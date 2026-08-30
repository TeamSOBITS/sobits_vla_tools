#!/usr/bin/env python3
# Copyright (c) 2026, Team SOBITS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS 2 training node for LeRobot VLA policies.

Reads all training parameters from the ROS parameter server (populated
from training_config.yaml), runs a VRAM pre-flight check, builds a
TrainPipelineConfig, and delegates to LeRobot's train() function.

Training runs in a dedicated thread so the ROS spin loop stays alive
for parameter introspection and clean shutdown via Ctrl-C / SIGTERM.
"""

from __future__ import annotations

import logging
from pathlib import Path
import signal
import threading
from typing import Any

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node

from sobits_vla_common import runtime_deps
from sobits_vla_common.lerobot_compat import apply_training_patches
from sobits_vla_common.param_schema import declare_from_schema, P, read_flat
from sobits_vla_training.preflight import run_preflight_checks

logger = logging.getLogger(__name__)


def _pd(desc: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=desc)


# Static-name declares only. peft.full_training_modules (dynamic_typing) and
# policy_overrides' undeclared-key discovery stay hand-written below.
_SCHEMA = {
    'policy': P('smolvla', descriptor=_pd(
        'Policy type: smolvla|pi0|pi05|pi0_fast|act|groot|vla_jepa|molmoact2')),
    'dataset': {
        'repo_id': P('', descriptor=_pd('HF Hub repo_id or local path to LeRobotDataset')),
        'num_workers': P(4, descriptor=_pd('DataLoader worker count')),
        'rename_map': P([''], descriptor=_pd('Feature renames, one "old:new" per entry')),
        'eval_split': P(0.0, descriptor=_pd(
            'Fraction of episodes per task held out for eval')),
    },
    'checkpoint': {
        'output_dir': P('', descriptor=_pd(
            'Checkpoint dir; relative -> lerobotmodel/, absolute -> verbatim')),
        'resume': P(False, descriptor=_pd('Resume from last checkpoint in output_dir')),
        'overwrite': P(False, descriptor=_pd(
            'Delete output_dir before training if it exists')),
        'pretrained_path': P('', descriptor=_pd(
            'Local path or HF repo_id for init weights')),
        'save_freq': P(20000, descriptor=_pd('Save checkpoint every N steps')),
        'save_checkpoint': P(True, descriptor=_pd('Whether to save checkpoints')),
    },
    'training': {
        'steps': P(100000, descriptor=_pd('Total gradient update steps')),
        'batch_size': P(32, descriptor=_pd('Per-GPU batch size')),
        'seed': P(1000, descriptor=_pd('Random seed')),
        'use_policy_training_preset': P(True, descriptor=_pd(
            'Use policy built-in optimizer preset')),
        'log_freq': P(200, descriptor=_pd('Log metrics every N steps')),
        'eval_steps': P(0, descriptor=_pd(
            'Held-out eval-loss every N steps; needs dataset.eval_split > 0. 0=off')),
        'optimizer_type': P('', descriptor=_pd(
            "Override optimizer algorithm (empty=preset, 'sgd'=SGDConfig)")),
        'optimizer_sgd': {
            'lr': P(1e-3, descriptor=_pd('SGD learning rate')),
            'momentum': P(0.0, descriptor=_pd('SGD momentum')),
            'dampening': P(0.0, descriptor=_pd('SGD dampening')),
            'nesterov': P(False, descriptor=_pd('SGD Nesterov momentum')),
            'weight_decay': P(0.0, descriptor=_pd('SGD weight decay')),
            'grad_clip_norm': P(10.0, descriptor=_pd('SGD gradient clip norm')),
        },
        'scheduler_warmup_steps_override': P(1000, descriptor=_pd(
            'Warmup steps for the override scheduler (optimizer_type set)')),
    },
    'num_gpus': P(1, descriptor=_pd('Number of GPUs (0=CPU, 1=single, >1=DDP)')),
    'wandb': {
        'project': P('sobits_vla_training', descriptor=_pd('W&B project name')),
        'entity': P('', descriptor=_pd('W&B entity (team/user)')),
        'run_name': P('', descriptor=_pd('W&B run name (auto-generated if empty)')),
        'run_id': P('', descriptor=_pd(
            'W&B run id to resume (empty = new run; auto-recovered on resume)')),
        'mode': P('', descriptor=_pd("W&B mode: online|offline ('' = disabled)")),
        'notes': P('', descriptor=_pd('W&B run notes')),
        'disable_artifact': P(False, descriptor=_pd(
            'Skip uploading checkpoint artifacts to W&B (HF hub still gets them)')),
    },
    'hub': {
        'push_to_hub': P(True, descriptor=_pd('Push final model to HF Hub after training')),
        'repo_id': P('', descriptor=_pd('HF Hub target repo_id for push')),
        'private': P(False, descriptor=_pd('Make Hub repo private')),
        'save_checkpoints': P(False, descriptor=_pd(
            'Push each saved checkpoint to Hub, not just final')),
    },
    # No ParameterDescriptor in the original code -- left plain to match.
    'policy_overrides': {
        'max_state_dim': P(32),
        'max_action_dim': P(32),
        'chunk_size': P(50),
        'n_action_steps': P(50),
        'n_obs_steps': P(1),
        'paligemma_variant': P('gemma_2b'),
        'action_expert_variant': P('gemma_300m'),
        'dtype': P('bfloat16'),
        'num_inference_steps': P(10),
        'image_resolution': P([224, 224]),
        'empty_cameras': P(0),
        'freeze_vision_encoder': P(False),
        'gradient_checkpointing': P(True),
        'train_expert_only': P(False),
        'use_peft': P(False),  # True = load existing adapter; keep False
        'tokenizer_max_length': P(200),
        'use_relative_actions': P(False),
        'relative_exclude_joints': P(['']),
        'optimizer_lr': P(2.5e-5),
        'optimizer_weight_decay': P(0.01),
        'optimizer_grad_clip_norm': P(1.0),
        'scheduler_warmup_steps': P(1000),
        'scheduler_decay_steps': P(30000),
        'scheduler_decay_lr': P(2.5e-6),
        'compile_model': P(False),
    },
    'peft': {
        'method_type': P('', descriptor=_pd('PEFT method: LORA or empty to disable')),
        'r': P(16, descriptor=_pd('LoRA rank')),
        'lora_alpha': P(32, descriptor=_pd('LoRA alpha (scaling = alpha/r)')),
        'lora_dropout': P(0.05, descriptor=_pd('LoRA dropout probability')),
        'target_modules': P('', descriptor=_pd(
            'LoRA target modules (empty = policy default)')),
    },
    'robot': {
        'descriptor_id': P('', descriptor=_pd('Robot descriptor ID')),
        # Trim the shared descriptor to this config's subset; unknown names
        # raise so a typo fails loudly instead of training a wrong morphology.
        'exclude': {
            'groups': P(['']),
            'cameras': P(['']),
            'ee_poses': P(['']),
            'joints': P(['']),
            'mobile_base': P(False, descriptor=_pd('Exclude the mobile base')),
        },
    },
}


class TrainNode(Node):
    """ROS 2 node that launches LeRobot training as a background thread."""

    def __init__(self) -> None:
        """Initialise the training node and declare all ROS 2 parameters."""
        super().__init__('sobits_vla_training', allow_undeclared_parameters=True)
        self._training_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()

        self._declare_parameters()

        from sobits_vla_common.lerobot_adapter import describe
        seam = describe()
        self.get_logger().info(
            f'lerobot seam: version={seam["version"]} is_v06={seam["is_v06"]} '
            f'unresolvable={seam["unresolvable"]}'
        )

        self.get_logger().info('sobits_vla_training node initialised.')

    def _declare_parameters(self) -> None:
        declare_from_schema(self, _SCHEMA)

        # dynamic_typing: an empty [] default infers BYTE_ARRAY, clashing with YAML
        # STRING_ARRAY overrides; values are normalized to list[str] in _collect_params.
        _ftm_desc = _pd('Modules to fully fine-tune alongside LoRA')
        _ftm_desc.dynamic_typing = True
        self.declare_parameter('peft.full_training_modules', [], _ftm_desc)

    def _collect_params(self) -> dict[str, Any]:
        """Read all declared parameters into a flat dict keyed by dotted name."""
        # policy_overrides.* is declared via the schema but read separately below
        # (nested dict, plus undeclared keys) -- exclude it here or it would also
        # land as flat 'policy_overrides.foo' keys nothing consumes.
        _flat_schema = {k: v for k, v in _SCHEMA.items() if k != 'policy_overrides'}
        params: dict[str, Any] = read_flat(self, _flat_schema)

        # dynamic_typing param, hand-read: not schema-expressible (see declare above).
        # YAML may deliver bytes/None; normalize to list[str] so
        # build_peft_config always sees a string list.
        ftm = self.get_parameter('peft.full_training_modules').value
        params['peft.full_training_modules'] = [str(m) for m in ftm] if ftm else []

        po: dict[str, Any] = {}
        try:
            overrides = getattr(self, '_parameter_overrides', None) or {}
            for pname, param in overrides.items():
                if pname.startswith('policy_overrides.'):
                    po[pname[len('policy_overrides.'):]] = param.value
        except Exception as exc:
            self.get_logger().warning(f'policy_overrides read failed: {exc}')

        if not po:
            try:
                result = self.list_parameters(prefixes=['policy_overrides'], depth=2)
                for pname in result.names:
                    if pname.startswith('policy_overrides.'):
                        key = pname[len('policy_overrides.'):]
                        try:
                            po[key] = self.get_parameter(pname).value
                        except Exception:
                            pass
            except Exception as exc:
                self.get_logger().warning(f'policy_overrides discovery failed: {exc}')
        # Strip the [''] empty-list sentinel (see param_schema._read_leaf) --
        # this hand-written path bypasses the schema reader's filtering.
        params['policy_overrides'] = {
            k: [x for x in v if x != ''] if isinstance(v, list) else v
            for k, v in po.items()
        }

        return params

    def start_training(self) -> None:
        """Spawn the background training thread."""
        self._training_thread = threading.Thread(
            target=self._training_worker,
            name='training_worker',
            daemon=True,
        )
        self._training_thread.start()
        self.get_logger().info('Training thread started.')

    def _training_worker(self) -> None:
        try:
            self._run_training()
        except Exception as exc:
            import traceback
            self.get_logger().error(
                f'Training failed: {exc}\n{traceback.format_exc()}'
            )
        finally:
            self._shutdown_event.set()
            rclpy.shutdown()

    def _run_training(self) -> None:
        params = self._collect_params()

        policy_type: str = params.get('policy', 'smolvla')
        self.get_logger().info(f'Policy: {policy_type}')

        run_preflight_checks(params, ros_logger=self.get_logger())

        from sobits_vla_training.config_builder import (
            build_accelerator,
            build_train_config,
            resolve_output_dir,
        )

        # Relative -> <package_src>/lerobotmodel/, absolute -> verbatim.
        # See resolve_output_dir() for the full table.
        out = resolve_output_dir(
            params.get('checkpoint.output_dir', ''),
            params.get('hub.repo_id', '') or '',
        )

        if (
            params.get('checkpoint.overwrite', False)
            and not params.get('checkpoint.resume', False)
        ):
            import shutil
            # Safety guard: only delete if it's a sub-directory and not CWD,
            # parent CWD, or root directory
            if (
                out.is_dir()
                and out != Path.cwd()
                and out != Path.cwd().parent
                and out != Path('/')
            ):
                shutil.rmtree(out)
                self.get_logger().info(f'Overwrite: removed existing output dir {out}')

        train_cfg, peft_extra = build_train_config(params, output_dir=out)

        num_gpus: int = params.get('num_gpus', 1)
        use_amp: bool = getattr(train_cfg.policy, 'use_amp', False)
        accelerator = build_accelerator(num_gpus=num_gpus, use_amp=use_amp)

        if train_cfg.peft is not None and peft_extra:
            # lora_dropout has no PeftConfig field; inject it dynamically so
            # asdict(cfg.peft) carries it into wrap_with_peft.
            import dataclasses as _dc
            from sobits_vla_common.lerobot_adapter import PeftConfig as _PeftConfig
            _known = {f.name for f in _dc.fields(train_cfg.peft)}
            _new_fields = [(k, type(v), _dc.field(default=v))
                           for k, v in peft_extra.items() if k not in _known]
            if _new_fields:
                _ExtendedPeft = _dc.make_dataclass(
                    'ExtendedPeftConfig',
                    _new_fields,
                    bases=(_PeftConfig,),
                )
                _base = _dc.asdict(train_cfg.peft)
                _base.update(peft_extra)
                train_cfg.peft = _ExtendedPeft(**_base)
        if train_cfg.peft is not None:
            targets = (
                'policy default' if not train_cfg.peft.target_modules
                else train_cfg.peft.target_modules
            )
            alpha = train_cfg.peft.lora_alpha
            self.get_logger().info(
                f'LoRA | r={train_cfg.peft.r} '
                f'alpha={alpha if alpha is not None else "default"} '
                f'dropout={peft_extra.get("lora_dropout", "default")} '
                f'targets={targets}'
            )

        self.get_logger().info(
            f'Starting training | steps={train_cfg.steps} '
            f'batch={train_cfg.batch_size} '
            f'output={train_cfg.output_dir}'
        )

        if train_cfg.resume:
            # lerobot's resume path reads --config_path from sys.argv (draccus CLI
            # plumbing our in-process train() call never provides); point it here.
            import sys as _sys

            ckpt_cfg = (
                Path(train_cfg.output_dir)
                / 'checkpoints' / 'last' / 'pretrained_model'
                / 'train_config.json'
            )
            if not ckpt_cfg.exists():
                raise RuntimeError(
                    'checkpoint.resume=true but no checkpoint found at '
                    f'{ckpt_cfg} — check output_dir and the checkpoints/last '
                    'symlink (and that checkpoint.overwrite is false).'
                )

            # PEFT checkpoints hold adapter_model.safetensors, not model.safetensors; resume
            # never reloads use_peft, so flip it here or the factory dies on the missing file.
            if (ckpt_cfg.parent / 'adapter_model.safetensors').exists():
                train_cfg.policy.use_peft = True
                self.get_logger().info(
                    'Adapter checkpoint detected — resuming via the PEFT '
                    'loading branch (policy.use_peft=true).'
                )

            # validate() only builds optimizer/scheduler presets when NOT resuming; build
            # here or make_optimizer_and_scheduler raises (STATE still restores from disk).
            if train_cfg.use_policy_training_preset and train_cfg.optimizer is None:
                train_cfg.optimizer = train_cfg.policy.get_optimizer_preset()
                train_cfg.scheduler = train_cfg.policy.get_scheduler_preset()

            _sys.argv = list(_sys.argv) + [f'--config_path={ckpt_cfg}']
            self.get_logger().info(f'Resuming from checkpoint: {ckpt_cfg}')

        apply_training_patches()

        from sobits_vla_common.lerobot_adapter import train
        train(train_cfg, accelerator=accelerator)

        self.get_logger().info('Training complete.')

        hub_repo_id: str = params.get('hub.repo_id', '')
        if params.get('hub.push_to_hub', True) and hub_repo_id:
            self.get_logger().info(f'Model pushed to HF Hub: {hub_repo_id}')
        elif not hub_repo_id:
            self.get_logger().info('hub.repo_id not set — Hub push skipped.')
        else:
            self.get_logger().info('hub.push_to_hub=false — Hub push skipped.')


def main(args=None) -> None:
    """Entry point for the sobits_vla_training ROS 2 node."""
    # Checked here, not at module import, so lint/pytest collection of this
    # package still works on environments without the ML stack installed.
    runtime_deps.ensure({
        'lerobot': 'pip install lerobot[training]~=0.6.0',
        'accelerate': 'pip install lerobot[training]~=0.6.0',
        'wandb': 'pip install lerobot[training]~=0.6.0',
    })
    rclpy.init(args=args)

    node = TrainNode()

    def _sigterm_handler(sig, frame):
        node.get_logger().info('SIGTERM received — shutting down.')
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _sigterm_handler)

    node.start_training()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
