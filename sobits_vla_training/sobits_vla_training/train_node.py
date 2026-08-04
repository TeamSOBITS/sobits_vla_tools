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

import rclpy
from rclpy.node import Node

from sobits_vla_common import runtime_deps
from sobits_vla_common.lerobot_compat import apply_training_patches
from sobits_vla_training.preflight import run_preflight_checks

logger = logging.getLogger(__name__)


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
        from rcl_interfaces.msg import ParameterDescriptor

        def _p(desc: str) -> ParameterDescriptor:
            d = ParameterDescriptor()
            d.description = desc
            return d

        self.declare_parameter(
            'policy', 'smolvla',
            _p('Policy type: smolvla|pi0|pi05|pi0_fast|act|groot|vla_jepa|molmoact2'))

        self.declare_parameter(
            'dataset.repo_id', '', _p('HF Hub repo_id or local path to LeRobotDataset'))
        self.declare_parameter(
            'dataset.num_workers', 4, _p('DataLoader worker count'))
        self.declare_parameter(
            'dataset.rename_map', [], _p('Feature rename map {old: new}'))

        self.declare_parameter(
            'checkpoint.output_dir', './outputs/train', _p('Output directory for checkpoints'))
        self.declare_parameter(
            'checkpoint.resume', False, _p('Resume from last checkpoint in output_dir'))
        self.declare_parameter(
            'checkpoint.overwrite', False, _p('Delete output_dir before training if it exists'))
        self.declare_parameter(
            'checkpoint.pretrained_path', '', _p('Local path or HF repo_id for init weights'))
        self.declare_parameter(
            'checkpoint.save_freq', 20000, _p('Save checkpoint every N steps'))
        self.declare_parameter(
            'checkpoint.save_checkpoint', True, _p('Whether to save checkpoints'))

        self.declare_parameter(
            'training.steps', 100000, _p('Total gradient update steps'))
        self.declare_parameter(
            'training.batch_size', 32, _p('Per-GPU batch size'))
        self.declare_parameter(
            'training.seed', 1000, _p('Random seed'))
        self.declare_parameter(
            'training.use_policy_training_preset', True,
            _p('Use policy built-in optimizer preset'))
        self.declare_parameter(
            'training.log_freq', 200, _p('Log metrics every N steps'))
        self.declare_parameter(
            'training.eval_freq', 20000, _p('Evaluate every N steps (0=disable)'))

        self.declare_parameter(
            'num_gpus', 1, _p('Number of GPUs (0=CPU, 1=single, >1=DDP)'))

        self.declare_parameter(
            'wandb.enable', True, _p('Enable Weights & Biases logging'))
        self.declare_parameter(
            'wandb.project', 'sobits_vla_training', _p('W&B project name'))
        self.declare_parameter(
            'wandb.entity', '', _p('W&B entity (team/user)'))
        self.declare_parameter(
            'wandb.run_name', '', _p('W&B run name (auto-generated if empty)'))
        self.declare_parameter(
            'wandb.notes', '', _p('W&B run notes'))

        self.declare_parameter(
            'hub.push_to_hub', True, _p('Push final model to HF Hub after training'))
        self.declare_parameter(
            'hub.repo_id', '', _p('HF Hub target repo_id for push'))
        self.declare_parameter(
            'hub.private', False, _p('Make Hub repo private'))

        # Policy override sub-parameters
        _po = 'policy_overrides.'
        self.declare_parameter(_po + 'max_state_dim', 32)
        self.declare_parameter(_po + 'max_action_dim', 32)
        self.declare_parameter(_po + 'chunk_size', 50)
        self.declare_parameter(_po + 'n_action_steps', 50)
        self.declare_parameter(_po + 'n_obs_steps', 1)
        self.declare_parameter(_po + 'paligemma_variant', 'gemma_2b')
        self.declare_parameter(_po + 'action_expert_variant', 'gemma_300m')
        self.declare_parameter(_po + 'dtype', 'bfloat16')
        self.declare_parameter(_po + 'num_inference_steps', 10)
        self.declare_parameter(_po + 'image_resolution', [224, 224])
        self.declare_parameter(_po + 'empty_cameras', 0)
        self.declare_parameter(_po + 'freeze_vision_encoder', False)
        self.declare_parameter(_po + 'gradient_checkpointing', True)
        self.declare_parameter(_po + 'train_expert_only', False)
        self.declare_parameter(_po + 'use_peft', False)  # True = load existing adapter; keep False
        self.declare_parameter(_po + 'tokenizer_max_length', 200)
        self.declare_parameter(_po + 'use_relative_actions', False)
        self.declare_parameter(_po + 'relative_exclude_joints', ['gripper'])
        self.declare_parameter(_po + 'optimizer_lr', 2.5e-5)
        self.declare_parameter(_po + 'optimizer_weight_decay', 0.01)
        self.declare_parameter(_po + 'optimizer_grad_clip_norm', 1.0)
        self.declare_parameter(_po + 'scheduler_warmup_steps', 1000)
        self.declare_parameter(_po + 'scheduler_decay_steps', 30000)
        self.declare_parameter(_po + 'scheduler_decay_lr', 2.5e-6)
        self.declare_parameter(_po + 'compile_model', False)

        self.declare_parameter(
            'peft.method_type', '', _p('PEFT method: LORA or empty to disable'))
        self.declare_parameter(
            'peft.r', 16, _p('LoRA rank'))
        self.declare_parameter(
            'peft.lora_alpha', 32, _p('LoRA alpha (scaling = alpha/r)'))
        self.declare_parameter(
            'peft.lora_dropout', 0.05, _p('LoRA dropout probability'))
        self.declare_parameter(
            'peft.target_modules', '', _p('LoRA target modules (empty = policy default)'))
        # dynamic_typing: an empty [] default infers BYTE_ARRAY and clashes
        # with YAML STRING_ARRAY overrides (and a hard STRING_ARRAY type
        # would clash with the `[]` most configs set). Values are normalized
        # to list[str] in _collect_params.
        _ftm_desc = _p('Modules to fully fine-tune alongside LoRA')
        _ftm_desc.dynamic_typing = True
        self.declare_parameter('peft.full_training_modules', [], _ftm_desc)

        from rclpy.parameter import Parameter
        self.declare_parameter('robot.descriptor_id', '', _p('Robot descriptor ID'))
        # Typed STRING_ARRAY (no default []): empty list infers BYTE_ARRAY,
        # clashing with the YAML STRING_ARRAY override.
        self.declare_parameter('robot.active_groups', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('robot.active_cameras', Parameter.Type.STRING_ARRAY)
        self.declare_parameter(
            'robot.active_mobile_base', True, _p('Whether to include mobile base')
        )

    def _collect_params(self) -> dict[str, Any]:
        """Read all declared parameters into a flat dict keyed by dotted name."""
        names = [
            'policy',
            'dataset.repo_id', 'dataset.num_workers',
            'dataset.rename_map',
            'checkpoint.output_dir', 'checkpoint.resume', 'checkpoint.overwrite',
            'checkpoint.pretrained_path', 'checkpoint.save_freq',
            'checkpoint.save_checkpoint',
            'training.steps', 'training.batch_size',
            'training.seed', 'training.use_policy_training_preset',
            'training.log_freq', 'training.eval_freq',
            'num_gpus',
            'wandb.enable', 'wandb.project', 'wandb.entity',
            'wandb.run_name', 'wandb.notes',
            'hub.push_to_hub', 'hub.repo_id', 'hub.private',
            'peft.method_type', 'peft.r', 'peft.lora_alpha', 'peft.lora_dropout',
            'peft.target_modules', 'peft.full_training_modules',
            'robot.descriptor_id', 'robot.active_groups',
            'robot.active_cameras', 'robot.active_mobile_base',
        ]

        params: dict[str, Any] = {}
        for name in names:
            try:
                params[name] = self.get_parameter(name).value
            except Exception:
                pass

        # dynamic_typing param: YAML may deliver bytes/None; normalize to
        # list[str] so build_peft_config always sees a string list.
        ftm = params.get('peft.full_training_modules')
        params['peft.full_training_modules'] = (
            [str(m) for m in ftm] if ftm else []
        )

        # Collect policy_overrides
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
        params['policy_overrides'] = po

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
            find_package_src_dir,
        )

        out_dir_raw = params.get('checkpoint.output_dir', '')
        package_src_dir = find_package_src_dir()

        if not out_dir_raw:
            out = package_src_dir / 'outputs'
        else:
            raw_path = Path(out_dir_raw).expanduser()
            if raw_path.is_absolute():
                out = raw_path
            else:
                if raw_path.parts and raw_path.parts[0] == 'outputs':
                    out = (package_src_dir / raw_path).resolve()
                else:
                    out = (package_src_dir / 'outputs' / raw_path).resolve()

        if (
            params.get('checkpoint.overwrite', False)
            and not params.get('checkpoint.resume', False)
        ):
            # Refuse to delete the shared outputs root (empty output_dir) —
            # that is the parent of every run, not one run's directory.
            if not out_dir_raw:
                raise RuntimeError(
                    'checkpoint.overwrite=true but checkpoint.output_dir is '
                    'empty — this would delete the shared outputs root '
                    f'({out}), containing every prior run. Set a run-named '
                    'checkpoint.output_dir.'
                )
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

        train_cfg, peft_extra = build_train_config(params)

        num_gpus: int = params.get('num_gpus', 1)
        use_amp: bool = getattr(train_cfg.policy, 'use_amp', False)
        accelerator = build_accelerator(num_gpus=num_gpus, use_amp=use_amp)

        if train_cfg.peft is not None and peft_extra:
            # lerobot 0.6.0 upstreamed `lora_alpha` onto PeftConfig (PR #3573),
            # so it's now in `_known` and flows through the plain `_base.update()`
            # below via native attribute assignment — no dynamic field needed for
            # it. `lora_dropout` still has no upstream field, so it's the only
            # one that ends up in `_new_fields` and gets injected via
            # make_dataclass. This block requires no version gate: it always
            # only injects whatever peft_extra keys are missing from upstream.
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
            targets = (
                'policy default' if not train_cfg.peft.target_modules
                else train_cfg.peft.target_modules
            )
            self.get_logger().info(
                f'LoRA | r={train_cfg.peft.r} '
                f'alpha={peft_extra.get("lora_alpha", "default")} '
                f'dropout={peft_extra.get("lora_dropout", "default")} '
                f'targets={targets}'
            )

        self.get_logger().info(
            f'Starting training | steps={train_cfg.steps} '
            f'batch={train_cfg.batch_size} '
            f'output={train_cfg.output_dir}'
        )

        if train_cfg.resume:
            # lerobot's resume path reads --config_path from sys.argv
            # (draccus CLI plumbing our in-process train() call never
            # provides). Point it at this run's latest checkpoint.
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

            # PEFT checkpoints contain adapter_model.safetensors, no
            # model.safetensors. Resume does NOT reload the policy config
            # from the checkpoint — it uses this in-process config, where
            # use_peft is false (it only means "load an adapter" to the
            # factory) — so the factory takes the plain-weights branch and
            # dies on the missing file. Flip the flag on OUR config when the
            # checkpoint is unambiguously an adapter; lerobot_train skips
            # wrap_with_peft when the policy already is a PeftModel.
            if (ckpt_cfg.parent / 'adapter_model.safetensors').exists():
                train_cfg.policy.use_peft = True
                self.get_logger().info(
                    'Adapter checkpoint detected — resuming via the PEFT '
                    'loading branch (policy.use_peft=true).'
                )

            # validate() only builds the optimizer/scheduler presets when NOT
            # resuming (the CLI flow reloads them from the saved train
            # config, which our in-process config never was) — build them
            # here or make_optimizer_and_scheduler raises. The optimizer
            # STATE is still restored from the checkpoint's training_state/.
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
