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
import signal
import threading
from typing import Any

import rclpy
from rclpy.node import Node

logger = logging.getLogger(__name__)


def _patch_bool_quantile_normalization() -> None:
    """Workaround for lerobot bug: QUANTILES normalization crashes on bool tensors.

    Dataset `.is_fresh` flags (dtype=bool) reach the quantile normalizer with
    bool stats (q01/q99). PyTorch does not support subtraction on bool tensors,
    so we cast to float before the q99-q01 step.
    """
    import torch
    from lerobot.processor.normalize_processor import _NormalizationMixin

    orig = _NormalizationMixin._apply_transform

    def _patched(self, tensor, key, feature_type, *, inverse=False):
        if tensor.dtype == torch.bool:
            tensor = tensor.float()
            # Also cast cached stats so q99 - q01 doesn't fail
            if key in self._tensor_stats:
                stats = self._tensor_stats[key]
                for stat_key in ('q01', 'q99', 'q10', 'q90', 'min', 'max', 'mean', 'std'):
                    if stat_key in stats and stats[stat_key].dtype == torch.bool:
                        stats[stat_key] = stats[stat_key].float()
        return orig(self, tensor, key, feature_type, inverse=inverse)

    _NormalizationMixin._apply_transform = _patched


def _patch_pi05_action_dim_padding() -> None:
    """Zero-pad action projection weights when max_action_dim > pretrained checkpoint dim.

    When max_action_dim is increased (e.g. 32→34 to add base x,y,θ), the pretrained
    action_in_proj.weight (1024x32) and action_out_proj.weight/bias (32x1024 / 32) no
    longer match the model's shapes (1024x34, 34x1024, 34). PyTorch load_state_dict
    raises a size-mismatch error, which the from_pretrained outer try/except catches by
    returning a fully random model — losing all pretrained weights.

    This patch intercepts _fix_pytorch_state_dict_keys (called just before load_state_dict)
    and pads the extra columns/rows with zeros, so pretrained weights load for dims 0..N-1
    and only the new dimensions start from zero.
    """
    import torch
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    orig_fix = PI05Policy._fix_pytorch_state_dict_keys

    def _patched_fix(self, state_dict, model_config):
        fixed = orig_fix(self, state_dict, model_config)
        model_action_dim = self.model.action_in_proj.in_features

        for key in list(fixed.keys()):
            val = fixed[key]
            # action_in_proj.weight: (width, ckpt_dim) → (width, model_dim)
            if key.endswith('action_in_proj.weight') and val.ndim == 2 and val.shape[1] < model_action_dim:
                extra = model_action_dim - val.shape[1]
                pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                fixed[key] = torch.cat([val, pad], dim=1)
            # action_out_proj.weight: (ckpt_dim, width) → (model_dim, width)
            elif key.endswith('action_out_proj.weight') and val.ndim == 2 and val.shape[0] < model_action_dim:
                extra = model_action_dim - val.shape[0]
                pad = torch.zeros(extra, val.shape[1], dtype=val.dtype, device=val.device)
                fixed[key] = torch.cat([val, pad], dim=0)
            # action_out_proj.bias: (ckpt_dim,) → (model_dim,)
            elif key.endswith('action_out_proj.bias') and val.ndim == 1 and val.shape[0] < model_action_dim:
                extra = model_action_dim - val.shape[0]
                pad = torch.zeros(extra, dtype=val.dtype, device=val.device)
                fixed[key] = torch.cat([val, pad], dim=0)

        return fixed

    PI05Policy._fix_pytorch_state_dict_keys = _patched_fix


class TrainNode(Node):
    """ROS 2 node that launches LeRobot training as a background thread."""

    def __init__(self) -> None:
        """Initialise the training node and declare all ROS 2 parameters."""
        super().__init__('sobits_vla_training', allow_undeclared_parameters=True)
        self._training_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()

        self._declare_parameters()
        self.get_logger().info('sobits_vla_training node initialised.')

    def _declare_parameters(self) -> None:
        from rcl_interfaces.msg import ParameterDescriptor

        def _p(desc: str) -> ParameterDescriptor:
            d = ParameterDescriptor()
            d.description = desc
            return d

        self.declare_parameter('policy', 'smolvla', _p('Policy type: smolvla|pi0|pi05|pi0_fast'))

        self.declare_parameter(
            'dataset.repo_id', '', _p('HF Hub repo_id or local path to LeRobotDataset'))
        self.declare_parameter(
            'dataset.val_split', 0.1, _p('Fraction of episodes for validation'))
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
            'training.grad_accum', 1, _p('Gradient accumulation steps'))
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
            'hub.push_on_finish', True, _p('Push model to HF Hub after training'))
        self.declare_parameter(
            'hub.repo_id', '', _p('HF Hub target repo_id for push'))
        self.declare_parameter(
            'hub.private', False, _p('Make Hub repo private'))
        self.declare_parameter(
            'hub.push_best', True, _p('Push best checkpoint (lowest val loss)'))

        self.declare_parameter(
            'vram.limit_gb', 15.5, _p('VRAM limit in GB (training aborts if exceeded)'))
        self.declare_parameter(
            'vram.verbose', True, _p('Log VRAM estimate even when passing'))

        # Policy override sub-parameters. ROS 2 only loads YAML values for declared
        # parameters, so every key that may appear under policy_overrides: in the YAML
        # must be declared here. make_policy_config filters to valid fields per policy.
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
        self.declare_parameter(
            'peft.full_training_modules', [], _p('Modules to fully fine-tune alongside LoRA'))

    def _collect_params(self) -> dict[str, Any]:
        """Read all declared parameters into a flat dict keyed by dotted name."""
        names = [
            'policy',
            'dataset.repo_id', 'dataset.val_split', 'dataset.num_workers',
            'dataset.rename_map',
            'checkpoint.output_dir', 'checkpoint.resume', 'checkpoint.overwrite',
            'checkpoint.pretrained_path', 'checkpoint.save_freq',
            'checkpoint.save_checkpoint',
            'training.steps', 'training.batch_size', 'training.grad_accum',
            'training.seed', 'training.use_policy_training_preset',
            'training.log_freq', 'training.eval_freq',
            'num_gpus',
            'wandb.enable', 'wandb.project', 'wandb.entity',
            'wandb.run_name', 'wandb.notes',
            'hub.push_on_finish', 'hub.repo_id', 'hub.private', 'hub.push_best',
            'vram.limit_gb', 'vram.verbose',
            'policy_overrides',
            'peft.method_type', 'peft.r', 'peft.lora_alpha', 'peft.lora_dropout',
            'peft.target_modules', 'peft.full_training_modules',
        ]

        params: dict[str, Any] = {}
        for name in names:
            try:
                params[name] = self.get_parameter(name).value
            except Exception:
                pass
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

        from sobits_vla_training.vram_estimator import check_vram
        check_vram(
            policy_type=policy_type,
            batch_size=params.get('training.batch_size', 32),
            limit_gb=params.get('vram.limit_gb', 15.5),
            verbose=params.get('vram.verbose', True),
        )

        if params.get('checkpoint.overwrite', False) and not params.get('checkpoint.resume', False):
            import shutil
            from pathlib import Path
            out = Path(params.get('checkpoint.output_dir', '')).expanduser().resolve()
            if out.is_dir():
                shutil.rmtree(out)
                self.get_logger().info(f'Overwrite: removed existing output dir {out}')

        from sobits_vla_training.config_builder import build_accelerator, build_train_config
        train_cfg, peft_extra = build_train_config(params)

        num_gpus: int = params.get('num_gpus', 1)
        use_amp: bool = getattr(train_cfg.policy, 'use_amp', False)
        accelerator = build_accelerator(num_gpus=num_gpus, use_amp=use_amp)

        # Inject lora_alpha / lora_dropout into peft config so lerobot_train
        # picks them up via dataclasses.asdict(cfg.peft) → wrap_with_peft.
        if train_cfg.peft is not None and peft_extra:
            import dataclasses as _dc
            for key, val in peft_extra.items():
                if key not in {f.name for f in _dc.fields(train_cfg.peft)}:
                    object.__setattr__(train_cfg.peft, key, val)
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

        _patch_bool_quantile_normalization()

        from lerobot.scripts.lerobot_train import train
        train(train_cfg, accelerator=accelerator)

        self.get_logger().info('Training complete.')

        hub_repo_id: str = params.get('hub.repo_id', '')
        if params.get('hub.push_on_finish', True) and hub_repo_id:
            self.get_logger().info(f'Model pushed to HF Hub: {hub_repo_id}')
        elif params.get('hub.push_on_finish', True) and not hub_repo_id:
            self.get_logger().info('hub.repo_id not set — skipping Hub push.')


def main(args=None) -> None:
    """Entry point for the sobits_vla_training ROS 2 node."""
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
