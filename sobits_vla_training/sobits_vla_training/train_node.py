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


def _patch_pi0fast_peft_targets() -> None:
    """PI0FastPolicy lacks _get_default_peft_targets in lerobot 0.5.1, so LoRA
    training (peft.method_type: LORA with empty target_modules) raises
    ValueError at wrap_with_peft. Provide the natural default: LoRA on the
    PaliGemma language-model attention q/v projections. pi0_fast has no action
    expert or action projections — actions are FAST tokens decoded through the
    LM head — so the language model is the only sensible adaptation target.
    The patch is skipped automatically if a future lerobot version adds its own.
    """
    try:
        from lerobot.policies.pi0_fast.modeling_pi0_fast import PI0FastPolicy
        qualname = getattr(PI0FastPolicy._get_default_peft_targets, '__qualname__', '')
        if not qualname.startswith('PI0FastPolicy'):
            def _targets(self) -> dict:
                return {
                    'target_modules': r'(.*\.language_model\..*\.self_attn\.(q|v)_proj)',
                    'modules_to_save': [],
                }
            PI0FastPolicy._get_default_peft_targets = _targets
            logger.info('Patched PI0FastPolicy._get_default_peft_targets (LM q/v projections).')
    except ImportError:
        pass


def _patch_processor_registry() -> None:
    """Register 'relative_actions_processor' as an alias for 'delta_actions_processor' for compatibility."""
    try:
        from lerobot.processor.pipeline import ProcessorStepRegistry
        if 'delta_actions_processor' in ProcessorStepRegistry._registry:
            ProcessorStepRegistry._registry['relative_actions_processor'] = ProcessorStepRegistry._registry['delta_actions_processor']
            logger.info("Registered alias 'relative_actions_processor' -> 'delta_actions_processor'.")
    except Exception as e:
        logger.warning(f"Could not register relative_actions_processor alias: {e}")


def _patch_pi05_action_dim_padding() -> None:
    """Zero-pad or truncate projection weights when action/state dims differ from pre-trained weights.

    Prevents PyTorch load_state_dict mismatch crashes which cause LeRobot to fall back
    to fully randomized weights.
    """
    import torch

    def make_patched_fix(orig_fix):
        def _patched_fix(self, state_dict, model_config):
            fixed = orig_fix(self, state_dict, model_config)
            
            # Action dimension remapping
            model_action_dim = self.model.action_in_proj.in_features
            
            # State dimension remapping (PI0 has state_proj, PI05 does not)
            model_state_dim = None
            if hasattr(self.model, 'state_proj'):
                model_state_dim = self.model.state_proj.in_features

            for key in list(fixed.keys()):
                val = fixed[key]
                
                # action_in_proj.weight: (width, ckpt_dim) → (width, model_dim)
                if key.endswith('action_in_proj.weight') and val.ndim == 2:
                    if val.shape[1] < model_action_dim:
                        extra = model_action_dim - val.shape[1]
                        pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                        fixed[key] = torch.cat([val, pad], dim=1)
                    elif val.shape[1] > model_action_dim:
                        fixed[key] = val[:, :model_action_dim]
                
                # action_out_proj.weight: (ckpt_dim, width) → (model_dim, width)
                elif key.endswith('action_out_proj.weight') and val.ndim == 2:
                    if val.shape[0] < model_action_dim:
                        extra = model_action_dim - val.shape[0]
                        pad = torch.zeros(extra, val.shape[1], dtype=val.dtype, device=val.device)
                        fixed[key] = torch.cat([val, pad], dim=0)
                    elif val.shape[0] > model_action_dim:
                        fixed[key] = val[:model_action_dim, :]
                
                # action_out_proj.bias: (ckpt_dim,) → (model_dim,)
                elif key.endswith('action_out_proj.bias') and val.ndim == 1:
                    if val.shape[0] < model_action_dim:
                        extra = model_action_dim - val.shape[0]
                        pad = torch.zeros(extra, dtype=val.dtype, device=val.device)
                        fixed[key] = torch.cat([val, pad], dim=0)
                    elif val.shape[0] > model_action_dim:
                        fixed[key] = val[:model_action_dim]
                        
                # state_proj.weight: (width, ckpt_dim) → (width, model_dim)
                elif key.endswith('state_proj.weight') and val.ndim == 2 and model_state_dim is not None:
                    if val.shape[1] < model_state_dim:
                        extra = model_state_dim - val.shape[1]
                        pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                        fixed[key] = torch.cat([val, pad], dim=1)
                    elif val.shape[1] > model_state_dim:
                        fixed[key] = val[:, :model_state_dim]
                        
            return fixed
        return _patched_fix

    # Intercept PI05 Policy if available
    try:
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        orig_fix_pi05 = PI05Policy._fix_pytorch_state_dict_keys
        PI05Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi05)
    except ImportError:
        pass

    # Intercept PI0 Policy if available
    try:
        from lerobot.policies.pi0.modeling_pi0 import PI0Policy
        orig_fix_pi = PI0Policy._fix_pytorch_state_dict_keys
        PI0Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi)
    except ImportError:
        pass

    # Intercept PI0Fast Policy if available
    try:
        from lerobot.policies.pi0_fast.modeling_pi0_fast import PI0FastPolicy
        orig_fix_pi_fast = PI0FastPolicy._fix_pytorch_state_dict_keys
        PI0FastPolicy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi_fast)
    except ImportError:
        pass



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
        self.declare_parameter(
            'peft.full_training_modules', [], _p('Modules to fully fine-tune alongside LoRA'))

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
        ]

        params: dict[str, Any] = {}
        for name in names:
            try:
                params[name] = self.get_parameter(name).value
            except Exception:
                pass

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

    @staticmethod
    def _load_dataset_info(repo_id: str) -> dict | None:
        """Read info.json from the local dataset cache without instantiating LeRobotDataset."""
        try:
            from lerobot.utils.constants import HF_LEROBOT_HOME
            candidate = HF_LEROBOT_HOME / repo_id / 'meta' / 'info.json'
            if not candidate.exists():
                candidate = Path(repo_id) / 'meta' / 'info.json'
            if not candidate.exists():
                from huggingface_hub import hf_hub_download
                candidate = Path(
                    hf_hub_download(repo_id, 'meta/info.json', repo_type='dataset')
                )
            import json
            with open(candidate) as f:
                return json.load(f)
        except Exception:
            return None

    def _preflight_dataset_checks(self, params: dict) -> None:
        """Run dataset-aware pre-flight checks that require info.json."""
        repo_id: str = params.get('dataset.repo_id', '')
        po: dict = params.get('policy_overrides', {})

        info = self._load_dataset_info(repo_id)
        if info is None:
            self.get_logger().warn(
                f'Could not read meta/info.json for dataset "{repo_id}" — '
                'skipping dataset-aware pre-flight checks.'
            )
            return

        features = info.get('features', {})
        action_feature = features.get('action', {})
        action_names: list[str] = action_feature.get('names') or []
        action_shape: list[int] = action_feature.get('shape') or []
        actual_action_dim: int = action_shape[0] if action_shape else len(action_names)

        # max_action_dim / max_state_dim vs actual dataset dim
        max_action_dim: int = po.get('max_action_dim', 32)
        max_state_dim: int = po.get('max_state_dim', 32)
        if actual_action_dim > 0:
            if max_action_dim < actual_action_dim:
                raise RuntimeError(
                    f'max_action_dim={max_action_dim} < dataset action dim={actual_action_dim} '
                    f'for "{repo_id}" — joints would be silently truncated during training. '
                    f'Set max_action_dim >= {actual_action_dim} in policy_overrides.'
                )
            if max_state_dim < actual_action_dim:
                raise RuntimeError(
                    f'max_state_dim={max_state_dim} < dataset state dim={actual_action_dim} '
                    f'for "{repo_id}" — state would be silently truncated during training. '
                    f'Set max_state_dim >= {actual_action_dim} in policy_overrides.'
                )
            self.get_logger().info(
                f'Dim pre-flight passed: actual={actual_action_dim} '
                f'max_action_dim={max_action_dim} max_state_dim={max_state_dim}'
            )

        # relative_exclude_joints validation against actual joint names
        if po.get('use_relative_actions', False) and action_names:
            exclude: list[str] = po.get('relative_exclude_joints', ['gripper'])
            if exclude == ['gripper']:
                self.get_logger().warn(
                    'use_relative_actions=true but relative_exclude_joints is the default '
                    '[\"gripper\"], which matches no joint in SOBIT HOME. '
                    'Velocity joints (base_x, base_y, base_theta) will be delta-converted. '
                    'Set relative_exclude_joints explicitly in policy_overrides.'
                )
            unknown = [j for j in exclude if j not in action_names]
            if unknown:
                self.get_logger().warn(
                    f'relative_exclude_joints contains names not found in dataset action '
                    f'feature names — these joints will not be excluded from delta conversion: '
                    f'{unknown}. Dataset action names: {action_names}'
                )
        elif po.get('use_relative_actions', False) and not action_names:
            # Fall back to the original default-only check when names are unavailable
            exclude = po.get('relative_exclude_joints', ['gripper'])
            if exclude == ['gripper']:
                self.get_logger().warn(
                    'use_relative_actions=true but relative_exclude_joints appears to be the '
                    'default [\"gripper\"], which matches no joint in SOBIT HOME. '
                    'Velocity joints (base_x, base_y, base_theta) will be delta-converted. '
                    'Set relative_exclude_joints explicitly in policy_overrides if this is unintended.'
                )

    def _run_training(self) -> None:
        params = self._collect_params()

        policy_type: str = params.get('policy', 'smolvla')
        self.get_logger().info(f'Policy: {policy_type}')

        self._preflight_dataset_checks(params)

        from sobits_vla_training.config_builder import find_package_src_dir, build_accelerator, build_train_config

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

        if params.get('checkpoint.overwrite', False) and not params.get('checkpoint.resume', False):
            import shutil
            # Safety guard: only delete if it's a sub-directory and not CWD, parent CWD, or root directory
            if out.is_dir() and out != Path.cwd() and out != Path.cwd().parent and out != Path('/'):
                shutil.rmtree(out)
                self.get_logger().info(f'Overwrite: removed existing output dir {out}')

        train_cfg, peft_extra = build_train_config(params)

        num_gpus: int = params.get('num_gpus', 1)
        use_amp: bool = getattr(train_cfg.policy, 'use_amp', False)
        accelerator = build_accelerator(num_gpus=num_gpus, use_amp=use_amp)

        if train_cfg.peft is not None and peft_extra:
            import dataclasses as _dc
            from lerobot.configs.default import PeftConfig as _PeftConfig
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

        _patch_bool_quantile_normalization()
        _patch_pi05_action_dim_padding()
        _patch_pi0fast_peft_targets()
        _patch_processor_registry()

        from lerobot.scripts.lerobot_train import train
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
