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
Build a LeRobot TrainPipelineConfig from ROS 2 parameters.

This module is the single translation layer between the ROS parameter
server (populated from training_config.yaml) and LeRobot's native
config dataclasses.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from sobits_vla_common.policy_registry import make_policy_config

logger = logging.getLogger(__name__)


def find_package_src_dir() -> Path:
    current_file = Path(__file__).resolve()

    # Check if 'build' or 'install' or 'site-packages' or 'dist-packages' is in the path parts
    in_workspace_build_or_install = any(
        part in current_file.parts
        for part in ('build', 'install', 'site-packages', 'dist-packages')
    )

    if not in_workspace_build_or_install:
        # We might be running directly from the source tree
        direct_parent = current_file.parent.parent
        if (direct_parent / 'package.xml').exists():
            return direct_parent

    # Try walking up to find a workspace root containing 'src'
    for p in current_file.parents:
        if (p / 'src').is_dir():
            src_dir = p / 'src'
            # Look for a directory containing package.xml and named 'sobits_vla_training'
            for path in src_dir.rglob('package.xml'):
                if path.parent.name == 'sobits_vla_training':
                    return path.parent
            break

    # Fallback to get_package_share_directory if available
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('sobits_vla_training'))
    except Exception:
        pass

    return current_file.parent.parent


def _default_model_root() -> Path:
    """Resolve the checkpoint output root: <package_src>/lerobotmodel/.

    Every non-absolute checkpoint.output_dir is resolved against this root, so
    it must resolve even before the directory exists (lerobot creates it) —
    hence the find_package_src_dir() fallback below.
    """
    candidate = Path(__file__).resolve().parent
    for _ in range(8):
        if (candidate / 'package.xml').exists() and (candidate / 'lerobotmodel').is_dir():
            return candidate / 'lerobotmodel'
        src_root = candidate / 'src'
        if src_root.is_dir():
            for pattern in ('*/sobits_vla_training', '*/*/sobits_vla_training'):
                for pkg_dir in src_root.glob(pattern):
                    if (pkg_dir / 'lerobotmodel').is_dir():
                        return pkg_dir / 'lerobotmodel'
        candidate = candidate.parent

    return find_package_src_dir() / 'lerobotmodel'


def resolve_output_dir(out_dir_raw: str, hub_repo_id: str) -> Path:
    """Resolve checkpoint.output_dir to the directory training writes into.

    Anything that is not absolute lands under <package_src>/lerobotmodel/::

        ""           -> lerobotmodel/<hub_repo_id>   (organization/model_name)
        "model_name" -> lerobotmodel/model_name
        "a/b"        -> lerobotmodel/a/b
        "/abs/path"  -> /abs/path                    (verbatim)

    Raises:
        RuntimeError: If both arguments are empty — with no identifier at all
            every run would collide on the shared lerobotmodel/ root.
    """
    model_root = _default_model_root()

    if not out_dir_raw:
        if not hub_repo_id:
            raise RuntimeError(
                'checkpoint.output_dir and hub.repo_id are both empty — '
                'refusing to default to a shared output root; set one '
                'explicitly.'
            )
        return model_root / hub_repo_id

    raw_path = Path(out_dir_raw).expanduser()
    if raw_path.is_absolute():
        return raw_path
    return (model_root / raw_path).resolve()


def _resolve_pretrained_path(raw: str) -> Path | str:
    """Return a Path for local files, or pass through HF Hub repo_id strings."""
    if not raw:
        return ''
    p = Path(raw)
    if p.exists():
        return p
    return raw


def build_train_config(params: dict[str, Any], output_dir: Path):
    """
    Construct a TrainPipelineConfig from a flat ROS parameter dict.

    Parameters
    ----------
    params : dict
        Flat dict of ROS parameter names to values.
    output_dir : Path
        Resolved checkpoint output directory (caller owns default/overwrite logic).

    Returns
    -------
    tuple[TrainPipelineConfig, dict]
        Fully initialised config and a dict of extra PEFT overrides
        (e.g. lora_alpha, lora_dropout) to pass to wrap_with_peft.

    """
    from sobits_vla_common.lerobot_adapter import DatasetConfig, TrainPipelineConfig, WandBConfig

    policy_type: str = params.get('policy', 'smolvla')
    device: str = _infer_device(params.get('num_gpus', 1))

    policy_overrides: dict = params.get('policy_overrides', {}) or {}

    desc_id = params.get('robot.descriptor_id', '')
    if desc_id:
        from sobits_vla_common.robot_descriptor import load_robot_descriptor
        desc = load_robot_descriptor(desc_id)

        active_groups = params.get('robot.active_groups', [])
        if not active_groups:
            active_groups = [g.name for g in desc.active_groups]
        active_mobile_base = params.get('robot.active_mobile_base', True)

        active_joint_features = []
        for g in desc.groups:
            if g.name in active_groups:
                active_joint_features.extend([j.feature for j in g.joints])

        n_base = 0
        if desc.mobile_base and active_mobile_base:
            n_base = len(desc.mobile_base.features)

        total_dim = len(active_joint_features) + n_base

        if 'max_state_dim' not in policy_overrides:
            policy_overrides['max_state_dim'] = max(32, total_dim)
        if 'max_action_dim' not in policy_overrides:
            policy_overrides['max_action_dim'] = max(32, total_dim)

        # Relative mode: keep base velocities + flagged groups absolute.
        # Descriptor-derived; explicit override wins.
        if (
            policy_overrides.get('use_relative_actions', False)
            and 'relative_exclude_joints' not in policy_overrides
        ):
            policy_overrides['relative_exclude_joints'] = desc.relative_exclude_features(
                active_groups=active_groups,
                active_mobile_base=active_mobile_base,
            )

    raw_pretrained = params.get('checkpoint.pretrained_path', '')
    if raw_pretrained:
        policy_overrides['pretrained_path'] = _resolve_pretrained_path(raw_pretrained)

    hub_repo_id: str = params.get('hub.repo_id', '') or ''
    push_to_hub: bool = bool(params.get('hub.push_to_hub', True))
    if hub_repo_id and push_to_hub:
        policy_overrides['repo_id'] = hub_repo_id
        policy_overrides['push_to_hub'] = True
        policy_overrides['private'] = bool(params.get('hub.private', False))
    else:
        policy_overrides['push_to_hub'] = False
        if hub_repo_id:
            policy_overrides['repo_id'] = hub_repo_id

    policy_cfg = make_policy_config(
        policy_type=policy_type,
        overrides=policy_overrides,
        device=device,
    )

    ds_repo_id: str = params.get('dataset.repo_id', '')
    if not ds_repo_id:
        raise ValueError('dataset.repo_id must be set in training_config.yaml.')

    eval_split: float = params.get('dataset.eval_split', 0.0)
    dataset_cfg = DatasetConfig(repo_id=ds_repo_id, eval_split=eval_split)

    # Version provenance: fold the lerobot version into notes since
    # WandBConfig has no dedicated metadata field. Keeps the W&B run
    # traceable to the lerobot version it trained under.
    from sobits_vla_common.lerobot_adapter import LEROBOT_VERSION
    lerobot_version_str = '.'.join(str(p) for p in LEROBOT_VERSION)
    user_notes = params.get('wandb.notes', '') or ''
    provenance_note = f'lerobot={lerobot_version_str}'
    notes = f'{user_notes} [{provenance_note}]' if user_notes else f'[{provenance_note}]'

    wandb_cfg = WandBConfig(
        enable=params.get('wandb.enable', True),
        project=params.get('wandb.project', 'sobits_vla_training'),
        entity=params.get('wandb.entity', None) or None,
        notes=notes,
    )
    # wandb.run_name is the display name (job_name), not a resume id --
    # run_id stays unset so each run starts a fresh W&B run.
    job_name = params.get('wandb.run_name', '') or None

    rename_map: dict = params.get('dataset.rename_map', {}) or {}

    peft_cfg, peft_extra = build_peft_config(params)

    train_kwargs: dict[str, Any] = dict(
        dataset=dataset_cfg,
        policy=policy_cfg,
        output_dir=output_dir,
        job_name=job_name,
        resume=params.get('checkpoint.resume', False),
        seed=params.get('training.seed', 1000),
        num_workers=params.get('dataset.num_workers', 4),
        batch_size=params.get('training.batch_size', 32),
        steps=params.get('training.steps', 100000),
        log_freq=params.get('training.log_freq', 200),
        env_eval_freq=params.get('training.eval_freq', 20000),
        save_checkpoint=params.get('checkpoint.save_checkpoint', True),
        save_freq=params.get('checkpoint.save_freq', 20000),
        use_policy_training_preset=params.get('training.use_policy_training_preset', True),
        wandb=wandb_cfg,
        peft=peft_cfg,
        rename_map=rename_map,
        save_checkpoint_to_hub=params.get('hub.save_checkpoints', False),
    )

    optimizer_override, scheduler_override = build_optimizer_scheduler_override(params)
    if optimizer_override is not None:
        # Override active: bypass the policy preset entirely.
        train_kwargs['use_policy_training_preset'] = False
        train_kwargs['optimizer'] = optimizer_override
        train_kwargs['scheduler'] = scheduler_override

    train_cfg = TrainPipelineConfig(**train_kwargs)

    return train_cfg, peft_extra


def build_optimizer_scheduler_override(params: dict[str, Any]):
    """
    Build an (optimizer, scheduler) override pair, or (None, None) if unset.

    Only active when training.optimizer_type is non-empty. Bypasses the
    policy's own optimizer/scheduler preset entirely (see TrainPipelineConfig
    .validate(): the preset always overwrites .optimizer/.scheduler unless
    use_policy_training_preset=False, and both become mandatory in that mode).
    Currently wires up SGDConfig only; add another `elif optimizer_type == ...`
    branch here to support a second optimizer type.
    """
    optimizer_type: str = params.get('training.optimizer_type', '') or ''
    if not optimizer_type:
        return None, None

    from sobits_vla_common.lerobot_adapter import ConstantWithWarmupSchedulerConfig, SGDConfig

    if optimizer_type != 'sgd':
        raise ValueError(
            f"Unsupported training.optimizer_type: {optimizer_type!r} (only 'sgd' is wired up)"
        )

    optimizer_override = SGDConfig(
        lr=params.get('training.optimizer_sgd.lr', 1e-3),
        momentum=params.get('training.optimizer_sgd.momentum', 0.0),
        dampening=params.get('training.optimizer_sgd.dampening', 0.0),
        nesterov=params.get('training.optimizer_sgd.nesterov', False),
        weight_decay=params.get('training.optimizer_sgd.weight_decay', 0.0),
        grad_clip_norm=params.get('training.optimizer_sgd.grad_clip_norm', 10.0),
    )
    scheduler_override = ConstantWithWarmupSchedulerConfig(
        num_warmup_steps=params.get('training.scheduler_warmup_steps_override', 1000),
    )
    return optimizer_override, scheduler_override


def build_peft_config(params: dict[str, Any]):
    """
    Build a (PeftConfig, extra_overrides) pair from ROS parameters.

    PeftConfig holds the 5 fields LeRobot serialises via dataclasses.asdict().
    extra_overrides carries lora_alpha / lora_dropout which are not PeftConfig
    fields but are consumed by wrap_with_peft via peft_cli_overrides.

    Returns
    -------
    tuple[PeftConfig, dict] or (None, {})
        PeftConfig instance and a dict of extra LoRA overrides, or (None, {})
        if peft.method_type is not set.

    """
    from sobits_vla_common.lerobot_adapter import PeftConfig

    method_type: str = params.get('peft.method_type', '') or ''
    if not method_type:
        return None, {}

    target_raw = params.get('peft.target_modules', '') or ''
    target_modules: list[str] | str | None = None
    if target_raw:
        # peft treats a plain string as a single regex fullmatch — only
        # split on ',' (a literal module-name list); a comma-free string
        # is a regex and must stay a string.
        target_modules = (
            target_raw if isinstance(target_raw, list) or ',' not in target_raw
            else [s.strip() for s in target_raw.split(',') if s.strip()]
        )

    full_training_modules = params.get('peft.full_training_modules', None)

    peft_cfg = PeftConfig(
        method_type=method_type,
        r=int(params.get('peft.r', 16)),
        target_modules=target_modules,
        full_training_modules=full_training_modules if full_training_modules is not None else [],
    )

    # lora_alpha and lora_dropout are extra LoRA args not on PeftConfig —
    # they are injected via peft_cli_overrides in wrap_with_peft.
    extra: dict[str, Any] = {}
    lora_alpha = params.get('peft.lora_alpha', None)
    if lora_alpha is not None:
        extra['lora_alpha'] = int(lora_alpha)
    lora_dropout = params.get('peft.lora_dropout', None)
    if lora_dropout is not None:
        extra['lora_dropout'] = float(lora_dropout)

    return peft_cfg, extra


def build_accelerator(num_gpus: int, use_amp: bool = False):
    """
    Build an accelerate.Accelerator for single- or multi-GPU training.

    Parameters
    ----------
    num_gpus : int
        Number of GPUs. 0 = CPU, 1 = single GPU, >1 = DDP.
    use_amp : bool
        Enable mixed precision (fp16 on CUDA).

    Returns
    -------
    Accelerator
        Configured accelerate.Accelerator instance.

    """
    try:
        from accelerate import Accelerator
        from accelerate.utils import DistributedDataParallelKwargs
    except ImportError as e:
        raise ImportError(
            'accelerate is required for training. '
            'Install with: pip install accelerate'
        ) from e

    mixed_precision = 'fp16' if use_amp else 'no'
    kwargs = []

    if num_gpus > 1:
        kwargs.append(DistributedDataParallelKwargs(find_unused_parameters=True))
        logger.info(f'Multi-GPU training enabled: {num_gpus} GPUs, DDP.')
    elif num_gpus == 1:
        logger.info('Single-GPU training.')
    else:
        logger.info('CPU training (num_gpus=0).')

    return Accelerator(
        mixed_precision=mixed_precision,
        kwargs_handlers=kwargs if kwargs else None,
    )


def _infer_device(num_gpus: int) -> str:
    if num_gpus == 0:
        return 'cpu'
    try:
        import torch
        if torch.cuda.is_available():
            return 'cuda'
    except ImportError:
        pass
    logger.warning('num_gpus > 0 but CUDA not available — falling back to CPU.')
    return 'cpu'
