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

from .policy_registry import make_policy_config

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


def _resolve_pretrained_path(raw: str) -> Path | str:
    """Return a Path for local files, or pass through HF Hub repo_id strings."""
    if not raw:
        return ''
    p = Path(raw)
    if p.exists():
        return p
    return raw


def build_train_config(params: dict[str, Any]):
    """
    Construct a TrainPipelineConfig from a flat ROS parameter dict.

    Parameters
    ----------
    params : dict
        Flat dict of ROS parameter names to values.

    Returns
    -------
    tuple[TrainPipelineConfig, dict]
        Fully initialised config and a dict of extra PEFT overrides
        (e.g. lora_alpha, lora_dropout) to pass to wrap_with_peft.

    """
    from lerobot.configs.default import DatasetConfig, WandBConfig
    from lerobot.configs.train import TrainPipelineConfig

    policy_type: str = params.get('policy', 'smolvla')
    device: str = _infer_device(params.get('num_gpus', 1))

    policy_overrides: dict = params.get('policy_overrides', {}) or {}

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

    dataset_cfg = DatasetConfig(repo_id=ds_repo_id)

    wandb_cfg = WandBConfig(
        enable=params.get('wandb.enable', True),
        project=params.get('wandb.project', 'sobits_vla_training'),
        entity=params.get('wandb.entity', None) or None,
        run_id=params.get('wandb.run_name', None) or None,
        notes=params.get('wandb.notes', '') or '',
    )

    output_dir_raw = params.get('checkpoint.output_dir', '')
    package_src_dir = find_package_src_dir()

    if not output_dir_raw:
        output_dir = package_src_dir / 'outputs'
    else:
        raw_path = Path(output_dir_raw).expanduser()
        if raw_path.is_absolute():
            output_dir = raw_path
        else:
            if raw_path.parts and raw_path.parts[0] == 'outputs':
                output_dir = (package_src_dir / raw_path).resolve()
            else:
                output_dir = (package_src_dir / 'outputs' / raw_path).resolve()

    rename_map: dict = params.get('dataset.rename_map', {}) or {}

    peft_cfg, peft_extra = build_peft_config(params)

    train_cfg = TrainPipelineConfig(
        dataset=dataset_cfg,
        policy=policy_cfg,
        output_dir=output_dir,
        resume=params.get('checkpoint.resume', False),
        seed=params.get('training.seed', 1000),
        num_workers=params.get('dataset.num_workers', 4),
        batch_size=params.get('training.batch_size', 32),
        steps=params.get('training.steps', 100000),
        log_freq=params.get('training.log_freq', 200),
        eval_freq=params.get('training.eval_freq', 20000),
        save_checkpoint=params.get('checkpoint.save_checkpoint', True),
        save_freq=params.get('checkpoint.save_freq', 20000),
        use_policy_training_preset=params.get('training.use_policy_training_preset', True),
        wandb=wandb_cfg,
        peft=peft_cfg,
        rename_map=rename_map,
    )

    return train_cfg, peft_extra


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
    from lerobot.configs.default import PeftConfig

    method_type: str = params.get('peft.method_type', '') or ''
    if not method_type:
        return None, {}

    target_raw = params.get('peft.target_modules', '') or ''
    target_modules: list[str] | str | None = None
    if target_raw:
        target_modules = (
            target_raw if isinstance(target_raw, list) else target_raw
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
