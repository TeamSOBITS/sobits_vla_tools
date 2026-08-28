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


from __future__ import annotations

from dataclasses import dataclass, fields
import importlib
import logging
from pathlib import Path
from typing import Any

import yaml


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PolicyEntry:
    policy_id: str           # 'pi05'
    class_path: str          # full dotted path to policy model class
    config_module: str
    config_class: str
    default_yaml: str        # filename in config/policies/ (training)
    default_pretrained: str  # '' = train from scratch
    has_device_field: bool   # config class accepts device= kwarg
    supports_rtc: bool       # exposes predict_action_chunk + rtc_config
    cast_bf16: bool          # load on CPU then cast to bfloat16


_REGISTRY: dict[str, PolicyEntry] = {
    'smolvla': PolicyEntry(
        policy_id='smolvla',
        class_path='lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy',
        config_module='lerobot.policies.smolvla.configuration_smolvla',
        config_class='SmolVLAConfig',
        default_yaml='smolvla.yaml',
        default_pretrained='lerobot/smolvla_base',
        # SmolVLAConfig accepts device= like every other policy config; False here
        # left device=None, triggering a "Device 'None' is not available" warning.
        has_device_field=True,
        supports_rtc=True,
        cast_bf16=False,
    ),
    'pi0': PolicyEntry(
        policy_id='pi0',
        class_path='lerobot.policies.pi0.modeling_pi0.PI0Policy',
        config_module='lerobot.policies.pi0.configuration_pi0',
        config_class='PI0Config',
        default_yaml='pi0.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=True,
        cast_bf16=True,
    ),
    'pi05': PolicyEntry(
        policy_id='pi05',
        class_path='lerobot.policies.pi05.modeling_pi05.PI05Policy',
        config_module='lerobot.policies.pi05.configuration_pi05',
        config_class='PI05Config',
        default_yaml='pi05.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=True,
        cast_bf16=True,
    ),
    'pi0_fast': PolicyEntry(
        policy_id='pi0_fast',
        class_path='lerobot.policies.pi0_fast.modeling_pi0_fast.PI0FastPolicy',
        config_module='lerobot.policies.pi0_fast.configuration_pi0_fast',
        config_class='PI0FastConfig',
        default_yaml='pi0_fast.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=True,
        cast_bf16=True,
    ),
    'act': PolicyEntry(
        policy_id='act',
        class_path='lerobot.policies.act.modeling_act.ACTPolicy',
        config_module='lerobot.policies.act.configuration_act',
        config_class='ACTConfig',
        default_yaml='act.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=False,
        cast_bf16=False,
    ),
    'groot': PolicyEntry(
        policy_id='groot',
        class_path='lerobot.policies.groot.modeling_groot.GrootPolicy',
        config_module='lerobot.policies.groot.configuration_groot',
        config_class='GrootConfig',
        default_yaml='groot.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=False,
        cast_bf16=True,
    ),
    'vla_jepa': PolicyEntry(
        policy_id='vla_jepa',
        class_path='lerobot.policies.vla_jepa.modeling_vla_jepa.VLAJEPAPolicy',
        config_module='lerobot.policies.vla_jepa.configuration_vla_jepa',
        config_class='VLAJEPAConfig',
        default_yaml='vla_jepa.yaml',
        default_pretrained='lerobot/VLA-JEPA-Pretrain',
        has_device_field=True,
        # No init_rtc_processor in modeling_vla_jepa (lerobot 0.6.0).
        supports_rtc=False,
        # Model manages its own dtype via config torch_dtype ('bfloat16').
        cast_bf16=False,
    ),
    'molmoact2': PolicyEntry(
        policy_id='molmoact2',
        class_path='lerobot.policies.molmoact2.modeling_molmoact2.MolmoAct2Policy',
        config_module='lerobot.policies.molmoact2.configuration_molmoact2',
        config_class='MolmoAct2Config',
        default_yaml='molmoact2.yaml',
        # Base weights load via the config's checkpoint_path field
        # (allenai/MolmoAct2), not via checkpoint.pretrained_path.
        default_pretrained='',
        has_device_field=True,
        supports_rtc=True,
        # Model manages its own dtype via config model_dtype ('bfloat16').
        cast_bf16=False,
    ),
    'diffusion': PolicyEntry(
        policy_id='diffusion',
        class_path='lerobot.policies.diffusion.modeling_diffusion.DiffusionPolicy',
        config_module='lerobot.policies.diffusion.configuration_diffusion',
        config_class='DiffusionConfig',
        default_yaml='diffusion.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=False,
        cast_bf16=False,
    ),
    'vqbet': PolicyEntry(
        policy_id='vqbet',
        class_path='lerobot.policies.vqbet.modeling_vqbet.VQBeTPolicy',
        config_module='lerobot.policies.vqbet.configuration_vqbet',
        config_class='VQBeTConfig',
        default_yaml='vqbet.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=False,
        cast_bf16=False,
    ),
    'multi_task_dit': PolicyEntry(
        policy_id='multi_task_dit',
        class_path='lerobot.policies.multi_task_dit.modeling_multi_task_dit.MultiTaskDiTPolicy',
        config_module='lerobot.policies.multi_task_dit.configuration_multi_task_dit',
        config_class='MultiTaskDiTConfig',
        default_yaml='multi_task_dit.yaml',
        default_pretrained='',
        has_device_field=True,
        supports_rtc=False,
        cast_bf16=False,
    ),
}

_CLASS_PATH_TO_ENTRY: dict[str, PolicyEntry] = {
    entry.class_path: entry for entry in _REGISTRY.values()
}

_POLICIES_DIR: Path | None = None


def get_entry(policy_id_or_class_path: str) -> PolicyEntry:
    if policy_id_or_class_path in _REGISTRY:
        return _REGISTRY[policy_id_or_class_path]
    if policy_id_or_class_path in _CLASS_PATH_TO_ENTRY:
        return _CLASS_PATH_TO_ENTRY[policy_id_or_class_path]
    raise ValueError(f'Unknown policy ID or class path: {policy_id_or_class_path}')


def available_policies() -> list[str]:
    return list(_REGISTRY.keys())


def _policies_dir() -> Path:
    global _POLICIES_DIR
    if _POLICIES_DIR is not None:
        return _POLICIES_DIR

    # 1. Environment variable override
    import os
    env_dir = os.environ.get('SOBITS_VLA_POLICIES_DIR')
    if env_dir:
        candidate = Path(env_dir)
        if candidate.is_dir():
            _POLICIES_DIR = candidate
            return _POLICIES_DIR

    # 2. ROS package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('sobits_vla_training'))
        candidate = share / 'config' / 'policies'
        if candidate.is_dir():
            _POLICIES_DIR = candidate
            return _POLICIES_DIR
    except Exception:
        pass

    # 3. Traversal through parents
    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        candidate = parent / 'sobits_vla_training' / 'config' / 'policies'
        if candidate.is_dir():
            _POLICIES_DIR = candidate
            return _POLICIES_DIR
        candidate = parent / 'config' / 'policies'
        if candidate.is_dir() and parent.name == 'sobits_vla_training':
            _POLICIES_DIR = candidate
            return _POLICIES_DIR

    raise FileNotFoundError(
        'Cannot locate config/policies/ directory. '
        'Build the package with colcon or set SOBITS_VLA_POLICIES_DIR.'
    )


def _load_policy_yaml(policy_type: str) -> dict[str, Any]:
    entry = get_entry(policy_type)
    yaml_path = _policies_dir() / entry.default_yaml
    with open(yaml_path) as f:
        return yaml.safe_load(f) or {}


def _import_config_class(policy_type: str):
    entry = get_entry(policy_type)
    mod = importlib.import_module(entry.config_module)
    return getattr(mod, entry.config_class)


def make_policy_config(
    policy_type: str,
    overrides: dict[str, Any] | None = None,
    device: str = 'cuda',
) -> Any:
    entry = get_entry(policy_type)
    ConfigClass = _import_config_class(policy_type)
    yaml_defaults = _load_policy_yaml(policy_type)

    yaml_defaults.pop('policy_type', None)

    valid_fields = {f.name for f in fields(ConfigClass)}

    # Process and clean overrides (filter by valid fields, warning on unknown ones)
    clean_overrides = {}
    if overrides:
        for k, v in overrides.items():
            if k not in valid_fields:
                logger.warning(
                    'policy_override %r is not a field of %s — ignored',
                    k, ConfigClass.__name__
                )
                continue
            clean_overrides[k] = v

    # Merge yaml defaults and overrides
    merged = {**yaml_defaults, **clean_overrides}

    # Filter by valid fields
    filtered = {k: v for k, v in merged.items() if k in valid_fields}

    # Coerce lists to tuples in defaults/merged dict
    for k, v in list(filtered.items()):
        if isinstance(v, list):
            ft = {f.name: f for f in fields(ConfigClass)}[k].type
            if ft in (tuple, 'tuple'):
                filtered[k] = tuple(v)

    if entry.has_device_field and 'device' in valid_fields:
        filtered.setdefault('device', device)

    cfg = ConfigClass(**filtered)
    if hasattr(cfg, 'device'):
        cfg.device = device

    return cfg
