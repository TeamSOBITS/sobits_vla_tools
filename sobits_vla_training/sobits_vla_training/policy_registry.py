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
Policy registry: maps policy_type string to (ConfigClass, default_yaml_path).

Adding a new policy requires only a new YAML in config/policies/<name>.yaml
and one new entry in _REGISTRY below.
"""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path
from typing import Any

import yaml

_REGISTRY: dict[str, dict] = {
    'smolvla': {
        'config_module': 'lerobot.policies.smolvla.configuration_smolvla',
        'config_class': 'SmolVLAConfig',
        'yaml': 'smolvla.yaml',
    },
    'pi0': {
        'config_module': 'lerobot.policies.pi0.configuration_pi0',
        'config_class': 'PI0Config',
        'yaml': 'pi0.yaml',
    },
    'pi05': {
        'config_module': 'lerobot.policies.pi05.configuration_pi05',
        'config_class': 'PI05Config',
        'yaml': 'pi05.yaml',
    },
    'pi0_fast': {
        'config_module': 'lerobot.policies.pi0_fast.configuration_pi0_fast',
        'config_class': 'PI0FastConfig',
        'yaml': 'pi0_fast.yaml',
    },
    'act': {
        'config_module': 'lerobot.policies.act.configuration_act',
        'config_class': 'ACTConfig',
        'yaml': 'act.yaml',
    },
    # GR00T N1.5 only — N1.6/N1.7 are fine-tuned via NVIDIA's Isaac-GR00T repo
    # (different backbone, Cosmos-Reason2); lerobot 0.5.1 ports N1.5-3B.
    # Requires flash-attn: pip install flash-attn --no-build-isolation
    'groot': {
        'config_module': 'lerobot.policies.groot.configuration_groot',
        'config_class': 'GrootConfig',
        'yaml': 'groot.yaml',
    },
}

_POLICIES_DIR: Path | None = None


def _policies_dir() -> Path:
    global _POLICIES_DIR
    if _POLICIES_DIR is not None:
        return _POLICIES_DIR

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('sobits_vla_training'))
        candidate = share / 'config' / 'policies'
        if candidate.is_dir():
            _POLICIES_DIR = candidate
            return _POLICIES_DIR
    except Exception:
        pass

    src_candidate = Path(__file__).parent.parent / 'config' / 'policies'
    if src_candidate.is_dir():
        _POLICIES_DIR = src_candidate
        return _POLICIES_DIR

    raise FileNotFoundError(
        'Cannot locate config/policies/ directory. '
        'Build the package with colcon or set SOBITS_VLA_POLICIES_DIR.'
    )


def available_policies() -> list[str]:
    """Return list of registered policy type names."""
    return list(_REGISTRY.keys())


def _load_policy_yaml(policy_type: str) -> dict[str, Any]:
    entry = _REGISTRY[policy_type]
    yaml_path = _policies_dir() / entry['yaml']
    with open(yaml_path) as f:
        return yaml.safe_load(f) or {}


def _import_config_class(policy_type: str):
    import importlib
    entry = _REGISTRY[policy_type]
    mod = importlib.import_module(entry['config_module'])
    return getattr(mod, entry['config_class'])


def make_policy_config(
    policy_type: str,
    overrides: dict[str, Any] | None = None,
    device: str = 'cuda',
) -> Any:
    """
    Build and return a fully initialised policy config dataclass.

    Parameters
    ----------
    policy_type : str
        One of the keys in _REGISTRY (e.g. ``'smolvla'``).
    overrides : dict, optional
        Field name to value pairs applied on top of the YAML defaults.
    device : str
        torch device string forwarded to ``config.device``.

    Returns
    -------
    Any
        An initialised ``PreTrainedConfig`` subclass instance.

    """
    if policy_type not in _REGISTRY:
        raise ValueError(
            f"Unknown policy '{policy_type}'. "
            f'Available: {available_policies()}'
        )

    ConfigClass = _import_config_class(policy_type)
    yaml_defaults = _load_policy_yaml(policy_type)

    yaml_defaults.pop('policy_type', None)

    merged = {**yaml_defaults, **(overrides or {})}

    valid_fields = {f.name for f in fields(ConfigClass)}
    filtered = {k: v for k, v in merged.items() if k in valid_fields}

    if 'device' in valid_fields:
        filtered.setdefault('device', device)

    cfg = ConfigClass(**filtered)
    cfg.device = device
    return cfg
