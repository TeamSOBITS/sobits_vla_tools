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
VLA-only robot settings layered over the shared robot descriptor.

Defaults live in ``config/robot_overrides_<robot_id>.yaml``; a stage config may
add a ``robot_overrides:`` block with the same schema, deep-merged on top.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional

import yaml

PARAM_PREFIX = 'robot_overrides'

_TOP_KEYS = {'descriptor_package', 'morphology', 'groups', 'mobile_base', 'cameras', 'ee'}
_GROUP_KEYS = {'active', 'max_joint_delta', 'relative_exclude', 'features'}
_BASE_KEYS = {'active', 'features', 'max_vel_x', 'max_vel_y', 'max_vel_z', 'max_vel_theta',
              'linear_deadband', 'angular_deadband'}
_STREAM_KEYS = {'active', 'compressed', 'encoding'}
_CAMERA_KEYS = _STREAM_KEYS | {'depth'}
_EE_KEYS = {'active'}


def merge_overrides(base: Mapping[str, Any], extra: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    """Deep-merge *extra* onto *base*; mappings merge by key, anything else replaces."""
    out = copy.deepcopy(dict(base))
    for key, value in (extra or {}).items():
        if isinstance(value, Mapping) and isinstance(out.get(key), Mapping):
            out[key] = merge_overrides(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def robot_overrides_from_params(
    params: Mapping[str, Any], prefix: str = PARAM_PREFIX,
) -> Dict[str, Any]:
    """
    Nested overrides from flat ``prefix.a.b`` parameter names.

    Values may be rclpy Parameter objects (``node._parameter_overrides``); an
    already nested ``params[prefix]`` mapping is merged in as well.
    """
    out: Dict[str, Any] = {}
    nested = params.get(prefix)
    if isinstance(nested, Mapping):
        out = merge_overrides(out, nested)
    head = prefix + '.'
    for name, raw in params.items():
        if not name.startswith(head):
            continue
        value = getattr(raw, 'value', raw)
        if value is None:
            continue
        keys = name[len(head):].split('.')
        node = out
        for k in keys[:-1]:
            node = node.setdefault(k, {})
        node[keys[-1]] = value
    return out


def resolve_overrides_path(robot_id: str) -> Optional[Path]:
    """Installed share file, else the source tree next to this module; None when absent."""
    name = f'robot_overrides_{robot_id}.yaml'
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory('sobits_vla_common')) / 'config' / name
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    for parent in Path(__file__).resolve().parents:
        for candidate in (parent / 'config' / name,
                          parent / 'sobits_vla_common' / 'config' / name):
            if candidate.is_file():
                return candidate
    return None


def load_robot_overrides(
    robot_id: str, extra: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """Per-robot defaults file (empty when missing) with *extra* merged on top."""
    base: Dict[str, Any] = {}
    path = resolve_overrides_path(robot_id)
    if path is not None:
        with open(path) as f:
            base = yaml.safe_load(f) or {}
        if not isinstance(base, Mapping):
            raise ValueError(f'{path}: robot overrides must be a mapping')
    return merge_overrides(base, extra)


def _check_keys(where: str, entry: Any, allowed: set) -> list:
    if entry is None:
        return []
    if not isinstance(entry, Mapping):
        return [f'{where} must be a mapping, got {type(entry).__name__}']
    unknown = sorted(set(entry) - allowed)
    return [f'{where}: unknown key(s) {unknown}; allowed {sorted(allowed)}'] if unknown else []


def _check_named(section: str, entries: Any, known: list, allowed: set) -> list:
    if entries is None:
        return []
    if not isinstance(entries, Mapping):
        return [f'{section} must be a mapping keyed by name']
    errors = []
    for name, entry in entries.items():
        if name not in known:
            errors.append(f'{section}: unknown name {name!r}; descriptor has {sorted(known)}')
        errors += _check_keys(f'{section}.{name}', entry, allowed)
    return errors


def check_overrides(ov: Mapping[str, Any], groups: list, cameras: list, ee: list,
                    has_base: bool) -> list:
    """Errors for unknown keys or names, so a typo fails loudly."""
    errors = _check_keys('robot_overrides', ov, _TOP_KEYS)
    errors += _check_named('groups', ov.get('groups'), groups, _GROUP_KEYS)
    errors += _check_named('cameras', ov.get('cameras'), cameras, _CAMERA_KEYS)
    errors += _check_named('ee', ov.get('ee'), ee, _EE_KEYS)
    for name, cam in (ov.get('cameras') or {}).items():
        if isinstance(cam, Mapping):
            errors += _check_keys(f'cameras.{name}.depth', cam.get('depth'), _STREAM_KEYS)
    if ov.get('mobile_base') is not None:
        if not has_base:
            errors.append('mobile_base: descriptor has no mobile_base')
        errors += _check_keys('mobile_base', ov.get('mobile_base'), _BASE_KEYS)
    return errors


def import_loader():
    """sobits_robot_descriptor, also when only the workspace install or source tree has it."""
    try:
        import sobits_robot_descriptor
        return sobits_robot_descriptor
    except ImportError:
        pass
    candidates: List[Path] = []
    for var in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for prefix in filter(None, os.environ.get(var, '').split(os.pathsep)):
            for sub in ('', 'sobits_robot_descriptor'):
                candidates += sorted(Path(prefix, sub).glob('lib/python3*/site-packages'))
    candidates += [p / 'sobits_robot_descriptor' for p in Path(__file__).resolve().parents]
    for cand in candidates:
        if (cand / 'sobits_robot_descriptor' / '__init__.py').is_file():
            sys.path.append(str(cand))
            import sobits_robot_descriptor
            return sobits_robot_descriptor
    raise ImportError(
        'sobits_robot_descriptor is not importable: build it in the workspace '
        '(colcon build --packages-select sobits_robot_descriptor) and source install/setup.bash.')


def descriptor_search_dirs(package: str) -> List[Path]:
    """<package>/config from the ament index, else from the source tree around this file."""
    dirs: List[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory(package)) / 'config')
    except Exception:
        pass
    for parent in Path(__file__).resolve().parents:
        dirs.append(parent / package / 'config')
        dirs.extend(sorted(parent.glob(f'*/{package}/config')))
    return [d for d in dirs if d.is_dir()]
