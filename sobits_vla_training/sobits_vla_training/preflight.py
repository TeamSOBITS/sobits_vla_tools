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

import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


def _local_dataset_root(repo_id: str) -> Path | None:
    """Local dataset root (HF_LEROBOT_HOME/repo_id, else repo_id as a path), or None."""
    try:
        from sobits_vla_common.lerobot_adapter import HF_LEROBOT_HOME
        candidates = [Path(HF_LEROBOT_HOME) / repo_id, Path(repo_id)]
    except Exception:
        candidates = [Path(repo_id)]
    for root in candidates:
        if (root / 'meta').is_dir():
            return root
    return None


def load_dataset_meta_file(repo_id: str, name: str) -> dict | None:
    """Read meta/<name> (JSON) from local cache or HF hub without instantiating LeRobotDataset."""
    try:
        root = _local_dataset_root(repo_id)
        candidate = root / 'meta' / name if root is not None else None
        if candidate is None or not candidate.exists():
            from huggingface_hub import hf_hub_download
            candidate = Path(
                hf_hub_download(repo_id, f'meta/{name}', repo_type='dataset')
            )
        with open(candidate) as f:
            return json.load(f)
    except Exception:
        return None


def load_dataset_info(repo_id: str) -> dict | None:
    """Read meta/info.json; see load_dataset_meta_file."""
    return load_dataset_meta_file(repo_id, 'info.json')


def _load_conversion_stats(repo_id: str) -> dict | None:
    """Local-only conversion_stats.yaml (written at the dataset root, never pushed)."""
    root = _local_dataset_root(repo_id)
    path = root / 'conversion_stats.yaml' if root is not None else None
    if path is None or not path.exists():
        return None
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return None


def _robot_ee_rotation(params: dict) -> str:
    from sobits_vla_common.robot_descriptor import EE_ROTATION_AXES, EE_ROTATION_DEFAULT

    rotation = params.get('robot.ee_rotation', EE_ROTATION_DEFAULT) or EE_ROTATION_DEFAULT
    if rotation not in EE_ROTATION_AXES:
        raise ValueError(
            f'robot.ee_rotation must be one of {sorted(EE_ROTATION_AXES)}, got {rotation!r}')
    return rotation


def _expected_ee_actions(desc, params: dict) -> list[str]:
    """
    Dataset action feature names for robot.ee_action_arms, or [] in joint mode.

    Mirrors config_builder._ee_action_dim: robot.ee_action_arms is an
    optional override of RobotDescriptor.derived_ee_action_arms() (empty
    derives from the descriptor; an explicit list is validated against the
    same active-ee/excluded-group rule, or joint and EE features would both
    land in expected_actions). robot.ee_rotation selects rotvec/rpy (6D) vs
    quat (7D) names, matching the dataset's conversion.
    """
    from sobits_vla_common.robot_descriptor import ee_action_features

    rotation = _robot_ee_rotation(params)

    arms = [a for a in params.get('robot.ee_action_arms', []) if a]
    if arms:
        derived = set(desc.derived_ee_action_arms())
        specs = desc.ee_control_for(arms)
        invalid = [s.ee_pose for s in specs if s.ee_pose not in derived]
        if invalid:
            still_active = [s.group for s in specs if s.ee_pose in invalid]
            raise ValueError(
                f'robot.ee_action_arms names arm(s) {invalid} whose group is not '
                f'excluded: {still_active}. Add them to robot.exclude.groups so '
                'joint and EE features do not both count.'
            )
    else:
        specs = desc.ee_control_for(desc.derived_ee_action_arms())

    features = []
    for s in specs:
        features.extend(ee_action_features(s.ee_pose, rotation=rotation))
    return features


# Legacy delta detection: an absolute EE action tracks its state, so the means
# agree within a fraction of the state spread (+2 cm floor for idle arms).
_DELTA_STD_FRACTION = 0.5
_DELTA_FLOOR_M = 0.02


def _delta_axes_from_stats(stats: dict, action_names: list, state_names: list) -> list:
    """EE translation action names whose mean is far from the state mean (per-step deltas)."""
    try:
        a_mean = stats['action']['mean']
        s_mean = stats['observation.state']['mean']
        s_std = stats['observation.state']['std']
    except (KeyError, TypeError):
        return []
    flagged = []
    for i, name in enumerate(action_names):
        if not (name.startswith('ee.') and name.rsplit('.', 1)[-1] in ('x', 'y', 'z')):
            continue
        if name not in state_names:
            continue
        j = state_names.index(name)
        if abs(a_mean[i] - s_mean[j]) > _DELTA_STD_FRACTION * s_std[j] + _DELTA_FLOOR_M:
            flagged.append(name)
    return flagged


def _check_action_convention(
    repo_id: str, info: dict, params: dict, log_warn, log_info,
) -> None:
    """Refuse datasets whose action encoding does not match what training assumes."""
    from sobits_vla_common.robot_descriptor import ee_rotation_from_names

    features = info.get('features', {})
    action_names = features.get('action', {}).get('names') or []
    state_names = features.get('observation.state', {}).get('names') or []
    has_ee = any(n.startswith('ee.') for n in action_names)

    sidecar = load_dataset_meta_file(repo_id, 'sobits_vla_info.json') or {}
    conv_stats = _load_conversion_stats(repo_id) or {}
    convention = sidecar.get('action_convention') or conv_stats.get('action_convention')

    if convention:
        mode = convention.get('action_mode', 'absolute')
        if mode != 'absolute':
            raise RuntimeError(
                f"Dataset '{repo_id}' has action_mode={mode!r}; training expects absolute "
                'actions. Reconvert it (relative is now a training-time option).'
            )
        rotation = convention.get('ee_rotation') or ee_rotation_from_names(action_names)
    else:
        rotation = ee_rotation_from_names(action_names)
        if conv_stats.get('use_relative_actions', False):
            raise RuntimeError(
                f"Dataset '{repo_id}' was converted with use_relative_actions=true "
                '(per-step deltas). Reconvert it as absolute.'
            )
        if has_ee:
            stats = load_dataset_meta_file(repo_id, 'stats.json') or {}
            delta_axes = _delta_axes_from_stats(stats, action_names, state_names)
            if delta_axes:
                raise RuntimeError(
                    f"Dataset '{repo_id}' EE actions {delta_axes} look like per-step deltas "
                    '(action mean far from state mean). Reconvert it as absolute.'
                )
        rot_note = f' with {rotation} rotation' if rotation else ''
        log_warn(
            f"Legacy dataset without action_convention: '{repo_id}'; "
            f'assuming absolute actions{rot_note}.'
        )

    if has_ee:
        expected = _robot_ee_rotation(params)
        if rotation != expected:
            raise RuntimeError(
                f"Dataset '{repo_id}' EE rotation is {rotation!r} but robot.ee_rotation is "
                f'{expected!r}. Set robot.ee_rotation to match or reconvert.'
            )
    log_info(f"Action convention pre-flight passed for '{repo_id}'.")


def run_preflight_checks(params: dict, ros_logger=None) -> None:
    """Run dataset-aware pre-flight checks that require info.json."""
    def log_warn(msg: str):
        if ros_logger:
            ros_logger.warning(msg)
        else:
            logger.warning(msg)

    def log_info(msg: str):
        if ros_logger:
            ros_logger.info(msg)
        else:
            logger.info(msg)

    repo_id: str = params.get('dataset.repo_id', '')
    po: dict = params.get('policy_overrides', {})

    info = load_dataset_info(repo_id)
    if info is None:
        log_warn(
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
        log_info(
            f'Dim pre-flight passed: actual={actual_action_dim} '
            f'max_action_dim={max_action_dim} max_state_dim={max_state_dim}'
        )

    # ── Descriptor Alignment Checks ───────────────────────────────────────
    desc_id = params.get('robot.descriptor_id', '')
    if desc_id:
        try:
            # Deprecated: robot.exclude.ee_poses was renamed to robot.exclude.ee.
            # rclpy silently ignores yaml params that were never declared, so an
            # old config setting exclude.ee_poses would otherwise stop excluding
            # without warning -- reject it loudly instead.
            if params.get('robot.exclude.ee_poses', []):
                raise ValueError('robot.exclude.ee_poses was renamed to robot.exclude.ee')

            from sobits_vla_common.robot_descriptor import load_robot_descriptor
            desc = load_robot_descriptor(desc_id)

            desc = desc.filtered(
                exclude_groups=params.get('robot.exclude.groups', []),
                exclude_cameras=params.get('robot.exclude.cameras', []),
                exclude_ee=params.get('robot.exclude.ee', []),
                exclude_joints=params.get('robot.exclude.joints', []),
            )

            active_cameras = [c.name for c in desc.active_cameras]
            active_mobile_base = not params.get('robot.exclude.mobile_base', False)

            # 1. Validate active cameras are in dataset
            for cam_name in active_cameras:
                img_key = f'observation.images.{cam_name}'
                if img_key not in features:
                    raise RuntimeError(
                        f"Active camera '{cam_name}' defined in robot descriptor '{desc_id}' "
                        f"is missing from dataset '{repo_id}' features ({img_key} not found)."
                    )

            # 2. Compare expected action features with dataset actions
            active_joint_features = []
            for g in desc.active_groups:
                active_joint_features.extend([j.feature for j in g.joints])

            ee_features = _expected_ee_actions(desc, params)

            active_base_features = []
            if desc.mobile_base and active_mobile_base:
                # active_groups=[] so only base features come back, not group joints
                # (those are already in active_joint_features above).
                active_base_features = desc.relative_exclude_features(
                    active_groups=[], active_mobile_base=active_mobile_base,
                )

            # Layout: [joints..., ee..., base...] to match dataset action order.
            expected_actions = active_joint_features + ee_features + active_base_features

            if action_names:
                missing = [a for a in expected_actions if a not in action_names]
                if missing:
                    log_warn(
                        f"Expected action features {missing} from descriptor '{desc_id}' "
                        f"are missing from dataset '{repo_id}' actions: {action_names}"
                    )
                extra = [a for a in action_names if a not in expected_actions]
                if extra:
                    log_warn(
                        f"Dataset '{repo_id}' contains action features {extra} "
                        f"not specified in active robot descriptor '{desc_id}' configuration."
                    )
        except Exception as exc:
            # ValueError: robot.ee_action_arms group-exclusion violation, must
            # surface like config_builder's; RuntimeError: dim/camera checks
            # above. Anything else is an unexpected descriptor/dataset issue.
            if isinstance(exc, (RuntimeError, ValueError)):
                raise
            log_warn(f'Failed to run robot-descriptor-based pre-flight checks: {exc}')

    _check_action_convention(repo_id, info, params, log_warn, log_info)

    ee_names = [a for a in action_names if a.startswith('ee.')]
    ee_relative = bool(params.get('robot.ee_relative_actions', False))
    if ee_names and po.get('use_relative_actions', False) and not ee_relative:
        raise RuntimeError(
            f"Dataset '{repo_id}' has EE actions {ee_names}: per-component relative EE is "
            'refused; set robot.ee_relative_actions (SE(3) relative) instead.'
        )

    # relative_exclude_joints validation (config_builder derives this from the
    # descriptor and appends ee.* names when ee_relative_actions is on).
    if po.get('use_relative_actions', False):
        exclude = po.get('relative_exclude_joints', [])
        if not exclude:
            log_warn(
                'use_relative_actions=true but relative_exclude_joints is empty — '
                'all features, including mobile-base velocities, would be delta-converted. '
                'Set robot.descriptor_id or an explicit override.'
                + (" ee.* names are excluded from LeRobot's per-component step "
                   'automatically (robot.ee_relative_actions).' if ee_names else '')
            )
        elif action_names:
            unknown = [j for j in exclude if j not in action_names]
            if unknown:
                log_warn(
                    f'relative_exclude_joints {unknown} not in dataset action names '
                    f'{action_names}; they will not be excluded from delta conversion.'
                )
