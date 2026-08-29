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


def load_dataset_info(repo_id: str) -> dict | None:
    """Read info.json from local cache or HF hub without instantiating LeRobotDataset."""
    try:
        from sobits_vla_common.lerobot_adapter import HF_LEROBOT_HOME
        candidate = HF_LEROBOT_HOME / repo_id / 'meta' / 'info.json'
        if not candidate.exists():
            candidate = Path(repo_id) / 'meta' / 'info.json'
        if not candidate.exists():
            from huggingface_hub import hf_hub_download
            candidate = Path(
                hf_hub_download(repo_id, 'meta/info.json', repo_type='dataset')
            )
        with open(candidate) as f:
            return json.load(f)
    except Exception:
        return None


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
            from sobits_vla_common.robot_descriptor import load_robot_descriptor
            desc = load_robot_descriptor(desc_id)
            desc = desc.filtered(
                exclude_groups=params.get('robot.exclude.groups', []),
                exclude_cameras=params.get('robot.exclude.cameras', []),
                exclude_ee_poses=params.get('robot.exclude.ee_poses', []),
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

            active_base_features = []
            if desc.mobile_base and active_mobile_base:
                # active_groups=[] so only base features come back, not group joints
                # (those are already in active_joint_features above).
                active_base_features = desc.relative_exclude_features(
                    active_groups=[], active_mobile_base=active_mobile_base,
                )

            expected_actions = active_joint_features + active_base_features

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
            if isinstance(exc, RuntimeError):
                raise
            log_warn(f'Failed to run robot-descriptor-based pre-flight checks: {exc}')

    # relative_exclude_joints validation (config_builder derives this from the
    # descriptor; warn only if it still looks wrong against the dataset).
    if po.get('use_relative_actions', False):
        exclude = po.get('relative_exclude_joints', [])
        if not exclude:
            log_warn(
                'use_relative_actions=true but relative_exclude_joints is empty — '
                'all features, including mobile-base velocities, would be delta-converted. '
                'Set robot.descriptor_id or an explicit override.'
            )
        elif action_names:
            unknown = [j for j in exclude if j not in action_names]
            if unknown:
                log_warn(
                    f'relative_exclude_joints {unknown} not in dataset action names '
                    f'{action_names}; they will not be excluded from delta conversion.'
                )
