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

"""Spec for the SE(3) EE relative steps and relative-space stats (lerobot_compat installs it)."""

from __future__ import annotations

from typing import Optional

from sobits_vla_training.ee_params import robot_ee_rotation


def relative_training_spec(
    params: dict, policy_cfg, info: Optional[dict], log_warn,
) -> Optional[dict]:
    """
    install_ee_relative_training spec, or None when nothing relative is on.

    info is the dataset's meta/info.json; an EE relative run cannot proceed
    without it (the step pair is keyed on action/state names), a joint-only
    relative run keeps absolute-space stats with a warning.
    """
    ee_relative = bool(params.get('robot.ee_relative_actions', False))
    joint_relative = bool(getattr(policy_cfg, 'use_relative_actions', False))
    if not (ee_relative or joint_relative):
        return None
    if info is None:
        if ee_relative:
            raise RuntimeError(
                'robot.ee_relative_actions needs meta/info.json for the action/state names; '
                'dataset not found locally.')
        log_warn('meta/info.json not found locally; action stats stay absolute-space.')
        return None
    features = info.get('features', {})
    return {
        'ee_relative': ee_relative,
        'joint_relative': joint_relative,
        'joint_exclude': list(getattr(policy_cfg, 'relative_exclude_joints', []) or []),
        'action_names': features.get('action', {}).get('names') or [],
        'state_names': features.get('observation.state', {}).get('names') or [],
        'ee_rotation': robot_ee_rotation(params),
    }
