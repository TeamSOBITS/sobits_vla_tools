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

"""robot.ee_* training parameters resolved against the (filtered) robot descriptor."""

from __future__ import annotations


def robot_ee_rotation(params: dict) -> str:
    """Return robot.ee_rotation (rotvec default), validated."""
    from sobits_vla_common.robot_descriptor import EE_ROTATION_AXES, EE_ROTATION_DEFAULT

    rotation = params.get('robot.ee_rotation', EE_ROTATION_DEFAULT) or EE_ROTATION_DEFAULT
    if rotation not in EE_ROTATION_AXES:
        raise ValueError(
            f'robot.ee_rotation must be one of {sorted(EE_ROTATION_AXES)}, got {rotation!r}')
    return rotation


def expected_ee_actions(desc, params: dict) -> list[str]:
    """Dataset action feature names for robot.ee_action_arms (derived when empty), or []."""
    from sobits_vla_common.robot_descriptor import ee_action_features, resolve_ee_action_specs

    rotation = robot_ee_rotation(params)
    specs = resolve_ee_action_specs(
        desc, params.get('robot.ee_action_arms', []), param='robot.ee_action_arms')
    return [n for s in specs for n in ee_action_features(s.ee_pose, rotation=rotation)]


def ee_action_dim(desc, params: dict) -> int:
    """Action/state dim contributed by EE channels, or 0 in joint mode."""
    return len(expected_ee_actions(desc, params))
