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

"""EE-servo wiring for model.action_space == 'ee', resolved from the filtered descriptor."""

from typing import List, Tuple

from sobits_vla_common.robot_descriptor import ee_action_features, EEControlSpec


def resolve_ee_control(
    desc, active_groups: List[str], ee_rotation: str,
) -> Tuple[List[EEControlSpec], List[str], List[tuple]]:
    """
    (ee_control, ee_features, ee_state_specs) for the arms the policy drives through servo.

    Each surviving ee_control spec's joint group must already be excluded
    (robot.exclude.groups): the servo bridge and the joint controller must
    not both command the same arm.
    """
    ee_control = desc.active_ee_control
    if not ee_control:
        raise RuntimeError(
            'model.action_space is "ee" but no ee_control spec survived '
            'robot.exclude filtering -- nothing to servo. Check '
            "robot.exclude.ee against the descriptor's ee_control list."
        )
    still_active = [c.group for c in ee_control if c.group in active_groups]
    if still_active:
        raise RuntimeError(
            'model.action_space is "ee" but joint group(s) {} are still '
            'active -- add them to robot.exclude.groups so the servo '
            'bridge and the joint controller do not both drive the same '
            'arm.'.format(still_active)
        )
    ee_features = [
        key for spec in ee_control
        for key in ee_action_features(spec.ee_pose, rotation=ee_rotation)
    ]
    known_ee_poses = {e.name: e for e in (desc.ee_poses or [])}
    ee_state_specs = [
        (e.name, e.source_frame, e.target_frame)
        for spec in ee_control
        for e in [known_ee_poses[spec.ee_pose]]
    ]
    return ee_control, ee_features, ee_state_specs
