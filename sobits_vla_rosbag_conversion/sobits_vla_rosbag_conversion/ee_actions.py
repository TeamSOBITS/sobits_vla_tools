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

"""EE-action arm resolution and the action-convention record written with every dataset."""

from typing import List, Tuple

from sobits_vla_common.robot_descriptor import EE_ROTATION_AXES, resolve_ee_action_specs


def resolve_ee_actions(
    desc, arms: List[str], rotation: str, log_info,
) -> List[Tuple[str, str, str]]:
    """
    (name, source_frame, target_frame) TF triples for the arms that get EE actions.

    ee_actions.arms is an optional override of the descriptor-derived list; an
    explicit arm is validated against the same rule (resolve_ee_action_specs).
    """
    if rotation not in EE_ROTATION_AXES:
        raise ValueError(
            f'ee_actions.rotation must be one of {sorted(EE_ROTATION_AXES)}, got {rotation!r}'
        )
    arms = [a for a in arms if a]
    specs = resolve_ee_action_specs(desc, arms, param='ee_actions.arms')
    if specs and not arms:
        log_info(f'ee_actions.arms not set -- derived from descriptor: '
                 f'{[s.ee_pose for s in specs]}')
    ee_by_name = {ee.name: ee for ee in (desc.ee_poses or [])}
    return [
        (spec.ee_pose, ee_by_name[spec.ee_pose].source_frame,
         ee_by_name[spec.ee_pose].target_frame)
        for spec in specs
    ]


def action_convention(ee_action_specs: list, ee_rotation: str) -> dict:
    """How action/state are encoded; persisted in the sidecar and conversion stats."""
    convention = {'action_mode': 'absolute'}
    if ee_action_specs:
        convention['ee_rotation'] = ee_rotation
        if ee_rotation == 'rpy':
            convention['rpy_convention'] = 'xyz_extrinsic'
        convention['ee_frames'] = {
            name: {'source': src, 'target': tgt} for name, src, tgt in ee_action_specs
        }
    return convention
