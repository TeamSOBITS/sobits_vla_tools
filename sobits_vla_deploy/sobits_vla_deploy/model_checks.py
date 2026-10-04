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

"""Deploy-time refusals: the config's action space must agree with what the checkpoint emits."""

from typing import List, Optional


def check_action_space_matches_model(
    *,
    action_space: str,
    ee_rotation: str,
    rtc_enabled: bool,
    model_repo_id: str,
    model_action_feature_names: Optional[List[str]],
    model_ee_rotation: str,
    model_ee_relative: bool,
    model_use_relative_actions: bool,
) -> None:
    """Raise RuntimeError on any config/checkpoint disagreement the node cannot resolve."""
    names = model_action_feature_names or []
    model_has_ee = any(n.startswith('ee.') for n in names)
    if action_space == 'joint' and model_has_ee:
        raise RuntimeError(
            'model.action_space is "joint" but the checkpoint {!r} emits '
            'ee.* action features -- set model.action_space: ee.'.format(model_repo_id)
        )
    if action_space == 'ee' and not model_has_ee:
        raise RuntimeError(
            'model.action_space is "ee" but the checkpoint {!r} emits no '
            'ee.* action features -- set model.action_space: joint.'.format(model_repo_id)
        )
    if model_has_ee and model_ee_rotation != ee_rotation:
        raise RuntimeError(
            'model.ee_rotation is {!r} but the checkpoint {!r} emits {!r} ee.* '
            'features -- set model.ee_rotation: {}.'.format(
                ee_rotation, model_repo_id, model_ee_rotation, model_ee_rotation,
            )
        )
    if model_has_ee and ee_rotation == 'quat':
        raise RuntimeError(
            'model.ee_rotation "quat" is not supported at deploy: ObsBuilder and '
            'ServoTargetPublisher support rotvec and rpy only.'
        )
    if rtc_enabled and (model_ee_relative or model_use_relative_actions):
        raise RuntimeError(
            'rtc.enabled with a relative-action checkpoint {!r}: the RTC prefix '
            'is not re-anchored to the new observation yet -- set '
            'rtc.enabled: false.'.format(model_repo_id)
        )
