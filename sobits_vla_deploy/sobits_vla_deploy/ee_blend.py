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

"""Blending helpers for ee.<arm>.* action keys shared by the chunk buffer and interpolator."""

from typing import Dict, List

from sobits_vla_common.geometry import slerp_rotvec
from sobits_vla_common.robot_descriptor import EE_ACTION_AXES, EE_ROTATION_AXES

_EE_ANGLE_AXES = set(EE_ACTION_AXES[3:])
_ROTVEC_AXES = tuple(EE_ROTATION_AXES['rotvec'][3:])


def is_ee_angle_key(key: str) -> bool:
    """Return True for 'ee.<arm>.{roll,pitch,yaw}' keys (unwrapped across +-pi)."""
    parts = key.split('.')
    return len(parts) == 3 and parts[0] == 'ee' and parts[2] in _EE_ANGLE_AXES


def rotvec_groups(keys) -> Dict[str, List[str]]:
    """Map arm -> [ee.<arm>.rx, .ry, .rz] for every arm whose three rotvec keys are all present."""
    arms = {k.split('.')[1] for k in keys if k.startswith('ee.') and k.count('.') == 2}
    groups = {}
    for arm in arms:
        group = [f'ee.{arm}.{ax}' for ax in _ROTVEC_AXES]
        if all(k in keys for k in group):
            groups[arm] = group
    return groups


def blend_rotvec_groups(
    out: Dict[str, float], old: Dict[str, float], new: Dict[str, float], t: float
) -> None:
    """
    Overwrite out's rotvec groups with the SO(3) interpolation old -> new at t.

    Per-axis lerp of rotvecs collapses toward identity when the pair straddles
    pi (sign flip), e.g. a top-down grasp; only groups present in both inputs
    are touched, a partial group keeps its per-axis value.
    """
    for keys in rotvec_groups(new).values():
        if not all(k in old for k in keys):
            continue
        a = [float(old[k]) for k in keys]
        b = [float(new[k]) for k in keys]
        for k, v in zip(keys, slerp_rotvec(a, b, t)):
            out[k] = float(v)
