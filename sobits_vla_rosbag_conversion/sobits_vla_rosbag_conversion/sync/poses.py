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

"""EE-pose synthesis via TF lookups: rotvec, rpy (per-axis unwrap, fb188e1) or quat."""

from sobits_vla_common.geometry import quat_shortest_arc, unwrap_rpy
from sobits_vla_rosbag_conversion.offline_tf_tree import (
    mat_to_pose6d, mat_to_pose6d_rotvec, mat_to_pose7d,
)

# Not sobits_vla_common.gz_utils.wrap_pi: that wraps an absolute angle to
# (-pi, pi]; this unwraps a delta against the previous sample (fb188e1).


def resolve_ee_pose(tf_tree, ee_src, ee_tgt, stamp_ns):
    """Look up one EE transform; returns a 4x4 matrix or None on failure."""
    return tf_tree.resolve(ee_tgt, ee_src, stamp_ns)


def synthesize_ee_action(tf_tree, ee_src, ee_tgt, t_ns, fps, prev_state_pose):
    """
    (state_pose6, action_pose6) or None when the state lookup fails.

    state = observed EE pose at t, action = pose at t + 1/fps (shift-forward),
    both unwrapped per-axis against prev_state_pose (fb188e1). None when
    either lookup fails (missing frame, or TF stale beyond the tree's
    max_age): a held pose would label the frame as zero motion.
    """
    state_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, t_ns)
    if state_mat is None:
        return None
    state_pose = mat_to_pose6d(state_mat)
    if prev_state_pose is not None:
        rpy = unwrap_rpy(state_pose[3:6], prev_state_pose[3:6])
        state_pose[3:6] = rpy

    action_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, _future_ns(t_ns, fps))
    if action_mat is None:
        return None
    action_pose = mat_to_pose6d(action_mat)
    rpy = unwrap_rpy(action_pose[3:6], state_pose[3:6])
    action_pose[3:6] = rpy

    return state_pose, action_pose


def _future_ns(t_ns: int, fps: float) -> int:
    """Stamp of the shift-forward action sample, one frame period ahead."""
    return t_ns + int(round((1.0 / fps) * 1e9)) if fps > 0 else t_ns


def synthesize_ee_action_rotvec(tf_tree, ee_src, ee_tgt, t_ns, fps):
    """
    (state_pose6, action_pose6) rotvec [x, y, z, rx, ry, rz], or None on state lookup failure.

    Same shift-forward rule as synthesize_ee_action; None when either lookup fails.
    No unwrap: scipy's rotvec is canonical (|rotvec| <= pi) and each pose is absolute.
    """
    state_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, t_ns)
    if state_mat is None:
        return None
    action_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, _future_ns(t_ns, fps))
    if action_mat is None:
        return None
    return mat_to_pose6d_rotvec(state_mat), mat_to_pose6d_rotvec(action_mat)


def synthesize_ee_action_quat(tf_tree, ee_src, ee_tgt, t_ns, fps, prev_state_quat):
    """
    (state_pose7, action_pose7) or None when the state lookup fails.

    Quaternion analogue of synthesize_ee_action: 7D = [x, y, z, qx, qy, qz,
    qw]. state = observed EE pose at t, action = pose at t + 1/fps
    (shift-forward); None when either lookup fails. Continuity via
    shortest-arc alignment (quat_shortest_arc) instead of per-axis unwrap:
    state's quat is aligned against prev_state_quat (or left as-is when
    prev is None), and action's quat is aligned against state's quat.
    """
    state_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, t_ns)
    if state_mat is None:
        return None
    state_pose = mat_to_pose7d(state_mat)
    if prev_state_quat is not None:
        state_pose[3:7] = quat_shortest_arc(state_pose[3:7], prev_state_quat[3:7])

    action_mat = resolve_ee_pose(tf_tree, ee_src, ee_tgt, _future_ns(t_ns, fps))
    if action_mat is None:
        return None
    action_pose = mat_to_pose7d(action_mat)
    action_pose[3:7] = quat_shortest_arc(action_pose[3:7], state_pose[3:7])

    return state_pose, action_pose
