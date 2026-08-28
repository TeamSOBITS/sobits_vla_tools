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
Joint-state/command alignment: measured state, ZOH commanded action, deltas.

Mobile-base cmd_vel/odom velocity lives here too -- the original code always
appends it onto the same state/action vectors right after the joint terms.
"""

from sobits_vla_rosbag_conversion.sync.core import hold_dict, interpolate_dict, interpolate_vector


def interpolate_joint_state(
    joint_pos_series, joint_vel_series, t_sec, action_features,
    joint_pos_times, joint_vel_times,
):
    joint_pos = interpolate_dict(joint_pos_series, t_sec, action_features, times=joint_pos_times)
    joint_vel = interpolate_dict(joint_vel_series, t_sec, action_features, times=joint_vel_times)
    return joint_pos, joint_vel


def interpolate_base_velocity(
    cmd_vel_series, odom_series, t_sec, base_keys, cmd_vel_times, odom_times,
):
    cmd_vel = interpolate_vector(cmd_vel_series, t_sec, len(base_keys), times=cmd_vel_times)
    odom_vel = interpolate_vector(odom_series, t_sec, len(base_keys), times=odom_times)
    return cmd_vel, odom_vel


def synthesize_action(
    t_sec, fps, action_features, cmd_series_by_feature,
    cmd_series_by_feature_times, joint_pos_series, joint_pos_times,
):
    """
    Commanded joint action via zero-order hold.

    Falls back to future measured state before the first command
    arrives (see core.hold_dict).
    """
    action = []
    for feat in action_features:
        cmd_series_for_feat = cmd_series_by_feature[feat]
        if cmd_series_for_feat and t_sec >= cmd_series_for_feat[0][0]:
            cmd_val = hold_dict(
                cmd_series_for_feat, t_sec, [feat],
                times=cmd_series_by_feature_times[feat],
            )[feat]
            action.append(cmd_val)
        else:
            t_next = t_sec + (1.0 / fps if fps > 0 else 0.1)
            next_joint_pos = interpolate_dict(
                joint_pos_series, t_next, [feat], times=joint_pos_times
            )
            action.append(next_joint_pos[feat])
    return action


def to_relative_action(action, state):
    # Delta command = commanded action minus current measured state.
    return [act_val - st_val for act_val, st_val in zip(action, state)]
