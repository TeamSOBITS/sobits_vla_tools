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

from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ActionExecutor:
    def __init__(
        self,
        joint_groups: List[Any],
        group_publishers: Dict[str, Any],
        base_pub: Optional[Any],
        mobile_base_features: List[str],
        max_vel_x: float,
        max_vel_y: float,
        max_vel_theta: float,
        step_duration: Any,
        logger=None,
        max_vel_z: float = 0.0,
        linear_deadband: float = 0.0,
        angular_deadband: float = 0.0,
    ):
        self.joint_groups = joint_groups
        self.group_publishers = group_publishers
        self.base_pub = base_pub
        self.mobile_base_features = mobile_base_features
        self.max_vel_x = max_vel_x
        self.max_vel_y = max_vel_y
        self.max_vel_theta = max_vel_theta
        self.max_vel_z = max_vel_z
        self.linear_deadband = linear_deadband
        self.angular_deadband = angular_deadband
        self.step_duration = step_duration
        self.logger = logger

    def execute_action(
        self,
        step: Dict[str, float],
        state_vector: Dict[str, float],
        cmd_vector: Dict[str, float],
        now_msg: Any,
    ) -> tuple[str, str]:
        joint_log_parts = []
        for group in self.joint_groups:
            msg = JointTrajectory()
            msg.header.stamp = now_msg
            msg.joint_names = group.joints_ros
            point = JointTrajectoryPoint()
            raw_positions = [
                float(step.get(feature, state_vector.get(feature, 0.0)))
                for feature in group.features
            ]
            delta = group.max_joint_delta
            if delta > 0.0:
                # Clamp goal vs MEASURED position
                point.positions = []
                for feat, raw in zip(group.features, raw_positions):
                    present = state_vector.get(feat, raw)
                    clamped = float(
                        max(present - delta, min(present + delta, raw))
                    )
                    point.positions.append(clamped)
                    cmd_vector[feat] = clamped
            else:
                point.positions = raw_positions
                for feat, pos in zip(group.features, raw_positions):
                    cmd_vector[feat] = pos
            point.time_from_start = self.step_duration
            msg.points = [point]
            self.group_publishers[group.name].publish(msg)

            pos_strs = ['{:.3f}'.format(p) for p in point.positions]
            joint_log_parts.append(
                '{}: [{}]'.format(group.name, ', '.join(pos_strs))
            )

        base_log = ''
        if self.base_pub is not None:
            cmd = Twist()
            # Only axes the descriptor declares are driven; the rest stay 0 so
            # a policy emitting an axis this base lacks cannot command it.
            feats = self.mobile_base_features

            def _axis(key: str, max_vel: float, deadband: float) -> float:
                if key not in feats:
                    return 0.0
                v = float(step.get(key, 0.0))
                if max_vel > 0.0:
                    v = max(-max_vel, min(max_vel, v))
                return 0.0 if abs(v) < deadband else v

            lin_db = self.linear_deadband
            cmd.linear.x = _axis('x.vel', self.max_vel_x, lin_db)
            cmd.linear.y = _axis('y.vel', self.max_vel_y, lin_db)
            cmd.linear.z = _axis('z.vel', self.max_vel_z, lin_db)
            cmd.angular.z = _axis(
                'theta.vel', self.max_vel_theta, self.angular_deadband
            )
            self.base_pub.publish(cmd)
            base_log = ' | BASE: x={:.3f} y={:.3f} th={:.3f}'.format(
                cmd.linear.x, cmd.linear.y, cmd.angular.z
            )

        joint_log = ' | '.join(joint_log_parts)
        return joint_log, base_log
