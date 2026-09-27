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

import math
from typing import Dict, List

from geometry_msgs.msg import TransformStamped
from sobits_vla_common.geometry import rpy_to_quat, unwrap_rpy
from sobits_vla_common.robot_descriptor import EE_ACTION_AXES, EEControlSpec
from std_msgs.msg import Bool


class ServoTargetPublisher:
    """
    Streams absolute EE targets to sobits_teleop's servo bridge.

    Broadcasts parent_frame -> spec.target_frame and latches Bool on
    spec.enable_topic; owns deploy-side target smoothing (the bridge has
    no rate limiting -- that lives in the quest teleop node we bypass).
    """

    def __init__(
        self,
        arms: List[EEControlSpec],
        parent_frame: str,
        tf_broadcaster,
        enable_publishers: Dict[str, object],
        max_lin_step_m: float,
        max_ang_step_rad: float,
        logger=None,
    ):
        self.arms = list(arms)
        self.parent_frame = parent_frame
        self.tf_broadcaster = tf_broadcaster
        self.enable_publishers = enable_publishers
        self.max_lin_step_m = max_lin_step_m
        self.max_ang_step_rad = max_ang_step_rad
        self.logger = logger

        # ee_pose name -> [x, y, z, roll, pitch, yaw] last commanded target.
        self._last_target: Dict[str, List[float]] = {}
        self._engaged = False

    def _warn(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.warning(msg)

    @property
    def engaged(self) -> bool:
        return self._engaged

    def engage(self, state_vector: Dict[str, float]) -> bool:
        """
        Seed each arm's last-target from state_vector and latch enable=true.

        Seeding from the current measured EE pose prevents a first-step jump
        to whatever stale target a prior episode left behind. Returns whether
        any arm was actually seeded, so callers can tell a no-op engage apart
        from a real one.
        """
        self._last_target = {}
        for spec in self.arms:
            keys = [f'ee.{spec.ee_pose}.{ax}' for ax in EE_ACTION_AXES]
            if any(k not in state_vector for k in keys):
                self._warn(
                    'ServoTargetPublisher.engage: missing {!r} in state_vector '
                    '-- arm {!r} not seeded.'.format(keys, spec.ee_pose)
                )
                continue
            values = [float(state_vector[k]) for k in keys]
            if all(v == 0.0 for v in values):
                # Fail toward "arm does not move": an all-zero pose means the
                # EE state was never measured -- enabling would servo to origin.
                self._warn(
                    'ServoTargetPublisher.engage: state_vector for arm {!r} is '
                    'all-zero (EE state never measured) -- arm left disabled.'.format(
                        spec.ee_pose
                    )
                )
                continue
            self._last_target[spec.ee_pose] = values

            pub = self.enable_publishers.get(spec.ee_pose)
            if pub is not None:
                pub.publish(Bool(data=True))
        self._engaged = bool(self._last_target)
        return self._engaged

    def publish_step(self, step: Dict[str, float], now_msg) -> None:
        """Clamp+broadcast one control step's EE targets. No-op if not engaged."""
        if not self._engaged:
            return
        for spec in self.arms:
            keys = [f'ee.{spec.ee_pose}.{ax}' for ax in EE_ACTION_AXES]
            if any(k not in step for k in keys):
                continue
            prev = self._last_target.get(spec.ee_pose)
            if prev is None:
                # Never seeded (engage skipped it) => enable was never latched
                # true for this arm; broadcasting targets would be dead weight.
                continue
            target = [float(step[k]) for k in keys]
            clamped = self._clamp_target(prev, target)
            self._last_target[spec.ee_pose] = clamped
            self._broadcast(spec, clamped, now_msg)

    def _clamp_target(
        self, prev: List[float], target: List[float]
    ) -> List[float]:
        px, py, pz, pr, pp, pyaw = prev
        tx, ty, tz, tr, tp, tyaw = target
        tr, tp, tyaw = unwrap_rpy((tr, tp, tyaw), (pr, pp, pyaw))

        dx, dy, dz = tx - px, ty - py, tz - pz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > self.max_lin_step_m and dist > 0.0:
            scale = self.max_lin_step_m / dist
            dx, dy, dz = dx * scale, dy * scale, dz * scale

        max_ang = self.max_ang_step_rad
        dr = max(-max_ang, min(max_ang, tr - pr))
        dp = max(-max_ang, min(max_ang, tp - pp))
        dyaw_ = max(-max_ang, min(max_ang, tyaw - pyaw))

        return [px + dx, py + dy, pz + dz, pr + dr, pp + dp, pyaw + dyaw_]

    def _broadcast(self, spec: EEControlSpec, target: List[float], now_msg) -> None:
        x, y, z, roll, pitch, yaw = target
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = self.parent_frame
        t.child_frame_id = spec.target_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def disable_tracking(self) -> None:
        """Latch enable=false on every arm and drop seeds. Idempotent."""
        for spec in self.arms:
            pub = self.enable_publishers.get(spec.ee_pose)
            if pub is not None:
                pub.publish(Bool(data=False))
        self._last_target = {}
        self._engaged = False
