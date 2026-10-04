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

"""ROS wiring for the deploy ~/status feed; the state machine is deploy_status.py."""

from threading import Lock
from time import monotonic

from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sobits_interfaces.msg import VlaStatus
from sobits_vla_deploy.deploy_status import DeployStatus, Snapshot


def to_msg(snap: Snapshot, stamp) -> VlaStatus:
    """Map a tracker Snapshot onto a VlaStatus message."""
    msg = VlaStatus()
    msg.stamp = stamp
    msg.stage = VlaStatus.STAGE_DEPLOY
    msg.state = snap.state
    msg.event = snap.event
    msg.event_seq = snap.event_seq
    msg.task_set = snap.task_set
    msg.task_name = snap.task_name
    msg.episode_name = snap.episode_name
    msg.elapsed_sec = float(snap.elapsed_sec)
    msg.detail = snap.detail
    msg.message = snap.message
    msg.policy = snap.policy
    msg.deadman_enabled = snap.deadman_enabled
    msg.deadman_engaged = snap.deadman_engaged
    msg.steps = snap.steps
    msg.inference_hz = float(snap.inference_hz)
    msg.outcome = snap.outcome
    return msg


class DeployStatusFeed:
    """Publish the tracker on ~/status: one message per event plus a heartbeat."""

    def __init__(self, node, policy: str, deadman_enabled: bool, task_name: str,
                 rate_hz: float) -> None:
        self._node = node
        self._tracker = DeployStatus(policy, deadman_enabled, task_name)
        self._lock = Lock()
        # Latched so a late HUD sees the current state, not a blank until the next beat.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = node.create_publisher(VlaStatus, '~/status', qos)
        # Steady clock: a paused sim must not stall the heartbeat the HUD ages out on.
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = node.create_timer(1.0 / max(rate_hz, 0.1), self.publish, clock=self._clock)
        self.publish()

    def publish(self) -> None:
        """Heartbeat, and the flush for any event recorded since the last message."""
        with self._lock:
            self._publish_locked()

    def _publish_locked(self) -> None:
        snap = self._tracker.snapshot(monotonic())
        self._pub.publish(to_msg(snap, self._node.get_clock().now().to_msg()))

    def _apply(self, method: str, *args) -> None:
        with self._lock:
            getattr(self._tracker, method)(*args)
            self._publish_locked()

    def started(self, clock_running: bool) -> None:
        self._apply('start', monotonic(), clock_running)

    def engaged(self) -> None:
        self._apply('engaged', monotonic())

    def released(self) -> None:
        self._apply('released', monotonic())

    def stopped(self, outcome: str) -> None:
        self._apply('stop', monotonic(), outcome)

    def resetting(self) -> None:
        self._apply('resetting', monotonic())

    def reset_done(self) -> None:
        self._apply('reset_done', monotonic())

    def task_set(self, name: str) -> None:
        self._apply('set_task', name)

    def rejected(self, why: str) -> None:
        self._apply('reject', why)

    def error(self, detail: str) -> None:
        self._apply('error', detail)

    def step(self) -> None:
        with self._lock:
            self._tracker.step()

    def chunk(self) -> None:
        with self._lock:
            self._tracker.chunk(monotonic())
