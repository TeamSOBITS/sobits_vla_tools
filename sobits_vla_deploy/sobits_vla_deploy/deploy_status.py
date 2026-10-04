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

"""Pure state tracker behind the deploy node's ~/status feed (no rclpy)."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

# Numbers mirror sobits_interfaces/msg/VlaStatus; test_deploy_status_wire checks them.
STATE_STOPPED = 0
STATE_PLAYING = 3
STATE_ERROR = 4
STATE_RESETTING = 5

EVENT_NONE = 0
EVENT_STARTED = 1
EVENT_ERROR = 7
EVENT_TASK_SET = 8
EVENT_REJECTED = 9
EVENT_STOPPED = 10
EVENT_EPISODE_DONE = 11
EVENT_ENGAGED = 12
EVENT_RELEASED = 13
EVENT_RESET_DONE = 14

_RATE_WINDOW = 8
_RATE_STALE_S = 5.0


@dataclass
class Snapshot:
    """Everything one VlaStatus message carries, minus stamp and stage."""

    state: int
    event: int
    event_seq: int
    task_set: bool
    task_name: str
    episode_name: str
    elapsed_sec: float
    detail: str
    message: str
    policy: str
    deadman_enabled: bool
    deadman_engaged: bool
    steps: int
    inference_hz: float
    outcome: str


def format_elapsed(seconds: float) -> str:
    """Return HH:MM:SS for a non-negative duration."""
    total = max(0, int(seconds))
    return '{:02d}:{:02d}:{:02d}'.format(total // 3600, total % 3600 // 60, total % 60)


class DeployStatus:
    """Deploy-stage state machine; `now` is always a monotonic time in seconds."""

    def __init__(self, policy: str, deadman_enabled: bool, task_name: str = '') -> None:
        self._policy = policy
        self._deadman_enabled = deadman_enabled
        self._task_name = task_name
        self._task_set = bool(task_name)
        self._state = STATE_STOPPED
        self._event = EVENT_NONE
        self._seq = 0
        self._detail = ''
        self._outcome = ''
        self._pending_outcome = ''
        self._episode = 0
        self._engaged = False
        self._t0: Optional[float] = None
        self._frozen = 0.0
        self._steps = 0
        self._chunks: Deque[float] = deque(maxlen=_RATE_WINDOW)

    def _emit(self, event: int, detail: str = '') -> None:
        self._event = event
        self._detail = detail
        self._seq += 1

    def start(self, now: float, clock_running: bool) -> None:
        """PLAY began; the clock starts now or, with a deadman, on first engagement."""
        self._state = STATE_PLAYING
        self._episode += 1
        self._engaged = False
        self._t0 = now if clock_running else None
        self._frozen = 0.0
        self._steps = 0
        self._chunks.clear()
        self._outcome = ''
        self._pending_outcome = ''
        self._emit(EVENT_STARTED)

    def engaged(self, now: float) -> None:
        self._engaged = True
        if self._t0 is None:
            self._t0 = now
        self._emit(EVENT_ENGAGED)

    def released(self, now: float) -> None:
        self._engaged = False
        self._emit(EVENT_RELEASED)

    def stop(self, now: float, outcome: str) -> None:
        """PLAY ended; elapsed freezes and the world reset begins."""
        self._frozen = self.elapsed_sec(now)
        self._state = STATE_RESETTING
        self._engaged = False
        self._outcome = outcome
        self._pending_outcome = outcome
        self._chunks.clear()
        self._emit(EVENT_STOPPED, outcome)

    def resetting(self, now: float) -> None:
        """Idle STOP/RESET: world reset starts without an episode, no event."""
        self._state = STATE_RESETTING
        self._outcome = ''
        self._pending_outcome = ''

    def reset_done(self, now: float) -> None:
        if self._state != STATE_RESETTING:
            return
        self._state = STATE_STOPPED
        if self._pending_outcome:
            self._emit(EVENT_EPISODE_DONE, self._pending_outcome)
        else:
            self._emit(EVENT_RESET_DONE)
        self._pending_outcome = ''

    def error(self, detail: str) -> None:
        self._state = STATE_ERROR
        self._emit(EVENT_ERROR, detail)

    def set_task(self, name: str) -> None:
        self._task_name = name
        self._task_set = bool(name)
        self._emit(EVENT_TASK_SET, name)

    def reject(self, why: str) -> None:
        self._emit(EVENT_REJECTED, why)

    def step(self) -> None:
        self._steps += 1

    def chunk(self, now: float) -> None:
        self._chunks.append(now)

    def elapsed_sec(self, now: float) -> float:
        if self._state != STATE_PLAYING:
            return self._frozen
        return 0.0 if self._t0 is None else max(0.0, now - self._t0)

    def inference_hz(self, now: float) -> float:
        if self._state != STATE_PLAYING or len(self._chunks) < 2:
            return 0.0
        if now - self._chunks[-1] > _RATE_STALE_S:
            return 0.0
        span = self._chunks[-1] - self._chunks[0]
        return (len(self._chunks) - 1) / span if span > 0 else 0.0

    def describe(self, now: float) -> str:
        """Return the human-readable message line."""
        if self._state == STATE_ERROR:
            return 'error'
        if self._state == STATE_RESETTING:
            return 'resetting'
        if self._state == STATE_PLAYING:
            held = ' (released)' if self._deadman_enabled and not self._engaged else ''
            return 'playing {}{}'.format(format_elapsed(self.elapsed_sec(now)), held)
        return 'done: ' + self._outcome if self._outcome else 'idle'

    def snapshot(self, now: float) -> Snapshot:
        """Return the current picture and clear the pending event."""
        snap = Snapshot(
            state=self._state, event=self._event, event_seq=self._seq,
            task_set=self._task_set, task_name=self._task_name,
            episode_name='episode_{}'.format(self._episode) if self._episode else '',
            elapsed_sec=self.elapsed_sec(now), detail=self._detail,
            message=self.describe(now), policy=self._policy,
            deadman_enabled=self._deadman_enabled, deadman_engaged=self._engaged,
            steps=self._steps, inference_hz=self.inference_hz(now), outcome=self._outcome,
        )
        self._event = EVENT_NONE
        self._detail = ''
        return snap
