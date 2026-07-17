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

from collections import deque
from threading import Lock
from typing import Any, Dict, List, Optional


class ActionChunkBuffer:
    def __init__(self, aggregate_fn_name: str) -> None:
        self._queue: deque[Dict[str, float]] = deque()
        # Model-space (normalised) tensor kept in parallel with _queue for RTC
        # left_over guidance. Shape: (T_remaining, A) on CPU.
        self._original_queue: Optional[Any] = None  # torch.Tensor | None
        self._lock = Lock()
        self._aggregate_fn_name = aggregate_fn_name

    def size(self) -> int:
        with self._lock:
            return len(self._queue)

    def pop(self) -> Optional[Dict[str, float]]:
        with self._lock:
            if self._original_queue is not None and len(self._original_queue) > 0:
                self._original_queue = self._original_queue[1:]
                if len(self._original_queue) == 0:
                    self._original_queue = None
            if not self._queue:
                return None
            return self._queue.popleft()

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()
            self._original_queue = None

    def left_over(self, count: int) -> Optional[Any]:
        """Return all remaining model-space steps as a Tensor (T, A), or None."""
        if count <= 0:
            return None
        with self._lock:
            if self._original_queue is None or len(self._original_queue) == 0:
                return None
            return self._original_queue.clone()

    def replace(
        self, original_actions: Any, processed_steps: List[Dict[str, float]], delay: int
    ) -> None:
        """RTC queue replacement: discard stale entries, start fresh."""
        if not processed_steps and original_actions is None:
            return
        with self._lock:
            clamped = 0
            if original_actions is not None and len(processed_steps) > 0:
                clamped = max(
                    0, min(delay, len(original_actions), len(processed_steps))
                )
            self._queue = deque(processed_steps[clamped:])
            if original_actions is not None and len(original_actions) > clamped:
                self._original_queue = original_actions[clamped:].clone().cpu()
            else:
                self._original_queue = None

    def merge(self, chunk: List[Dict[str, float]], overlap: int) -> None:
        if not chunk:
            return
        with self._lock:
            overlap_steps = min(overlap, len(self._queue), len(chunk))
            for idx in range(overlap_steps):
                self._queue[idx] = self._aggregate(self._queue[idx], chunk[idx])
            for step in chunk[overlap_steps:]:
                self._queue.append(step)

    def merge_aligned(self, chunk: List[Dict[str, float]], q_len_at_obs: int) -> None:
        if not chunk:
            return
        with self._lock:
            q_len_now = len(self._queue)
            steps_executed = max(0, q_len_at_obs - q_len_now)
            overlap_steps = min(q_len_now, max(0, len(chunk) - steps_executed))
            for idx in range(overlap_steps):
                self._queue[idx] = self._aggregate(
                    self._queue[idx], chunk[idx + steps_executed]
                )
            for step in chunk[overlap_steps + steps_executed:]:
                self._queue.append(step)

    def _aggregate(
        self,
        old_step: Dict[str, float],
        new_step: Dict[str, float],
    ) -> Dict[str, float]:
        out = dict(old_step)
        for key, new_val in new_step.items():
            old_val = old_step.get(key, new_val)
            if self._aggregate_fn_name == 'newest':
                out[key] = float(new_val)
            else:
                out[key] = float(0.5 * old_val + 0.5 * new_val)
        return out
