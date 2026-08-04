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

from typing import Dict, List, Optional


class ActionInterpolator:
    # Dict-space port of lerobot.utils.action_interpolator.ActionInterpolator.
    def __init__(self, multiplier: int) -> None:
        if multiplier < 1:
            raise ValueError(f'multiplier must be >= 1, got {multiplier}')
        self.multiplier = multiplier
        self._prev: Optional[Dict[str, float]] = None
        self._buffer: List[Dict[str, float]] = []
        self._idx = 0

    @property
    def enabled(self) -> bool:
        return self.multiplier > 1

    def reset(self) -> None:
        self._prev = None
        self._buffer = []
        self._idx = 0

    def needs_new_action(self) -> bool:
        return self._idx >= len(self._buffer)

    def add(self, action: Dict[str, float]) -> None:
        if self.multiplier > 1 and self._prev is not None:
            prev = self._prev
            self._buffer = []
            for i in range(1, self.multiplier + 1):
                t = i / self.multiplier
                interp = {k: prev.get(k, v) + t * (v - prev.get(k, v)) for k, v in action.items()}
                self._buffer.append(interp)
        else:
            # First step: no previous action yet, run at base rate.
            self._buffer = [dict(action)]
        self._prev = dict(action)
        self._idx = 0

    def get(self) -> Optional[Dict[str, float]]:
        if self._idx >= len(self._buffer):
            return None
        step = self._buffer[self._idx]
        self._idx += 1
        return step
