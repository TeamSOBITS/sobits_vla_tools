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

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.deadman import DeadmanGate, Edge  # noqa: E402


def test_starts_released_and_silent():
    gate = DeadmanGate()
    assert not gate.engaged
    assert gate.update(False) is None


def test_engage_and_release_edges():
    gate = DeadmanGate()
    assert gate.update(True) is Edge.ENGAGED
    assert gate.engaged
    assert gate.update(True) is None
    assert gate.update(False) is Edge.RELEASED
    assert not gate.engaged
    assert gate.update(False) is None


def test_grip_held_across_stop_play_reengages():
    # The bug: the latch lived only while playing, so a held grip never re-engaged.
    gate = DeadmanGate()
    assert gate.update(True) is Edge.ENGAGED
    gate.reset()
    assert gate.update(True) is Edge.ENGAGED


def test_reset_while_released_is_a_noop():
    gate = DeadmanGate()
    gate.reset()
    assert gate.update(False) is None
