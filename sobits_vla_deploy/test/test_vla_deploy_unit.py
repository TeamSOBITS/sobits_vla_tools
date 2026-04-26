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

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sobits_vla_deploy'))

from sobits_vla_deploy import ActionChunkBuffer  # noqa: E402


# ---------------------------------------------------------------------------
# ActionChunkBuffer tests
# ---------------------------------------------------------------------------

def _make_step(val: float) -> dict:
    return {'j0': val, 'j1': val * 2}


class TestActionChunkBufferBasic:

    def test_size_empty(self):
        buf = ActionChunkBuffer('weighted_average')
        assert buf.size() == 0

    def test_pop_empty_returns_none(self):
        buf = ActionChunkBuffer('weighted_average')
        assert buf.pop() is None

    def test_merge_and_pop(self):
        buf = ActionChunkBuffer('weighted_average')
        chunk = [_make_step(float(i)) for i in range(5)]
        buf.merge(chunk, overlap=0)
        assert buf.size() == 5
        first = buf.pop()
        assert first == {'j0': 0.0, 'j1': 0.0}
        assert buf.size() == 4

    def test_clear(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([_make_step(1.0), _make_step(2.0)], overlap=0)
        buf.clear()
        assert buf.size() == 0
        assert buf.pop() is None

    def test_merge_empty_chunk_is_noop(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([], overlap=0)
        assert buf.size() == 0


class TestActionChunkBufferAggregate:

    def test_weighted_average_overlap(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([{'j0': 0.0}], overlap=0)
        buf.merge([{'j0': 1.0}], overlap=1)
        step = buf.pop()
        assert step is not None
        assert abs(step['j0'] - 0.5) < 1e-9

    def test_newest_overlap(self):
        buf = ActionChunkBuffer('newest')
        buf.merge([{'j0': 0.0}], overlap=0)
        buf.merge([{'j0': 1.0}], overlap=1)
        step = buf.pop()
        assert step is not None
        assert abs(step['j0'] - 1.0) < 1e-9

    def test_overlap_larger_than_queue(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([_make_step(1.0)], overlap=0)
        buf.merge([_make_step(0.0), _make_step(2.0)], overlap=5)
        assert buf.size() == 2

    def test_left_over(self):
        buf = ActionChunkBuffer('weighted_average')
        chunk = [_make_step(float(i)) for i in range(4)]
        buf.merge(chunk, overlap=0)
        result = buf.left_over(2)
        assert len(result) == 2
        assert result[0]['j0'] == 0.0
        assert result[1]['j0'] == 1.0


# ---------------------------------------------------------------------------
# _to_action_steps tests (via VlaDeployNode with mocked ROS)
# ---------------------------------------------------------------------------

class MockNode:
    """Minimal mock to allow importing VlaDeployNode._to_action_steps."""

    def __init__(self):
        self._joint_features = ['j0', 'j1', 'j2']
        self._mobile_base_features = ['x.vel']

    def _to_action_steps(self, raw_actions):
        from sobits_vla_deploy import LeRobotDeployNode as VlaDeployNode
        return VlaDeployNode._to_action_steps(self, raw_actions)


class TestToActionSteps:

    def _node(self):
        return MockNode()

    def test_1d_numpy(self):
        node = self._node()
        raw = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
        steps = node._to_action_steps(raw)
        assert len(steps) == 1
        assert steps[0] == {'j0': 1.0, 'j1': 2.0, 'j2': 3.0, 'x.vel': 4.0}

    def test_2d_numpy(self):
        node = self._node()
        raw = np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]], dtype=np.float32)
        steps = node._to_action_steps(raw)
        assert len(steps) == 2
        assert steps[0]['j0'] == 1.0
        assert steps[1]['j0'] == 5.0

    def test_1d_truncated(self):
        node = self._node()
        raw = np.array([1.0, 2.0], dtype=np.float32)
        steps = node._to_action_steps(raw)
        assert len(steps) == 1
        assert 'j0' in steps[0]
        assert 'j1' in steps[0]
        assert 'j2' not in steps[0]

    def test_empty_action_keys(self):
        node = self._node()
        node._joint_features = []
        node._mobile_base_features = []
        raw = np.array([1.0, 2.0], dtype=np.float32)
        steps = node._to_action_steps(raw)
        assert steps == []

    def test_dict_input_flat(self):
        node = self._node()
        raw = {'j0': 0.5, 'j1': 1.5, 'j2': 2.5, 'x.vel': 0.1, 'extra': 99.0}
        steps = node._to_action_steps(raw)
        assert len(steps) == 1
        assert steps[0]['j0'] == 0.5
        assert 'extra' not in steps[0]

    def test_dict_input_chunked(self):
        node = self._node()
        raw = {'j0': [0.0, 1.0], 'j1': [2.0, 3.0], 'j2': [4.0, 5.0], 'x.vel': [0.1, 0.2]}
        steps = node._to_action_steps(raw)
        assert len(steps) == 2
        assert steps[0]['j0'] == 0.0
        assert steps[1]['j0'] == 1.0

    def test_unsupported_returns_empty(self):
        node = self._node()
        steps = node._to_action_steps('not_a_valid_type')
        assert steps == []
