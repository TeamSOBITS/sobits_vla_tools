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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.inference_engine import InferenceEngine  # noqa: E402
from sobits_vla_deploy.sobits_vla_deploy import ActionChunkBuffer  # noqa: E402


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
        import torch
        buf = ActionChunkBuffer('weighted_average')
        original_actions = torch.tensor([[0.0, 0.0], [1.0, 2.0], [2.0, 4.0], [3.0, 6.0]])
        processed_steps = [_make_step(float(i)) for i in range(4)]
        buf.replace(original_actions, processed_steps, delay=1)
        result = buf.left_over(2)
        assert result is not None
        assert result.shape == (3, 2)
        assert torch.allclose(result[0], torch.tensor([1.0, 2.0]))


# ---------------------------------------------------------------------------
# _to_action_steps tests (the live copy, InferenceEngine)
# ---------------------------------------------------------------------------


def _make_engine(
    joint_features=None, mobile_base_features=None, relative_exclude_features=None,
    postprocessor=None,
) -> InferenceEngine:
    """Minimal InferenceEngine with only what _to_action_steps reads set."""
    return InferenceEngine(
        policy=None,
        model_device='cpu',
        model_use_amp=False,
        control_hz=10.0,
        actions_per_chunk=50,
        chunk_size_threshold=0.6,
        async_enabled=True,
        single_step_mode=False,
        rtc_enabled=False,
        rtc_inference_delay=4,
        preprocessor=None,
        postprocessor=postprocessor,
        expected_state_dim=None,
        model_action_feature_names=None,
        model_use_relative_actions=False,
        joint_features=joint_features if joint_features is not None else ['j0', 'j1', 'j2'],
        mobile_base_features=mobile_base_features if mobile_base_features is not None else ['x.vel'],
        relative_exclude_features=relative_exclude_features,
    )


class TestToActionSteps:

    def test_1d_numpy(self):
        engine = _make_engine()
        raw = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
        steps = engine._to_action_steps(raw)
        assert len(steps) == 1
        assert steps[0] == {'j0': 1.0, 'j1': 2.0, 'j2': 3.0, 'x.vel': 4.0}

    def test_2d_numpy(self):
        engine = _make_engine()
        raw = np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]], dtype=np.float32)
        steps = engine._to_action_steps(raw)
        assert len(steps) == 2
        assert steps[0]['j0'] == 1.0
        assert steps[1]['j0'] == 5.0

    def test_1d_truncated(self):
        engine = _make_engine()
        raw = np.array([1.0, 2.0], dtype=np.float32)
        steps = engine._to_action_steps(raw)
        assert len(steps) == 1
        assert 'j0' in steps[0]
        assert 'j1' in steps[0]
        assert 'j2' not in steps[0]

    def test_empty_action_keys(self):
        engine = _make_engine(joint_features=[], mobile_base_features=[])
        raw = np.array([1.0, 2.0], dtype=np.float32)
        steps = engine._to_action_steps(raw)
        assert steps == []

    def test_dict_input_flat(self):
        engine = _make_engine()
        raw = {'j0': 0.5, 'j1': 1.5, 'j2': 2.5, 'x.vel': 0.1, 'extra': 99.0}
        steps = engine._to_action_steps(raw)
        assert len(steps) == 1
        assert steps[0]['j0'] == 0.5
        assert 'extra' not in steps[0]

    def test_dict_input_chunked(self):
        engine = _make_engine()
        raw = {'j0': [0.0, 1.0], 'j1': [2.0, 3.0], 'j2': [4.0, 5.0], 'x.vel': [0.1, 0.2]}
        steps = engine._to_action_steps(raw)
        assert len(steps) == 2
        assert steps[0]['j0'] == 0.0
        assert steps[1]['j0'] == 1.0

    def test_unsupported_returns_empty(self):
        engine = _make_engine()
        steps = engine._to_action_steps('not_a_valid_type')
        assert steps == []


# ---------------------------------------------------------------------------
# _apply_manual_delta tests
# ---------------------------------------------------------------------------


class TestApplyManualDelta:

    def test_adds_state_to_delta(self):
        engine = _make_engine(joint_features=['j0'], mobile_base_features=[])
        steps = [{'j0': 0.1}]
        engine._apply_manual_delta(steps, state_vector={'j0': 1.0})
        assert abs(steps[0]['j0'] - 1.1) < 1e-9

    def test_skips_mobile_base_features(self):
        engine = _make_engine(joint_features=['j0'], mobile_base_features=['x.vel'])
        steps = [{'j0': 0.1, 'x.vel': 0.5}]
        engine._apply_manual_delta(steps, state_vector={'j0': 1.0, 'x.vel': 2.0})
        assert abs(steps[0]['j0'] - 1.1) < 1e-9
        assert steps[0]['x.vel'] == 0.5

    def test_skips_relative_exclude_features(self):
        # e.g. hand_left fingers: never delta-converted regardless of the
        # policy's use_relative_actions mode.
        engine = _make_engine(
            joint_features=['j0', 'hand_left_finger_l_mcp_joint'],
            mobile_base_features=[],
            relative_exclude_features=['hand_left_finger_l_mcp_joint'],
        )
        steps = [{'j0': 0.1, 'hand_left_finger_l_mcp_joint': 0.3}]
        engine._apply_manual_delta(
            steps,
            state_vector={'j0': 1.0, 'hand_left_finger_l_mcp_joint': 1.5},
        )
        assert abs(steps[0]['j0'] - 1.1) < 1e-9
        assert steps[0]['hand_left_finger_l_mcp_joint'] == 0.3

    def test_noop_when_postprocessor_has_absolute_step(self):
        try:
            from sobits_vla_common.lerobot_adapter import AbsoluteActionsProcessorStep
        except ImportError:
            import pytest
            pytest.skip('sobits_vla_common.lerobot_adapter not importable in this env')

        class _FakePostprocessor:
            steps = [AbsoluteActionsProcessorStep.__new__(AbsoluteActionsProcessorStep)]

        engine = _make_engine(
            joint_features=['j0'], mobile_base_features=[],
            postprocessor=_FakePostprocessor(),
        )
        steps = [{'j0': 0.1}]
        engine._apply_manual_delta(steps, state_vector={'j0': 1.0})
        assert steps[0]['j0'] == 0.1
