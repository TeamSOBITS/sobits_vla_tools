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
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.action_interpolator import ActionInterpolator  # noqa: E402
from sobits_vla_deploy.deploy_node import ActionChunkBuffer  # noqa: E402
from sobits_vla_deploy.inference_engine import InferenceEngine  # noqa: E402
from sobits_vla_deploy.vla_episode_logger import EpisodeLogger  # noqa: E402


# --- ActionChunkBuffer tests ---

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


# --- _to_action_steps tests (the live copy, InferenceEngine) ---


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
        mobile_base_features=(
            mobile_base_features if mobile_base_features is not None else ['x.vel']
        ),
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


# --- _apply_manual_delta tests ---


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


# --- ActionChunkBuffer: push/pop order + replace() ---


class TestActionChunkBufferOrder:

    def test_pop_returns_fifo_order(self):
        buf = ActionChunkBuffer('weighted_average')
        chunk = [_make_step(float(i)) for i in range(4)]
        buf.merge(chunk, overlap=0)
        popped = [buf.pop()['j0'] for _ in range(4)]
        assert popped == [0.0, 1.0, 2.0, 3.0]

    def test_replace_discards_delay_steps(self):
        import torch
        buf = ActionChunkBuffer('weighted_average')
        original = torch.tensor([[0.0], [1.0], [2.0], [3.0]])
        processed = [_make_step(float(i)) for i in range(4)]
        buf.replace(original, processed, delay=2)
        assert buf.size() == 2
        assert buf.pop()['j0'] == 2.0

    def test_replace_empty_inputs_is_noop(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([_make_step(1.0)], overlap=0)
        buf.replace(None, [], delay=0)
        assert buf.size() == 1


# --- ActionInterpolator ---


class TestActionInterpolator:

    def test_multiplier_one_is_passthrough(self):
        interp = ActionInterpolator(1)
        assert not interp.enabled
        assert interp.needs_new_action()
        interp.add({'j0': 1.0})
        assert interp.get() == {'j0': 1.0}
        assert interp.needs_new_action()
        assert interp.get() is None

    def test_multiplier_greater_than_one_interpolates(self):
        interp = ActionInterpolator(4)
        assert interp.enabled
        interp.add({'j0': 0.0})  # first step: no prev, base-rate passthrough
        assert interp.get() == {'j0': 0.0}
        assert interp.needs_new_action()

        interp.add({'j0': 4.0})
        values = [interp.get()['j0'] for _ in range(4)]
        assert values == [1.0, 2.0, 3.0, 4.0]
        assert interp.needs_new_action()
        assert interp.get() is None

    def test_reset_clears_buffer_and_prev(self):
        interp = ActionInterpolator(3)
        interp.add({'j0': 0.0})
        interp.add({'j0': 3.0})
        interp.reset()
        assert interp.needs_new_action()
        interp.add({'j0': 9.0})
        # No prev after reset -> base-rate passthrough, not interpolated.
        assert interp.get() == {'j0': 9.0}

    def test_new_key_defaults_prev_to_its_own_value(self):
        interp = ActionInterpolator(2)
        interp.add({'j0': 0.0})
        interp.add({'j0': 0.0, 'j1': 10.0})
        first = interp.get()
        assert first['j1'] == 10.0  # prev.get('j1', v) == v -> no jump

    def test_invalid_multiplier_raises(self):
        with pytest.raises(ValueError):
            ActionInterpolator(0)


# --- EpisodeLogger.evaluate_termination ---


def _make_logger(tmp_path, **kwargs):
    """sim_enabled=False skips the gz poller thread and blocking begin_episode reads."""
    defaults = {
        'log_dir': str(tmp_path),
        'world_name': 'test_world',
        'robot_name': 'robot',
        'block_name': 'block',
        'spawn_z': 0.0,
        'block_z': 0.5,
        'tilt_threshold_deg': 30.0,
        'episode_timeout_s': 60.0,
        'lift_success_m': 0.05,
        'fall_z_drop_m': 0.15,
        'success_settle_s': 2.0,
        'sim_enabled': False,
    }
    defaults.update(kwargs)
    logger = EpisodeLogger(**defaults)
    logger.begin_episode()
    return logger


def _set_poses(logger, block=None, robot=None):
    with logger._pose_lock:
        if block is not None:
            logger._cached_block_pose = block
        if robot is not None:
            logger._cached_robot_pose = robot


def _pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}


class TestEvaluateTerminationPick:

    def test_lift_success_after_settle(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, block=_pose(z=0.5 + 0.10))
        outcome = logger.evaluate_termination(elapsed_sim_s=3.0)
        assert outcome == 'success_lift'

    def test_lift_ignored_during_settle(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, block=_pose(z=0.5 + 0.10))
        outcome = logger.evaluate_termination(elapsed_sim_s=0.5)
        assert outcome is None
        # Still tracked for the end-of-episode summary.
        assert logger._max_block_lift > 0.05

    def test_lift_below_threshold_no_success(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, block=_pose(z=0.5 + 0.01))
        outcome = logger.evaluate_termination(elapsed_sim_s=3.0)
        assert outcome is None

    def test_timeout(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, block=_pose(z=0.5), robot=_pose(z=0.0))
        outcome = logger.evaluate_termination(elapsed_sim_s=60.0)
        assert outcome == 'timeout'

    def test_fallen_by_tilt(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, robot=_pose(z=0.0, roll=0.7))  # > 30 deg (0.524 rad)
        outcome = logger.evaluate_termination(elapsed_sim_s=1.0)
        assert outcome == 'fallen'

    def test_fallen_by_z_drop(self, tmp_path):
        logger = _make_logger(tmp_path)
        _set_poses(logger, robot=_pose(z=-0.20))  # drop 0.20 > fall_z_drop_m 0.15
        outcome = logger.evaluate_termination(elapsed_sim_s=1.0)
        assert outcome == 'fallen'

    def test_no_cached_poses_no_crash(self, tmp_path):
        logger = _make_logger(tmp_path)
        outcome = logger.evaluate_termination(elapsed_sim_s=1.0)
        assert outcome is None

    def test_inactive_logger_returns_none(self, tmp_path):
        logger = _make_logger(tmp_path)
        logger.end_episode('manual_stop')
        _set_poses(logger, block=_pose(z=10.0))
        assert logger.evaluate_termination(elapsed_sim_s=3.0) is None


class TestEvaluateTerminationPlace:

    def _place_logger(self, tmp_path, monkeypatch, goal_xy=(1.0, 1.0), **kwargs):
        kwargs.setdefault('place_z_max_m', 0.0)
        logger = _make_logger(
            tmp_path,
            goal_name='goal_bin',
            place_radius_m=0.12,
            place_settle_s=1.0,
            **kwargs,
        )
        monkeypatch.setattr(
            'sobits_vla_deploy.vla_episode_logger.gz_get_pose',
            lambda world, name, timeout=None: {
                'x': goal_xy[0], 'y': goal_xy[1], 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            },
        )
        return logger

    def test_goal_radius_and_z_condition_then_settle(self, tmp_path, monkeypatch):
        logger = self._place_logger(tmp_path, monkeypatch)
        # First tick: lift crosses threshold and settled -> sets _lifted,
        # but block isn't at the goal yet, so no success returned.
        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.5 + 0.10))
        assert logger.evaluate_termination(elapsed_sim_s=3.0) is None
        assert logger._lifted is True

        # Now at the goal, radius+z satisfied, but settle timer hasn't elapsed.
        _set_poses(logger, block=_pose(x=1.0, y=1.0, z=0.0))
        assert logger.evaluate_termination(elapsed_sim_s=3.5) is None

        # Settle timer elapsed (>= place_settle_s after goal_reached_at).
        outcome = logger.evaluate_termination(elapsed_sim_s=4.5)
        assert outcome == 'success_place'

    def test_leaving_goal_resets_settle_timer(self, tmp_path, monkeypatch):
        logger = self._place_logger(tmp_path, monkeypatch)
        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.5 + 0.10))
        logger.evaluate_termination(elapsed_sim_s=3.0)

        _set_poses(logger, block=_pose(x=1.0, y=1.0, z=0.0))
        logger.evaluate_termination(elapsed_sim_s=3.5)  # goal_reached_at = 3.5

        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.0))  # leaves goal
        logger.evaluate_termination(elapsed_sim_s=4.0)

        _set_poses(logger, block=_pose(x=1.0, y=1.0, z=0.0))  # back at goal
        assert logger.evaluate_termination(elapsed_sim_s=4.6) is None  # only 0.6s since re-entry

    def test_z_max_condition_blocks_success(self, tmp_path, monkeypatch):
        logger = self._place_logger(tmp_path, monkeypatch, place_z_max_m=0.05)
        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.5 + 0.10))
        logger.evaluate_termination(elapsed_sim_s=3.0)

        _set_poses(logger, block=_pose(x=1.0, y=1.0, z=0.20))  # in radius, too high
        outcome = logger.evaluate_termination(elapsed_sim_s=5.0)
        assert outcome is None

    def test_drop_abort_fires_after_dwell(self, tmp_path, monkeypatch):
        logger = self._place_logger(
            tmp_path, monkeypatch, drop_abort_s=1.0, drop_abort_z_max_m=0.05,
        )
        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.5 + 0.10))
        logger.evaluate_termination(elapsed_sim_s=3.0)  # sets _lifted

        # Object at rest, low, far from goal -> dwell clock starts.
        _set_poses(logger, block=_pose(x=9.0, y=9.0, z=0.0))
        assert logger.evaluate_termination(elapsed_sim_s=3.5) is None
        assert logger.evaluate_termination(elapsed_sim_s=4.0) is None
        outcome = logger.evaluate_termination(elapsed_sim_s=4.6)  # >= 1.0s dwell
        assert outcome == 'dropped'

    def test_drop_abort_disabled_by_default(self, tmp_path, monkeypatch):
        logger = self._place_logger(tmp_path, monkeypatch)  # drop_abort_s=0.0
        _set_poses(logger, block=_pose(x=5.0, y=5.0, z=0.5 + 0.10))
        logger.evaluate_termination(elapsed_sim_s=3.0)

        _set_poses(logger, block=_pose(x=9.0, y=9.0, z=0.0))
        outcome = logger.evaluate_termination(elapsed_sim_s=50.0)  # under the 60s timeout
        assert outcome is None
