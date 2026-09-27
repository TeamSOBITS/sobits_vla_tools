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


"""
Unit tests for absolute EE action synthesis (chunk B).

synthesize_ee_action is tested against a stub tf_tree (no OfflineTFTree
dependency); _build_features and ee_actions validation are tested against
RobotDescriptor fixtures built in-line rather than real robot yaml.
"""

import importlib.util
import math
from pathlib import Path
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.sync.poses import (  # noqa: E402
    synthesize_ee_action, synthesize_ee_action_quat,
)

# conversion_node imports lerobot/rosbags at module scope (runtime_deps.ensure);
# only importable inside the pixi env, same gate as test_conversion_golden.py.
_DEPS_AVAILABLE = (
    importlib.util.find_spec('rclpy') is not None
    and importlib.util.find_spec('lerobot') is not None
)
skip_no_rclpy = pytest.mark.skipif(
    not _DEPS_AVAILABLE, reason='rclpy/lerobot not importable outside the pixi env.'
)


def _quat_mul(q1, q2):
    """Hamilton product q1 (x)(x) q2, both (x, y, z, w) -- matches rpy_to_quat's convention."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class _StubTFTree:
    """resolve() returns a translation-only 4x4 matrix keyed by (target, source, stamp_ns)."""

    def __init__(self, lookups: dict):
        self._lookups = lookups

    def resolve(self, target, source, stamp_ns):
        val = self._lookups.get((target, source, stamp_ns))
        if val is None:
            return None
        x, y, z, roll, pitch, yaw = val
        from scipy.spatial.transform import Rotation
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        mat[0, 3], mat[1, 3], mat[2, 3] = x, y, z
        return mat


FPS = 10
STEP_NS = int(round(1e9 / FPS))


class TestSynthesizeEEAction:

    def test_shift_forward_action_is_future_state(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        })
        result = synthesize_ee_action(tree, 'ee', 'base', 0, FPS, None)
        assert result is not None
        state, action = result
        np.testing.assert_allclose(state[:3], [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(action[:3], [0.1, 0.0, 0.0], atol=1e-6)

    def test_unresolvable_future_falls_back_to_state(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.2, 0.3, 0.0, 0.0, 0.0, 0.0),
        })
        result = synthesize_ee_action(tree, 'ee', 'base', 0, FPS, None)
        assert result is not None
        state, action = result
        np.testing.assert_allclose(action, state, atol=1e-6)

    def test_state_lookup_failure_returns_none(self):
        tree = _StubTFTree({})
        result = synthesize_ee_action(tree, 'ee', 'base', 0, FPS, None)
        assert result is None

    def test_unwrap_continuity_across_pi_crossing(self):
        # prev yaw close to +pi; new raw sample reads back near -pi (same
        # physical angle wrapped by the rotation lib) -- must unwrap forward.
        near_pi = np.pi - 0.05
        prev_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, near_pi], dtype=np.float32)
        wrapped_yaw = -np.pi + 0.05
        tree = _StubTFTree({
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, 0.0, 0.0, wrapped_yaw),
        })
        result = synthesize_ee_action(tree, 'ee', 'base', STEP_NS, FPS, prev_state)
        assert result is not None
        state, _ = result
        assert abs(state[5] - (near_pi + 0.1)) < 1e-3

    def test_action_unwrapped_against_state_not_prev(self):
        # state yaw near -pi, action's raw sample near +pi (one small step
        # forward that wrapped) -- action must unwrap against state, not prev.
        prev_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        state_yaw = -np.pi + 0.02
        action_yaw_wrapped = np.pi - 0.03
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, state_yaw),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, 0.0, 0.0, action_yaw_wrapped),
        })
        result = synthesize_ee_action(tree, 'ee', 'base', 0, FPS, prev_state)
        assert result is not None
        state, action = result
        assert abs(state[5] - state_yaw) < 1e-3
        assert abs(action[5] - (state_yaw - 0.05)) < 1e-3


class TestSynthesizeEEActionQuat:
    """Quaternion analogue of TestSynthesizeEEAction: 7D [x,y,z,qx,qy,qz,qw]."""

    def _quat_lookup(self, roll, pitch, yaw):
        from scipy.spatial.transform import Rotation
        return tuple(Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat())

    def test_shape_and_unit_norm(self):
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.0)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx, qy, qz, qw),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, qx, qy, qz, qw),
        })
        result = synthesize_ee_action_quat(tree, 'ee', 'base', 0, FPS, None)
        assert result is not None
        state, action = result
        assert state.shape == (7,)
        assert action.shape == (7,)
        assert abs(np.linalg.norm(state[3:7]) - 1.0) < 1e-5
        assert abs(np.linalg.norm(action[3:7]) - 1.0) < 1e-5
        np.testing.assert_allclose(action[:3], [0.1, 0.0, 0.0], atol=1e-6)

    def test_state_lookup_failure_returns_none(self):
        tree = _StubTFTreeQuat({})
        assert synthesize_ee_action_quat(tree, 'ee', 'base', 0, FPS, None) is None

    def test_unresolvable_future_falls_back_to_state(self):
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.3)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.2, 0.3, 0.0, qx, qy, qz, qw),
        })
        result = synthesize_ee_action_quat(tree, 'ee', 'base', 0, FPS, None)
        assert result is not None
        state, action = result
        np.testing.assert_allclose(action, state, atol=1e-6)

    def test_shortest_arc_continuity_across_sign_flip(self):
        # Same physical orientation, but the raw quaternion sample flips sign
        # (double cover) relative to prev_state -- must realign, not jump.
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.5)
        prev_state = np.array([0.0, 0.0, 0.0, qx, qy, qz, qw], dtype=np.float32)
        flipped = (-qx, -qy, -qz, -qw)
        tree = _StubTFTreeQuat({
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0) + flipped,
        })
        result = synthesize_ee_action_quat(tree, 'ee', 'base', STEP_NS, FPS, prev_state)
        assert result is not None
        state, _ = result
        # Realigned quat should match prev_state's sign convention, not the flipped raw sample.
        dot = np.dot(state[3:7], prev_state[3:7])
        assert dot > 0.0
        np.testing.assert_allclose(state[3:7], [qx, qy, qz, qw], atol=1e-5)

    def test_action_realigned_against_state_not_prev(self):
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.4)
        flipped = (-qx, -qy, -qz, -qw)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx, qy, qz, qw),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0) + flipped,
        })
        result = synthesize_ee_action_quat(tree, 'ee', 'base', 0, FPS, None)
        assert result is not None
        state, action = result
        dot = np.dot(action[3:7], state[3:7])
        assert dot > 0.0


class _StubTFTreeQuat:
    """resolve() returns a matrix built directly from a 7D (x,y,z,qx,qy,qz,qw) tuple."""

    def __init__(self, lookups: dict):
        self._lookups = lookups

    def resolve(self, target, source, stamp_ns):
        val = self._lookups.get((target, source, stamp_ns))
        if val is None:
            return None
        x, y, z, qx, qy, qz, qw = val
        from scipy.spatial.transform import Rotation
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        mat[0, 3], mat[1, 3], mat[2, 3] = x, y, z
        return mat


class TestAppendEEChannelsRelative:
    """_append_ee_channels is a plain method; drive it via a stub self (R1: no rclpy)."""

    def _synthesizer(self, use_relative_actions, ee_rotation='rpy', ee_frame='base'):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            use_relative_actions=use_relative_actions,
            ee_rotation=ee_rotation,
            ee_frame=ee_frame,
            log_warn=lambda msg: None,
            _append_ee_channels=FrameSynthesizer._append_ee_channels,
            _relative_ee_pose6=staticmethod(FrameSynthesizer._relative_ee_pose6),
            _relative_ee_pose7=staticmethod(FrameSynthesizer._relative_ee_pose7),
        )

    def _run(self, synth, tree, t_sec, prev):
        state, action = [], []
        prev_ee_action_poses = {'left': prev}
        ok = synth._append_ee_channels(
            synth, state, action, tree, t_sec, prev_ee_action_poses,
            {'tf': 0},
        )
        return ok, np.array(state, dtype=np.float32), np.array(action, dtype=np.float32)

    def test_relative_mode_appends_delta_state_stays_absolute(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, 0.2, 0.0, 0.0, 0.0, 0.0),
        })
        synth = self._synthesizer(use_relative_actions=True)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(state[:3], [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(action[:3], [0.1, 0.2, 0.0], atol=1e-6)

    def test_integration_invariant_state_plus_action_equals_next_state(self):
        # state(t) + action(t) == state(t+1) is exactly the delta roundtrip.
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, -0.05, 0.02, 0.0, 0.0, 0.0),
        })
        synth = self._synthesizer(use_relative_actions=True)
        ok, state_t, action_t = self._run(synth, tree, 0.0, None)
        assert ok
        _, state_t1, _ = self._run(synth, tree, 1.0 / FPS, state_t)
        np.testing.assert_allclose(state_t + action_t, state_t1, atol=1e-5)

    def test_pi_crossing_delta_stays_small(self):
        # state near +pi, next raw sample near -pi (same physical motion,
        # wrapped) -- unwrap keeps the delta small, never near 2*pi.
        near_pi = np.pi - 0.05
        wrapped_next = -np.pi + 0.05
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, near_pi),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, 0.0, 0.0, wrapped_next),
        })
        synth = self._synthesizer(use_relative_actions=True)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        assert abs(action[5] - 0.1) < 1e-3
        assert abs(action[5]) < np.pi

    def test_absolute_mode_unchanged(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        })
        synth = self._synthesizer(use_relative_actions=False)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(state[:3], [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(action[:3], [0.1, 0.0, 0.0], atol=1e-6)


class TestAppendEEChannelsRelativeQuat:
    """Quat-mode _append_ee_channels: state absolute 7D, action = [dp, q_rel]."""

    def _synthesizer(self, use_relative_actions, ee_frame='base'):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            use_relative_actions=use_relative_actions,
            ee_rotation='quat',
            ee_frame=ee_frame,
            log_warn=lambda msg: None,
            _append_ee_channels=FrameSynthesizer._append_ee_channels,
            _relative_ee_pose7=staticmethod(FrameSynthesizer._relative_ee_pose7),
        )

    def _run(self, synth, tree, t_sec, prev):
        state, action = [], []
        prev_ee_action_poses = {'left': prev}
        ok = synth._append_ee_channels(
            synth, state, action, tree, t_sec, prev_ee_action_poses,
            {'tf': 0},
        )
        return ok, np.array(state, dtype=np.float32), np.array(action, dtype=np.float32)

    def _quat(self, roll, pitch, yaw):
        from scipy.spatial.transform import Rotation
        return tuple(Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat())

    def test_relative_mode_state_absolute_action_is_delta_pose_and_rel_quat(self):
        qx0, qy0, qz0, qw0 = self._quat(0.0, 0.0, 0.0)
        qx1, qy1, qz1, qw1 = self._quat(0.0, 0.0, 0.2)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx0, qy0, qz0, qw0),
            ('base', 'ee', STEP_NS): (0.1, 0.2, 0.0, qx1, qy1, qz1, qw1),
        })
        synth = self._synthesizer(use_relative_actions=True)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        assert state.shape == (7,)
        assert action.shape == (7,)
        np.testing.assert_allclose(state[:3], [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(state[3:7], [qx0, qy0, qz0, qw0], atol=1e-6)
        np.testing.assert_allclose(action[:3], [0.1, 0.2, 0.0], atol=1e-6)
        assert abs(np.linalg.norm(action[3:7]) - 1.0) < 1e-5

    def test_composing_state_and_rel_quat_gives_next_state_quat(self):
        from sobits_vla_common.geometry import quat_shortest_arc
        qx0, qy0, qz0, qw0 = self._quat(0.0, 0.0, 0.0)
        qx1, qy1, qz1, qw1 = self._quat(0.1, -0.2, 0.3)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx0, qy0, qz0, qw0),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, qx1, qy1, qz1, qw1),
        })
        synth = self._synthesizer(use_relative_actions=True)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok

        def quat_mul(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return (
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            )

        composed = quat_mul(tuple(state[3:7]), tuple(action[3:7]))
        composed = quat_shortest_arc(composed, (qx1, qy1, qz1, qw1))
        np.testing.assert_allclose(composed, [qx1, qy1, qz1, qw1], atol=1e-5)

    def test_absolute_mode_appends_pose7_as_is(self):
        qx, qy, qz, qw = self._quat(0.0, 0.0, 0.1)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx, qy, qz, qw),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, qx, qy, qz, qw),
        })
        synth = self._synthesizer(use_relative_actions=False)
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(action[:3], [0.1, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(action[3:7], [qx, qy, qz, qw], atol=1e-6)


class TestAppendEEChannelsBodyFrame:
    """ee_frame='body': deltas expressed in the EE's own axes at t (state pose)."""

    def _synth_rpy(self, ee_frame='body'):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            use_relative_actions=True,
            ee_rotation='rpy',
            ee_frame=ee_frame,
            log_warn=lambda msg: None,
            _append_ee_channels=FrameSynthesizer._append_ee_channels,
            _relative_ee_pose6=staticmethod(FrameSynthesizer._relative_ee_pose6),
            _relative_ee_pose7=staticmethod(FrameSynthesizer._relative_ee_pose7),
        )

    def _synth_quat(self, ee_frame='body'):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            use_relative_actions=True,
            ee_rotation='quat',
            ee_frame=ee_frame,
            log_warn=lambda msg: None,
            _append_ee_channels=FrameSynthesizer._append_ee_channels,
            _relative_ee_pose6=staticmethod(FrameSynthesizer._relative_ee_pose6),
            _relative_ee_pose7=staticmethod(FrameSynthesizer._relative_ee_pose7),
        )

    def _run(self, synth, tree, t_sec, prev):
        state, action = [], []
        prev_ee_action_poses = {'left': prev}
        ok = synth._append_ee_channels(
            synth, state, action, tree, t_sec, prev_ee_action_poses,
            {'tf': 0},
        )
        return ok, np.array(state, dtype=np.float32), np.array(action, dtype=np.float32)

    def _quat(self, roll, pitch, yaw):
        from scipy.spatial.transform import Rotation
        return tuple(Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat())

    def test_rpy_body_translation_matches_known_value(self):
        # state yaw=+90deg, base motion [0, 0.03, 0] -> body delta [0.03, 0, 0]
        # (R(t)^T rotates the +y base motion onto the EE's own +x axis).
        yaw = np.pi / 2.0
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, yaw),
            ('base', 'ee', STEP_NS): (0.0, 0.03, 0.0, 0.0, 0.0, yaw),
        })
        synth = self._synth_rpy()
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(action[:3], [0.03, 0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(action[3:6], [0.0, 0.0, 0.0], atol=1e-5)

    def test_rpy_body_reconstruction_invariant_translation(self):
        # p(t+1) == p(t) + R(t) . delta_p
        yaw = np.pi / 2.0
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.1, 0.2, 0.05, 0.0, 0.0, yaw),
            ('base', 'ee', STEP_NS): (0.1, 0.23, 0.05, 0.0, 0.0, yaw),
        })
        synth = self._synth_rpy()
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        from sobits_vla_common.geometry import quat_rotate_vec, rpy_to_quat
        state_q = rpy_to_quat(*state[3:6])
        p_next = np.array(state[:3]) + np.array(quat_rotate_vec(state_q, action[:3]))
        np.testing.assert_allclose(p_next, [0.1, 0.23, 0.05], atol=1e-5)

    def test_rpy_body_reconstruction_invariant_rotation(self):
        # q(t) (x) rpy_to_quat(delta_rpy) == q(t+1) up to sign.
        from sobits_vla_common.geometry import quat_shortest_arc, rpy_to_quat
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.1, 0.2, 0.3),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, 0.15, 0.1, 0.5),
        })
        synth = self._synth_rpy()
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        q_state = rpy_to_quat(*state[3:6])
        q_delta = rpy_to_quat(*action[3:6])
        q_next_expected = rpy_to_quat(*(0.15, 0.1, 0.5))
        composed = _quat_mul(q_state, q_delta)
        composed = quat_shortest_arc(composed, q_next_expected)
        for a, b in zip(composed, q_next_expected):
            assert math.isclose(a, b, abs_tol=1e-5)

    def test_rpy_body_rotation_differs_from_per_axis_subtraction(self):
        # Proper relative rotation (quat route) != naive per-axis rpy subtraction
        # once more than one axis is involved -- guards against regressing to it.
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.2, 0.3, 0.4),
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0, 0.5, -0.1, 0.9),
        })
        synth = self._synth_rpy()
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        naive = np.array((0.5, -0.1, 0.9)) - np.array((0.2, 0.3, 0.4))
        assert not np.allclose(action[3:6], naive, atol=1e-3)

    def test_quat_body_translation_matches_known_value(self):
        qx, qy, qz, qw = self._quat(0.0, 0.0, np.pi / 2.0)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx, qy, qz, qw),
            ('base', 'ee', STEP_NS): (0.0, 0.03, 0.0, qx, qy, qz, qw),
        })
        synth = self._synth_quat()
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(action[:3], [0.03, 0.0, 0.0], atol=1e-5)

    def test_quat_body_rotation_unchanged_from_base(self):
        # Rotation channel is body-frame already (quat_relative); base vs body
        # must agree bit-for-bit on the quaternion delta.
        qx0, qy0, qz0, qw0 = self._quat(0.0, 0.0, 0.0)
        qx1, qy1, qz1, qw1 = self._quat(0.1, -0.2, 0.3)
        lookups = {
            ('base', 'ee', 0): (0.0, 0.0, 0.0, qx0, qy0, qz0, qw0),
            ('base', 'ee', STEP_NS): (0.05, 0.0, 0.0, qx1, qy1, qz1, qw1),
        }
        tree_base = _StubTFTreeQuat(lookups)
        tree_body = _StubTFTreeQuat(lookups)
        ok_b, _, action_base = self._run(self._synth_quat('base'), tree_base, 0.0, None)
        ok_body, _, action_body = self._run(self._synth_quat('body'), tree_body, 0.0, None)
        assert ok_b and ok_body
        np.testing.assert_allclose(action_base[3:7], action_body[3:7], atol=1e-9)

    def test_base_frame_unchanged_regression(self):
        # ee_frame='base' must byte-match the pre-existing behaviour.
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2.0),
            ('base', 'ee', STEP_NS): (0.0, 0.03, 0.0, 0.0, 0.0, np.pi / 2.0),
        })
        synth = self._synth_rpy('base')
        ok, state, action = self._run(synth, tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(action[:3], [0.0, 0.03, 0.0], atol=1e-6)


class TestEEActionFeatures:

    def test_feature_names_in_order(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        assert ee_action_features('left') == [
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.roll', 'ee.left.pitch', 'ee.left.yaw',
        ]

    def test_quat_feature_names_in_order(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        assert ee_action_features('left', rotation='quat') == [
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.qx', 'ee.left.qy', 'ee.left.qz', 'ee.left.qw',
        ]

    def test_invalid_rotation_raises(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        with pytest.raises(ValueError):
            ee_action_features('left', rotation='bogus')


@skip_no_rclpy
class TestBuildFeaturesWithEE:
    """_build_features is a bound method; drive it via a stub `self` (R1: no rclpy.init needed)."""

    def _stub_node(self, **overrides):
        base = {
            'action_features': ['head_pan_joint', 'head_tilt_joint'],
            'ee_action_specs': [('left', 'ee_l', 'base')],
            'ee_rotation': 'rpy',
            'has_mobile_base': True, 'has_cmd_vel_y': False, 'has_cmd_vel_z': False,
            'ee_pose_enabled': False, 'ee_configs': [], 'has_subtasks': False,
            'skip_cameras': True, 'camera_topics': {}, 'camera_shapes': {},
            'depth_camera_topics': {}, 'depth_camera_shapes': {},
        }
        base.update(overrides)
        return types.SimpleNamespace(**base)

    def test_names_ordered_joints_then_ee_then_base(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._stub_node()
        features = RosbagConversionNode._build_features(node, all_tasks=[])
        assert features['action']['names'] == [
            'head_pan_joint', 'head_tilt_joint',
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.roll', 'ee.left.pitch', 'ee.left.yaw',
            'base_x', 'base_theta',
        ]
        assert features['observation.state']['names'] == features['action']['names']

    def test_dim_matches_joint_plus_ee_plus_base(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._stub_node()
        features = RosbagConversionNode._build_features(node, all_tasks=[])
        assert features['action']['shape'] == (10,)
        assert features['observation.state']['shape'] == (10,)

    def test_ee_disabled_matches_pre_ee_layout(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._stub_node(ee_action_specs=[])
        features = RosbagConversionNode._build_features(node, all_tasks=[])
        assert features['action']['names'] == [
            'head_pan_joint', 'head_tilt_joint', 'base_x', 'base_theta',
        ]
        assert features['action']['shape'] == (4,)

    def test_quat_rotation_gives_7_names_per_arm(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._stub_node(ee_rotation='quat')
        features = RosbagConversionNode._build_features(node, all_tasks=[])
        assert features['action']['names'] == [
            'head_pan_joint', 'head_tilt_joint',
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.qx', 'ee.left.qy', 'ee.left.qz', 'ee.left.qw',
            'base_x', 'base_theta',
        ]
        assert features['action']['shape'] == (11,)
        assert features['observation.state']['shape'] == (11,)


class TestEEControlValidation:

    def _descriptor(self):
        from sobits_vla_common.robot_descriptor import (
            EEControlSpec, EEPoseSpec, GroupSpec, JointSpec, RobotDescriptor,
        )
        arm = GroupSpec(
            name='arm_left', command_topic='/cmd', command_action=None,
            state_topic='', max_joint_delta=0.0, active=True,
            joints=[JointSpec(ros_name='j1', feature='arm_left_j1')],
        )
        return RobotDescriptor(
            robot_id='test_robot', joint_states_topic='/joint_states',
            groups=[arm],
            ee_poses=[EEPoseSpec(name='left', source_frame='ee_l', target_frame='base')],
            ee_control=[EEControlSpec(
                ee_pose='left', group='arm_left',
                target_frame='left_target', enable_topic='arm_left/enabled',
            )],
        )

    def test_ee_control_for_resolves_matching_arm(self):
        desc = self._descriptor()
        specs = desc.ee_control_for(['left'])
        assert len(specs) == 1
        assert specs[0].group == 'arm_left'

    def test_ee_control_for_unknown_arm_raises(self):
        desc = self._descriptor()
        with pytest.raises(ValueError):
            desc.ee_control_for(['right'])

    def test_filtered_drops_ee_control_when_ee_pose_excluded(self):
        desc = self._descriptor()
        filtered = desc.filtered(exclude_ee=['left'])
        with pytest.raises(ValueError):
            filtered.ee_control_for(['left'])


@skip_no_rclpy
class TestResolveEEActionsValidation:
    """_resolve_ee_actions is a plain method; drive it via stub self/params (no ROS needed)."""

    def _descriptor(self, two_arms=False, exclude_groups=()):
        """
        Build a descriptor and apply .filtered(exclude_groups=...) up front.

        Mirrors production: conversion_node.__init__ always calls
        desc.filtered(...) before passing desc to _resolve_ee_actions, so
        derived_ee_action_arms() (which reads desc.active_groups) sees the
        post-exclude state, not the raw descriptor.
        """
        from sobits_vla_common.robot_descriptor import (
            EEControlSpec, EEPoseSpec, GroupSpec, JointSpec, RobotDescriptor,
        )
        arm_left = GroupSpec(
            name='arm_left', command_topic='/cmd', command_action=None,
            state_topic='', max_joint_delta=0.0, active=True,
            joints=[JointSpec(ros_name='j1', feature='arm_left_j1')],
        )
        groups = [arm_left]
        ee_poses = [EEPoseSpec(name='left', source_frame='ee_l', target_frame='base')]
        ee_control = [EEControlSpec(
            ee_pose='left', group='arm_left',
            target_frame='left_target', enable_topic='arm_left/enabled',
        )]
        if two_arms:
            arm_right = GroupSpec(
                name='arm_right', command_topic='/cmd_r', command_action=None,
                state_topic='', max_joint_delta=0.0, active=True,
                joints=[JointSpec(ros_name='j2', feature='arm_right_j1')],
            )
            groups.append(arm_right)
            ee_poses.append(
                EEPoseSpec(name='right', source_frame='ee_r', target_frame='base')
            )
            ee_control.append(EEControlSpec(
                ee_pose='right', group='arm_right',
                target_frame='right_target', enable_topic='arm_right/enabled',
            ))
        desc = RobotDescriptor(
            robot_id='test_robot', joint_states_topic='/joint_states',
            groups=groups, ee_poses=ee_poses, ee_control=ee_control,
        )
        if exclude_groups:
            desc = desc.filtered(exclude_groups=list(exclude_groups))
        return desc

    def _node(self, use_relative_actions=False, skip_static_threshold=0.0):
        return types.SimpleNamespace(
            use_relative_actions=use_relative_actions,
            skip_static_threshold=skip_static_threshold,
            get_logger=lambda: types.SimpleNamespace(
                warning=lambda msg: None, info=lambda msg: None,
            ),
        )

    def _params(self, arms=(), exclude_groups=(), rotation='rpy', frame='base'):
        return types.SimpleNamespace(
            ee_actions=types.SimpleNamespace(
                arms=list(arms), rotation=rotation, frame=frame,
            ),
            exclude=types.SimpleNamespace(groups=list(exclude_groups)),
        )

    def test_explicit_arm_raises_when_superseded_group_not_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(arms=['left'], exclude_groups=[])  # arm_left NOT excluded
        with pytest.raises(ValueError, match='derivation rule'):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_explicit_arm_passes_when_superseded_group_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(arms=['left'], exclude_groups=['arm_left'])
        desc = self._descriptor(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert specs == [('left', 'ee_l', 'base')]

    def test_derives_when_group_excluded_and_arms_unset(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'])
        desc = self._descriptor(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert specs == [('left', 'ee_l', 'base')]

    def test_derives_both_arms_when_all_groups_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left', 'arm_right'])
        desc = self._descriptor(two_arms=True, exclude_groups=['arm_left', 'arm_right'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert sorted(s[0] for s in specs) == ['left', 'right']

    def test_derives_only_arm_whose_group_is_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'])
        desc = self._descriptor(two_arms=True, exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert [s[0] for s in specs] == ['left']

    def test_relative_actions_allowed(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node(use_relative_actions=True)
        params = self._params(exclude_groups=['arm_left'])
        desc = self._descriptor(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert specs == [('left', 'ee_l', 'base')]

    def test_no_active_ee_returns_empty_without_validating(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=[])  # arm_left still active -> nothing derives
        assert RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params) == []

    def test_quat_rotation_sets_ee_rotation_attr(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'], rotation='quat')
        desc = self._descriptor(exclude_groups=['arm_left'])
        RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert node.ee_rotation == 'quat'

    def test_invalid_rotation_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'], rotation='axis_angle')
        with pytest.raises(ValueError, match='rotation'):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_body_frame_with_relative_actions_allowed(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node(use_relative_actions=True)
        params = self._params(exclude_groups=['arm_left'], frame='body')
        desc = self._descriptor(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, desc, params)
        assert specs == [('left', 'ee_l', 'base')]
        assert node.ee_frame == 'body'

    def test_body_frame_without_relative_actions_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node(use_relative_actions=False)
        params = self._params(exclude_groups=['arm_left'], frame='body')
        with pytest.raises(ValueError, match='use_relative_actions'):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_invalid_frame_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'], frame='world')
        with pytest.raises(ValueError, match='frame'):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_base_frame_default_sets_ee_frame_attr(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'])
        RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)
        assert node.ee_frame == 'base'


@skip_no_rclpy
class TestDeprecatedParams:
    """_check_deprecated_params is a staticmethod; no ROS node needed."""

    def _params(self, ee_poses=(), ee_actions_enabled=''):
        return types.SimpleNamespace(
            exclude=types.SimpleNamespace(ee_poses=list(ee_poses)),
            ee_actions=types.SimpleNamespace(enabled=ee_actions_enabled),
        )

    def test_old_exclude_key_set_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        with pytest.raises(ValueError, match='exclude.ee_poses was renamed'):
            RosbagConversionNode._check_deprecated_params(self._params(ee_poses=['left']))

    def test_old_exclude_key_empty_is_allowed(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        RosbagConversionNode._check_deprecated_params(self._params(ee_poses=[]))

    def test_old_ee_actions_enabled_key_set_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        with pytest.raises(ValueError, match='ee_actions.enabled was removed'):
            RosbagConversionNode._check_deprecated_params(
                self._params(ee_actions_enabled='true')
            )

    def test_old_ee_actions_enabled_key_empty_is_allowed(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        RosbagConversionNode._check_deprecated_params(self._params(ee_actions_enabled=''))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
