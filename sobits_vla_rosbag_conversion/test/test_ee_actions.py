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
# refactor-exempt: file over 600 lines, test module

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.sync.poses import (  # noqa: E402
    synthesize_ee_action, synthesize_ee_action_quat, synthesize_ee_action_rotvec,
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

    def test_unresolvable_future_skips_frame(self):
        # A held pose would label the frame as zero motion; refuse instead.
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.2, 0.3, 0.0, 0.0, 0.0, 0.0),
        })
        assert synthesize_ee_action(tree, 'ee', 'base', 0, FPS, None) is None

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
            ('base', 'ee', 2 * STEP_NS): (0.0, 0.0, 0.0, 0.0, 0.0, wrapped_yaw),
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

    def test_unresolvable_future_skips_frame(self):
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.3)
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.2, 0.3, 0.0, qx, qy, qz, qw),
        })
        assert synthesize_ee_action_quat(tree, 'ee', 'base', 0, FPS, None) is None

    def test_shortest_arc_continuity_across_sign_flip(self):
        # Same physical orientation, but the raw quaternion sample flips sign
        # (double cover) relative to prev_state -- must realign, not jump.
        qx, qy, qz, qw = self._quat_lookup(0.0, 0.0, 0.5)
        prev_state = np.array([0.0, 0.0, 0.0, qx, qy, qz, qw], dtype=np.float32)
        flipped = (-qx, -qy, -qz, -qw)
        tree = _StubTFTreeQuat({
            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0) + flipped,
            ('base', 'ee', 2 * STEP_NS): (0.0, 0.0, 0.0) + flipped,
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


class TestSynthesizeEEActionRotvec:
    """Rotvec analogue: 6D [x,y,z,rx,ry,rz], no unwrap state."""

    def test_shift_forward_action_is_future_state(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, 0.0, 0.0, 0.3),
        })
        result = synthesize_ee_action_rotvec(tree, 'ee', 'base', 0, FPS)
        assert result is not None
        state, action = result
        assert state.shape == (6,)
        assert action.shape == (6,)
        np.testing.assert_allclose(state, np.zeros(6), atol=1e-6)
        np.testing.assert_allclose(action, [0.1, 0.0, 0.0, 0.0, 0.0, 0.3], atol=1e-6)

    def test_unresolvable_future_skips_frame(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.2, 0.3, 0.0, 0.1, -0.2, 0.3),
        })
        assert synthesize_ee_action_rotvec(tree, 'ee', 'base', 0, FPS) is None

    def test_state_lookup_failure_returns_none(self):
        assert synthesize_ee_action_rotvec(_StubTFTree({}), 'ee', 'base', 0, FPS) is None

    def test_matches_scipy_rotvec(self):
        from scipy.spatial.transform import Rotation
        rpy = (0.4, -0.3, 2.9)
        tree = _StubTFTree({('base', 'ee', 0): (0.0, 0.0, 0.0) + rpy,
                            ('base', 'ee', STEP_NS): (0.0, 0.0, 0.0) + rpy})
        state, _ = synthesize_ee_action_rotvec(tree, 'ee', 'base', 0, FPS)
        expected = Rotation.from_euler('xyz', rpy).as_rotvec()
        np.testing.assert_allclose(state[3:6], expected, atol=1e-5)
        assert np.linalg.norm(state[3:6]) <= np.pi + 1e-6


class TestAppendEEChannels:
    """_append_ee_channels is a plain method; drive it via a stub self (R1: no rclpy)."""

    def _synthesizer(self, ee_rotation):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            ee_rotation=ee_rotation,
            log_warn=lambda msg: None,
            _append_ee_channels=FrameSynthesizer._append_ee_channels,
        )

    def _run(self, synth, tree, t_sec, prev):
        state, action = [], []
        prev_ee_action_poses = {'left': prev}
        ok = synth._append_ee_channels(
            synth, state, action, tree, t_sec, prev_ee_action_poses,
            {'tf': 0},
        )
        return ok, np.array(state, dtype=np.float32), np.array(action, dtype=np.float32)

    def test_rotvec_appends_absolute_state_and_future_pose(self):
        from scipy.spatial.transform import Rotation
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2.0),
            ('base', 'ee', STEP_NS): (0.1, 0.2, 0.0, 0.0, 0.0, np.pi / 2.0),
        })
        ok, state, action = self._run(self._synthesizer('rotvec'), tree, 0.0, None)
        assert ok
        rotvec = Rotation.from_euler('xyz', [0.0, 0.0, np.pi / 2.0]).as_rotvec()
        np.testing.assert_allclose(state, [0.0, 0.0, 0.0, *rotvec], atol=1e-5)
        np.testing.assert_allclose(action, [0.1, 0.2, 0.0, *rotvec], atol=1e-5)

    def test_rpy_appends_absolute_pose(self):
        tree = _StubTFTree({
            ('base', 'ee', 0): (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0, 0.0, 0.0, 0.2),
        })
        ok, state, action = self._run(self._synthesizer('rpy'), tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(state, np.zeros(6), atol=1e-6)
        np.testing.assert_allclose(action, [0.1, 0.0, 0.0, 0.0, 0.0, 0.2], atol=1e-6)

    def test_quat_appends_absolute_pose7(self):
        from scipy.spatial.transform import Rotation
        q = tuple(Rotation.from_euler('xyz', [0.0, 0.0, 0.1]).as_quat())
        tree = _StubTFTreeQuat({
            ('base', 'ee', 0): (0.0, 0.0, 0.0) + q,
            ('base', 'ee', STEP_NS): (0.1, 0.0, 0.0) + q,
        })
        ok, state, action = self._run(self._synthesizer('quat'), tree, 0.0, None)
        assert ok
        np.testing.assert_allclose(action[:3], [0.1, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(action[3:7], q, atol=1e-6)

    def test_lookup_failure_skips_frame(self):
        ok, _, _ = self._run(self._synthesizer('rotvec'), _StubTFTree({}), 0.0, None)
        assert not ok


def _tf_msg(t_s: float, parent: str, child: str, x: float, z: float = 0.0):
    sec = int(t_s)
    ns = types.SimpleNamespace
    return ns(transforms=[ns(
        header=ns(frame_id=parent, stamp=ns(sec=sec, nanosec=int(round((t_s - sec) * 1e9)))),
        child_frame_id=child,
        transform=ns(translation=ns(x=x, y=0.0, z=z), rotation=ns(x=0.0, y=0.0, z=0.0, w=1.0)),
    )])


class TestSynthesizerWithOfflineTFTree:
    """FrameSynthesizer._append_ee_channels on a real OfflineTFTree fed from /tf messages."""

    TF_HZ = 20

    def _tree(self, max_age_s, gap=(None, None)):
        from sobits_vla_rosbag_conversion.offline_tf_tree import OfflineTFTree
        tree = OfflineTFTree(max_age_ns=int(max_age_s * 1e9))
        # base -> lift (constant lift) -> ee moving at 0.5 m/s for 1 s; optional dropout.
        for i in range(self.TF_HZ + 1):
            t = i / self.TF_HZ
            if gap[0] is not None and gap[0] < t <= gap[1]:
                continue
            tree.ingest(_tf_msg(t, 'base', 'lift', 0.0, z=0.1), is_static=False)
            tree.ingest(_tf_msg(t, 'lift', 'ee', 0.5 * t), is_static=False)
        return tree

    def _append(self, tree, t_sec):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        synth = types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')], fps=FPS, ee_rotation='rotvec',
            log_warn=lambda msg: None, _append_ee_channels=FrameSynthesizer._append_ee_channels,
        )
        state, action, counters = [], [], {'tf': 0}
        ok = synth._append_ee_channels(synth, state, action, tree, t_sec, {'left': None}, counters)
        return ok, state, action, counters

    def test_state_at_t_and_action_one_frame_ahead_through_chain(self):
        ok, state, action, _ = self._append(self._tree(0.5), 0.3)
        assert ok
        np.testing.assert_allclose(state[:3], [0.15, 0.0, 0.1], atol=1e-6)
        np.testing.assert_allclose(action[:3], [0.20, 0.0, 0.1], atol=1e-6)

    def test_tf_dropout_beyond_max_age_skips_frame(self):
        tree = self._tree(0.1, gap=(0.6, 0.9))
        ok, _, _, counters = self._append(tree, 0.75)
        assert not ok and counters['tf'] == 1
        ok, state, action, _ = self._append(tree, 0.95)  # back to live samples
        assert ok
        np.testing.assert_allclose(state[0], 0.475, atol=1e-6)

    def test_action_past_end_of_recording_within_max_age_holds_last_pose(self):
        ok, state, action, _ = self._append(self._tree(0.1), 1.0)
        assert ok
        np.testing.assert_allclose(action[0], 0.5, atol=1e-6)

    def test_action_past_end_of_recording_beyond_max_age_skips_frame(self):
        ok, _, _, counters = self._append(self._tree(0.1), 1.05)
        assert not ok and counters['tf'] == 1


class TestEEActionFeatures:

    def test_feature_names_in_order(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        assert ee_action_features('left') == [
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.rx', 'ee.left.ry', 'ee.left.rz',
        ]

    def test_rpy_feature_names_in_order(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        assert ee_action_features('left', rotation='rpy') == [
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
            'ee_rotation': 'rotvec',
            'has_mobile_base': True, 'has_cmd_vel_y': False, 'has_cmd_vel_z': False,
            'has_subtasks': False,
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
            'ee.left.rx', 'ee.left.ry', 'ee.left.rz',
            'base_x', 'base_theta',
        ]
        assert features['observation.state']['names'] == features['action']['names']

    def test_no_ee_pose_side_channel(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._stub_node()
        features = RosbagConversionNode._build_features(node, all_tasks=[])
        assert not any(k.startswith('observation.ee_pose') for k in features)
        assert not any(k.endswith('.delta') for k in features)

    def test_tf_enabled_only_with_ee_actions(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        tf_enabled = RosbagConversionNode.tf_enabled.fget
        assert tf_enabled(self._stub_node()) is True
        assert tf_enabled(self._stub_node(ee_action_specs=[])) is False

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

    def _node(self, skip_static_threshold=0.0):
        quiet = types.SimpleNamespace(warning=lambda msg: None, info=lambda msg: None)
        return types.SimpleNamespace(
            skip_static_threshold=skip_static_threshold,
            get_logger=lambda: quiet, log=quiet,
        )

    def _params(self, arms=(), exclude_groups=(), rotation='rotvec'):
        return types.SimpleNamespace(
            ee_actions=types.SimpleNamespace(arms=list(arms), rotation=rotation),
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

    def test_rotvec_is_default_rotation(self):
        from sobits_vla_rosbag_conversion.conversion_node import _SCHEMA
        assert _SCHEMA['ee_actions']['rotation'].default == 'rotvec'

    def test_invalid_rotation_lists_choices(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'], rotation='euler')
        with pytest.raises(ValueError, match=r"\['quat', 'rotvec', 'rpy'\]"):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_rotvec_sets_ee_rotation_attr(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'])
        RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)
        assert node.ee_rotation == 'rotvec'


@skip_no_rclpy
class TestActionConvention:
    """_action_convention feeds both conversion_stats.yaml and meta/sobits_vla_info.json."""

    def _node(self, specs, rotation='rotvec'):
        return types.SimpleNamespace(ee_action_specs=specs, ee_rotation=rotation)

    def test_joint_only_dataset_is_absolute(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        conv = RosbagConversionNode._action_convention(self._node([]))
        assert conv == {'action_mode': 'absolute'}

    def test_ee_rotvec_records_rotation_and_frames(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        conv = RosbagConversionNode._action_convention(
            self._node([('left', 'ee_l', 'base_footprint')])
        )
        assert conv == {
            'action_mode': 'absolute', 'ee_rotation': 'rotvec',
            'ee_frames': {'left': {'source': 'ee_l', 'target': 'base_footprint'}},
        }

    def test_rpy_records_euler_convention(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        conv = RosbagConversionNode._action_convention(
            self._node([('left', 'ee_l', 'base')], rotation='rpy')
        )
        assert conv['rpy_convention'] == 'xyz_extrinsic'

    def test_sidecar_contains_action_convention(self, tmp_path):
        from sobits_vla_rosbag_conversion.dataset_writer import (
            read_custom_info, write_custom_info,
        )
        conv = {'action_mode': 'absolute', 'ee_rotation': 'rotvec'}
        write_custom_info(tmp_path, robot_info={}, user_info={}, action_convention=conv)
        assert read_custom_info(tmp_path)['action_convention'] == conv


@skip_no_rclpy
class TestDeprecatedParams:
    """_check_deprecated_params is a staticmethod; no ROS node needed."""

    def _params(self, ee_poses=(), ee_actions_enabled='', relative=False, frame=''):
        return types.SimpleNamespace(
            exclude=types.SimpleNamespace(ee_poses=list(ee_poses)),
            ee_actions=types.SimpleNamespace(enabled=ee_actions_enabled, frame=frame),
            use_relative_actions=relative,
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

    def test_use_relative_actions_true_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        with pytest.raises(ValueError, match='robot.ee_relative_actions'):
            RosbagConversionNode._check_deprecated_params(self._params(relative=True))

    def test_ee_actions_frame_set_raises(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        for frame in ('base', 'body'):
            with pytest.raises(ValueError, match='ee_actions.frame was removed'):
                RosbagConversionNode._check_deprecated_params(self._params(frame=frame))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
