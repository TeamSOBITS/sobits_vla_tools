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
from pathlib import Path
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.sync.poses import synthesize_ee_action  # noqa: E402

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

    def test_end_of_bag_falls_back_to_state(self):
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


class TestAppendEEChannelsRelative:
    """_append_ee_channels is a plain method; drive it via a stub self (R1: no rclpy)."""

    def _synthesizer(self, use_relative_actions):
        from sobits_vla_rosbag_conversion.frame_synthesizer import FrameSynthesizer
        return types.SimpleNamespace(
            ee_action_specs=[('left', 'ee', 'base')],
            fps=FPS,
            use_relative_actions=use_relative_actions,
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


class TestEEActionFeatures:

    def test_feature_names_in_order(self):
        from sobits_vla_common.robot_descriptor import ee_action_features
        assert ee_action_features('left') == [
            'ee.left.x', 'ee.left.y', 'ee.left.z',
            'ee.left.roll', 'ee.left.pitch', 'ee.left.yaw',
        ]


@skip_no_rclpy
class TestBuildFeaturesWithEE:
    """_build_features is a bound method; drive it via a stub `self` (R1: no rclpy.init needed)."""

    def _stub_node(self, **overrides):
        base = {
            'action_features': ['head_pan_joint', 'head_tilt_joint'],
            'ee_action_specs': [('left', 'ee_l', 'base')],
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
        filtered = desc.filtered(exclude_ee_poses=['left'])
        with pytest.raises(ValueError):
            filtered.ee_control_for(['left'])


@skip_no_rclpy
class TestResolveEEActionsValidation:
    """_resolve_ee_actions is a plain method; drive it via stub self/params (no ROS needed)."""

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

    def _node(self, use_relative_actions=False, skip_static_threshold=0.0):
        return types.SimpleNamespace(
            use_relative_actions=use_relative_actions,
            skip_static_threshold=skip_static_threshold,
            get_logger=lambda: types.SimpleNamespace(warning=lambda msg: None),
        )

    def _params(self, enabled=True, arms=('left',), exclude_groups=()):
        return types.SimpleNamespace(
            ee_actions=types.SimpleNamespace(enabled=enabled, arms=list(arms)),
            exclude=types.SimpleNamespace(groups=list(exclude_groups)),
        )

    def test_raises_when_superseded_group_not_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=[])  # arm_left NOT excluded
        with pytest.raises(ValueError, match='exclude.groups'):
            RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)

    def test_passes_when_superseded_group_excluded(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)
        assert specs == [('left', 'ee_l', 'base')]

    def test_relative_actions_allowed(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node(use_relative_actions=True)
        params = self._params(exclude_groups=['arm_left'])
        specs = RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params)
        assert specs == [('left', 'ee_l', 'base')]

    def test_disabled_returns_empty_without_validating(self):
        from sobits_vla_rosbag_conversion.conversion_node import RosbagConversionNode
        node = self._node()
        params = self._params(enabled=False, arms=(), exclude_groups=[])
        assert RosbagConversionNode._resolve_ee_actions(node, self._descriptor(), params) == []


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
