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

"""Shared descriptor + robot_overrides adapter: shipped robots, merging, typo checks."""

import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

import pytest  # noqa: E402

from sobits_vla_common.robot_descriptor import JointSpec, load_robot_descriptor  # noqa: E402
from sobits_vla_common.robot_overrides import (  # noqa: E402
    merge_overrides, robot_overrides_from_params,
)
from test_robot_descriptor import (  # noqa: E402
    _MINIMAL_YAML, _parse_descriptor_file, _write_yaml,
)


def test_sobit_home_topics_are_absolute():
    desc = load_robot_descriptor('sobit_home')
    assert desc.joint_states_topic == '/sobit_home/joint_states'
    head = desc.groups[0]
    assert head.command_topic == '/sobit_home/head_position_controller/joint_trajectory'
    assert head.command_action == '/sobit_home/head_position_controller/follow_joint_trajectory'
    assert desc.mobile_base.command_topic == '/sobit_home/cmd_vel'
    assert desc.mobile_base.odom_topic == '/sobit_home/odom'
    cam = desc.active_cameras[0]
    assert cam.raw_topic == '/sobit_home/head_camera/color/image_raw'
    assert cam.info_topic == '/sobit_home/head_camera/color/camera_info'


def test_sobit_home_keeps_v1_features_and_lab_defaults():
    desc = load_robot_descriptor('sobit_home')
    assert [g.name for g in desc.active_groups] == [
        'head', 'body', 'arm_left', 'arm_right', 'hand_left', 'hand_right']
    features = desc.all_joint_features
    assert len(features) == 31
    assert features[:3] == ['head_pan_joint', 'head_tilt_joint', 'body_lift_joint']
    assert 'arm_left_lower_flex_joint' in features
    assert all(g.max_joint_delta == 0.0 for g in desc.groups)
    mb = desc.mobile_base
    assert mb.features == ['x.vel', 'y.vel', 'theta.vel']
    assert (mb.linear_deadband, mb.angular_deadband) == (0.015, 0.005)
    rel = desc.relative_exclude_features()
    assert rel[:3] == ['base_x', 'base_y', 'base_theta']
    assert 'hand_left_finger_l_mcp_joint' in rel and 'arm_left_elbow_joint' not in rel
    assert [w for w in desc.excluded_joints if w.startswith('wheel_')] == [
        f'wheel_{k}_{c}_joint' for k in ('steer', 'drive') for c in ('f_l', 'f_r', 'b_l', 'b_r')]
    assert 'hand_left_finger_r_mcp_joint' in desc.excluded_joints


def test_sobit_home_depth_pairs_as_inactive_depth_entry():
    desc = load_robot_descriptor('sobit_home')
    assert [c.name for c in desc.active_cameras] == [
        'head_camera', 'hand_left_camera', 'hand_right_camera']
    assert desc.active_depth_cameras == []
    depth = next(c for c in desc.sensors['cameras'] if c.is_depth)
    assert depth.name == 'head_camera_depth'
    assert depth.encoding == '16UC1'
    assert depth.raw_topic == '/sobit_home/head_camera/depth/image_raw'
    assert depth.compressed_topic == '/sobit_home/head_camera/depth/image_raw/compressedDepth'
    assert depth.info_topic == '/sobit_home/head_camera/depth/camera_info'
    assert all(c.compressed and c.encoding == '' for c in desc.active_cameras)


def test_stage_overrides_merge_over_defaults():
    desc = load_robot_descriptor('sobit_home', overrides={
        'groups': {'arm_left': {'max_joint_delta': 0.2}, 'hand_right': {'active': False}},
        'cameras': {'head_camera': {'depth': {'active': True}}},
        'mobile_base': {'linear_deadband': 0.0},
    })
    arm_left = next(g for g in desc.groups if g.name == 'arm_left')
    assert arm_left.max_joint_delta == 0.2
    assert 'hand_right' not in [g.name for g in desc.active_groups]
    assert 'hand_right_finger_l_mcp_joint' in desc.all_excluded_ros_names
    hand_left = next(g for g in desc.groups if g.name == 'hand_left')
    assert hand_left.relative_exclude
    assert [c.name for c in desc.active_depth_cameras] == ['head_camera_depth']
    assert desc.mobile_base.linear_deadband == 0.0
    assert desc.mobile_base.angular_deadband == 0.005


def test_filtered_on_loaded_sobit_home_derives_ee_arm():
    desc = load_robot_descriptor('sobit_home').filtered(
        exclude_groups=['arm_left'], exclude_cameras=['head_camera_depth'])
    assert desc.derived_ee_action_arms() == ['left']
    assert 'arm_left_elbow_joint' in desc.all_excluded_ros_names
    assert 'arm_left_elbow_joint' not in desc.all_joint_features


def test_sobit_home_v1_1_loads_older_arm_revision():
    desc = load_robot_descriptor('sobit_home_v1_1')
    assert desc.robot_id == 'sobit_home_v1_1'
    assert desc.version == '1.1.0'
    assert len(desc.all_joint_features) == 29
    assert not any('lower_flex' in f for f in desc.all_joint_features)
    for side in ('left', 'right'):
        for j in ('upper_roll', 'upper_flex', 'elbow'):
            assert f'arm_{side}_{j}_sub_joint' in desc.excluded_joints
    assert desc.joint_states_topic == '/sobit_home/joint_states'


def test_sobit_light_feature_order_follows_descriptor():
    desc = load_robot_descriptor('sobit_light')
    assert desc.all_joint_features == [
        'head_yaw_joint', 'head_pitch_joint',
        'arm_shoulder_roll_joint', 'arm_shoulder_pitch_joint', 'arm_elbow_pitch_joint',
        'arm_forearm_roll_joint', 'arm_wrist_pitch_joint', 'arm_wrist_roll_joint',
        'hand_joint',
    ]
    assert 'hand_sub_joint' in desc.excluded_joints
    assert [c.name for c in desc.active_cameras] == ['head_camera', 'hand_camera']
    assert desc.mobile_base.features == ['x.vel', 'theta.vel']
    assert desc.filtered().ee_poses == []


def _write_overrides_case(tmp, overrides):
    return _parse_descriptor_file(_write_yaml(tmp, _MINIMAL_YAML), overrides)


@pytest.mark.parametrize('overrides, match', [
    ({'groups': {'arm_center': {'active': False}}}, "unknown name 'arm_center'"),
    ({'groups': {'arm_left': {'max_delta': 0.1}}}, r"unknown key\(s\) \['max_delta'\]"),
    ({'cameras': {'head_camera': {}}}, "unknown name 'head_camera'"),
    ({'mobile_base': {'active': True}}, 'no mobile_base'),
    ({'bogus': 1}, r"unknown key\(s\) \['bogus'\]"),
    ({'groups': {'arm_left': {'features': ['a', 'b']}}}, '2 names for 1 joints'),
])
def test_overrides_reject_typos(overrides, match):
    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(ValueError, match=match):
            _write_overrides_case(tmp, overrides)


def test_overrides_rename_features_and_drop_ee():
    with tempfile.TemporaryDirectory() as tmp:
        desc = _write_overrides_case(tmp, {
            'groups': {'arm_left': {'features': ['left_shoulder']}},
            'ee': {'right': {'active': False}},
        })
    assert desc.groups[0].joints == [JointSpec(ros_name='shoulder', feature='left_shoulder')]
    assert desc.groups[0].command_topic == '/arm_left_controller/joint_trajectory'
    assert [e.name for e in desc.filtered().ee_poses] == ['left']


def test_merge_overrides_is_deep_and_by_name():
    base = {'groups': {'a': {'active': True, 'max_joint_delta': 0.1}}, 'mobile_base': {'x': 1}}
    out = merge_overrides(base, {'groups': {'a': {'active': False}, 'b': {'active': True}}})
    assert out['groups'] == {'a': {'active': False, 'max_joint_delta': 0.1},
                             'b': {'active': True}}
    assert base['groups']['a']['active'] is True


def test_robot_overrides_from_flat_params():
    class _Param:
        def __init__(self, value):
            self.value = value

    flat = {
        'robot_overrides.groups.arm_left.max_joint_delta': _Param(0.3),
        'robot_overrides.cameras.head_camera.depth.active': True,
        'robot_overrides.ee.left.active': _Param(None),
        'robot.descriptor_id': 'sobit_home',
    }
    assert robot_overrides_from_params(flat) == {
        'groups': {'arm_left': {'max_joint_delta': 0.3}},
        'cameras': {'head_camera': {'depth': {'active': True}}},
    }
