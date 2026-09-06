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

"""Unit tests for RobotDescriptor.filtered(), focused on exclude_joints."""

from dataclasses import replace
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402

from sobits_vla_common.robot_descriptor import (  # noqa: E402
    _parse_descriptor_file, CameraSpec, ee_action_features, EEControlSpec,
    EEPoseSpec, GroupSpec, JointSpec, MobileBaseSpec, RobotDescriptor,
    validate_descriptor,
)


def _joint(feature, ros_name=None):
    return JointSpec(ros_name=ros_name or feature, feature=feature)


def _make_descriptor():
    """Two-group robot: arm (3 joints) + gripper (1 joint), one camera, one ee_pose."""
    arm = GroupSpec(
        name='arm', command_topic='/arm/cmd', command_action=None, state_topic='',
        max_joint_delta=0.15, active=True,
        joints=[_joint('shoulder'), _joint('elbow'), _joint('wrist')],
    )
    gripper = GroupSpec(
        name='gripper', command_topic='/gripper/cmd', command_action=None, state_topic='',
        max_joint_delta=0.30, active=True, relative_exclude=True,
        joints=[_joint('grip')],
    )
    mobile_base = MobileBaseSpec(
        name='mobile_base', command_topic='/cmd_vel', odom_topic='/odom',
        has_vel_x=True, has_vel_y=False, has_vel_z=False, has_vel_theta=True,
        max_vel_x=0.0, max_vel_y=0.0, max_vel_z=0.0, max_vel_theta=0.0,
        features=['x.vel', 'theta.vel'],
    )
    camera = CameraSpec(
        name='head_camera', compressed_topic='/head/compressed', raw_topic='/head/raw',
        info_topic='/head/info', encoding='', compressed=True, active=True,
    )
    ee_pose = EEPoseSpec(name='left', source_frame='hand_link', target_frame='base')
    return RobotDescriptor(
        robot_id='test_robot', joint_states_topic='/joint_states',
        groups=[arm, gripper], mobile_base=mobile_base,
        sensors={'cameras': [camera]}, ee_poses=[ee_pose],
        excluded_joints=['wheel_1', 'wheel_2'],
    )


def test_exclude_joints_removes_from_active_group():
    desc = _make_descriptor()
    filtered = desc.filtered(exclude_joints=['elbow'])
    arm = next(g for g in filtered.active_groups if g.name == 'arm')
    assert [j.feature for j in arm.joints] == ['shoulder', 'wrist']
    # Other groups untouched.
    gripper = next(g for g in filtered.active_groups if g.name == 'gripper')
    assert [j.feature for j in gripper.joints] == ['grip']


def test_exclude_joints_unknown_name_raises():
    desc = _make_descriptor()
    with pytest.raises(ValueError, match='Unknown joint'):
        desc.filtered(exclude_joints=['not_a_real_joint'])


def test_exclude_joints_emptying_group_drops_it_entirely():
    desc = _make_descriptor()
    filtered = desc.filtered(exclude_joints=['grip'])
    assert 'gripper' not in [g.name for g in filtered.groups]
    assert 'gripper' not in [g.name for g in filtered.active_groups]
    # The arm group is unaffected.
    assert [g.name for g in filtered.groups] == ['arm']


def test_exclude_joints_ros_name_joins_all_excluded_ros_names():
    desc = _make_descriptor()
    filtered = desc.filtered(exclude_joints=['elbow'])
    assert 'elbow' in filtered.all_excluded_ros_names
    # Pre-existing excluded_joints entries survive alongside the new one.
    assert 'wheel_1' in filtered.all_excluded_ros_names
    assert 'wheel_2' in filtered.all_excluded_ros_names


def test_exclude_joints_ros_name_differs_from_feature_name():
    """ros_name (what shows up in joint_states) must be recorded, not feature."""
    arm = GroupSpec(
        name='arm', command_topic='/arm/cmd', command_action=None, state_topic='',
        max_joint_delta=0.15, active=True,
        joints=[_joint('shoulder_feature', ros_name='shoulder_joint_ros')],
    )
    other = GroupSpec(
        name='other', command_topic='/other/cmd', command_action=None, state_topic='',
        max_joint_delta=0.10, active=True, joints=[_joint('other_feature')],
    )
    desc = RobotDescriptor(
        robot_id='test_robot', joint_states_topic='/joint_states', groups=[arm, other],
    )
    filtered = desc.filtered(exclude_joints=['shoulder_feature'])
    # arm is emptied and dropped; only ros_name (not feature name) is recorded.
    assert [g.name for g in filtered.groups] == ['other']
    assert 'shoulder_joint_ros' in filtered.all_excluded_ros_names
    assert 'shoulder_feature' not in filtered.all_excluded_ros_names


def test_exclude_joints_combined_with_exclude_groups():
    desc = _make_descriptor()
    filtered = desc.filtered(exclude_groups=['gripper'], exclude_joints=['elbow'])
    assert [g.name for g in filtered.active_groups] == ['arm']
    arm = filtered.groups[0]
    assert [j.feature for j in arm.joints] == ['shoulder', 'wrist']
    # gripper marked inactive (not dropped -- excluded via group, not emptied
    # by joint removal) so its joint still surfaces via all_excluded_ros_names.
    assert 'grip' in filtered.all_excluded_ros_names
    assert 'elbow' in filtered.all_excluded_ros_names


def test_exclude_joints_no_args_returns_self():
    desc = _make_descriptor()
    assert desc.filtered() is desc


def test_exclude_joints_empty_list_is_noop():
    desc = _make_descriptor()
    filtered = desc.filtered(exclude_joints=[])
    assert filtered is desc


_MINIMAL_YAML = """
schema_version: 1
robot_id: test_robot
joint_states_topic: /joint_states
groups:
  - name: arm_left
    command_topic: /arm_left/cmd
    max_joint_delta: 0.0
    active: true
    joints:
      - ros_name: shoulder
        feature: shoulder
  - name: arm_right
    command_topic: /arm_right/cmd
    max_joint_delta: 0.0
    active: true
    joints:
      - ros_name: shoulder_r
        feature: shoulder_r
ee:
  - name: left
    ee_link: hand_left_link
    reference_frame: base_footprint
    control:
      group: arm_left
      command_frame: left_target_link
      enable_topic: arm_left/moveit_track_enabled
  - name: right
    ee_link: hand_right_link
    reference_frame: base_footprint
    control:
      group: arm_right
      command_frame: right_target_link
      enable_topic: arm_right/moveit_track_enabled
"""


def _write_yaml(tmp_path, text):
    path = os.path.join(tmp_path, 'test_robot.robot.yaml')
    with open(path, 'w') as f:
        f.write(text)
    return path


def test_ee_control_parses_from_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    assert len(desc.ee_control) == 2
    left = next(c for c in desc.ee_control if c.ee_pose == 'left')
    assert left.group == 'arm_left'
    assert left.target_frame == 'left_target_link'
    assert left.enable_topic == 'arm_left/moveit_track_enabled'


def test_ee_control_missing_key_defaults_to_empty_list():
    text = _MINIMAL_YAML.split('ee:')[0]
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        desc = _parse_descriptor_file(path)
    assert desc.ee_control == []
    assert desc.ee_poses is None


def test_ee_missing_key_yields_none_poses_and_empty_control():
    """No 'ee:' key at all -> ee_poses stays None, ee_control stays []."""
    text = _MINIMAL_YAML.split('ee:')[0]
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        desc = _parse_descriptor_file(path)
    assert desc.ee_poses is None
    assert desc.ee_control == []


def test_ee_empty_list_yields_empty_poses_and_empty_control():
    text = _MINIMAL_YAML.split('ee:')[0] + 'ee: []\n'
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        desc = _parse_descriptor_file(path)
    assert desc.ee_poses == []
    assert desc.ee_control == []


def test_ee_entry_without_control_parses_pose_only():
    text = _MINIMAL_YAML.split('ee:')[0] + """ee:
  - name: left
    ee_link: hand_left_link
    reference_frame: base_footprint
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        desc = _parse_descriptor_file(path)
    assert len(desc.ee_poses) == 1
    left = desc.ee_poses[0]
    assert left.name == 'left'
    assert left.source_frame == 'hand_left_link'
    assert left.target_frame == 'base_footprint'
    assert desc.ee_control == []


def test_old_ee_poses_key_raises_value_error():
    text = _MINIMAL_YAML.split('ee:')[0] + """ee_poses:
  - name: left
    source_frame: hand_left_link
    target_frame: base_footprint
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        with pytest.raises(ValueError, match="removed 'ee_poses'"):
            _parse_descriptor_file(path)


def test_old_ee_control_key_raises_value_error():
    text = _MINIMAL_YAML.split('ee:')[0] + """ee_control:
  - ee_pose: left
    group: arm_left
    target_frame: left_target_link
    enable_topic: arm_left/moveit_track_enabled
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, text)
        with pytest.raises(ValueError, match="removed 'ee_poses'"):
            _parse_descriptor_file(path)


def test_ee_control_validate_unknown_ee_pose():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    bad = desc.ee_control[0]
    desc.ee_control[0] = replace(bad, ee_pose='center')
    errors = validate_descriptor(desc)
    assert any('unknown ee_pose' in e for e in errors)


def test_ee_control_validate_unknown_group():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    desc.ee_control[0] = replace(desc.ee_control[0], group='arm_center')
    errors = validate_descriptor(desc)
    assert any('unknown group' in e for e in errors)


def test_ee_control_validate_empty_enable_topic():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    desc.ee_control[0] = replace(desc.ee_control[0], enable_topic='')
    errors = validate_descriptor(desc)
    assert any('empty enable_topic' in e for e in errors)


def test_ee_control_validate_duplicate_group():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    desc.ee_control[1] = replace(desc.ee_control[1], group='arm_left')
    errors = validate_descriptor(desc)
    assert any('Duplicate ee_control group' in e for e in errors)


def test_ee_control_filtered_drops_excluded_ee_pose():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    filtered = desc.filtered(exclude_ee_poses=['right'])
    assert [c.ee_pose for c in filtered.ee_control] == ['left']


def test_ee_action_features_axis_order():
    assert ee_action_features('left') == [
        'ee.left.x', 'ee.left.y', 'ee.left.z',
        'ee.left.roll', 'ee.left.pitch', 'ee.left.yaw',
    ]


def test_ee_action_features_default_matches_explicit_rpy():
    assert ee_action_features('left') == ee_action_features('left', rotation='rpy')


def test_ee_action_features_quat_axis_order():
    assert ee_action_features('left', rotation='quat') == [
        'ee.left.x', 'ee.left.y', 'ee.left.z',
        'ee.left.qx', 'ee.left.qy', 'ee.left.qz', 'ee.left.qw',
    ]


def test_ee_action_features_invalid_rotation_raises():
    with pytest.raises(ValueError, match='rotation'):
        ee_action_features('left', rotation='axis_angle')


def test_ee_control_for_returns_matching_specs():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    specs = desc.ee_control_for(['left'])
    assert [s.ee_pose for s in specs] == ['left']


def test_ee_control_for_unknown_name_raises():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    with pytest.raises(ValueError, match='Unknown ee_pose'):
        desc.ee_control_for(['center'])


def test_active_ee_control_excludes_removed_ee_pose():
    with tempfile.TemporaryDirectory() as tmp:
        path = _write_yaml(tmp, _MINIMAL_YAML)
        desc = _parse_descriptor_file(path)
    filtered = desc.filtered(exclude_ee_poses=['right'])
    assert [c.ee_pose for c in filtered.active_ee_control] == ['left']


def test_filtered_all_groups_excluded_without_ee_control_raises():
    desc = _make_descriptor()
    with pytest.raises(ValueError, match='every joint group'):
        desc.filtered(exclude_groups=['arm', 'gripper'])


def test_filtered_all_groups_excluded_allowed_in_pure_ee_mode():
    desc = replace(_make_descriptor(), ee_control=[EEControlSpec(
        ee_pose='left', group='arm', target_frame='left_target_link',
        enable_topic='arm/moveit_track_enabled',
    )])
    filtered = desc.filtered(exclude_groups=['arm', 'gripper'])
    assert filtered.active_groups == []
    assert [c.ee_pose for c in filtered.ee_control] == ['left']
