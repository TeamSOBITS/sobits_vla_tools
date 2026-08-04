from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class JointSpec:
    ros_name: str
    feature: str


@dataclass
class GroupSpec:
    name: str
    command_topic: str
    command_action: Optional[str]
    max_joint_delta: float
    active: bool
    joints: List[JointSpec]
    # Keep this group's features absolute (never delta-convert) in relative
    # mode — e.g. a gripper whose open/close is absolute, not incremental.
    relative_exclude: bool = False


@dataclass
class MobileBaseSpec:
    command_topic: str
    odom_topic: str
    has_vel_x: bool
    has_vel_y: bool
    has_vel_z: bool
    has_vel_theta: bool
    max_vel_x: float
    max_vel_y: float
    max_vel_z: float
    max_vel_theta: float
    features: List[str]


@dataclass
class CameraSpec:
    name: str
    compressed_topic: str
    raw_topic: str
    info_topic: str
    encoding: str
    compressed: bool
    active: bool
    is_depth: bool = False


@dataclass
class EEPoseSpec:
    name: str
    source_frame: str
    target_frame: str


@dataclass
class RobotDescriptor:
    robot_id: str
    joint_states_topic: str
    groups: List[GroupSpec] = field(default_factory=list)
    mobile_base: Optional[MobileBaseSpec] = None
    sensors: Dict[str, List[Any]] = field(default_factory=dict)
    ee_poses: Optional[List[EEPoseSpec]] = None
    excluded_joints: List[str] = field(default_factory=list)

    @property
    def active_groups(self) -> List[GroupSpec]:
        return [g for g in self.groups if g.active]

    @property
    def active_cameras(self) -> List[CameraSpec]:
        cameras = self.sensors.get('cameras', [])
        return [c for c in cameras if c.active and not c.is_depth]

    @property
    def active_depth_cameras(self) -> List[CameraSpec]:
        cameras = self.sensors.get('cameras', [])
        return [c for c in cameras if c.active and c.is_depth]

    @property
    def all_joint_features(self) -> List[str]:
        features = []
        for g in self.active_groups:
            for j in g.joints:
                features.append(j.feature)
        return features

    @property
    def active_ros_names(self) -> List[str]:
        ros_names = []
        for g in self.active_groups:
            for j in g.joints:
                ros_names.append(j.ros_name)
        return ros_names

    @property
    def all_excluded_ros_names(self) -> List[str]:
        ros_names = list(self.excluded_joints)
        for g in self.groups:
            if not g.active:
                for j in g.joints:
                    ros_names.append(j.ros_name)
        return ros_names

    # Maps mobile_base feature keys (x.vel/...) to dataset action feature names.
    _BASE_FEATURE_MAP = {
        'x.vel': 'base_x',
        'y.vel': 'base_y',
        'z.vel': 'base_z',
        'theta.vel': 'base_theta',
    }

    def relative_exclude_features(
        self,
        active_groups: Optional[List[str]] = None,
        active_mobile_base: bool = True,
    ) -> List[str]:
        """
        Dataset feature names to keep absolute in relative-action mode.

        Mobile-base velocities and any group flagged relative_exclude (e.g. a
        gripper) are never delta-converted. ``active_groups`` defaults to all
        active groups; pass a subset to match a specific config selection.
        """
        if active_groups is None:
            active_groups = [g.name for g in self.active_groups]
        features: List[str] = []
        if self.mobile_base and active_mobile_base:
            features.extend(
                self._BASE_FEATURE_MAP[f]
                for f in self.mobile_base.features
                if f in self._BASE_FEATURE_MAP
            )
        for g in self.groups:
            if g.name in active_groups and g.relative_exclude:
                features.extend(j.feature for j in g.joints)
        return features


def _parse_descriptor_file(path: Path) -> RobotDescriptor:
    with open(path) as f:
        data = yaml.safe_load(f)

    # Build Groups
    groups = []
    for g in (data.get('groups') or []):
        joints = [
            JointSpec(ros_name=j['ros_name'], feature=j['feature'])
            for j in (g.get('joints') or [])
        ]
        groups.append(GroupSpec(
            name=g['name'],
            command_topic=g['command_topic'],
            command_action=g.get('command_action'),
            max_joint_delta=float(g.get('max_joint_delta', 0.0)),
            active=bool(g.get('active', True)),
            joints=joints,
            relative_exclude=bool(g.get('relative_exclude', False)),
        ))

    # Build Mobile Base
    mb = data.get('mobile_base')
    mobile_base = None
    if mb:
        mobile_base = MobileBaseSpec(
            command_topic=mb['command_topic'],
            odom_topic=mb['odom_topic'],
            has_vel_x=bool(mb.get('has_vel_x', False)),
            has_vel_y=bool(mb.get('has_vel_y', False)),
            has_vel_z=bool(mb.get('has_vel_z', False)),
            has_vel_theta=bool(mb.get('has_vel_theta', False)),
            max_vel_x=float(mb.get('max_vel_x', 0.0)),
            max_vel_y=float(mb.get('max_vel_y', 0.0)),
            max_vel_z=float(mb.get('max_vel_z', 0.0)),
            max_vel_theta=float(mb.get('max_vel_theta', 0.0)),
            features=list(mb.get('features') or [])
        )
    # Build Sensors
    sensors = {}
    s_dict = data.get('sensors') or {}
    for s_type, s_list in s_dict.items():
        if s_type == 'cameras':
            cameras = []
            for c in (s_list or []):
                cameras.append(CameraSpec(
                    name=c['name'],
                    compressed_topic=c.get('compressed_topic', ''),
                    raw_topic=c.get('raw_topic', ''),
                    info_topic=c.get('info_topic', ''),
                    encoding=c.get('encoding', ''),
                    compressed=bool(c.get('compressed', False)),
                    active=bool(c.get('active', True)),
                    is_depth=bool(c.get('is_depth', False))
                ))
            sensors['cameras'] = cameras
        else:
            sensors[s_type] = s_list

    # Build ee_poses
    ee_poses = None
    ee_list = data.get('ee_poses')
    if ee_list:
        ee_poses = [
            EEPoseSpec(
                name=e['name'],
                source_frame=e['source_frame'],
                target_frame=e['target_frame']
            )
            for e in ee_list
        ]

    return RobotDescriptor(
        robot_id=data['robot_id'],
        joint_states_topic=data['joint_states_topic'],
        groups=groups,
        mobile_base=mobile_base,
        sensors=sensors,
        ee_poses=ee_poses,
        excluded_joints=list(data.get('excluded_joints') or [])
    )


def resolve_descriptor_path(robot_id: str) -> Path:
    """Return the path to ``<robot_id>.robot.yaml`` or raise FileNotFoundError."""
    # 1. Try ament index if available
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('sobits_vla_common'))
        candidate = share / 'robots' / f'{robot_id}.robot.yaml'
        if candidate.is_file():
            return candidate
    except Exception:
        pass

    # 2. Try source tree lookup relative to this file
    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        candidate = parent / 'robots' / f'{robot_id}.robot.yaml'
        if candidate.is_file():
            return candidate
        candidate = parent / 'sobits_vla_common' / 'robots' / f'{robot_id}.robot.yaml'
        if candidate.is_file():
            return candidate

    # 3. FileNotFoundError with a hint to new_robot scaffolder
    raise FileNotFoundError(
        f"Could not find robot descriptor for '{robot_id}'. "
        f'Ensure it exists at <share>/robots/{robot_id}.robot.yaml. '
        'You can generate a new scaffold with: '
        f'`ros2 run sobits_vla_common new_robot --robot_id {robot_id}`'
    )


def load_robot_descriptor(robot_id: str) -> RobotDescriptor:
    return _parse_descriptor_file(resolve_descriptor_path(robot_id))


def validate_descriptor(desc: RobotDescriptor) -> List[str]:
    errors = []

    # 1. No duplicate feature names
    features = []
    for g in desc.groups:
        for j in g.joints:
            features.append(j.feature)
    if desc.mobile_base:
        features.extend(desc.mobile_base.features)
    duplicates = {f for f in features if features.count(f) > 1}
    if duplicates:
        errors.append(f'Duplicate feature names found: {list(duplicates)}')

    # 2. At least one active group
    if not desc.active_groups:
        errors.append('Robot must have at least one active joint group.')

    # 3. No empty topics on active components
    for g in desc.active_groups:
        if not g.command_topic:
            errors.append(
                f"Active group '{g.name}' must have "
                'a command_topic defined.'
            )

    if desc.mobile_base and not desc.mobile_base.command_topic:
        errors.append('mobile_base has an empty command_topic.')

    cameras = desc.sensors.get('cameras', [])
    for c in cameras:
        if c.active:
            if not c.compressed_topic and not c.raw_topic:
                errors.append(
                    f"Active camera '{c.name}' must have "
                    'compressed_topic or raw_topic defined.'
                )

    return errors
