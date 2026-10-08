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


# refactor-exempt: file over 600 lines, descriptor schema, loader and filter belong together

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Mapping, Optional

from sobits_vla_common.robot_overrides import (
    check_overrides, descriptor_search_dirs, import_loader, load_robot_overrides,
)


# Dataset action feature name -> mobile_base velocity key. Single source of
# truth for both directions; RobotDescriptor._BASE_FEATURE_MAP is the inverse.
BASE_KEY_ALIASES: Dict[str, str] = {
    'base_x': 'x.vel',
    'base_y': 'y.vel',
    'base_z': 'z.vel',
    'base_theta': 'theta.vel',
}

# EE pose axes in feature-name order, expressed in each arm's reference_frame.
# Datasets store absolute poses; relative-to-observation is a training processor.
# rpy = scipy 'xyz' extrinsic (ROS RPY); rotvec = axis-angle, no wrap/gimbal.
EE_ACTION_AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
EE_ACTION_AXES_ROTVEC = ('x', 'y', 'z', 'rx', 'ry', 'rz')
EE_ACTION_AXES_QUAT = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')

EE_ROTATION_AXES: Dict[str, tuple] = {
    'rotvec': EE_ACTION_AXES_ROTVEC,
    'rpy': EE_ACTION_AXES,
    'quat': EE_ACTION_AXES_QUAT,
}
EE_ROTATION_DEFAULT = 'rotvec'

# A depth stream becomes its own camera entry; the name keeps dataset keys apart.
DEPTH_SUFFIX = '_depth'


def ee_action_features(name: str, rotation: str = EE_ROTATION_DEFAULT) -> List[str]:
    """
    Dataset action feature names for one EE pose, e.g. 'ee.left.x'.

    rotation='rotvec' (default) -> 6D (x,y,z,rx,ry,rz); 'rpy' -> 6D
    (x,y,z,roll,pitch,yaw); 'quat' -> 7D (x,y,z,qx,qy,qz,qw).
    """
    try:
        axes = EE_ROTATION_AXES[rotation]
    except KeyError:
        raise ValueError(
            f'rotation must be one of {sorted(EE_ROTATION_AXES)}, got {rotation!r}') from None
    return [f'ee.{name}.{ax}' for ax in axes]


def ee_rotation_from_names(names: List[str]) -> str:
    """Infer the EE rotation representation from dataset feature names; '' if no ee.* names."""
    for rotation, axes in EE_ROTATION_AXES.items():
        if any(n.startswith('ee.') and n.endswith('.' + axes[-1]) for n in names):
            return rotation
    return ''


def resolve_ee_action_specs(
    desc: 'RobotDescriptor', arms: Optional[List[str]] = None, *, param: str = 'ee_action_arms',
) -> List['EEControlSpec']:
    """
    Return the EEControlSpec per EE-action arm: derived, or *arms* validated by the same rule.

    One resolver for conversion (ee_actions.arms) and training (robot.ee_action_arms):
    an explicit arm must still satisfy derived_ee_action_arms(), or joint and EE
    features would both count for it. *param* names the setting in errors.
    """
    arms = [a for a in (arms or []) if a]
    if not arms:
        return desc.ee_control_for(desc.derived_ee_action_arms())
    derived = set(desc.derived_ee_action_arms())
    try:
        specs = desc.ee_control_for(arms)
    except ValueError as exc:
        raise ValueError(
            f'{param}: {exc} (an arm must have a surviving ee entry -- check exclude.ee)'
        ) from exc
    invalid = [s.ee_pose for s in specs if s.ee_pose not in derived]
    if invalid:
        still_active = [s.group for s in specs if s.ee_pose in invalid]
        raise ValueError(
            f'{param} names arm(s) {invalid} whose group is not excluded ({still_active} '
            'still active). The EE-action derivation rule needs the arm active and its '
            'control.group not active: add the group to exclude.groups, or mark it '
            'active: false in the descriptor, so joint and EE features do not both count.'
        )
    return specs


@dataclass
class JointSpec:
    ros_name: str
    feature: str


@dataclass
class GroupSpec:
    name: str
    command_topic: str
    command_action: Optional[str]
    # Empty means "derive from command_topic" (ros2_control naming fallback).
    state_topic: str
    max_joint_delta: float
    active: bool
    joints: List[JointSpec]
    # Keep this group's features absolute (never delta-convert) in relative
    # mode — e.g. a gripper whose open/close is absolute, not incremental.
    relative_exclude: bool = False


@dataclass
class MobileBaseSpec:
    name: str
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
    # Commands below these magnitudes are sent as zero, so regression noise
    # around 0 cannot creep the base during a stationary step.
    linear_deadband: float = 0.0
    angular_deadband: float = 0.0


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
    # Mirrors GroupSpec.active: filtered() treats active=False like an
    # excluded entry (see its docstring) -- it drops out of ee_poses.
    active: bool = True


@dataclass
class EEControlSpec:
    ee_pose: str        # references an ee[].name ('left'/'right')
    group: str          # joint group superseded by servo control ('arm_left')
    target_frame: str   # TF child frame streamed to the servo bridge
    enable_topic: str   # relative topic, e.g. 'arm_left/moveit_track_enabled'


@dataclass
class RobotDescriptor:
    robot_id: str
    joint_states_topic: str
    version: str = '1.0.0'
    morphology: str = 'mobile_manipulator'
    groups: List[GroupSpec] = field(default_factory=list)
    mobile_base: Optional[MobileBaseSpec] = None
    sensors: Dict[str, List[Any]] = field(default_factory=dict)
    ee_poses: Optional[List[EEPoseSpec]] = None
    ee_control: List[EEControlSpec] = field(default_factory=list)
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

    @property
    def active_ee_control(self) -> List[EEControlSpec]:
        known = {e.name for e in (self.ee_poses or [])}
        return [c for c in self.ee_control if c.ee_pose in known]

    def ee_control_for(self, names: List[str]) -> List[EEControlSpec]:
        """Return EEControlSpec entries matching the given ee_pose names, in order."""
        by_pose = {c.ee_pose: c for c in self.ee_control}
        unknown = [n for n in names if n not in by_pose]
        if unknown:
            raise ValueError(
                f'Unknown ee_pose name(s) in ee_control_for: {unknown}. '
                f'Available: {sorted(by_pose)}'
            )
        return [by_pose[n] for n in names]

    def derived_ee_action_arms(self) -> List[str]:
        """
        ee_pose names whose EE channels should become dataset ACTION features.

        Single source of truth for the derivation rule shared by conversion
        (ee_actions.arms) and training (robot.ee_action_arms): an ee entry
        contributes an EE action iff it survived filtering (active, not
        excluded via exclude.ee -- filtered() already dropped anything else
        from ee_poses) AND its ee_control.group is NOT active (excluded via
        exclude.groups or active: false on the group) -- i.e. the group is
        no longer commanding that arm via joint features, so the EE channels
        replace them instead of duplicating them. An active ee_pose whose
        group is still active is state/observation-only (existing
        joint-dataset behaviour) and is correctly excluded here. An ee_pose
        with no ee_control block at all cannot drive an action either.
        """
        active_group_names = {g.name for g in self.active_groups}
        return [
            c.ee_pose for c in self.ee_control
            if c.group not in active_group_names
        ]

    def filtered(  # refactor-exempt: exclude rules must be applied together
        self,
        exclude_groups: Optional[List[str]] = None,
        exclude_cameras: Optional[List[str]] = None,
        exclude_ee: Optional[List[str]] = None,
        exclude_joints: Optional[List[str]] = None,
    ) -> 'RobotDescriptor':
        """
        Return a copy with the named components deactivated/removed.

        Lets one descriptor describe the full robot while a consumer config
        trims it to the subset it uses (e.g. left-arm-only conversion).
        Excluded groups/cameras are marked inactive rather than dropped, so
        their joints still surface via ``all_excluded_ros_names`` and get
        filtered out of joint_states. Excluded joints (matched by feature
        name) are removed from their group's joint list outright, and their
        ros_names fold into ``excluded_joints`` the same way; a group left
        with no joints is dropped entirely. Unknown names raise ValueError so
        a typo fails loudly instead of silently converting a wrong morphology.

        An ee[] entry with ``active: false`` in the yaml is dropped from
        ``ee_poses`` here too, exactly as if it had been named in
        ``exclude_ee`` -- there is exactly one code path that removes ee
        entries, so consumers iterating ``desc.ee_poses`` never need to
        separately check an active flag.
        """
        ex_g = list(exclude_groups or [])
        ex_c = list(exclude_cameras or [])
        ex_e = list(exclude_ee or [])
        ex_j = list(exclude_joints or [])
        inactive_ee = [e.name for e in (self.ee_poses or []) if not e.active]
        if not (ex_g or ex_c or ex_e or ex_j or inactive_ee):
            return self

        cameras = self.sensors.get('cameras', [])
        known_g = {g.name for g in self.groups}
        known_c = {c.name for c in cameras}
        known_e = {e.name for e in (self.ee_poses or [])}
        known_j = {j.feature for g in self.groups for j in g.joints}

        for names, known, kind in (
            (ex_g, known_g, 'group'),
            (ex_c, known_c, 'camera'),
            (ex_e, known_e, 'ee_pose'),
            (ex_j, known_j, 'joint'),
        ):
            unknown = [n for n in names if n not in known]
            if unknown:
                raise ValueError(
                    f'Unknown {kind} name(s) in exclude list: {unknown}. '
                    f'Available: {sorted(known)}'
                )

        groups = [
            replace(g, active=False) if g.name in ex_g else g
            for g in self.groups
        ]

        removed_ros_names = []
        if ex_j:
            trimmed = []
            for g in groups:
                kept = [j for j in g.joints if j.feature not in ex_j]
                removed_ros_names.extend(
                    j.ros_name for j in g.joints if j.feature in ex_j
                )
                if not kept:
                    continue
                trimmed.append(replace(g, joints=kept) if len(kept) != len(g.joints) else g)
            groups = trimmed

        ex_e_all = set(ex_e) | set(inactive_ee)
        ee_poses = (
            [e for e in self.ee_poses if e.name not in ex_e_all]
            if self.ee_poses is not None
            else None
        )
        surviving_ee = {e.name for e in ee_poses} if ee_poses is not None else set()
        ee_control = [c for c in self.ee_control if c.ee_pose in surviving_ee]

        # Zero active joint groups is valid only in pure-EE mode, where every
        # arm is driven through a surviving ee_control spec instead.
        if not any(g.active for g in groups) and not ee_control:
            raise ValueError(
                'exclude.groups would deactivate every joint group; '
                'at least one must remain active (or an ee_control spec '
                'must survive for pure-EE action mode).'
            )

        sensors = dict(self.sensors)
        sensors['cameras'] = [
            replace(c, active=False) if c.name in ex_c else c
            for c in cameras
        ]
        excluded_joints = list(self.excluded_joints) + removed_ros_names
        return replace(
            self, groups=groups, sensors=sensors, ee_poses=ee_poses,
            ee_control=ee_control, excluded_joints=excluded_joints,
        )

    # Maps mobile_base feature keys (x.vel/...) to dataset action feature names.
    _BASE_FEATURE_MAP = {v: k for k, v in BASE_KEY_ALIASES.items()}

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


def _abs(desc: Any, rel: Optional[str]) -> str:
    return desc.topic(rel) if rel else ''


def _group_spec(desc: Any, g: Any, o: Dict[str, Any]) -> GroupSpec:
    features = list(o.get('features') or g.joints)
    if len(features) != len(g.joints):
        raise ValueError(
            f'robot_overrides groups.{g.name}.features has {len(features)} names for '
            f'{len(g.joints)} joints {g.joints}')
    return GroupSpec(
        name=g.name,
        command_topic=_abs(desc, g.command_topic),
        command_action=_abs(desc, g.command_action) or None,
        state_topic=_abs(desc, g.state_topic),
        max_joint_delta=float(o.get('max_joint_delta', 0.0)),
        active=bool(o.get('active', True)),
        joints=[JointSpec(ros_name=j, feature=f) for j, f in zip(g.joints, features)],
        relative_exclude=bool(o.get('relative_exclude', False)),
    )


def _mobile_base_spec(desc: Any, o: Dict[str, Any]) -> Optional[MobileBaseSpec]:
    mb = desc.mobile_base
    if mb is None or not o.get('active', True):
        return None
    axes = ('x', 'y', 'z', 'theta')
    default_features = [f'{a}.vel' for a in axes if getattr(mb, f'has_vel_{a}')]
    return MobileBaseSpec(
        name='mobile_base',
        command_topic=_abs(desc, mb.command_topic),
        odom_topic=_abs(desc, mb.odom_topic),
        **{f'has_vel_{a}': bool(getattr(mb, f'has_vel_{a}')) for a in axes},
        **{f'max_vel_{a}': float(o.get(f'max_vel_{a}', 0.0)) for a in axes},
        features=list(o.get('features') or default_features),
        linear_deadband=float(o.get('linear_deadband', 0.0)),
        angular_deadband=float(o.get('angular_deadband', 0.0)),
    )


def _stream_spec(desc: Any, name: str, s: Any, o: Dict[str, Any],
                 is_depth: bool) -> CameraSpec:
    return CameraSpec(
        name=name,
        compressed_topic=_abs(desc, s.compressed_topic),
        raw_topic=_abs(desc, s.raw_topic),
        info_topic=_abs(desc, s.info_topic),
        encoding=str(o.get('encoding', s.encoding) or ''),
        compressed=bool(o.get('compressed', bool(s.compressed_topic))),
        # Depth is opt-in: it multiplies bag size and few policies consume it.
        active=bool(o.get('active', not is_depth)),
        is_depth=is_depth,
    )


def _camera_specs(desc: Any, ov: Dict[str, Any]) -> List[CameraSpec]:
    """v1 shape: colour entry under the camera name, depth as '<name>_depth' with is_depth."""
    out = []
    for cam in desc.cameras:
        o = ov.get(cam.name) or {}
        if cam.color is not None:
            out.append(_stream_spec(desc, cam.name, cam.color, o, is_depth=False))
        if cam.depth is not None:
            out.append(_stream_spec(desc, f'{cam.name}{DEPTH_SUFFIX}', cam.depth,
                                    o.get('depth') or {}, is_depth=True))
    return out


def from_descriptor(desc: Any, overrides: Optional[Mapping[str, Any]] = None) -> RobotDescriptor:
    """Map a shared (schema v2) descriptor plus VLA overrides onto the VLA dataclasses."""
    ov = dict(overrides or {})
    errors = check_overrides(
        ov, [g.name for g in desc.groups], [c.name for c in desc.cameras],
        [e.name for e in desc.ee], desc.mobile_base is not None)
    if errors:
        raise ValueError(f'Invalid robot_overrides for {desc.robot_id!r}:\n  - '
                         + '\n  - '.join(errors))
    group_ov = ov.get('groups') or {}
    ee_ov = ov.get('ee') or {}
    ee_poses = [
        EEPoseSpec(name=e.name, source_frame=desc.frame(e.ee_link),
                   target_frame=desc.frame(e.reference_frame),
                   active=bool((ee_ov.get(e.name) or {}).get('active', True)))
        for e in desc.ee
    ]
    ee_control = [
        EEControlSpec(ee_pose=e.name, group=e.control.group,
                      target_frame=desc.frame(e.control.command_frame),
                      enable_topic=e.control.enable_topic)
        for e in desc.ee if e.control is not None
    ]
    return RobotDescriptor(
        robot_id=desc.robot_id,
        joint_states_topic=desc.topic(desc.joint_states_topic),
        version=desc.version,
        morphology=str(ov.get('morphology', 'mobile_manipulator')),
        groups=[_group_spec(desc, g, group_ov.get(g.name) or {}) for g in desc.groups],
        mobile_base=_mobile_base_spec(desc, ov.get('mobile_base') or {}),
        sensors={'cameras': _camera_specs(desc, ov.get('cameras') or {})},
        ee_poses=ee_poses,
        ee_control=ee_control,
        # Mimic joints appear in joint_states but are never dataset features.
        excluded_joints=desc.all_excluded_joints + [
            j for g in desc.groups for j in g.uncommanded_joints],
    )


def load_robot_descriptor(
    robot_id: str, validate: bool = True, overrides: Optional[Mapping[str, Any]] = None,
) -> RobotDescriptor:
    """
    Load the shared ``<robot_id>.robot.yaml`` and apply the VLA overrides.

    ``overrides`` (a stage config's ``robot_overrides`` block) is merged over
    ``config/robot_overrides_<robot_id>.yaml``.
    """
    srd = import_loader()
    ov = load_robot_overrides(robot_id, overrides)
    package = ov.get('descriptor_package') or f'{robot_id}_description'
    shared = srd.load(robot_id, validate=validate,
                      search_dirs=descriptor_search_dirs(package))
    desc = from_descriptor(shared, ov)
    if validate:
        errors = validate_descriptor(desc)
        if errors:
            raise ValueError('Invalid robot descriptor {}:\n  - {}'.format(
                shared.path, '\n  - '.join(errors)))
    return desc


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

    # 4. ee_control specs must reference real ee poses/groups, non-empty
    # frame/topic, and not double-claim an ee_pose or a group. Unreachable
    # via yaml today (control is nested under its ee[] entry) but kept for
    # dataclasses constructed directly.
    known_ee = {e.name for e in (desc.ee_poses or [])}
    known_groups = {g.name for g in desc.groups}
    ee_poses_seen = []
    groups_seen = []
    for c in desc.ee_control:
        if c.ee_pose not in known_ee:
            errors.append(
                f"ee_control entry references unknown ee_pose '{c.ee_pose}'."
            )
        if c.group not in known_groups:
            errors.append(
                f"ee_control entry references unknown group '{c.group}'."
            )
        if not c.target_frame:
            errors.append(
                f"ee_control entry for ee_pose '{c.ee_pose}' has an empty "
                'target_frame.'
            )
        if not c.enable_topic:
            errors.append(
                f"ee_control entry for ee_pose '{c.ee_pose}' has an empty "
                'enable_topic.'
            )
        ee_poses_seen.append(c.ee_pose)
        groups_seen.append(c.group)

    dup_ee = {n for n in ee_poses_seen if ee_poses_seen.count(n) > 1}
    if dup_ee:
        errors.append(f'Duplicate ee_control ee_pose name(s): {sorted(dup_ee)}')
    dup_groups = {n for n in groups_seen if groups_seen.count(n) > 1}
    if dup_groups:
        errors.append(f'Duplicate ee_control group name(s): {sorted(dup_groups)}')

    return errors
