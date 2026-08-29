#!/usr/bin/env python3
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

from __future__ import annotations

import os
os.environ['PYTORCH_ALLOC_CONF'] = 'expandable_segments:True'

from threading import Lock, Thread  # noqa: E402
from typing import Any, Dict, List, Optional  # noqa: E402

from builtin_interfaces.msg import Duration  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy  # noqa: E402
from sobits_interfaces.srv import VlaCommand, VlaResetWorld, VlaUpdateTask  # noqa: E402
from sobits_vla_common import runtime_deps  # noqa: E402
from sobits_vla_common.image_codec import decode_image_message  # noqa: E402
from sobits_vla_common.lerobot_compat import apply_deploy_patches  # noqa: E402
from sobits_vla_common.param_schema import (  # noqa: E402
    declare_from_schema, P, read_schema, Template,
)
from sobits_vla_deploy.action_chunk_buffer import ActionChunkBuffer  # noqa: E402
from sobits_vla_deploy.action_executor import ActionExecutor  # noqa: E402
from sobits_vla_deploy.action_interpolator import ActionInterpolator  # noqa: E402
from sobits_vla_deploy.episode_logger import EpisodeLogger  # noqa: E402
from sobits_vla_deploy.inference_engine import InferenceEngine  # noqa: E402
from sobits_vla_deploy.obs_builder import ObsBuilder  # noqa: E402
from sobits_vla_deploy.policy_loader import PolicyLoader  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
import tf2_ros  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

apply_deploy_patches()


# Static-name params from _configure_parameters only. robot.*, logging.scene_config's
# YAML-derived baselines, and anything using a ParameterDescriptor stay hand-written.
_SCHEMA = {
    'model': {
        'repo_id': P(''),
        'policy_class': P('lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy'),
        'device': P('cuda'),
        'use_amp': P(True),
        'use_relative_actions': P(False),
        'default_task_label': P(''),
        'dataset_repo_id': P(''),
    },
    'runtime': {
        'control_hz': P(10.0),
        'actions_per_chunk': P(50),
        'chunk_size_threshold': P(0.6),
        'aggregate_fn_name': P('weighted_average'),
        'async_enabled': P(True),
        'single_step_mode': P(False),
        # Chunked-mode Nx control rate via linear interp; 1 = off (default).
        'action_interpolation_multiplier': P(1),
    },
    'rtc': {
        'enabled': P(True),
        'execution_horizon': P(10),
        'max_guidance_weight': P(10.0),
        'prefix_attention_schedule': P('EXP'),
        'inference_delay': P(4),
        'debug': P(False),
    },
    'gamepad': {
        # Gamepad input arrives via the shared GamepadClient node, which
        # calls the VlaCommand service below; no direct /joy subscription.
        'command_service': P('~/command'),
        # Deadman trigger (real robot): actions commanded only while held.
        'controller': P('quest'),
        '<item>': Template('gamepad.controller', {
            'button_mapping': {'deploy': {'safety': {
                'enabled': P(False),
                'trigger_index': P(-4),
                'joy_timeout_s': P(0.5),
            }}},
        }),
    },
    'logging': {
        'enabled': P(False),
        'log_dir': P('/tmp/vla_logs'),
        'scene_config': P(''),
        'scene_preset': P('default'),
    },
    'task': {
        'mode': P('pick'),
        'common': {
            'tilt_threshold_deg': P(30.0),
            'episode_timeout_s': P(60.0),
            # Grace period after PLAY during which a success crossing is
            # ignored, so the world reset settling cannot be scored as a pick.
            'success_settle_s': P(2.0),
            'fall_z_drop_m': P(0.15),
            'robot_model_name': P(''),
            'tracked_model_name': P(''),
        },
        # lift_success_m appears in both: 'place' needs a lift before the
        # object can count as placed.
        'pick': {'lift_success_m': P(0.05)},
        'place': {
            'lift_success_m': P(0.05),
            'goal_model_name': P(''),
            'place_radius_m': P(0.12),
            'place_settle_s': P(1.0),
            # Object must also be below this world z to count as placed
            # (0 = no height condition).
            'place_z_max_m': P(0.0),
            # Abort an episode whose object is lying low and away from the
            # goal for this long (sim seconds). 0 disables.
            'drop_abort_s': P(0.0),
            'drop_abort_z_max_m': P(0.0),
        },
    },
    'reset': {
        'world_service': P('world_reset_node/reset_world'),
        # Scene preset to request. Empty defers to the reset node's
        # world_reset.active_preset; set this only to override it here.
        'preset': P(''),
        # Keep above the reset node's worst case, else resets overlap.
        'timeout_s': P(45.0),
    },
}


class JointGroupConfig:
    def __init__(
        self,
        name: str,
        command_topic: str,
        joints_ros: List[str],
        features: List[str],
        max_joint_delta: float = 0.0,
    ):
        self.name = name
        self.command_topic = command_topic
        self.joints_ros = joints_ros
        self.features = features
        self.max_joint_delta = max_joint_delta


class LeRobotDeployNode(Node):
    def __init__(self) -> None:
        super().__init__('sobits_vla_deploy')

        from sobits_vla_common.lerobot_adapter import describe
        seam = describe()
        self.get_logger().info(
            f'lerobot seam: version={seam["version"]} is_v06={seam["is_v06"]} '
            f'unresolvable={seam["unresolvable"]}'
        )

        self._cb_group = ReentrantCallbackGroup()
        self._lock = Lock()
        self._reset_lock = Lock()
        self._episode_start_lock = Lock()

        self._configure_parameters()
        self._load_robot_profile()

        self._init_policy()
        self._init_collaborators()
        self._init_io()
        self._init_logging()

        self.get_logger().info(
            'Loaded profile {!r} with {} joint features, {} cameras. '
            'Async={}, RTC={}.'.format(
                self._active_profile,
                len(self._joint_features),
                len(self._camera_names),
                self._async_enabled,
                self._rtc_enabled,
            )
        )

    def _init_policy(self) -> None:
        loader = PolicyLoader(
            model_repo_id=self._model_repo_id,
            policy_class_path=self._policy_class_path,
            model_device=self._model_device,
            model_use_amp=self._model_use_amp,
            model_dataset_repo_id=self._model_dataset_repo_id,
            rtc_enabled=self._rtc_enabled,
            rtc_execution_horizon=self._rtc_execution_horizon,
            rtc_max_guidance_weight=self._rtc_max_guidance_weight,
            rtc_prefix_attention_schedule=self._rtc_prefix_attention_schedule,
            rtc_inference_delay=self._rtc_inference_delay,
            rtc_debug=self._rtc_debug,
            control_hz=self._control_hz,
            logger=self.get_logger(),
        )

        bundle = loader.load_policy(self._joint_features, self._mobile_base_features)

        self._policy = bundle.policy
        self._rtc_enabled = bundle.rtc_enabled
        self._model_action_feature_names = bundle.model_action_feature_names
        self._model_use_relative_actions = bundle.model_use_relative_actions
        self._expected_state_dim = bundle.expected_state_dim
        self._preprocessor = bundle.preprocessor
        self._postprocessor = bundle.postprocessor

        # model.use_relative_actions was dead (checkpoint always won silently);
        # enforce that an explicit config value agrees with the checkpoint.
        if (
            self._model_use_relative_actions_set
            and self._model_use_relative_actions_param != self._model_use_relative_actions
        ):
            raise RuntimeError(
                'model.use_relative_actions={} but checkpoint {!r} resolves '
                'to use_relative_actions={} -- refusing to guess which is '
                'correct.'.format(
                    self._model_use_relative_actions_param,
                    self._model_repo_id,
                    self._model_use_relative_actions,
                )
            )

    def _init_collaborators(self) -> None:
        self._obs_builder = ObsBuilder(
            joint_features=self._joint_features,
            mobile_base_features=self._mobile_base_features,
            camera_names=self._camera_names,
        )

        self._play_enabled = False

        self._chunk_buffer = ActionChunkBuffer(self._aggregate_fn_name)
        self._interpolator = ActionInterpolator(self._action_interpolation_multiplier)

        self._task_label = str(self.get_parameter('model.default_task_label').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cmd_vector: Dict[str, float] = {}

        self._inference_engine = InferenceEngine(
            policy=self._policy,
            model_device=self._model_device,
            model_use_amp=self._model_use_amp,
            control_hz=self._control_hz,
            actions_per_chunk=self._actions_per_chunk,
            chunk_size_threshold=self._chunk_size_threshold,
            async_enabled=self._async_enabled,
            single_step_mode=self._single_step_mode,
            rtc_enabled=self._rtc_enabled,
            rtc_inference_delay=self._rtc_inference_delay,
            preprocessor=self._preprocessor,
            postprocessor=self._postprocessor,
            expected_state_dim=self._expected_state_dim,
            model_action_feature_names=self._model_action_feature_names,
            model_use_relative_actions=self._model_use_relative_actions,
            joint_features=self._joint_features,
            mobile_base_features=self._mobile_base_features,
            relative_exclude_features=self._relative_exclude_features,
            logger=self.get_logger(),
        )
        self._inference_engine.update_task_label(self._task_label)

    def _init_subscriptions(self) -> QoSProfile:
        # Split out of _init_io (R2 80-line cap); same construction order.
        qos = QoSProfile(depth=1)

        self._play_sub = self.create_subscription(
            Bool,
            '~/play',
            self._on_play,
            qos,
            callback_group=self._cb_group,
        )
        self._task_sub = self.create_subscription(
            String,
            '~/task',
            self._on_task,
            qos,
            callback_group=self._cb_group,
        )
        self._joint_sub = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_state,
            qos,
            callback_group=self._cb_group,
        )

        self._odom_sub = None
        if self._odom_topic:
            self._odom_sub = self.create_subscription(
                Odometry,
                self._odom_topic,
                self._on_odom,
                qos,
                callback_group=self._cb_group,
            )

        # Camera drivers publish BEST_EFFORT; a RELIABLE subscriber is an
        # incompatible QoS match and silently gets nothing — no error at all.
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._camera_subs = []
        for cam_name, cam_topic in self._camera_topics.items():
            is_compressed = self._camera_compressed.get(cam_name, False)
            msg_type = CompressedImage if is_compressed else Image
            sub = self.create_subscription(
                msg_type,
                cam_topic,
                lambda msg, name=cam_name: self._on_image(msg, name),
                image_qos,
                callback_group=self._cb_group,
            )
            self._camera_subs.append(sub)

        return qos

    def _init_io(self) -> None:
        qos = self._init_subscriptions()

        self._group_publishers = {
            group.name: self.create_publisher(
                JointTrajectory,
                group.command_topic,
                qos,
                callback_group=self._cb_group,
            )
            for group in self._joint_groups
        }

        self._base_pub = None
        if self._mobile_base_cmd_topic:
            self._base_pub = self.create_publisher(
                Twist,
                self._mobile_base_cmd_topic,
                qos,
                callback_group=self._cb_group,
            )

        self._update_task_srv = self.create_service(
            VlaUpdateTask,
            '~/update_task',
            self._on_update_task,
            callback_group=self._cb_group,
        )

        self._command_srv = self.create_service(
            VlaCommand,
            self._command_service,
            self._on_command,
            callback_group=self._cb_group,
        )

        self._step_duration = Duration(
            sec=0,
            nanosec=int((1.0 / (self._control_hz * self._action_interpolation_multiplier)) * 1e9),
        )

        self._action_executor = ActionExecutor(
            joint_groups=self._joint_groups,
            group_publishers=self._group_publishers,
            base_pub=self._base_pub,
            mobile_base_features=self._mobile_base_features,
            max_vel_x=self._max_vel_x,
            max_vel_y=self._max_vel_y,
            max_vel_theta=self._max_vel_theta,
            max_vel_z=self._max_vel_z,
            linear_deadband=self._base_linear_deadband,
            angular_deadband=self._base_angular_deadband,
            step_duration=self._step_duration,
            logger=self.get_logger(),
        )

        self._control_timer = self.create_timer(
            1.0 / (self._control_hz * self._action_interpolation_multiplier),
            self._publish_next_action,
            callback_group=self._cb_group,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._inference_engine.start(
            obs_builder=self._obs_builder,
            chunk_buffer=self._chunk_buffer,
            tf_buffer=self._tf_buffer,
            ee_poses=[(ee.name, ee.source_frame, ee.target_frame) for ee in self._ee_poses],
        )

    def _init_logging(self) -> None:
        self._episode_logger = EpisodeLogger(
            log_dir=self._log_dir,
            world_name=self._log_world_name,
            robot_name=self._log_robot_model,
            block_name=self._log_tracked_model,
            spawn_z=self._log_robot_spawn_z,
            block_z=self._log_tracked_z,
            tilt_threshold_deg=self._log_tilt_deg,
            episode_timeout_s=self._episode_timeout_s,
            lift_success_m=self._lift_success_m,
            success_settle_s=self._success_settle_s,
            goal_name=self._log_goal_model,
            place_radius_m=self._log_place_radius_m,
            place_settle_s=self._log_place_settle_s,
            place_z_max_m=self._log_place_z_max_m,
            drop_abort_s=self._log_drop_abort_s,
            drop_abort_z_max_m=self._log_drop_abort_z_max_m,
            fall_z_drop_m=self._fall_z_drop_m,
            enabled=self._logging_enabled,
            model_repo_id=self._model_repo_id,
            sim_enabled=self._sim_enabled,
            joint_groups={g.name: list(g.features) for g in self._joint_groups},
        )
        if self._logging_enabled:
            self.get_logger().info(
                'Episode logging enabled → {}'.format(self._log_dir)
            )

        # world_reset_node owns teleports + arm reset-pose. Callback runs on
        # the reentrant group so the reset worker thread can block safely.
        self._reset_world_client = self.create_client(
            VlaResetWorld, self._reset_world_service,
            callback_group=self._cb_group,
        )

        # Deadman safety trigger state (Joy timestamped with a monotonic
        # clock so a paused sim can't keep a stale press alive).
        self._last_joy: Optional[Joy] = None
        self._last_joy_rx: float = 0.0
        self._safety_was_pressed = False
        if self._safety_enabled:
            self._joy_sub = self.create_subscription(
                Joy, 'joy', self._on_joy, QoSProfile(depth=10)
            )
            self.get_logger().info(
                'Safety trigger ENABLED (index {}, joy timeout {:.2f}s) — '
                'actions are commanded only while held.'.format(
                    self._safety_trigger_index, self._safety_joy_timeout_s
                )
            )

        # Sim-time of the current episode start, and a topic the experiment
        # runner listens on to know when an episode has auto-terminated.
        self._episode_t0 = None
        self._episode_done_pub = self.create_publisher(
            String, '~/episode_done', QoSProfile(depth=10)
        )

    def destroy_node(self) -> None:
        self._inference_engine.stop()
        self._episode_logger.shutdown()
        super().destroy_node()

    def _configure_parameters(self) -> None:
        declare_from_schema(self, _SCHEMA)
        params = read_schema(self, _SCHEMA)

        self._model_repo_id = str(params.model.repo_id)
        if not self._model_repo_id:
            raise RuntimeError(
                'model.repo_id is required -- refusing to default to a '
                'robot-specific value.'
            )
        self._policy_class_path = str(params.model.policy_class)
        self._model_device = str(params.model.device)
        self._model_use_amp = bool(params.model.use_amp)
        self._model_dataset_repo_id = str(params.model.dataset_repo_id)
        self._model_use_relative_actions_param = bool(params.model.use_relative_actions)
        # Only enforce when the config explicitly set this key -- otherwise
        # it's just the declared default, not an operator claim to check.
        self._model_use_relative_actions_set = (
            'model.use_relative_actions' in (getattr(self, '_parameter_overrides', None) or {})
        )

        self._control_hz = float(params.runtime.control_hz)
        self._actions_per_chunk = int(params.runtime.actions_per_chunk)
        self._chunk_size_threshold = float(params.runtime.chunk_size_threshold)
        self._aggregate_fn_name = str(params.runtime.aggregate_fn_name)
        self._async_enabled = bool(params.runtime.async_enabled)
        self._single_step_mode = bool(params.runtime.single_step_mode)
        self._action_interpolation_multiplier = int(
            params.runtime.action_interpolation_multiplier
        )

        self._rtc_enabled = bool(params.rtc.enabled)
        self._rtc_execution_horizon = int(params.rtc.execution_horizon)
        self._rtc_max_guidance_weight = float(params.rtc.max_guidance_weight)
        self._rtc_prefix_attention_schedule = str(params.rtc.prefix_attention_schedule)
        self._rtc_inference_delay = int(params.rtc.inference_delay)
        self._rtc_debug = bool(params.rtc.debug)

        self._command_service = str(params.gamepad.command_service)

        self._logging_enabled = bool(params.logging.enabled)
        self._log_dir = str(params.logging.log_dir)
        self._log_tilt_deg = float(params.task.common.tilt_threshold_deg)
        self._episode_timeout_s = float(params.task.common.episode_timeout_s)
        self._success_settle_s = float(params.task.common.success_settle_s)
        self._fall_z_drop_m = float(params.task.common.fall_z_drop_m)
        # Sim vs real from use_sim_time: in sim the reset node teleports and
        # the logger polls gz poses; on real hardware the operator re-stages it.
        self._sim_enabled = bool(self.get_parameter('use_sim_time').value)
        self._reset_world_service = str(params.reset.world_service)
        self._reset_preset = str(params.reset.preset or '')
        self._reset_timeout_s = float(params.reset.timeout_s)
        # Template-expanded: safety lives under the controller name Template
        # resolved (gamepad.<controller>.button_mapping.deploy.safety).
        controller_name = str(params.gamepad.controller)
        safety = getattr(params.gamepad, controller_name).button_mapping.deploy.safety
        self._safety_enabled = bool(safety.enabled)
        self._safety_trigger_index = int(safety.trigger_index)
        self._safety_joy_timeout_s = float(safety.joy_timeout_s)
        # Optional: without it, fall detection is unavailable but the EE-frame
        # metrics and lift/place scoring still work.
        self._log_robot_model = str(params.task.common.robot_model_name)
        self._log_tracked_model = str(params.task.common.tracked_model_name)
        # Only the selected mode's block is read, so a stale goal under
        # task.place can't leak into 'pick'. Unknown mode is a config error.
        self._task_mode = str(params.task.mode).strip().lower()
        if self._task_mode not in ('pick', 'place'):
            raise RuntimeError(
                'task.mode must be "pick" or "place", got {!r}.'.format(
                    self._task_mode
                )
            )
        self._lift_success_m = float(
            getattr(params.task, self._task_mode).lift_success_m
        )
        self._log_goal_model = ''
        self._log_place_radius_m = float(params.task.place.place_radius_m)
        self._log_place_z_max_m = float(params.task.place.place_z_max_m)
        self._log_drop_abort_s = float(params.task.place.drop_abort_s)
        self._log_drop_abort_z_max_m = float(params.task.place.drop_abort_z_max_m)
        self._log_place_settle_s = float(params.task.place.place_settle_s)
        if self._task_mode == 'place':
            self._log_goal_model = str(params.task.place.goal_model_name)
            if not self._log_goal_model:
                raise RuntimeError(
                    'task.mode is "place" but task.place.goal_model_name is '
                    'unset -- there is nothing to place into.'
                )
        self._load_scene_baselines()
        self._actions_per_chunk = max(self._actions_per_chunk, 1)
        self._control_hz = max(self._control_hz, 1.0)
        self._chunk_size_threshold = min(max(self._chunk_size_threshold, 0.0), 1.0)
        self._action_interpolation_multiplier = max(self._action_interpolation_multiplier, 1)

    def _load_robot_profile(self) -> None:
        self.declare_parameter('robot.descriptor_id', '')
        desc_id = str(self.get_parameter('robot.descriptor_id').value)

        if not desc_id:
            raise RuntimeError(
                'robot.descriptor_id is unset -- refusing to default to a '
                'robot-specific profile. Set it to a descriptor id under '
                'sobits_vla_common/robots/<id>.robot.yaml.'
            )

        from sobits_vla_common.robot_descriptor import load_robot_descriptor
        desc = load_robot_descriptor(desc_id)
        self._active_profile = desc_id

        # Trim the shared descriptor to the subset this model drives. Unknown
        # names raise, so a typo fails loudly instead of running a wrong body.
        self.declare_parameter('robot.exclude.groups', [''])
        self.declare_parameter('robot.exclude.cameras', [''])
        self.declare_parameter('robot.exclude.ee_poses', [''])
        self.declare_parameter('robot.exclude.joints', [''])
        self.declare_parameter('robot.exclude.mobile_base', False)

        desc = desc.filtered(
            exclude_groups=self._str_list('robot.exclude.groups'),
            exclude_cameras=self._str_list('robot.exclude.cameras'),
            exclude_ee_poses=self._str_list('robot.exclude.ee_poses'),
            exclude_joints=self._str_list('robot.exclude.joints'),
        )
        active_groups_list = [g.name for g in desc.active_groups]
        active_cameras_list = [c.name for c in desc.active_cameras]
        active_mobile_base = not bool(
            self.get_parameter('robot.exclude.mobile_base').value
        )

        self._joint_states_topic = desc.joint_states_topic
        self._joint_groups = []
        self._joint_features = []
        self._joint_feature_to_ros = {}
        self._ee_poses = desc.ee_poses or []

        for group in desc.groups:
            if group.name in active_groups_list:
                joints_ros = [j.ros_name for j in group.joints]
                features = [j.feature for j in group.joints]
                self._joint_groups.append(
                    JointGroupConfig(
                        name=group.name,
                        command_topic=group.command_topic,
                        joints_ros=joints_ros,
                        features=features,
                        max_joint_delta=group.max_joint_delta,
                    )
                )
                for ros_name, feature in zip(joints_ros, features):
                    self._joint_features.append(feature)
                    self._joint_feature_to_ros[feature] = ros_name

        if desc.mobile_base and active_mobile_base:
            self._odom_topic = desc.mobile_base.odom_topic
            self._mobile_base_cmd_topic = desc.mobile_base.command_topic
            self._mobile_base_features = desc.mobile_base.features
            self._max_vel_x = desc.mobile_base.max_vel_x
            self._max_vel_y = desc.mobile_base.max_vel_y
            self._max_vel_theta = desc.mobile_base.max_vel_theta
            self._max_vel_z = desc.mobile_base.max_vel_z
            self._base_linear_deadband = desc.mobile_base.linear_deadband
            self._base_angular_deadband = desc.mobile_base.angular_deadband
        else:
            self._odom_topic = ''
            self._mobile_base_cmd_topic = ''
            self._mobile_base_features = []
            self._max_vel_x = 0.0
            self._max_vel_y = 0.0
            self._max_vel_theta = 0.0
            self._max_vel_z = 0.0
            self._base_linear_deadband = 0.0
            self._base_angular_deadband = 0.0

        self._relative_exclude_features = desc.relative_exclude_features(
            active_groups=active_groups_list,
            active_mobile_base=active_mobile_base,
        )

        self._camera_names = []
        self._camera_topics = {}
        self._camera_encodings = {}
        self._camera_compressed = {}

        for cam in desc.sensors.get('cameras', []):
            if cam.name in active_cameras_list:
                self._camera_names.append(cam.name)
                if cam.compressed and cam.compressed_topic:
                    self._camera_topics[cam.name] = cam.compressed_topic
                    self._camera_compressed[cam.name] = True
                else:
                    self._camera_topics[cam.name] = cam.raw_topic
                    self._camera_compressed[cam.name] = False
                self._camera_encodings[cam.name] = cam.encoding if cam.encoding else 'rgb8'

    def _load_scene_baselines(self) -> None:
        """
        Read world name and rest heights from the world_reset scene YAML.

        Sharing that file with world_reset_node keeps the lift/fall baselines
        pinned to the same poses the reset teleports to. Required whenever
        episode logging runs against Gazebo: a guessed baseline silently
        mis-scores lift-success and fall termination for every episode.
        """
        # Placeholders only: any run that actually scores lifts overwrites
        # these from the scene YAML or raises below.
        self._log_world_name = ''
        self._log_tracked_z = 0.0
        self._log_robot_spawn_z = 0.0

        # Baselines are read from Gazebo poses, so they only matter for a
        # logged sim run; a real-robot or logging-off run needs no scene.
        scoring_required = self._logging_enabled and self._sim_enabled

        path = str(self.get_parameter('logging.scene_config').value)
        if not path or not os.path.isfile(path):
            if scoring_required:
                raise RuntimeError(
                    'logging.enabled=true with use_sim_time=true but '
                    'logging.scene_config is {} -- refusing to score episodes '
                    'against a guessed lift baseline. Point it at the '
                    'world_reset scene YAML used for this run.'.format(
                        'unset' if not path else '{!r} (not found)'.format(path)
                    )
                )
            return

        import yaml
        with open(path, 'r') as f:
            params = (yaml.safe_load(f) or {}).get('/**', {}).get(
                'ros__parameters', {})
        scene = params.get('world_reset', {})
        self._log_world_name = str(scene.get('world_name', self._log_world_name))

        preset_name = str(self.get_parameter('logging.scene_preset').value)
        preset = scene.get(preset_name, {})
        for name, attr in (
            (self._log_tracked_model, '_log_tracked_z'),
            (self._log_robot_model, '_log_robot_spawn_z'),
        ):
            pose = (preset.get(name) or {}).get('pose') if name else None
            if pose is None or 'z' not in pose:
                if scoring_required and name:
                    raise RuntimeError(
                        'Model {!r} has no pose.z in scene preset {!r} of {} -- '
                        'cannot derive a lift/fall baseline.'.format(
                            name, preset_name, os.path.basename(path)
                        )
                    )
                continue
            setattr(self, attr, float(pose['z']))

        self.get_logger().info(
            'Logger baselines from {}: world={!r}, {}.z={:.4f}, {}.z={:.4f}'.format(
                os.path.basename(path), self._log_world_name,
                self._log_tracked_model, self._log_tracked_z,
                self._log_robot_model, self._log_robot_spawn_z,
            )
        )

    def _str_list(self, name: str) -> List[str]:
        """Read a string-array parameter, dropping the empty-default sentinel."""
        raw = self.get_parameter(name).get_parameter_value().string_array_value
        return [s for s in raw if s]

    def _on_set_parameters(self, params: List[Any]) -> Any:
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'runtime.chunk_size_threshold':
                self._chunk_size_threshold = float(min(max(p.value, 0.0), 1.0))
                self._inference_engine.chunk_size_threshold = self._chunk_size_threshold
                self.get_logger().info(
                    'chunk_size_threshold → {}'.format(self._chunk_size_threshold)
                )
            elif p.name == 'runtime.aggregate_fn_name':
                self._aggregate_fn_name = str(p.value)
                self._chunk_buffer._aggregate_fn_name = self._aggregate_fn_name
                self.get_logger().info('aggregate_fn_name → {}'.format(self._aggregate_fn_name))
            elif p.name == 'runtime.async_enabled':
                self._async_enabled = bool(p.value)
                self._inference_engine.async_enabled = self._async_enabled
                self.get_logger().info('async_enabled → {}'.format(self._async_enabled))
            elif p.name == 'runtime.control_hz':
                hz = float(max(p.value, 1.0))
                self._control_hz = hz
                period_ns = int((1.0 / (hz * self._action_interpolation_multiplier)) * 1e9)
                self._control_timer.timer_period_ns = period_ns
                self._step_duration = Duration(sec=0, nanosec=period_ns)
                self._action_executor.step_duration = self._step_duration
                self._inference_engine.control_hz = hz
                self.get_logger().info('control_hz → {}'.format(hz))
            elif p.name == 'runtime.actions_per_chunk':
                self._actions_per_chunk = max(int(p.value), 1)
                self._inference_engine.actions_per_chunk = self._actions_per_chunk
                self.get_logger().info('actions_per_chunk → {}'.format(self._actions_per_chunk))
            elif p.name == 'runtime.single_step_mode':
                self._single_step_mode = bool(p.value)
                self._inference_engine.single_step_mode = self._single_step_mode
                self.get_logger().info('single_step_mode → {}'.format(self._single_step_mode))
            elif p.name == 'runtime.action_interpolation_multiplier':
                mult = max(int(p.value), 1)
                self._action_interpolation_multiplier = mult
                self._interpolator.multiplier = mult
                self._interpolator.reset()
                period_ns = int((1.0 / (self._control_hz * mult)) * 1e9)
                self._control_timer.timer_period_ns = period_ns
                self._step_duration = Duration(sec=0, nanosec=period_ns)
                self._action_executor.step_duration = self._step_duration
                self.get_logger().info('action_interpolation_multiplier → {}'.format(mult))
            elif p.name == 'logging.enabled':
                self._logging_enabled = bool(p.value)
                self._episode_logger.enabled = self._logging_enabled
                self.get_logger().info('logging.enabled → {}'.format(self._logging_enabled))
            elif p.name == 'task.common.tilt_threshold_deg':
                import math as _math
                self._log_tilt_deg = float(p.value)
                self._episode_logger._tilt_rad = _math.radians(self._log_tilt_deg)
                self.get_logger().info(
                    'task.common.tilt_threshold_deg → {}'.format(self._log_tilt_deg)
                )
            elif p.name == 'task.common.episode_timeout_s':
                self._episode_timeout_s = float(p.value)
                self._episode_logger._episode_timeout_s = self._episode_timeout_s
                self.get_logger().info(
                    'task.common.episode_timeout_s → {}'.format(self._episode_timeout_s)
                )
            elif p.name == 'task.{}.lift_success_m'.format(self._task_mode):
                self._lift_success_m = float(p.value)
                self._episode_logger._lift_success_m = self._lift_success_m
                self.get_logger().info(
                    'task.{}.lift_success_m → {}'.format(self._task_mode, self._lift_success_m)
                )
            elif p.name == 'task.common.success_settle_s':
                self._success_settle_s = float(p.value)
                self._episode_logger._success_settle_s = self._success_settle_s
                self.get_logger().info(
                    'task.common.success_settle_s → {}'.format(self._success_settle_s)
                )
            elif p.name == 'task.common.fall_z_drop_m':
                self._fall_z_drop_m = float(p.value)
                self._episode_logger._fall_z_drop_m = self._fall_z_drop_m
                self.get_logger().info(
                    'task.common.fall_z_drop_m → {}'.format(self._fall_z_drop_m)
                )
        return SetParametersResult(successful=True)

    def _reset_episode_state(self, outcome: Optional[str] = None) -> None:
        """
        Reset all per-episode model state after a stop/world-reset.

        When ``outcome`` is given, ~/episode_done is published by the
        reset thread AFTER the world/pose reset completes — so a listener
        (the experiment runner) knows the system is ready for the next
        PLAY without any time-based settle.
        """
        self._chunk_buffer.clear()
        self._interpolator.reset()
        if hasattr(self._policy, 'reset'):
            self._policy.reset()
        for pipeline in (self._preprocessor, self._postprocessor):
            if pipeline is not None:
                for step in pipeline.steps:
                    if hasattr(step, 'reset'):
                        step.reset()
        with self._lock:
            self._cmd_vector.clear()
        self._obs_builder.clear_prev_ee_pose()
        self._inference_engine.clear_single_step_result()
        self.get_logger().info('Episode model state reset.')
        Thread(target=self._do_world_reset, args=(outcome,), daemon=True).start()

    @staticmethod
    def _wait_future(future, timeout_s: float) -> bool:
        """Block a worker thread until an rclpy future resolves (executor spins it)."""
        from threading import Event
        done = Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout=timeout_s)

    def _do_world_reset(self, outcome: Optional[str] = None) -> None:
        """
        Reset the scene via the shared world_reset_node; blocking.

        Serialized: a second reset landing mid-flight would race the first on
        the same set_pose service and both would report failures.
        """
        with self._reset_lock:
            try:
                self._run_world_reset()
            finally:
                if outcome is not None:
                    self._publish_episode_done(outcome)

    def _run_world_reset(self) -> None:
        """Call VlaResetWorld on the shared reset node; blocking."""
        if not self._reset_world_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'World reset service {!r} unavailable — scene NOT reset.'.format(
                    self._reset_world_service
                )
            )
            return

        req = VlaResetWorld.Request()
        # Empty -> the reset node's world_reset.active_preset decides.
        req.preset = self._reset_preset
        fut = self._reset_world_client.call_async(req)
        # Must exceed the reset node's own budget (pose call + settle wait +
        # teleports); a shorter timeout here just starts a competing reset.
        if not self._wait_future(fut, timeout_s=self._reset_timeout_s):
            self.get_logger().error(
                'World reset call timed out after {:.0f}s.'.format(
                    self._reset_timeout_s
                )
            )
            return

        result = fut.result()
        if result is None:
            self.get_logger().error('World reset call returned no response.')
            return
        self.get_logger().info(
            'World reset: {} ({})'.format(
                'OK' if result.success else 'FAIL', result.message
            )
        )

    def _on_update_task(
        self,
        request: VlaUpdateTask.Request,
        response: VlaUpdateTask.Response,
    ) -> VlaUpdateTask.Response:
        self._chunk_buffer.clear()
        self._interpolator.reset()
        if hasattr(self._policy, 'reset'):
            self._policy.reset()
        self._task_label = request.label
        self._inference_engine.update_task_label(request.label)
        self.get_logger().info('Task label updated to {!r}.'.format(request.label))
        response.success = True
        response.message = 'succeeded'
        return response

    def _do_begin_episode(self) -> None:
        """Run begin_episode's blocking gz pose reads off the callback thread."""
        with self._episode_start_lock:
            self._episode_logger.begin_episode()
            self._inference_engine.update_play_enabled(True)

    def _start_play(self) -> bool:
        """Atomically start PLAY if not already running. Returns True if it started."""
        with self._lock:
            if self._play_enabled:
                return False
            self._cmd_vector = dict(self._obs_builder.state_vector)
            # With safety trigger enabled, start the episode clock on first
            # engagement, not at PLAY — the robot stays frozen till then.
            self._episode_t0 = (
                None if self._safety_enabled else self.get_clock().now()
            )
            self._play_enabled = True
        Thread(target=self._do_begin_episode, daemon=True).start()
        return True

    def _stop_play(self, outcome: str) -> bool:
        """Atomically stop PLAY if running. Returns True if it stopped."""
        with self._lock:
            if not self._play_enabled:
                return False
            self._play_enabled = False
        # Serialize against an in-flight _do_begin_episode so end_episode()
        # cannot land before begin_episode() has opened the episode file.
        with self._episode_start_lock:
            self._episode_logger.end_episode(outcome)
        self._inference_engine.update_play_enabled(False)
        self._reset_episode_state(outcome=outcome)
        return True

    def _on_command(
        self,
        request: VlaCommand.Request,
        response: VlaCommand.Response,
    ) -> VlaCommand.Response:
        cmd = request.command
        if cmd == VlaCommand.Request.PLAY:
            if self._start_play():
                self.get_logger().info('VLA execution started via service command PLAY.')
            response.success = True
            response.message = 'PLAY execution enabled'
            response.status = VlaCommand.Response.STATE_PLAYING
        elif cmd == VlaCommand.Request.STOP:
            if self._stop_play('manual_stop'):
                self.get_logger().info('VLA execution stopped via service command STOP.')
            else:
                # Idle STOP resets the world to start pose — the experiment
                # runner issues this before episode 1 to avoid a stale pose.
                self.get_logger().info('STOP while idle → resetting world to start pose.')
                self._reset_episode_state(outcome='reset')
            response.success = True
            response.message = 'STOP execution disabled'
            response.status = VlaCommand.Response.STATE_STOPPED
        elif cmd == VlaCommand.Request.RESET:
            # _stop_play already resets the world as part of stopping; only
            # trigger a standalone reset when play was not running.
            if self._stop_play('manual_stop'):
                self.get_logger().info('World reset requested via RESET (was playing).')
            else:
                self.get_logger().info('World reset requested via RESET (was idle).')
                self._reset_episode_state(outcome='reset')
            response.success = True
            response.message = 'World reset triggered'
            response.status = VlaCommand.Response.STATE_STOPPED
        else:
            response.success = False
            response.message = f'Command code {cmd} not supported in deploy stage.'
            response.status = (
                VlaCommand.Response.STATE_PLAYING if self._play_enabled
                else VlaCommand.Response.STATE_STOPPED
            )
        return response

    def _on_play(self, msg: Bool) -> None:
        if msg.data:
            if self._start_play():
                self.get_logger().info('VLA execution started via ~/play topic.')
        else:
            if self._stop_play('manual_stop'):
                self.get_logger().info('VLA execution stopped via ~/play topic.')

    def _auto_stop_episode(self, outcome: str) -> None:
        """Terminate the current episode automatically and notify the runner."""
        self.get_logger().info(
            'Episode auto-terminated: {}. Resetting world.'.format(outcome)
        )
        # Stop the base immediately so the robot does not drift during reset.
        if self._base_pub is not None:
            self._base_pub.publish(Twist())
        self._stop_play(outcome)

    def _publish_episode_done(self, outcome: str) -> None:
        self._episode_t0 = None
        msg = String()
        msg.data = outcome
        self._episode_done_pub.publish(msg)

    def _on_task(self, msg: String) -> None:
        label = msg.data.strip()
        if label:
            self._chunk_buffer.clear()
            self._interpolator.reset()
            if hasattr(self._policy, 'reset'):
                self._policy.reset()
            self._task_label = label
            self._inference_engine.update_task_label(label)
            self.get_logger().info('Task label updated to {!r} via ~/task topic.'.format(label))

    def _on_joint_state(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        for feature, ros_joint in self._joint_feature_to_ros.items():
            if ros_joint in by_name:
                self._obs_builder.update_joint_state(feature, float(by_name[ros_joint]))

    def _on_odom(self, msg: Odometry) -> None:
        if 'x.vel' in self._mobile_base_features:
            self._obs_builder.update_odom('x.vel', float(msg.twist.twist.linear.x))
        if 'y.vel' in self._mobile_base_features:
            self._obs_builder.update_odom('y.vel', float(msg.twist.twist.linear.y))
        if 'z.vel' in self._mobile_base_features:
            self._obs_builder.update_odom('z.vel', float(msg.twist.twist.linear.z))
        if 'theta.vel' in self._mobile_base_features:
            self._obs_builder.update_odom('theta.vel', float(msg.twist.twist.angular.z))

    def _on_image(self, msg: Any, cam_name: str) -> None:
        encoding = self._camera_encodings.get(cam_name, 'rgb8')
        is_compressed = self._camera_compressed.get(cam_name, False)
        # cv_bridge segfaults under this env's NumPy 2; avoid it entirely.
        try:
            image = decode_image_message(msg)
        except Exception as exc:
            image = None
            decode_exc = exc
        else:
            decode_exc = None
        if image is None:
            self.get_logger().warning(
                'Image decode failed for {!r} (encoding={!r}, compressed={}): {}'.format(
                    cam_name, encoding, is_compressed, decode_exc
                ),
                throttle_duration_sec=5.0,
            )
            return
        self._obs_builder.update_image(cam_name, image)

    def _on_joy(self, msg: Joy) -> None:
        from time import monotonic
        self._last_joy = msg
        self._last_joy_rx = monotonic()

    def _safety_pressed(self) -> bool:
        """Deadman state; released when stale, missing, or not held."""
        from time import monotonic
        if self._last_joy is None:
            return False
        if monotonic() - self._last_joy_rx > self._safety_joy_timeout_s:
            return False
        idx = self._safety_trigger_index
        try:
            if idx < 0:
                return float(self._last_joy.axes[abs(idx)]) > 0.5
            return int(self._last_joy.buttons[idx]) != 0
        except IndexError:
            return False

    def _publish_next_action(self) -> None:
        if not self._play_enabled:
            if self._base_pub is not None:
                cmd = Twist()
                self._base_pub.publish(cmd)
            return

        # Auto-termination (success/fall/timeout) checked before consuming the
        # queue. Snapshot to a local: the reset thread can null this mid-check.
        episode_t0 = self._episode_t0
        if self._logging_enabled and episode_t0 is not None:
            elapsed = (self.get_clock().now() - episode_t0).nanoseconds * 1e-9
            outcome = self._episode_logger.evaluate_termination(elapsed)
            if outcome is not None:
                self._auto_stop_episode(outcome)
                return

        if self._safety_enabled:
            if not self._safety_pressed():
                if self._safety_was_pressed:
                    self.get_logger().warning(
                        'Safety trigger released — holding commands.'
                    )
                    self._safety_was_pressed = False
                if self._base_pub is not None:
                    self._base_pub.publish(Twist())
                return
            if not self._safety_was_pressed:
                # (Re)engaged: drop actions queued while held so execution
                # resumes only with freshly inferred chunks.
                self._chunk_buffer.clear()
                self._interpolator.reset()
                self._safety_was_pressed = True
                if self._episode_t0 is None:
                    # Deferred episode clock: timing/timeout start now, not
                    # while the scene was staged with the trigger released.
                    self._episode_t0 = self.get_clock().now()
                self.get_logger().info(
                    'Safety trigger engaged — resuming with fresh actions.'
                )
                return

        if self._single_step_mode:
            step = self._inference_engine.pop_single_step_result()
            if step is None:
                self.get_logger().warning(
                    'Single-step inference not ready yet.',
                    throttle_duration_sec=2.0,
                )
                if self._base_pub is not None:
                    cmd = Twist()
                    self._base_pub.publish(cmd)
                return
        else:
            queue_len = self._chunk_buffer.size()
            threshold_len = max(1, int(self._actions_per_chunk * self._chunk_size_threshold))
            if self._async_enabled and queue_len <= threshold_len:
                self._inference_engine.update_play_enabled(True)

            if not self._interpolator.enabled:
                step = self._chunk_buffer.pop()
            else:
                if self._interpolator.needs_new_action():
                    popped = self._chunk_buffer.pop()
                    if popped is not None:
                        self._interpolator.add(popped)
                step = self._interpolator.get()
            if step is None:
                self.get_logger().warning(
                    'Action queue empty. Increase actions_per_chunk or lower control_hz.',
                    throttle_duration_sec=2.0,
                )
                if self._base_pub is not None:
                    cmd = Twist()
                    self._base_pub.publish(cmd)
                return

        now = self.get_clock().now().to_msg()
        joint_log, base_log = self._action_executor.execute_action(
            step=step,
            state_vector=self._obs_builder.state_vector,
            cmd_vector=self._cmd_vector,
            now_msg=now,
        )

        if self._logging_enabled:
            self._log_step(step, joint_log, base_log)

        self.get_logger().debug('CMD -> {}{}'.format(joint_log, base_log))

    def _log_step(self, step, joint_log, base_log) -> None:
        log_joints: Dict[str, float] = {}
        log_joints_measured: Dict[str, float] = {}
        measured_state = self._obs_builder.state_vector
        for group in self._joint_groups:
            for feat in group.features:
                log_joints[feat] = float(self._cmd_vector.get(feat, 0.0))
                log_joints_measured[feat] = float(measured_state.get(feat, 0.0))
        log_base = {
            'x': float(step.get('x.vel', 0.0)) if step else 0.0,
            'y': float(step.get('y.vel', 0.0)) if step else 0.0,
            'theta': float(step.get('theta.vel', 0.0)) if step else 0.0,
        }
        # Logger takes one EE pose; use the descriptor's first entry.
        ee = None
        if self._ee_poses:
            first = self._ee_poses[0]
            ee = self._obs_builder.get_ee_pose(
                self._tf_buffer, first.target_frame, first.source_frame
            )
        self._episode_logger.log_step(
            joints=log_joints,
            base_vel=log_base,
            ee_pose=ee.tolist() if ee is not None else None,
            joints_measured=log_joints_measured,
        )


def main(args: Optional[List[str]] = None) -> None:
    # Checked here, not at module import, so lint/pytest collection of this
    # package still works on environments without the ML stack installed.
    runtime_deps.ensure({
        'lerobot': 'pip install lerobot[training]~=0.6.0',
        'huggingface_hub': 'pip install huggingface_hub',
        'safetensors': 'pip install lerobot[training]~=0.6.0',
    })
    rclpy.init(args=args)
    node = LeRobotDeployNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # On SIGINT (launch teardown) rclpy's signal handler may already have
        # shut down the context; calling shutdown() again raises RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
