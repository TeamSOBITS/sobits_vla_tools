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
import cv2  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
import rclpy.duration  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
import rclpy.time  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy  # noqa: E402
from sobits_interfaces.action import MoveToPose  # noqa: E402
from sobits_interfaces.srv import VlaCommand, VlaUpdateTask  # noqa: E402
from sobits_vla_common import runtime_deps  # noqa: E402
from sobits_vla_common.lerobot_compat import apply_deploy_patches  # noqa: E402
from sobits_vla_deploy.action_chunk_buffer import ActionChunkBuffer  # noqa: E402
from sobits_vla_deploy.action_executor import ActionExecutor  # noqa: E402
from sobits_vla_deploy.inference_engine import InferenceEngine  # noqa: E402
from sobits_vla_deploy.obs_builder import ObsBuilder  # noqa: E402
from sobits_vla_deploy.policy_loader import PolicyLoader  # noqa: E402
from sobits_vla_deploy.vla_episode_logger import EpisodeLogger  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
import tf2_ros  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

apply_deploy_patches()


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
        self._bridge = CvBridge()

        self._configure_parameters()
        self._load_robot_profile()

        # Build policy config and load policy
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

        loaded = loader.load_policy(self._joint_features, self._mobile_base_features)

        self._policy = loaded['policy']
        self._rtc_enabled = loaded['rtc_enabled']
        self._model_action_feature_names = loaded['model_action_feature_names']
        self._model_use_relative_actions = loaded['model_use_relative_actions']
        self._expected_state_dim = loaded['expected_state_dim']
        self._preprocessor = loaded['preprocessor']
        self._postprocessor = loaded['postprocessor']

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

        # Initialize ObsBuilder
        self._obs_builder = ObsBuilder(
            joint_features=self._joint_features,
            mobile_base_features=self._mobile_base_features,
            camera_names=self._camera_names,
        )

        self._play_enabled = False

        # Action chunk buffer
        self._chunk_buffer = ActionChunkBuffer(self._aggregate_fn_name)

        self._task_label = str(self.get_parameter('model.default_task_label').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cmd_vector: Dict[str, float] = {}

        # Initialize InferenceEngine
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
            logger=self.get_logger(),
        )
        self._inference_engine.update_task_label(self._task_label)

        qos = QoSProfile(depth=1)

        self._play_sub = self.create_subscription(
            Bool,
            '/vla/play',
            self._on_play,
            qos,
            callback_group=self._cb_group,
        )
        self._task_sub = self.create_subscription(
            String,
            '/vla/task',
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

        # Camera drivers (orbbec, realsense, ...) publish sensor data
        # BEST_EFFORT. A RELIABLE subscriber is an incompatible QoS match and
        # silently receives NOTHING -- no error, no callback, just a policy
        # that never sees an image. Sensor QoS is required here.
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
            '/vla/update_task',
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
            nanosec=int((1.0 / self._control_hz) * 1e9),
        )

        # Initialize ActionExecutor
        self._action_executor = ActionExecutor(
            joint_groups=self._joint_groups,
            group_publishers=self._group_publishers,
            base_pub=self._base_pub,
            mobile_base_features=self._mobile_base_features,
            max_vel_x=self._max_vel_x,
            max_vel_y=self._max_vel_y,
            max_vel_theta=self._max_vel_theta,
            step_duration=self._step_duration,
            logger=self.get_logger(),
        )

        self._control_timer = self.create_timer(
            1.0 / self._control_hz,
            self._publish_next_action,
            callback_group=self._cb_group,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Start InferenceEngine
        self._inference_engine.start(
            obs_builder=self._obs_builder,
            chunk_buffer=self._chunk_buffer,
            tf_buffer=self._tf_buffer,
            ee_poses=[(ee.name, ee.source_frame, ee.target_frame) for ee in self._ee_poses],
        )

        sx, sy, sz, sqx, sqy, sqz, sqw = self._sim_spawn
        bx, by, bz = self._sim_block_reset
        self._episode_logger = EpisodeLogger(
            log_dir=self._log_dir,
            world_name=self._sim_world_name,
            robot_name=self._sim_robot_model,
            block_name=self._sim_block_model,
            spawn_x=sx, spawn_y=sy, spawn_z=sz,
            spawn_qx=sqx, spawn_qy=sqy, spawn_qz=sqz, spawn_qw=sqw,
            block_x=bx, block_y=by, block_z=bz,
            tilt_threshold_deg=self._log_tilt_deg,
            episode_timeout_s=self._episode_timeout_s,
            lift_success_m=self._lift_success_m,
            fall_z_drop_m=self._fall_z_drop_m,
            enabled=self._logging_enabled,
            model_repo_id=self._model_repo_id,
            sim_enabled=self._sim_enabled,
        )
        if self._logging_enabled:
            self.get_logger().info(
                'Episode logging enabled → {}'.format(self._log_dir)
            )

        # Reset pose action client — callbacks run on the node's reentrant
        # group, served by the spinning MultiThreadedExecutor, so the reset
        # worker thread can block on the futures safely.
        self._reset_pose_client = ActionClient(
            self, MoveToPose, self._reset_action_name,
            callback_group=self._cb_group,
        )
        # Lazy ros_gz SetEntityPose client (sim teleports); falls back to
        # the gz CLI when the bridge doesn't expose the service.
        self._set_pose_client = None

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
            String, '/vla/episode_done', QoSProfile(depth=10)
        )

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

    def destroy_node(self) -> None:
        self._inference_engine.stop()
        self._episode_logger.shutdown()
        super().destroy_node()

    def _configure_parameters(self) -> None:
        self.declare_parameter('model.repo_id', '')
        self.declare_parameter(
            'model.policy_class',
            'lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy',
        )
        self.declare_parameter('model.device', 'cuda')
        self.declare_parameter('model.use_amp', True)
        self.declare_parameter('model.use_relative_actions', False)
        self.declare_parameter('model.default_task_label', '')
        self.declare_parameter('model.dataset_repo_id', '')

        self.declare_parameter('runtime.control_hz', 10.0)
        self.declare_parameter('runtime.actions_per_chunk', 50)
        self.declare_parameter('runtime.chunk_size_threshold', 0.6)
        self.declare_parameter('runtime.aggregate_fn_name', 'weighted_average')
        self.declare_parameter('runtime.async_enabled', True)
        self.declare_parameter('runtime.single_step_mode', False)

        self.declare_parameter('rtc.enabled', True)
        self.declare_parameter('rtc.execution_horizon', 10)
        self.declare_parameter('rtc.max_guidance_weight', 10.0)
        self.declare_parameter('rtc.prefix_attention_schedule', 'EXP')
        self.declare_parameter('rtc.inference_delay', 4)
        self.declare_parameter('rtc.debug', False)

        # Gamepad input arrives via the shared GamepadClient node
        # (sobits_vla_common), which calls the VlaCommand service below.
        # This node no longer subscribes to /joy directly.
        self.declare_parameter('gamepad.command_service', '/vla/command')

        self._model_repo_id = str(self.get_parameter('model.repo_id').value)
        if not self._model_repo_id:
            raise RuntimeError(
                'model.repo_id is required -- refusing to default to a '
                'robot-specific value.'
            )
        self._policy_class_path = str(self.get_parameter('model.policy_class').value)
        self._model_device = str(self.get_parameter('model.device').value)
        self._model_use_amp = bool(self.get_parameter('model.use_amp').value)
        self._model_dataset_repo_id = str(self.get_parameter('model.dataset_repo_id').value)
        self._model_use_relative_actions_param = bool(
            self.get_parameter('model.use_relative_actions').value
        )
        # Only enforce when the config explicitly set this key -- otherwise
        # it's just the declared default, not an operator claim to check.
        self._model_use_relative_actions_set = (
            'model.use_relative_actions' in (getattr(self, '_parameter_overrides', None) or {})
        )

        self._control_hz = float(self.get_parameter('runtime.control_hz').value)
        self._actions_per_chunk = int(self.get_parameter('runtime.actions_per_chunk').value)
        self._chunk_size_threshold = float(
            self.get_parameter('runtime.chunk_size_threshold').value
        )
        self._aggregate_fn_name = str(self.get_parameter('runtime.aggregate_fn_name').value)
        self._async_enabled = bool(self.get_parameter('runtime.async_enabled').value)
        self._single_step_mode = bool(
            self.get_parameter('runtime.single_step_mode').value
        )

        self._rtc_enabled = bool(self.get_parameter('rtc.enabled').value)
        self._rtc_execution_horizon = int(self.get_parameter('rtc.execution_horizon').value)
        self._rtc_max_guidance_weight = float(
            self.get_parameter('rtc.max_guidance_weight').value
        )
        self._rtc_prefix_attention_schedule = str(
            self.get_parameter('rtc.prefix_attention_schedule').value
        )
        self._rtc_inference_delay = int(self.get_parameter('rtc.inference_delay').value)
        self._rtc_debug = bool(self.get_parameter('rtc.debug').value)

        self._command_service = str(
            self.get_parameter('gamepad.command_service').value
        )

        # Episode logging parameters
        self.declare_parameter('logging.enabled', False)
        self.declare_parameter('logging.log_dir', '/tmp/vla_logs')
        self.declare_parameter('logging.tilt_threshold_deg', 30.0)
        # Automatic episode termination thresholds.
        self.declare_parameter('logging.episode_timeout_s', 60.0)
        self.declare_parameter('logging.lift_success_m', 0.05)
        self.declare_parameter('logging.fall_z_drop_m', 0.15)

        # Simulation reset parameters
        # Reset motion (applies in sim AND on the real robot): move to a
        # predefined pose via the action server with the given duration.
        self.declare_parameter('reset.pose_name', 'initial_pose')
        # Time allowance handed to the action for the reset motion — raise
        # on the real robot where fast transitions are unsafe.
        self.declare_parameter('reset.duration_s', 1.5)
        self.declare_parameter('reset.action_name', '')

        # Deadman safety trigger (real robot): generated actions are only
        # commanded while the trigger is held. Values come from the shared
        # gamepad_config.yaml; index follows the GamepadClient convention
        # (negative = axis with >0.5 pressed, non-negative = button).
        self.declare_parameter('gamepad.safety.enabled', False)
        self.declare_parameter('gamepad.safety.trigger_index', -4)
        self.declare_parameter('gamepad.safety.joy_timeout_s', 0.5)
        self.declare_parameter('sim.world_name', 'simple_data_collection')
        self.declare_parameter('sim.robot_model_name', '')
        self.declare_parameter('sim.block_model_name', 'box_to_pick')
        self.declare_parameter('sim.spawn_x', 2.0)
        self.declare_parameter('sim.spawn_y', -1.5)
        self.declare_parameter('sim.spawn_z', 0.0)
        self.declare_parameter('sim.spawn_qx', 0.0)
        self.declare_parameter('sim.spawn_qy', 0.0)
        self.declare_parameter('sim.spawn_qz', 0.7071)
        self.declare_parameter('sim.spawn_qw', 0.7071)
        self.declare_parameter('sim.block_x', 2.0)
        self.declare_parameter('sim.block_y', -0.5)
        self.declare_parameter('sim.block_z', 0.45)

        self._logging_enabled = bool(self.get_parameter('logging.enabled').value)
        self._log_dir = str(self.get_parameter('logging.log_dir').value)
        self._log_tilt_deg = float(self.get_parameter('logging.tilt_threshold_deg').value)
        self._episode_timeout_s = float(
            self.get_parameter('logging.episode_timeout_s').value
        )
        self._lift_success_m = float(self.get_parameter('logging.lift_success_m').value)
        self._fall_z_drop_m = float(self.get_parameter('logging.fall_z_drop_m').value)
        # Sim vs real is derived from use_sim_time (set true by the sim
        # launches): in sim, world resets also teleport robot+block via
        # Gazebo and the episode logger polls gz poses; on the real robot
        # only the reset pose motion runs.
        self._sim_enabled = bool(self.get_parameter('use_sim_time').value)
        self._reset_pose_name = str(self.get_parameter('reset.pose_name').value)
        self._reset_pose_duration_s = float(
            self.get_parameter('reset.duration_s').value
        )
        self._reset_action_name = str(self.get_parameter('reset.action_name').value)
        # Reset runs on every STOP -- required, no robot-specific default.
        if not self._reset_action_name:
            raise RuntimeError(
                'reset.action_name is required -- refusing to default to a '
                'robot-specific value.'
            )
        self._safety_enabled = bool(self.get_parameter('gamepad.safety.enabled').value)
        self._safety_trigger_index = int(
            self.get_parameter('gamepad.safety.trigger_index').value
        )
        self._safety_joy_timeout_s = float(
            self.get_parameter('gamepad.safety.joy_timeout_s').value
        )
        self._sim_world_name = str(self.get_parameter('sim.world_name').value)
        self._sim_robot_model = str(self.get_parameter('sim.robot_model_name').value)
        if self._sim_enabled and not self._sim_robot_model:
            raise RuntimeError(
                'sim.robot_model_name is required when running in sim -- '
                'refusing to default to a robot-specific value.'
            )
        self._sim_block_model = str(self.get_parameter('sim.block_model_name').value)
        self._sim_spawn = (
            float(self.get_parameter('sim.spawn_x').value),
            float(self.get_parameter('sim.spawn_y').value),
            float(self.get_parameter('sim.spawn_z').value),
            float(self.get_parameter('sim.spawn_qx').value),
            float(self.get_parameter('sim.spawn_qy').value),
            float(self.get_parameter('sim.spawn_qz').value),
            float(self.get_parameter('sim.spawn_qw').value),
        )
        self._sim_block_reset = (
            float(self.get_parameter('sim.block_x').value),
            float(self.get_parameter('sim.block_y').value),
            float(self.get_parameter('sim.block_z').value),
        )

        self._actions_per_chunk = max(self._actions_per_chunk, 1)
        self._control_hz = max(self._control_hz, 1.0)
        self._chunk_size_threshold = min(max(self._chunk_size_threshold, 0.0), 1.0)

    def _load_robot_profile(self) -> None:
        self.declare_parameter('robot.descriptor_id', '')
        desc_id = str(self.get_parameter('robot.descriptor_id').value)

        if not desc_id:
            raise RuntimeError(
                'robot.descriptor_id, robot.name, and robot.active_profile '
                'are all unset -- refusing to default to a robot-specific '
                'profile.'
            )

        from sobits_vla_common.robot_descriptor import load_robot_descriptor
        desc = load_robot_descriptor(desc_id)
        self._active_profile = desc_id

        self.declare_parameter('robot.active_groups', [g.name for g in desc.active_groups])
        self.declare_parameter('robot.active_cameras', [c.name for c in desc.active_cameras])
        self.declare_parameter('robot.active_mobile_base', True)

        active_groups_list = list(self.get_parameter('robot.active_groups').value)
        active_cameras_list = list(self.get_parameter('robot.active_cameras').value)
        active_mobile_base = bool(self.get_parameter('robot.active_mobile_base').value)

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
        else:
            self._odom_topic = ''
            self._mobile_base_cmd_topic = ''
            self._mobile_base_features = []
            self._max_vel_x = 0.0
            self._max_vel_y = 0.0
            self._max_vel_theta = 0.0

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
                self._control_timer.timer_period_ns = int((1.0 / hz) * 1e9)
                self._step_duration = Duration(sec=0, nanosec=int((1.0 / hz) * 1e9))
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
            elif p.name == 'logging.enabled':
                self._logging_enabled = bool(p.value)
                self._episode_logger.enabled = self._logging_enabled
                self.get_logger().info('logging.enabled → {}'.format(self._logging_enabled))
            elif p.name == 'logging.tilt_threshold_deg':
                import math as _math
                self._log_tilt_deg = float(p.value)
                self._episode_logger._tilt_rad = _math.radians(self._log_tilt_deg)
                self.get_logger().info(
                    'logging.tilt_threshold_deg → {}'.format(self._log_tilt_deg)
                )
            elif p.name == 'logging.episode_timeout_s':
                self._episode_timeout_s = float(p.value)
                self._episode_logger._episode_timeout_s = self._episode_timeout_s
                self.get_logger().info(
                    'logging.episode_timeout_s → {}'.format(self._episode_timeout_s)
                )
            elif p.name == 'logging.lift_success_m':
                self._lift_success_m = float(p.value)
                self._episode_logger._lift_success_m = self._lift_success_m
                self.get_logger().info(
                    'logging.lift_success_m → {}'.format(self._lift_success_m)
                )
            elif p.name == 'logging.fall_z_drop_m':
                self._fall_z_drop_m = float(p.value)
                self._episode_logger._fall_z_drop_m = self._fall_z_drop_m
                self.get_logger().info(
                    'logging.fall_z_drop_m → {}'.format(self._fall_z_drop_m)
                )
        return SetParametersResult(successful=True)

    def _reset_episode_state(self, outcome: Optional[str] = None) -> None:
        """
        Reset all per-episode model state after a stop/world-reset.

        When ``outcome`` is given, /vla/episode_done is published by the
        reset thread AFTER the world/pose reset completes — so a listener
        (the experiment runner) knows the system is ready for the next
        PLAY without any time-based settle.
        """
        self._chunk_buffer.clear()
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

    def _set_entity_pose(
        self, name: str,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> bool:
        """
        Teleport a Gazebo entity.

        Prefers the bridged ros_gz SetEntityPose service
        (/world/<world>/set_pose); falls back to the `gz service` CLI when
        the bridge does not expose it.
        """
        try:
            from ros_gz_interfaces.msg import Entity
            from ros_gz_interfaces.srv import SetEntityPose

            if self._set_pose_client is None:
                self._set_pose_client = self.create_client(
                    SetEntityPose,
                    '/world/{}/set_pose'.format(self._sim_world_name),
                    callback_group=self._cb_group,
                )
            if self._set_pose_client.wait_for_service(timeout_sec=1.0):
                req = SetEntityPose.Request()
                req.entity = Entity(name=name, type=Entity.MODEL)
                req.pose.position.x = float(x)
                req.pose.position.y = float(y)
                req.pose.position.z = float(z)
                req.pose.orientation.x = float(qx)
                req.pose.orientation.y = float(qy)
                req.pose.orientation.z = float(qz)
                req.pose.orientation.w = float(qw)
                fut = self._set_pose_client.call_async(req)
                if self._wait_future(fut, timeout_s=3.0) and fut.result() is not None:
                    return bool(fut.result().success)
                self.get_logger().warn(
                    'SetEntityPose service call timed out — falling back to gz CLI.'
                )
        except ImportError:
            pass
        from sobits_vla_deploy.vla_episode_logger import _gz_set_pose
        return _gz_set_pose(self._sim_world_name, name, x, y, z, qx, qy, qz, qw)

    def _do_world_reset(self, outcome: Optional[str] = None) -> None:
        """Reset the scene: teleports (sim only), then the reset pose action."""
        try:
            self._run_world_reset()
        finally:
            if outcome is not None:
                self._publish_episode_done(outcome)

    def _run_world_reset(self) -> None:
        """Teleports (sim only) + reset pose action; blocking."""
        if self._sim_enabled:
            sx, sy, sz, sqx, sqy, sqz, sqw = self._sim_spawn
            bx, by, bz = self._sim_block_reset
            ok_robot = self._set_entity_pose(
                self._sim_robot_model, sx, sy, sz, sqx, sqy, sqz, sqw
            )
            ok_block = self._set_entity_pose(
                self._sim_block_model, bx, by, bz, 0.0, 0.0, 0.0, 1.0
            )
            self.get_logger().info(
                'World reset: robot={} block={}'.format(ok_robot, ok_block)
            )
        else:
            self.get_logger().info(
                'Real-robot reset: sending pose {!r} only.'.format(
                    self._reset_pose_name
                )
            )

        if not self._reset_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                'Reset action server {!r} unavailable — robot NOT re-posed.'.format(
                    self._reset_action_name
                )
            )
            return

        goal = MoveToPose.Goal()
        goal.pose_name = self._reset_pose_name
        goal.time_allowance.sec = int(self._reset_pose_duration_s)
        goal.time_allowance.nanosec = int(
            (self._reset_pose_duration_s - int(self._reset_pose_duration_s)) * 1e9
        )

        send_fut = self._reset_pose_client.send_goal_async(goal)
        if not self._wait_future(send_fut, timeout_s=5.0):
            self.get_logger().error('Reset goal send timed out.')
            return
        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Reset goal rejected by the action server.')
            return

        result_fut = goal_handle.get_result_async()
        if not self._wait_future(
            result_fut, timeout_s=self._reset_pose_duration_s + 10.0
        ):
            self.get_logger().error(
                'Reset motion did not finish within {:.0f}s — cancelling.'.format(
                    self._reset_pose_duration_s + 10.0
                )
            )
            goal_handle.cancel_goal_async()
            return

        result = result_fut.result().result
        self.get_logger().info(
            'move_to_pose {}: {}{}'.format(
                self._reset_pose_name,
                'OK' if result.success else 'FAIL',
                ' ({})'.format(result.message) if result.message else '',
            )
        )

    def _on_update_task(
        self,
        request: VlaUpdateTask.Request,
        response: VlaUpdateTask.Response,
    ) -> VlaUpdateTask.Response:
        self._chunk_buffer.clear()
        if hasattr(self._policy, 'reset'):
            self._policy.reset()
        self._task_label = request.label
        self._inference_engine.update_task_label(request.label)
        self.get_logger().info('Task label updated to {!r}.'.format(request.label))
        response.success = True
        response.message = 'succeeded'
        return response

    def _start_play(self) -> bool:
        """Atomically start PLAY if not already running. Returns True if it started."""
        with self._lock:
            if self._play_enabled:
                return False
            self._cmd_vector = dict(self._obs_builder.state_vector)
            # With the safety trigger enabled the robot stays frozen until the
            # operator engages it — start the episode clock on first
            # engagement instead of at PLAY (see the safety gate).
            self._episode_t0 = (
                None if self._safety_enabled else self.get_clock().now()
            )
            self._play_enabled = True
        self._episode_logger.begin_episode()
        self._inference_engine.update_play_enabled(True)
        return True

    def _stop_play(self, outcome: str) -> bool:
        """Atomically stop PLAY if running. Returns True if it stopped."""
        with self._lock:
            if not self._play_enabled:
                return False
            self._play_enabled = False
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
                # Idle STOP = reset the world to the start pose. The experiment
                # runner issues this before episode 1 so the first episode does
                # not start from a stale (un-reset) pose.
                self.get_logger().info('STOP while idle → resetting world to start pose.')
                self._reset_episode_state(outcome='reset')
            response.success = True
            response.message = 'STOP execution disabled'
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
                self.get_logger().info('VLA execution started via /vla/play topic.')
        else:
            if self._stop_play('manual_stop'):
                self.get_logger().info('VLA execution stopped via /vla/play topic.')

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
            if hasattr(self._policy, 'reset'):
                self._policy.reset()
            self._task_label = label
            self._inference_engine.update_task_label(label)
            self.get_logger().info('Task label updated to {!r} via /vla/task topic.'.format(label))

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
        try:
            if is_compressed:
                # NOT cv_bridge.compressed_imgmsg_to_cv2: apt's cv_bridge_boost
                # is built against NumPy 1.x and SEGFAULTS (SIGSEGV, not an
                # exception -- the except below cannot catch it) under this
                # env's NumPy 2 whenever desired_encoding forces a cvtColor2
                # conversion, which 'rgb8' always does. cv2.imdecode is the
                # env's own NumPy-2-native build, and this is exactly what
                # cv_bridge does internally: decode to BGR, then convert.
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                image = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # always BGR
                if image is None:
                    raise ValueError('cv2.imdecode returned None')
                if encoding == 'rgb8':
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as exc:
            self.get_logger().warn(
                'Image decode failed for {!r}: {}'.format(cam_name, exc),
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

        # Automatic termination: success (block lifted), failure (fall), or
        # timeout. Checked before consuming the action queue so an empty queue
        # cannot stall a timeout. Uses the sim-time-aware node clock.
        # Snapshot to a local: the reset thread can null this out between the
        # check and the subtraction below, raising TypeError.
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
                    self.get_logger().warn(
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
                self._safety_was_pressed = True
                if self._episode_t0 is None:
                    # Deferred episode clock: timing (and the episode
                    # timeout) starts now, not while the scene was being
                    # staged with the trigger released.
                    self._episode_t0 = self.get_clock().now()
                self.get_logger().info(
                    'Safety trigger engaged — resuming with fresh actions.'
                )
                return

        if self._single_step_mode:
            step = self._inference_engine.pop_single_step_result()
            if step is None:
                self.get_logger().warn(
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

            step = self._chunk_buffer.pop()
            if step is None:
                self.get_logger().warn(
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

        # Episode logging: commanded + measured joints, base vel, EE pose
        if self._logging_enabled:
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
                ee = self._obs_builder._get_ee_pose(
                    self._tf_buffer, first.target_frame, first.source_frame
                )
            self._episode_logger.log_step(
                joints=log_joints,
                base_vel=log_base,
                ee_pose=ee.tolist() if ee is not None else None,
                joints_measured=log_joints_measured,
            )

        self.get_logger().info('CMD -> {}{}'.format(joint_log, base_log))

    _BASE_KEY_ALIASES: Dict[str, str] = {
        'base_x': 'x.vel',
        'base_y': 'y.vel',
        'base_z': 'z.vel',
        'base_theta': 'theta.vel',
    }

    def _to_action_steps(self, raw_actions: Any) -> List[Dict[str, float]]:
        import torch

        action_keys = self._joint_features + self._mobile_base_features
        if not action_keys:
            return []

        if isinstance(raw_actions, dict):
            values = list(raw_actions.values())
            if values and isinstance(values[0], (list, tuple, np.ndarray, torch.Tensor)):
                chunk_len = len(values[0])
                chunk: List[Dict[str, float]] = []
                for step_idx in range(chunk_len):
                    step = {}
                    for key in action_keys:
                        if key in raw_actions:
                            step[key] = float(raw_actions[key][step_idx])
                    if step:
                        chunk.append(step)
                return chunk
            return [{k: float(v) for k, v in raw_actions.items() if k in action_keys}]

        if isinstance(raw_actions, torch.Tensor):
            raw_actions = raw_actions.detach().cpu().numpy()

        if isinstance(raw_actions, np.ndarray):
            if raw_actions.ndim == 3 and raw_actions.shape[0] == 1:
                raw_actions = raw_actions.squeeze(0)
            model_names = getattr(self, '_model_action_feature_names', None)
            if model_names:
                action_keys_set = set(action_keys)

                def _resolve(name: str) -> Optional[str]:
                    if name in action_keys_set:
                        return name
                    aliased = self._BASE_KEY_ALIASES.get(name)
                    if aliased and aliased in action_keys_set:
                        return aliased
                    return None

                if raw_actions.ndim == 1:
                    step: Dict[str, float] = {}
                    for i in range(min(len(model_names), raw_actions.shape[0])):
                        resolved = _resolve(model_names[i])
                        if resolved is not None:
                            step[resolved] = float(raw_actions[i])
                    return [step]
                if raw_actions.ndim == 2:
                    result = []
                    for step_idx in range(raw_actions.shape[0]):
                        step = {}
                        for i in range(min(len(model_names), raw_actions.shape[1])):
                            resolved = _resolve(model_names[i])
                            if resolved is not None:
                                step[resolved] = float(raw_actions[step_idx, i])
                        result.append(step)
                    return result
            else:
                if raw_actions.ndim == 1:
                    return [
                        {
                            key: float(raw_actions[i])
                            for i, key in enumerate(action_keys)
                            if i < raw_actions.shape[0]
                        }
                    ]
                if raw_actions.ndim == 2:
                    return [
                        {
                            key: float(raw_actions[step_idx, i])
                            for i, key in enumerate(action_keys)
                            if i < raw_actions.shape[1]
                        }
                        for step_idx in range(raw_actions.shape[0])
                    ]

        return []


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
