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
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
import rclpy.duration  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
import rclpy.time  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image, JointState  # noqa: E402
from sobits_interfaces.srv import VlaCommand, VlaUpdateTask  # noqa: E402
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
        self._ee_left_base_frame = 'base_footprint'
        self._ee_left_target_frame = 'hand_left_end_effector_link'
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

        self._camera_subs = []
        for cam_name, cam_topic in self._camera_topics.items():
            is_compressed = self._camera_compressed.get(cam_name, False)
            msg_type = CompressedImage if is_compressed else Image
            sub = self.create_subscription(
                msg_type,
                cam_topic,
                lambda msg, name=cam_name: self._on_image(msg, name),
                qos,
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
            ee_left_base_frame=self._ee_left_base_frame,
            ee_left_target_frame=self._ee_left_target_frame,
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
        )
        if self._logging_enabled:
            self.get_logger().info(
                'Episode logging enabled → {}'.format(self._log_dir)
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
        self.declare_parameter('model.repo_id', 'team-sobits/sobit_home_smolvla')
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
        self._policy_class_path = str(self.get_parameter('model.policy_class').value)
        self._model_device = str(self.get_parameter('model.device').value)
        self._model_use_amp = bool(self.get_parameter('model.use_amp').value)
        self._model_dataset_repo_id = str(self.get_parameter('model.dataset_repo_id').value)

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
        self.declare_parameter('sim.world_name', 'simple_data_collection')
        self.declare_parameter('sim.robot_model_name', 'sobit_home')
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
        self._sim_world_name = str(self.get_parameter('sim.world_name').value)
        self._sim_robot_model = str(self.get_parameter('sim.robot_model_name').value)
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

        if desc_id:
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

        else:
            self.declare_parameter('robot.name', '')
            self.declare_parameter('robot.active_profile', '')

            robot_name = str(self.get_parameter('robot.name').value)
            active_profile = str(self.get_parameter('robot.active_profile').value)

            if robot_name and active_profile and robot_name != active_profile:
                self.get_logger().warn(
                    'Both robot.name={!r} and robot.active_profile={!r} are set. '
                    'Using robot.name.'.format(robot_name, active_profile)
                )

            if robot_name:
                self._active_profile = robot_name
                ns = 'robot'
            elif active_profile:
                self._active_profile = active_profile
                ns = 'robot'
            else:
                self._active_profile = 'sobit_home'
                ns = 'robot'

            self.declare_parameter(f'{ns}.joint_states_topic', '/joint_states')
            self.declare_parameter(f'{ns}.odom_topic', '')
            self.declare_parameter(f'{ns}.groups.names', ['arm'])
            self.declare_parameter(f'{ns}.mobile_base.command_topic', '')
            self.declare_parameter(f'{ns}.mobile_base.features', ['x.vel', 'theta.vel'])
            self.declare_parameter(f'{ns}.mobile_base.max_vel_x', 0.0)
            self.declare_parameter(f'{ns}.mobile_base.max_vel_y', 0.0)
            self.declare_parameter(f'{ns}.mobile_base.max_vel_theta', 0.0)
            self.declare_parameter(f'{ns}.cameras.names', ['head'])

            self._joint_states_topic = str(self.get_parameter(f'{ns}.joint_states_topic').value)
            self._odom_topic = str(self.get_parameter(f'{ns}.odom_topic').value)
            self._mobile_base_cmd_topic = str(
                self.get_parameter(f'{ns}.mobile_base.command_topic').value
            )
            self._mobile_base_features = list(
                self.get_parameter(f'{ns}.mobile_base.features').value
            )
            self._max_vel_x = float(
                self.get_parameter(f'{ns}.mobile_base.max_vel_x').value
            )
            self._max_vel_y = float(
                self.get_parameter(f'{ns}.mobile_base.max_vel_y').value
            )
            self._max_vel_theta = float(
                self.get_parameter(f'{ns}.mobile_base.max_vel_theta').value
            )
            self._camera_names = list(self.get_parameter(f'{ns}.cameras.names').value)

            group_names = list(self.get_parameter(f'{ns}.groups.names').value)
            self._joint_groups: List[JointGroupConfig] = []
            self._joint_features: List[str] = []
            self._joint_feature_to_ros: Dict[str, str] = {}

            for group_name in group_names:
                group_ns = f'{ns}.groups.{group_name}'
                self.declare_parameter(f'{group_ns}.command_topic', '')
                self.declare_parameter(f'{group_ns}.joints_ros', [''])
                self.declare_parameter(f'{group_ns}.features', [''])
                self.declare_parameter(f'{group_ns}.max_joint_delta', -1.0)

                command_topic = str(self.get_parameter(f'{group_ns}.command_topic').value)
                joints_ros = [j for j in self.get_parameter(f'{group_ns}.joints_ros').value if j]
                features = [f for f in self.get_parameter(f'{group_ns}.features').value if f]
                group_delta = float(self.get_parameter(f'{group_ns}.max_joint_delta').value)

                if not command_topic:
                    raise RuntimeError(
                        'Missing command topic for group {!r}.'.format(group_name)
                    )
                if len(joints_ros) != len(features):
                    raise RuntimeError(
                        'Group {!r} must have the same number of joints_ros and '
                        'features.'.format(group_name)
                    )

                self._joint_groups.append(
                    JointGroupConfig(
                        name=group_name,
                        command_topic=command_topic,
                        joints_ros=joints_ros,
                        features=features,
                        max_joint_delta=group_delta,
                    )
                )

                for joint_name, feature in zip(joints_ros, features):
                    self._joint_features.append(feature)
                    self._joint_feature_to_ros[feature] = joint_name

            self.declare_parameter(f'{ns}.cameras.default_encoding', 'rgb8')
            self.declare_parameter(f'{ns}.cameras.default_compressed', True)
            default_encoding = str(self.get_parameter(f'{ns}.cameras.default_encoding').value)
            default_compressed = bool(
                self.get_parameter(f'{ns}.cameras.default_compressed').value
            )
            self._camera_topics: Dict[str, str] = {}
            self._camera_encodings: Dict[str, str] = {}
            self._camera_compressed: Dict[str, bool] = {}
            for cam_name in self._camera_names:
                cam_ns = f'{ns}.cameras.{cam_name}'
                self.declare_parameter(f'{cam_ns}.topic', '')
                self.declare_parameter(f'{cam_ns}.encoding', default_encoding)
                self.declare_parameter(f'{cam_ns}.compressed', default_compressed)
                cam_topic = str(self.get_parameter(f'{cam_ns}.topic').value)
                if not cam_topic:
                    raise RuntimeError('Camera {!r} must define a topic.'.format(cam_name))
                self._camera_topics[cam_name] = cam_topic
                self._camera_encodings[cam_name] = str(
                    self.get_parameter(f'{cam_ns}.encoding').value
                )
                self._camera_compressed[cam_name] = bool(
                    self.get_parameter(f'{cam_ns}.compressed').value
                )

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

    def _reset_episode_state(self) -> None:
        """Reset all per-episode model state after a stop/world-reset."""
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
        Thread(target=self._do_world_reset, daemon=True).start()

    def _do_world_reset(self) -> None:
        """Teleport robot+block, then move robot to initial_pose via action."""
        import subprocess
        from sobits_vla_deploy.vla_episode_logger import _gz_set_pose
        sx, sy, sz, sqx, sqy, sqz, sqw = self._sim_spawn
        bx, by, bz = self._sim_block_reset
        ok_robot = _gz_set_pose(
            self._sim_world_name, self._sim_robot_model,
            sx, sy, sz, sqx, sqy, sqz, sqw,
        )
        ok_block = _gz_set_pose(
            self._sim_world_name, self._sim_block_model,
            bx, by, bz, 0.0, 0.0, 0.0, 1.0,
        )
        self.get_logger().info(
            'World reset: robot={} block={}'.format(ok_robot, ok_block)
        )
        goal = (
            "pose_name: 'initial_pose'\n"
            'time_allowance:\n'
            '  sec: 1\n'
            '  nanosec: 500000000'
        )
        try:
            result = subprocess.run(
                [
                    'ros2', 'action', 'send_goal',
                    '/sobit_home/move_to_pose',
                    'sobits_interfaces/action/MoveToPose',
                    goal,
                ],
                capture_output=True, text=True, timeout=10.0,
            )
            ok_pose = 'succeeded' in result.stdout.lower() or result.returncode == 0
            self.get_logger().info('move_to_pose initial_pose: {}'.format(
                'OK' if ok_pose else 'FAIL (rc={})'.format(result.returncode)
            ))
        except subprocess.TimeoutExpired:
            self.get_logger().warn('move_to_pose timed out after 10s')
        except Exception as exc:
            self.get_logger().warn('move_to_pose error: {}'.format(exc))

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

    def _on_command(
        self,
        request: VlaCommand.Request,
        response: VlaCommand.Response,
    ) -> VlaCommand.Response:
        cmd = request.command
        if cmd == VlaCommand.Request.PLAY:
            if not self._play_enabled:
                self.get_logger().info('VLA execution started via service command PLAY.')
                with self._lock:
                    self._cmd_vector = dict(self._obs_builder.state_vector)
                self._episode_t0 = self.get_clock().now()
                self._play_enabled = True
                self._episode_logger.begin_episode()
                self._inference_engine.update_play_enabled(True)
            response.success = True
            response.message = 'PLAY execution enabled'
            response.status = VlaCommand.Response.STATE_PLAYING
        elif cmd == VlaCommand.Request.STOP:
            if self._play_enabled:
                self.get_logger().info('VLA execution stopped via service command STOP.')
                self._episode_logger.end_episode('manual_stop')
                self._play_enabled = False
                self._inference_engine.update_play_enabled(False)
                self._reset_episode_state()
                self._publish_episode_done('manual_stop')
            else:
                # Idle STOP = reset the world to the start pose. The experiment
                # runner issues this before episode 1 so the first episode does
                # not start from a stale (un-reset) pose.
                self.get_logger().info('STOP while idle → resetting world to start pose.')
                self._reset_episode_state()
                self._publish_episode_done('reset')
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
        if msg.data and not self._play_enabled:
            self.get_logger().info('VLA execution started via /vla/play topic.')
            with self._lock:
                self._cmd_vector = dict(self._obs_builder.state_vector)
            self._episode_t0 = self.get_clock().now()
            self._play_enabled = True
            self._episode_logger.begin_episode()
            self._inference_engine.update_play_enabled(True)
        elif not msg.data and self._play_enabled:
            self.get_logger().info('VLA execution stopped via /vla/play topic.')
            self._episode_logger.end_episode('manual_stop')
            self._play_enabled = False
            self._inference_engine.update_play_enabled(False)
            self._reset_episode_state()
            self._publish_episode_done('manual_stop')

    def _auto_stop_episode(self, outcome: str) -> None:
        """Terminate the current episode automatically and notify the runner."""
        self.get_logger().info(
            'Episode auto-terminated: {}. Resetting world.'.format(outcome)
        )
        self._episode_logger.end_episode(outcome)
        self._play_enabled = False
        self._inference_engine.update_play_enabled(False)
        # Stop the base immediately so the robot does not drift during reset.
        if self._base_pub is not None:
            self._base_pub.publish(Twist())
        self._reset_episode_state()
        self._publish_episode_done(outcome)

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
                image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding=encoding)
            else:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as exc:
            self.get_logger().warn(
                'Image decode failed for {!r}: {}'.format(cam_name, exc),
                throttle_duration_sec=5.0,
            )
            return
        self._obs_builder.update_image(cam_name, image)

    def _publish_next_action(self) -> None:
        if not self._play_enabled:
            if self._base_pub is not None:
                cmd = Twist()
                self._base_pub.publish(cmd)
            return

        # Automatic termination: success (block lifted), failure (fall), or
        # timeout. Checked before consuming the action queue so an empty queue
        # cannot stall a timeout. Uses the sim-time-aware node clock.
        if self._logging_enabled and self._episode_t0 is not None:
            elapsed = (self.get_clock().now() - self._episode_t0).nanoseconds * 1e-9
            outcome = self._episode_logger.evaluate_termination(elapsed)
            if outcome is not None:
                self._auto_stop_episode(outcome)
                return

        if self._single_step_mode:
            step = self._inference_engine.get_single_step_result()
            self._inference_engine.clear_single_step_result()
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
            ee = self._obs_builder._get_ee_pose_left(
                self._tf_buffer, self._ee_left_base_frame, self._ee_left_target_frame
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
