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

from collections import deque
from dataclasses import dataclass
from importlib import import_module
from threading import Condition, Lock, Thread
from time import monotonic  # wall-clock: correct for GPU latency measurement, not sim time
from typing import Any, Dict, List, Optional

from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy
from std_msgs.msg import Bool, String
from sobits_interfaces.srv import VlaUpdateTask
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# LeRobot v0.5.1 imports — paths changed from v0.4.x
try:
    from lerobot.datasets.feature_utils import build_dataset_frame, hw_to_dataset_features
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.utils.control_utils import predict_action
    _LEROBOT_AVAILABLE = True
except ImportError:
    _LEROBOT_AVAILABLE = False

try:
    from lerobot.policies.rtc.configuration_rtc import RTCConfig
    from lerobot.configs.types import RTCAttentionSchedule
    _RTC_AVAILABLE = True
except ImportError:
    RTCConfig = None  # type: ignore[assignment,misc]
    RTCAttentionSchedule = None  # type: ignore[assignment]
    _RTC_AVAILABLE = False

# Registry: policy_class_path -> (config_module, config_class, config_has_device_field)
_POLICY_CONFIG_REGISTRY: Dict[str, tuple] = {
    'lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy': (
        'lerobot.policies.smolvla.configuration_smolvla', 'SmolVLAConfig', False,
    ),
    'lerobot.policies.pi0.modeling_pi0.PI0Policy': (
        'lerobot.policies.pi0.configuration_pi0', 'PI0Config', True,
    ),
    'lerobot.policies.pi05.modeling_pi05.PI05Policy': (
        'lerobot.policies.pi05.configuration_pi05', 'PI05Config', True,
    ),
    'lerobot.policies.pi0_fast.modeling_pi0_fast.PI0FastPolicy': (
        'lerobot.policies.pi0_fast.configuration_pi0_fast', 'PI0FastConfig', True,
    ),
}


@dataclass
class JointGroupConfig:
    name: str
    command_topic: str
    joints_ros: List[str]
    features: List[str]


class ActionChunkBuffer:
    def __init__(self, aggregate_fn_name: str) -> None:
        self._queue: deque[Dict[str, float]] = deque()
        self._lock = Lock()
        self._aggregate_fn_name = aggregate_fn_name

    def size(self) -> int:
        with self._lock:
            return len(self._queue)

    def pop(self) -> Optional[Dict[str, float]]:
        with self._lock:
            if not self._queue:
                return None
            return self._queue.popleft()

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()

    def left_over(self, count: int) -> List[Dict[str, float]]:
        if count <= 0:
            return []
        with self._lock:
            return list(self._queue)[:count]

    def merge(self, chunk: List[Dict[str, float]], overlap: int) -> None:
        if not chunk:
            return
        with self._lock:
            overlap_steps = min(overlap, len(self._queue), len(chunk))
            for idx in range(overlap_steps):
                self._queue[idx] = self._aggregate(self._queue[idx], chunk[idx])
            for step in chunk[overlap_steps:]:
                self._queue.append(step)

    def _aggregate(
        self,
        old_step: Dict[str, float],
        new_step: Dict[str, float],
    ) -> Dict[str, float]:
        out = dict(old_step)
        for key, new_val in new_step.items():
            old_val = old_step.get(key, new_val)
            if self._aggregate_fn_name == 'newest':
                out[key] = float(new_val)
            else:
                out[key] = float(0.5 * old_val + 0.5 * new_val)
        return out


class LeRobotDeployNode(Node):
    def __init__(self) -> None:
        super().__init__('sobits_vla_deploy')

        self._cb_group = ReentrantCallbackGroup()
        self._lock = Lock()
        self._bridge = CvBridge()
        self._inference_cond = Condition()

        self._configure_parameters()
        self._load_robot_profile()
        self._load_policy()

        self._state_vector: Dict[str, float] = {
            feature: 0.0 for feature in self._joint_features
        }
        self._images: Dict[str, Optional[np.ndarray]] = {
            cam_name: None for cam_name in self._camera_names
        }
        self._obs_features: Optional[Dict[str, Any]] = None
        self._play_enabled = False
        self._shutdown_inference = False

        self._chunk_buffer = ActionChunkBuffer(self._aggregate_fn_name)
        self._single_step_result: Optional[Dict[str, float]] = None
        self._single_step_lock = Lock()
        self._task_label: str = ''

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._ee_left_base_frame = 'base_footprint'
        self._ee_left_target_frame = 'hand_left_end_effector_link'
        self._prev_ee_pose_left: Optional[np.ndarray] = None

        qos = QoSProfile(depth=1)

        self._joy_sub = self.create_subscription(
            Joy,
            self._joy_topic,
            self._on_joy,
            qos,
            callback_group=self._cb_group,
        )
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

        self._control_timer = self.create_timer(
            1.0 / self._control_hz,
            self._publish_next_action,
            callback_group=self._cb_group,
        )

        self._step_duration = Duration(
            sec=0,
            nanosec=int((1.0 / self._control_hz) * 1e9),
        )

        self._inference_thread = Thread(target=self._inference_worker, daemon=True)
        self._inference_thread.start()

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
        self._shutdown_inference = True
        with self._inference_cond:
            self._inference_cond.notify_all()
        if hasattr(self, '_inference_thread') and self._inference_thread.is_alive():
            self._inference_thread.join(timeout=2.0)
        super().destroy_node()

    def _configure_parameters(self) -> None:
        self.declare_parameter('model.repo_id', 'team-sobits/sobit_home_smolvla')
        self.declare_parameter(
            'model.policy_class',
            'lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy',
        )
        self.declare_parameter('model.device', 'cuda')
        self.declare_parameter('model.use_amp', True)

        self.declare_parameter('runtime.control_hz', 10.0)
        self.declare_parameter('runtime.actions_per_chunk', 50)
        self.declare_parameter('runtime.chunk_size_threshold', 0.6)
        self.declare_parameter('runtime.aggregate_fn_name', 'weighted_average')
        self.declare_parameter('runtime.async_enabled', True)
        # When true: infer every tick, publish only step[0], discard the rest.
        # Eliminates chunk-boundary jumps at the cost of higher GPU usage.
        self.declare_parameter('runtime.single_step_mode', False)
        # Max radians any joint may move in a single control step. Prevents violent
        # jumps at chunk boundaries when the queue runs empty. 0.0 = disabled.
        self.declare_parameter('runtime.max_joint_delta_per_step', 0.15)

        self.declare_parameter('rtc.enabled', True)
        self.declare_parameter('rtc.execution_horizon', 10)
        self.declare_parameter('rtc.max_guidance_weight', 10.0)
        self.declare_parameter('rtc.prefix_attention_schedule', 'EXP')
        self.declare_parameter('rtc.inference_delay', 4)
        self.declare_parameter('rtc.debug', False)

        self.declare_parameter('gamepad.topic', '/joy')
        self.declare_parameter('gamepad.name', 'default')
        self.declare_parameter('gamepad.controllers', [''])

        self._model_repo_id = str(self.get_parameter('model.repo_id').value)
        self._policy_class_path = str(self.get_parameter('model.policy_class').value)
        self._model_device = str(self.get_parameter('model.device').value)
        self._model_use_amp = bool(self.get_parameter('model.use_amp').value)

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
        self._max_joint_delta = float(
            self.get_parameter('runtime.max_joint_delta_per_step').value
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

        self._joy_topic = str(self.get_parameter('gamepad.topic').value)
        self._gamepad_name = str(self.get_parameter('gamepad.name').value)
        self._gamepad_controllers = [
            c for c in self.get_parameter('gamepad.controllers').value if c
        ]
        if self._gamepad_name and self._gamepad_name not in self._gamepad_controllers:
            self._gamepad_controllers.append(self._gamepad_name)

        self._play_buttons: List[int] = []
        self._stop_buttons: List[int] = []

        for controller in self._gamepad_controllers:
            base = f'gamepad.{controller}.button_mapping'
            self.declare_parameter(f'{base}.play', -1)
            self.declare_parameter(f'{base}.stop', -1)
            play_idx = int(self.get_parameter(f'{base}.play').value)
            stop_idx = int(self.get_parameter(f'{base}.stop').value)
            if play_idx >= 0:
                self._play_buttons.append(play_idx)
            if stop_idx >= 0:
                self._stop_buttons.append(stop_idx)

        self._play_buttons = sorted(set(self._play_buttons))
        self._stop_buttons = sorted(set(self._stop_buttons))

        self._actions_per_chunk = max(self._actions_per_chunk, 1)
        self._control_hz = max(self._control_hz, 1.0)
        self._chunk_size_threshold = min(max(self._chunk_size_threshold, 0.0), 1.0)

    def _load_robot_profile(self) -> None:
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
        self.declare_parameter(f'{ns}.cameras.names', ['head'])

        self._joint_states_topic = str(self.get_parameter(f'{ns}.joint_states_topic').value)
        self._odom_topic = str(self.get_parameter(f'{ns}.odom_topic').value)
        self._mobile_base_cmd_topic = str(
            self.get_parameter(f'{ns}.mobile_base.command_topic').value
        )
        self._mobile_base_features = list(
            self.get_parameter(f'{ns}.mobile_base.features').value
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

            command_topic = str(self.get_parameter(f'{group_ns}.command_topic').value)
            joints_ros = [j for j in self.get_parameter(f'{group_ns}.joints_ros').value if j]
            features = [f for f in self.get_parameter(f'{group_ns}.features').value if f]

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

    def _build_rtc_config(self) -> Optional[Any]:
        """Build RTCConfig if RTC enabled and policy supports it."""
        if not self._rtc_enabled or not _RTC_AVAILABLE:
            return None
        try:
            schedule = RTCAttentionSchedule[self._rtc_prefix_attention_schedule]
            return RTCConfig(
                enabled=True,
                execution_horizon=self._rtc_execution_horizon,
                max_guidance_weight=self._rtc_max_guidance_weight,
                prefix_attention_schedule=schedule,
                debug=self._rtc_debug,
            )
        except Exception as exc:
            self.get_logger().warn('RTCConfig build failed: {}. RTC disabled.'.format(exc))
            self._rtc_enabled = False
            return None

    def _build_policy_config(self, rtc_cfg: Optional[Any]) -> Optional[Any]:
        """Build typed policy config using registry, injecting RTCConfig if supported."""
        entry = _POLICY_CONFIG_REGISTRY.get(self._policy_class_path)
        if entry is None:
            self.get_logger().info(
                'Policy {!r} not in registry — loading with pretrained defaults.'.format(
                    self._policy_class_path
                )
            )
            return None
        config_module_path, config_class_name, has_device = entry
        try:
            config_mod = import_module(config_module_path)
            config_cls = getattr(config_mod, config_class_name)
            kwargs: Dict[str, Any] = {}
            if rtc_cfg is not None:
                kwargs['rtc_config'] = rtc_cfg
            if has_device and self._model_device:
                kwargs['device'] = self._model_device
            cfg = config_cls(**kwargs)
            self.get_logger().info(
                'Built {} with RTC={}, device={}.'.format(
                    config_class_name, rtc_cfg is not None, self._model_device
                )
            )
            return cfg
        except Exception as exc:
            self.get_logger().warn(
                'Could not build {}: {}. Loading with pretrained defaults.'.format(
                    config_class_name, exc
                )
            )
            self._rtc_enabled = False
            return None

    def _is_peft_adapter_repo(self, repo_id: str) -> bool:
        """Return True if repo_id is a PEFT adapter (has adapter_config.json)."""
        try:
            from huggingface_hub import file_exists
            return file_exists(repo_id, 'adapter_config.json')
        except Exception:
            pass
        try:
            from pathlib import Path
            return (Path(repo_id) / 'adapter_config.json').exists()
        except Exception:
            return False

    def _patch_input_features_from_adapter(self, repo_id: str) -> None:
        """Overwrite policy.config.input_features with values from the adapter repo's config.json."""
        try:
            import json
            from huggingface_hub import hf_hub_download
            from lerobot.configs.policies import PolicyFeature, FeatureType
            cfg_path = hf_hub_download(repo_id, 'config.json')
            with open(cfg_path) as fh:
                cfg_dict = json.load(fh)
            raw_features = cfg_dict.get('input_features', {})
            if not raw_features:
                return
            patched: Dict[str, Any] = {}
            for k, v in raw_features.items():
                patched[k] = PolicyFeature(type=FeatureType[v['type']], shape=tuple(v['shape']))
            self._policy.config.input_features = patched
            img_keys = [k for k, f in patched.items() if f.type is FeatureType.VISUAL]
            self.get_logger().info(
                'Patched input_features from adapter config. Image keys: {}'.format(img_keys)
            )
        except Exception as exc:
            self.get_logger().warn(
                'Could not patch input_features from adapter config: {}'.format(exc)
            )

    @staticmethod
    def _patch_pi05_action_dim_padding() -> None:
        """Zero-pad action_in/out_proj weights when loading a base checkpoint whose
        max_action_dim is smaller than the model being built (e.g. pi05_base has 32,
        but a LoRA adapter trained on 34 dims requires the base to also be 34-dim).
        Without padding, strict=False silently leaves the extra rows/cols random,
        which causes PEFT's load_state_dict to see a shape mismatch on the LoRA weights.
        """
        import torch
        try:
            from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        except ImportError:
            return

        orig_fix = PI05Policy._fix_pytorch_state_dict_keys

        def _patched_fix(self, state_dict, model_config):
            fixed = orig_fix(self, state_dict, model_config)
            model_action_dim = self.model.action_in_proj.in_features
            for key in list(fixed.keys()):
                val = fixed[key]
                if key.endswith('action_in_proj.weight') and val.ndim == 2 and val.shape[1] < model_action_dim:
                    extra = model_action_dim - val.shape[1]
                    pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                    fixed[key] = torch.cat([val, pad], dim=1)
                elif key.endswith('action_out_proj.weight') and val.ndim == 2 and val.shape[0] < model_action_dim:
                    extra = model_action_dim - val.shape[0]
                    pad = torch.zeros(extra, val.shape[1], dtype=val.dtype, device=val.device)
                    fixed[key] = torch.cat([val, pad], dim=0)
                elif key.endswith('action_out_proj.bias') and val.ndim == 1 and val.shape[0] < model_action_dim:
                    extra = model_action_dim - val.shape[0]
                    pad = torch.zeros(extra, dtype=val.dtype, device=val.device)
                    fixed[key] = torch.cat([val, pad], dim=0)
            return fixed

        PI05Policy._fix_pytorch_state_dict_keys = _patched_fix

    def _load_policy(self) -> None:
        import torch
        module_path, class_name = self._policy_class_path.rsplit('.', 1)
        policy_module = import_module(module_path)
        policy_cls = getattr(policy_module, class_name)

        # Apply action-dim padding patch before any from_pretrained call so that
        # base checkpoints with fewer action dims (e.g. pi05_base has 32) load
        # cleanly into a larger model (e.g. 34-dim for joints + base velocity).
        self._patch_pi05_action_dim_padding()

        rtc_cfg = self._build_rtc_config()
        cfg = self._build_policy_config(rtc_cfg)

        # Load weights directly in bfloat16 on CPU to halve peak RAM during load
        # (~5-6 GB instead of ~11 GB for pi05_base in float32).  PI0/PI05/PI0Fast
        # call self.model.to(config.device) in __init__ before weights arrive, so
        # we keep device='cpu' here and move to GPU after the load is complete.
        load_device = self._model_device
        if cfg is not None and hasattr(cfg, 'device'):
            cfg.device = 'cpu'

        if self._is_peft_adapter_repo(self._model_repo_id):
            # LoRA adapter repo: load base model, apply adapter, then merge.
            # Use the adapter repo's config.json (not the base model's) so that
            # input_features / image_features reflect the fine-tuned camera names.
            import json
            from huggingface_hub import hf_hub_download
            adapter_cfg_path = hf_hub_download(self._model_repo_id, 'adapter_config.json')
            with open(adapter_cfg_path) as fh:
                adapter_meta = json.load(fh)
            base_model_id = adapter_meta.get('base_model_name_or_path', '')
            self.get_logger().info(
                'LoRA adapter detected. Loading base model {!r} ...'.format(base_model_id)
            )

            # Build typed policy config from the adapter's config.json so that
            # input_features / image_features reflect the fine-tuned camera names.
            # We cannot use PreTrainedConfig.from_pretrained because draccus does not
            # recognise the 'type' field; instead we parse the JSON manually.
            try:
                from lerobot.configs.types import PolicyFeature, FeatureType as FT
                adapter_policy_json_path = hf_hub_download(self._model_repo_id, 'config.json')
                with open(adapter_policy_json_path) as fh:
                    adapter_policy_dict = json.load(fh)
                in_feats = {
                    k: PolicyFeature(type=FT[v['type']], shape=tuple(v['shape']))
                    for k, v in adapter_policy_dict.get('input_features', {}).items()
                }
                out_feats = {
                    k: PolicyFeature(type=FT[v['type']], shape=tuple(v['shape']))
                    for k, v in adapter_policy_dict.get('output_features', {}).items()
                }
                # max_action_dim / max_state_dim must match the adapter's training dims
                # (e.g. 34 for 31 joints + 3 base) so the base model is built with
                # the right projection shapes before the adapter weights are applied.
                adapter_policy_cfg = policy_cls.config_class(
                    input_features=in_feats,
                    output_features=out_feats,
                    device='cpu',
                    chunk_size=adapter_policy_dict.get('chunk_size', 50),
                    n_action_steps=adapter_policy_dict.get('n_action_steps', 50),
                    paligemma_variant=adapter_policy_dict.get('paligemma_variant', 'gemma_2b'),
                    action_expert_variant=adapter_policy_dict.get(
                        'action_expert_variant', 'gemma_300m'
                    ),
                    max_action_dim=adapter_policy_dict.get('max_action_dim', 32),
                    max_state_dim=adapter_policy_dict.get('max_state_dim', 32),
                )
                self.get_logger().info(
                    'Built adapter policy config. Image features: {}'.format(
                        list(getattr(adapter_policy_cfg, 'image_features', {}).keys())
                    )
                )
                load_cfg = adapter_policy_cfg
            except Exception as exc:
                self.get_logger().warn(
                    'Could not build adapter policy config ({}). Using ROS-built config.'.format(exc)
                )
                load_cfg = cfg

            load_kwargs: Dict[str, Any] = {'strict': False, 'torch_dtype': torch.bfloat16}
            if load_cfg is not None:
                load_kwargs['config'] = load_cfg

            # --- memory diagnostics before the heavyweight from_pretrained call ---
            import sys, gc, traceback as _tb
            try:
                import psutil as _psutil
                _proc = _psutil.Process()
                _rss_before = _proc.memory_info().rss / 1024 ** 3
            except Exception:
                _rss_before = float('nan')
            try:
                import torch as _torch
                if _torch.cuda.is_available():
                    _torch.cuda.synchronize()
                    _vram_free_before, _vram_total = _torch.cuda.mem_get_info()
                    _vram_free_before /= 1024 ** 3
                    _vram_total /= 1024 ** 3
                else:
                    _vram_free_before = _vram_total = float('nan')
            except Exception:
                _vram_free_before = _vram_total = float('nan')
            self.get_logger().info(
                'from_pretrained START: base={!r}  RAM_used={:.2f}GB  '
                'VRAM_free={:.2f}/{:.2f}GB  kwargs={}'.format(
                    base_model_id, _rss_before,
                    _vram_free_before, _vram_total,
                    {k: (v if not hasattr(v, '__class__') else v.__class__.__name__)
                     for k, v in load_kwargs.items()},
                )
            )
            import sys as _sys; _sys.stdout.flush(); _sys.stderr.flush()
            # ---

            try:
                self._policy = policy_cls.from_pretrained(base_model_id, **load_kwargs)
            except Exception as _exc:
                self.get_logger().error(
                    'from_pretrained FAILED for base={!r}: {}\n{}'.format(
                        base_model_id, _exc, _tb.format_exc()
                    )
                )
                _sys.stdout.flush(); _sys.stderr.flush()
                raise

            try:
                _rss_after = _psutil.Process().memory_info().rss / 1024 ** 3
            except Exception:
                _rss_after = float('nan')
            try:
                if _torch.cuda.is_available():
                    _torch.cuda.synchronize()
                    _vram_free_after, _ = _torch.cuda.mem_get_info()
                    _vram_free_after /= 1024 ** 3
                else:
                    _vram_free_after = float('nan')
            except Exception:
                _vram_free_after = float('nan')
            self.get_logger().info(
                'from_pretrained DONE: RAM_used={:.2f}GB  VRAM_free={:.2f}GB'.format(
                    _rss_after, _vram_free_after
                )
            )
            _sys.stdout.flush(); _sys.stderr.flush()

            self.get_logger().info('Applying LoRA adapter from {!r} ...'.format(
                self._model_repo_id
            ))
            try:
                from peft import PeftModel
                self.get_logger().info('PeftModel.from_pretrained START ...')
                _sys.stdout.flush(); _sys.stderr.flush()
                self._policy = PeftModel.from_pretrained(
                    self._policy, self._model_repo_id
                )
                self.get_logger().info('PeftModel.from_pretrained DONE. Merging ...')
                _sys.stdout.flush(); _sys.stderr.flush()
                self._policy = self._policy.merge_and_unload()
                self.get_logger().info('LoRA adapter merged.')
                _sys.stdout.flush(); _sys.stderr.flush()
            except Exception as exc:
                self.get_logger().warn(
                    'PEFT merge failed ({}). Running without adapter.'.format(exc)
                )
                _sys.stdout.flush(); _sys.stderr.flush()
        else:
            load_kwargs: Dict[str, Any] = {'strict': False, 'torch_dtype': torch.bfloat16}
            if cfg is not None:
                load_kwargs['config'] = cfg
            self._policy = policy_cls.from_pretrained(self._model_repo_id, **load_kwargs)

        if hasattr(self._policy, 'reset'):
            self._policy.reset()

        # Build pre/post processor pipelines for v0.5.1 predict_action API
        self._preprocessor = None
        self._postprocessor = None
        if _LEROBOT_AVAILABLE:
            try:
                self._preprocessor, self._postprocessor = make_pre_post_processors(
                    self._policy.config, self._model_repo_id
                )
            except Exception as exc:
                self.get_logger().warn(
                    'Could not build pre/post processors: {}. '
                    'Direct policy.select_action will be used.'.format(exc)
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
        self.get_logger().info('Task label updated to {!r}.'.format(request.label))
        response.success = True
        response.message = 'succeeded'
        return response

    def _on_joy(self, msg: Joy) -> None:
        if self._any_button_pressed(msg, self._play_buttons):
            if not self._play_enabled:
                self.get_logger().info('VLA execution started by gamepad play button.')
                self._play_enabled = True
                with self._inference_cond:
                    self._inference_cond.notify_all()

        if self._any_button_pressed(msg, self._stop_buttons):
            if self._play_enabled:
                self.get_logger().info('VLA execution stopped by gamepad stop button.')
            self._play_enabled = False
            self._chunk_buffer.clear()

    def _on_play(self, msg: Bool) -> None:
        if msg.data and not self._play_enabled:
            self.get_logger().info('VLA execution started via /vla/play topic.')
            self._play_enabled = True
            with self._inference_cond:
                self._inference_cond.notify_all()
        elif not msg.data and self._play_enabled:
            self.get_logger().info('VLA execution stopped via /vla/play topic.')
            self._play_enabled = False
            self._chunk_buffer.clear()

    def _on_task(self, msg: String) -> None:
        label = msg.data.strip()
        if label:
            self._chunk_buffer.clear()
            if hasattr(self._policy, 'reset'):
                self._policy.reset()
            self._task_label = label
            self.get_logger().info('Task label updated to {!r} via /vla/task topic.'.format(label))

    def _any_button_pressed(self, msg: Joy, button_indices: List[int]) -> bool:
        for idx in button_indices:
            if 0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1:
                return True
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        with self._lock:
            for feature, ros_joint in self._joint_feature_to_ros.items():
                if ros_joint in by_name:
                    self._state_vector[feature] = float(by_name[ros_joint])

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            if 'x.vel' in self._mobile_base_features:
                self._state_vector['x.vel'] = float(msg.twist.twist.linear.x)
            if 'y.vel' in self._mobile_base_features:
                self._state_vector['y.vel'] = float(msg.twist.twist.linear.y)
            if 'z.vel' in self._mobile_base_features:
                self._state_vector['z.vel'] = float(msg.twist.twist.linear.z)
            if 'theta.vel' in self._mobile_base_features:
                self._state_vector['theta.vel'] = float(msg.twist.twist.angular.z)

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
        with self._lock:
            self._images[cam_name] = image

    def _get_ee_pose_left(self) -> Optional[np.ndarray]:
        """Return left EE pose as [x, y, z, roll, pitch, yaw] in base_footprint frame."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._ee_left_base_frame,
                self._ee_left_target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            tx = t.transform.translation
            q = t.transform.rotation
            try:
                from scipy.spatial.transform import Rotation as _R
                rpy = _R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            except ImportError:
                import math as _math
                sinr = 2.0 * (q.w * q.x + q.y * q.z)
                cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                roll = _math.atan2(sinr, cosr)
                sinp = 2.0 * (q.w * q.y - q.z * q.x)
                pitch = _math.asin(max(-1.0, min(1.0, sinp)))
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = _math.atan2(siny, cosy)
                rpy = [roll, pitch, yaw]
            return np.array([tx.x, tx.y, tx.z, rpy[0], rpy[1], rpy[2]], dtype=np.float32)
        except Exception:
            return None

    def _snapshot_observation(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if any(self._images[c] is None for c in self._camera_names):
                return None
            obs = dict(self._state_vector)
            for cam_name, image in self._images.items():
                obs[cam_name] = image.copy()

        if self._obs_features is None and _LEROBOT_AVAILABLE:
            # Build dataset feature spec: float for joints+base, (H,W,C) tuple for cameras.
            # Mobile base velocities must be included here so that hw_to_dataset_features
            # maps them into observation.state alongside joint positions.  The dataset
            # convention uses base_x/base_y/base_theta; _BASE_KEY_ALIASES maps those to
            # x.vel/y.vel/theta.vel at action publish time, but for the *observation*
            # the state vector stores them under the deploy keys (x.vel etc.), so we
            # use those same keys here and rely on the model's normalizer ordering.
            hw_features: Dict[str, Any] = {feature: float for feature in self._joint_features}
            for base_feat in self._mobile_base_features:
                hw_features[base_feat] = float
            for cam_name in self._camera_names:
                img = obs[cam_name]
                hw_features[cam_name] = img.shape  # (H, W, C)
            self._obs_features = hw_to_dataset_features(hw_features, 'observation')

        if not _LEROBOT_AVAILABLE or self._obs_features is None:
            return obs

        # v0.5.1: ds_features first, values second, then prefix
        frame = build_dataset_frame(self._obs_features, obs, 'observation')

        # Add features that are not produced by hw_to_dataset_features but are required
        # by the model as separate dataset keys.
        ee_pose = self._get_ee_pose_left()
        if ee_pose is not None:
            frame['observation.ee_pose.left'] = ee_pose
            frame['observation.ee_pose.left.delta'] = (
                ee_pose - self._prev_ee_pose_left
                if self._prev_ee_pose_left is not None
                else np.zeros(6, dtype=np.float32)
            )
            self._prev_ee_pose_left = ee_pose.copy()
        else:
            frame['observation.ee_pose.left'] = np.zeros(6, dtype=np.float32)
            frame['observation.ee_pose.left.delta'] = np.zeros(6, dtype=np.float32)

        # All joint states are fresh: published at 200 Hz, far above the 10 Hz control rate.
        state_dim = (
            frame['observation.state'].shape[-1]
            if 'observation.state' in frame and hasattr(frame['observation.state'], 'shape')
            else len(self._joint_features) + len(self._mobile_base_features)
        )
        frame['observation.state.is_fresh'] = np.ones(state_dim, dtype=np.float32)

        # The normalizer's stats were computed on a state vector with all joints
        # including mimic joints excluded from the YAML (e.g. r_mcp_joint ×2).
        # Pad observation.state to the expected_state_dim set during _load_policy,
        # filling missing joints with 0.0 in model-ordering order.
        import torch
        expected_state_dim = getattr(self, '_expected_state_dim', None)
        if expected_state_dim is not None and 'observation.state' in frame:
            state_arr = frame['observation.state']
            current_dim = state_arr.shape[-1] if hasattr(state_arr, 'shape') else len(state_arr)
            if current_dim < expected_state_dim:
                model_names = getattr(self, '_model_action_feature_names', None)
                if model_names is not None:
                    model_joint_names = [
                        n for n in model_names
                        if n not in self._BASE_KEY_ALIASES
                        and n not in self._BASE_KEY_ALIASES.values()
                    ]
                else:
                    # fallback: just zero-pad to expected dim
                    model_joint_names = None
                yaml_index = {name: i for i, name in enumerate(self._joint_features)}
                state_tensor = (
                    torch.from_numpy(state_arr)
                    if isinstance(state_arr, np.ndarray)
                    else torch.tensor(state_arr, dtype=torch.float32)
                )
                padded = torch.zeros(expected_state_dim, dtype=state_tensor.dtype)
                if model_joint_names is not None:
                    for i, name in enumerate(model_joint_names):
                        if i < expected_state_dim and name in yaml_index:
                            padded[i] = state_tensor[yaml_index[name]]
                else:
                    padded[:current_dim] = state_tensor
                frame['observation.state'] = padded.numpy()

        return frame

    def _predict_actions(self, obs_frame: Dict[str, Any]) -> List[Dict[str, float]]:
        import torch

        device = torch.device(self._model_device)

        if self._rtc_enabled and hasattr(self._policy, 'predict_action_chunk'):
            try:
                prev_left_over = self._chunk_buffer.left_over(self._rtc_inference_delay)
                t0 = monotonic()
                raw_chunk = self._policy.predict_action_chunk(
                    obs_frame,
                    inference_delay=self._rtc_inference_delay,
                    prev_chunk_left_over=prev_left_over,
                )
                elapsed = monotonic() - t0
                if elapsed > 0.2:
                    self.get_logger().warn(
                        'RTC inference {:.0f}ms > 200ms threshold.'.format(elapsed * 1000)
                    )
                steps = self._to_action_steps(raw_chunk)
                if steps:
                    return steps
            except Exception as exc:
                self.get_logger().warn(
                    'RTC chunk inference failed, falling back: {}'.format(exc)
                )

        if _LEROBOT_AVAILABLE and self._preprocessor is not None:
            t0 = monotonic()
            raw_action = predict_action(
                obs_frame,
                self._policy,
                device,
                self._preprocessor,
                self._postprocessor,
                self._model_use_amp,
            )
            elapsed = monotonic() - t0
            if elapsed > 0.2:
                self.get_logger().warn(
                    'Inference {:.0f}ms > 200ms threshold.'.format(elapsed * 1000)
                )
        else:
            with torch.inference_mode():
                raw_action = self._policy.select_action(obs_frame)

        return self._to_action_steps(raw_action)

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

    def _inference_worker(self) -> None:
        # single_step_mode: publish timer pops _single_step_result directly;
        # this thread just keeps running inference and depositing results.
        if self._single_step_mode:
            while rclpy.ok() and not self._shutdown_inference:
                with self._inference_cond:
                    if not self._play_enabled:
                        self._inference_cond.wait(timeout=0.5)
                        continue

                obs_frame = self._snapshot_observation()
                if obs_frame is None:
                    with self._inference_cond:
                        self._inference_cond.wait(timeout=0.05)
                    continue

                try:
                    steps = self._predict_actions(obs_frame)
                except Exception as exc:
                    import traceback as _tb
                    self.get_logger().error(
                        'Inference error: {}. Retrying.\n{}'.format(exc, _tb.format_exc()),
                        throttle_duration_sec=2.0,
                    )
                    continue

                if steps:
                    with self._single_step_lock:
                        self._single_step_result = steps[0]
            return

        # Default chunked mode
        while rclpy.ok() and not self._shutdown_inference:
            with self._inference_cond:
                play = self._play_enabled
                queue_len = self._chunk_buffer.size()
                threshold_len = max(
                    1, int(self._actions_per_chunk * self._chunk_size_threshold)
                )
                need_infer = play and (
                    (self._async_enabled and queue_len <= threshold_len)
                    or (not self._async_enabled and queue_len == 0)
                )
                if not need_infer:
                    self._inference_cond.wait(timeout=0.5)
                    continue

            obs_frame = self._snapshot_observation()
            if obs_frame is None:
                with self._inference_cond:
                    self._inference_cond.wait(timeout=0.1)
                continue

            try:
                chunk = self._predict_actions(obs_frame)
            except Exception as exc:
                import traceback as _tb
                self.get_logger().error(
                    'Inference error: {}. Retrying.\n{}'.format(exc, _tb.format_exc()),
                    throttle_duration_sec=2.0,
                )
                with self._inference_cond:
                    self._inference_cond.wait(timeout=1.0)
                continue

            if chunk:
                overlap = self._rtc_inference_delay if self._rtc_enabled else 0
                self._chunk_buffer.merge(chunk, overlap)

    def _publish_next_action(self) -> None:
        if not self._play_enabled:
            return

        if self._single_step_mode:
            with self._single_step_lock:
                step = self._single_step_result
                self._single_step_result = None
            if step is None:
                self.get_logger().warn(
                    'Single-step inference not ready yet.',
                    throttle_duration_sec=2.0,
                )
                return
        else:
            queue_len = self._chunk_buffer.size()
            threshold_len = max(1, int(self._actions_per_chunk * self._chunk_size_threshold))
            if self._async_enabled and queue_len <= threshold_len:
                with self._inference_cond:
                    self._inference_cond.notify_all()

            step = self._chunk_buffer.pop()
            if step is None:
                self.get_logger().warn(
                    'Action queue empty. Increase actions_per_chunk or lower control_hz.',
                    throttle_duration_sec=2.0,
                )
                return

        now = self.get_clock().now().to_msg()
        for group in self._joint_groups:
            msg = JointTrajectory()
            msg.header.stamp = now
            msg.joint_names = group.joints_ros
            point = JointTrajectoryPoint()
            raw_positions = [
                float(step.get(feature, self._state_vector.get(feature, 0.0)))
                for feature in group.features
            ]
            if self._max_joint_delta > 0.0:
                point.positions = [
                    float(
                        max(
                            self._state_vector.get(feat, raw) - self._max_joint_delta,
                            min(
                                self._state_vector.get(feat, raw) + self._max_joint_delta,
                                raw,
                            ),
                        )
                    )
                    for feat, raw in zip(group.features, raw_positions)
                ]
            else:
                point.positions = raw_positions
            point.time_from_start = self._step_duration
            msg.points = [point]
            self._group_publishers[group.name].publish(msg)

        if self._base_pub is not None:
            cmd = Twist()
            cmd.linear.x = float(step.get('x.vel', 0.0))
            cmd.linear.y = float(step.get('y.vel', 0.0))
            cmd.linear.z = float(step.get('z.vel', 0.0))
            cmd.angular.z = float(step.get('theta.vel', 0.0))
            self._base_pub.publish(cmd)
            print(
                'BASE cmd_vel  x={:.4f}  y={:.4f}  theta={:.4f}'.format(
                    cmd.linear.x, cmd.linear.y, cmd.angular.z
                ),
                flush=True,
            )


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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
