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

import math
import os
os.environ["PYTORCH_ALLOC_CONF"] = "expandable_segments:True"

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

from sobits_vla_deploy.vla_episode_logger import EpisodeLogger

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
    from lerobot.policies.rtc.latency_tracker import LatencyTracker
    from lerobot.configs.types import RTCAttentionSchedule
    _RTC_AVAILABLE = True
except ImportError:
    RTCConfig = None  # type: ignore[assignment,misc]
    LatencyTracker = None  # type: ignore[assignment]
    RTCAttentionSchedule = None  # type: ignore[assignment]
    _RTC_AVAILABLE = False

# ---------------------------------------------------------------------------
# Monkey-patch PI05Policy.from_pretrained to honour torch_dtype.
#
# The upstream lerobot implementation is a fully custom loader that ignores
# torch_dtype: it constructs the model skeleton in float32, immediately moves
# it to config.device (OOM on 12 GB VRAM), and calls load_file() without a
# dtype argument (loads the 14 GB checkpoint as float32, doubling peak RAM).
#
# This patch intercepts from_pretrained BEFORE the first call and injects:
#   1. Build skeleton on CPU regardless of config.device.
#   2. Cast skeleton to torch_dtype right after construction.
#   3. Cast each state-dict tensor to torch_dtype after load_file().
#   4. Move the final bfloat16 model to the target device.
# ---------------------------------------------------------------------------
def _patch_pi05_from_pretrained() -> None:
    try:
        import types
        import torch as _torch
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        from safetensors.torch import load_file as _sf_load_file
        from transformers.utils import cached_file as _cached_file

        _original_from_pretrained = PI05Policy.from_pretrained.__func__  # type: ignore[attr-defined]

        @classmethod  # type: ignore[misc]
        def _patched_from_pretrained(
            cls,
            pretrained_name_or_path,
            *,
            config=None,
            strict: bool = True,
            **kwargs,
        ):
            import builtins, gc
            from lerobot.configs.policies import PreTrainedConfig

            torch_dtype = kwargs.get('torch_dtype', None)

            # --- build config if not provided (mirrors upstream logic) ---
            if config is None:
                config = PreTrainedConfig.from_pretrained(
                    pretrained_name_or_path=pretrained_name_or_path, **kwargs
                )

            # --- construct skeleton on CPU; defer GPU move until after load ---
            target_device = getattr(config, 'device', 'cpu') or 'cpu'
            config.device = 'cpu'
            model = cls(config, **kwargs)
            config.device = target_device

            if torch_dtype is not None:
                model.to(dtype=torch_dtype)

            # --- resolve and load safetensors ---
            try:
                resolved_file = _cached_file(
                    pretrained_name_or_path,
                    'model.safetensors',
                    cache_dir=kwargs.get('cache_dir'),
                    force_download=kwargs.get('force_download', False),
                    resume_download=kwargs.get('resume_download'),
                    proxies=kwargs.get('proxies'),
                    token=kwargs.get('token'),
                    revision=kwargs.get('revision'),
                    local_files_only=kwargs.get('local_files_only', False),
                )
                state_dict = _sf_load_file(resolved_file)
                if torch_dtype is not None:
                    state_dict = {k: v.to(dtype=torch_dtype) for k, v in state_dict.items()}
            except Exception as exc:
                import logging
                logging.getLogger(__name__).warning(
                    'PI05 patch: could not load state dict: %s', exc
                )
                return model

            # --- key remapping (mirrors upstream) ---
            state_dict = model._fix_pytorch_state_dict_keys(state_dict, model.config)
            state_dict = {
                (k if k.startswith('model.') else f'model.{k}'): v
                for k, v in state_dict.items()
            }
            model.load_state_dict(state_dict, strict=strict)
            del state_dict
            gc.collect()

            # --- move bfloat16 model to target device ---
            if target_device and target_device != 'cpu':
                model.model.to(target_device)
                gc.collect()
                if _torch.cuda.is_available():
                    _torch.cuda.empty_cache()

            return model

        PI05Policy.from_pretrained = _patched_from_pretrained
    except Exception:
        pass  # lerobot not installed or PI05 not available; no-op


_patch_pi05_from_pretrained()

# Registry: policy_class_path ->
#   (config_module, config_class, config_has_device_field, supports_rtc, cast_bf16)
# supports_rtc: config exposes rtc_config (flow-matching pi-family policies).
# cast_bf16: load weights in bfloat16 and keep the model in bf16 on GPU.
_POLICY_CONFIG_REGISTRY: Dict[str, tuple] = {
    'lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy': (
        'lerobot.policies.smolvla.configuration_smolvla', 'SmolVLAConfig', False, True, False,
    ),
    'lerobot.policies.pi0.modeling_pi0.PI0Policy': (
        'lerobot.policies.pi0.configuration_pi0', 'PI0Config', True, True, True,
    ),
    'lerobot.policies.pi05.modeling_pi05.PI05Policy': (
        'lerobot.policies.pi05.configuration_pi05', 'PI05Config', True, True, True,
    ),
    'lerobot.policies.pi0_fast.modeling_pi0_fast.PI0FastPolicy': (
        'lerobot.policies.pi0_fast.configuration_pi0_fast', 'PI0FastConfig', True, True, True,
    ),
    'lerobot.policies.act.modeling_act.ACTPolicy': (
        'lerobot.policies.act.configuration_act', 'ACTConfig', True, False, False,
    ),
    'lerobot.policies.groot.modeling_groot.GrootPolicy': (
        'lerobot.policies.groot.configuration_groot', 'GrootConfig', True, False, True,
    ),
}


def _registry_flag(policy_class_path: str, index: int, default: bool) -> bool:
    entry = _POLICY_CONFIG_REGISTRY.get(policy_class_path)
    if entry is None or len(entry) <= index:
        return default
    return bool(entry[index])


@dataclass
class JointGroupConfig:
    name: str
    command_topic: str
    joints_ros: List[str]
    features: List[str]
    max_joint_delta: float = 0.0  # 0.0 = no clamping


class ActionChunkBuffer:
    def __init__(self, aggregate_fn_name: str) -> None:
        self._queue: deque[Dict[str, float]] = deque()
        # Model-space (normalised) tensor kept in parallel with _queue for RTC
        # left_over guidance. Shape: (T_remaining, A) on CPU.
        self._original_queue: Optional[Any] = None  # torch.Tensor | None
        self._lock = Lock()
        self._aggregate_fn_name = aggregate_fn_name

    def size(self) -> int:
        with self._lock:
            return len(self._queue)

    def pop(self) -> Optional[Dict[str, float]]:
        with self._lock:
            if self._original_queue is not None and len(self._original_queue) > 0:
                self._original_queue = self._original_queue[1:]
                if len(self._original_queue) == 0:
                    self._original_queue = None
            if not self._queue:
                return None
            return self._queue.popleft()

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()
            self._original_queue = None

    def left_over(self, count: int) -> Optional[Any]:
        """Return all remaining model-space steps as a Tensor (T, A), or None.

        Mirrors LeRobot ActionQueue.get_left_over() which returns
        original_queue[last_index:] — i.e. everything unconsumed, not capped
        to *count*. The RTC processor (modeling_rtc.py) handles zero-padding
        internally when the leftover is shorter than the chunk.

        The *count* argument is retained for call-site compatibility but is not
        used to truncate; it only guards against returning anything when no
        delay has been estimated yet (count <= 0 → first inference, no guidance).
        """
        if count <= 0:
            return None
        with self._lock:
            if self._original_queue is None or len(self._original_queue) == 0:
                return None
            return self._original_queue.clone()

    def replace(self, original_actions: Any, processed_steps: List[Dict[str, float]], delay: int) -> None:
        """RTC queue replacement (Bug 2 fix): discard stale entries, start fresh.

        Mirrors LeRobot's ActionQueue._replace_actions_queue: drop all unexecuted
        old steps and begin from *delay* steps into the new chunk so execution stays
        synchronised with real time.
        """
        if not processed_steps and original_actions is None:
            return
        import torch
        with self._lock:
            clamped = 0
            if original_actions is not None and len(processed_steps) > 0:
                clamped = max(0, min(delay, len(original_actions), len(processed_steps)))
            self._queue = deque(processed_steps[clamped:])
            if original_actions is not None and len(original_actions) > clamped:
                self._original_queue = original_actions[clamped:].clone().cpu()
            else:
                self._original_queue = None

    def merge(self, chunk: List[Dict[str, float]], overlap: int) -> None:
        if not chunk:
            return
        with self._lock:
            overlap_steps = min(overlap, len(self._queue), len(chunk))
            for idx in range(overlap_steps):
                self._queue[idx] = self._aggregate(self._queue[idx], chunk[idx])
            for step in chunk[overlap_steps:]:
                self._queue.append(step)

    def merge_aligned(self, chunk: List[Dict[str, float]], q_len_at_obs: int) -> None:
        if not chunk:
            return
        with self._lock:
            q_len_now = len(self._queue)
            steps_executed = max(0, q_len_at_obs - q_len_now)
            overlap_steps = min(q_len_now, max(0, len(chunk) - steps_executed))
            for idx in range(overlap_steps):
                self._queue[idx] = self._aggregate(self._queue[idx], chunk[idx + steps_executed])
            for step in chunk[overlap_steps + steps_executed:]:
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
        self._model_use_relative_actions = False
        self._load_robot_profile()
        self._load_policy()

        self._state_vector: Dict[str, float] = {
            feature: 0.0 for feature in self._joint_features
        }
        self._cmd_vector: Dict[str, float] = {}
        self._images: Dict[str, Optional[np.ndarray]] = {
            cam_name: None for cam_name in self._camera_names
        }
        self._obs_features: Optional[Dict[str, Any]] = None
        self._play_enabled = False
        self._shutdown_inference = False

        self._chunk_buffer = ActionChunkBuffer(self._aggregate_fn_name)
        # Dynamic latency tracker for RTC inference delay
        self._latency_tracker: Optional[Any] = None
        if self._rtc_enabled and _RTC_AVAILABLE and LatencyTracker is not None:
            self._latency_tracker = LatencyTracker(maxlen=20)
            seed_latency = self._rtc_inference_delay / max(self._control_hz, 1.0)
            self._latency_tracker.add(seed_latency)
        self._single_step_result: Optional[Dict[str, float]] = None
        self._single_step_lock = Lock()
        self._task_label = str(self.get_parameter('model.default_task_label').value)

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

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._inference_thread = Thread(target=self._inference_worker, daemon=True)
        self._inference_thread.start()

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
            enabled=self._logging_enabled,
        )
        if self._logging_enabled:
            self.get_logger().info(
                'Episode logging enabled → {}'.format(self._log_dir)
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

        self.declare_parameter('gamepad.topic', '/joy')
        self.declare_parameter('gamepad.name', 'default')
        self.declare_parameter('gamepad.controllers', [''])

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

        # Episode logging parameters
        self.declare_parameter('logging.enabled', False)
        self.declare_parameter('logging.log_dir', '/tmp/vla_logs')
        self.declare_parameter('logging.tilt_threshold_deg', 30.0)

        # Simulation reset parameters (used by EpisodeLogger._reset_world)
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
        self._max_vel_x = float(self.get_parameter(f'{ns}.mobile_base.max_vel_x').value)
        self._max_vel_y = float(self.get_parameter(f'{ns}.mobile_base.max_vel_y').value)
        self._max_vel_theta = float(self.get_parameter(f'{ns}.mobile_base.max_vel_theta').value)
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

    def _build_rtc_config(self) -> Optional[Any]:
        """Build RTCConfig if RTC enabled and policy supports it."""
        if not self._rtc_enabled or not _RTC_AVAILABLE:
            return None
        if not _registry_flag(self._policy_class_path, 3, default=True):
            self.get_logger().info(
                'Policy {!r} does not support RTC (no rtc_config field) — '
                'disabling RTC; chunked execution will be used.'.format(
                    self._policy_class_path
                )
            )
            self._rtc_enabled = False
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
        config_module_path, config_class_name, has_device = entry[0], entry[1], entry[2]
        supports_rtc = entry[3] if len(entry) > 3 else True
        try:
            config_mod = import_module(config_module_path)
            config_cls = getattr(config_mod, config_class_name)
            kwargs: Dict[str, Any] = {}
            if rtc_cfg is not None and supports_rtc:
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

    def _build_cfg_from_repo_json(self, policy_cls) -> Optional[Any]:
        """Build a typed policy config from the model repo's config.json.

        Required for FULL-model repos (no adapter): passing a bare ROS-built
        config to from_pretrained clobbers the repo's input_features, leaving
        image_features empty ("All image features are missing from the
        batch"). Mirrors the adapter branch's manual JSON parsing; generic
        across policies by filtering to the config class's dataclass fields.
        """
        try:
            import json
            from dataclasses import fields as _dc_fields
            from lerobot.configs.types import PolicyFeature, FeatureType as FT

            cfg_path = self._fetch_model_file(self._model_repo_id, 'config.json')
            with open(cfg_path) as fh:
                d = json.load(fh)
            config_cls = policy_cls.config_class
            valid = {f.name for f in _dc_fields(config_cls)}
            kwargs: Dict[str, Any] = {k: v for k, v in d.items() if k in valid}
            for feat_key in ('input_features', 'output_features'):
                if d.get(feat_key):
                    kwargs[feat_key] = {
                        k: PolicyFeature(type=FT[v['type']], shape=tuple(v['shape']))
                        for k, v in d[feat_key].items()
                    }
            if isinstance(kwargs.get('image_resolution'), list):
                kwargs['image_resolution'] = tuple(kwargs['image_resolution'])
            # Nested/enum-valued fields not constructible from raw JSON — use
            # class defaults (rtc_config is injected post-load anyway).
            kwargs.pop('rtc_config', None)
            kwargs.pop('normalization_mapping', None)
            # Training-only flags must not leak into deployment:
            #  - compile_model=true stalls the FIRST inference for minutes
            #    (max-autotune kernel compilation) while the action queue
            #    starves, and RTC's autograd guidance inside a compiled
            #    sample_actions is untested interplay;
            #  - gradient_checkpointing is meaningless at inference.
            # Only set them where the config class HAS the field — smolvla
            # has neither and an unknown kwarg aborts the whole repo-config
            # build (falling back to a featureless config).
            for _flag in ('compile_model', 'gradient_checkpointing'):
                if _flag in valid:
                    kwargs[_flag] = False
            # CPU-first load; moved to GPU after weights arrive.
            kwargs['device'] = 'cpu'
            cfg = config_cls(**kwargs)
            self.get_logger().info(
                'Built {} from repo config.json. Image features: {}'.format(
                    config_cls.__name__,
                    list(getattr(cfg, 'image_features', {}) or {}),
                )
            )
            return cfg
        except Exception as exc:
            self.get_logger().warn(
                'Could not build config from repo config.json: {}'.format(exc)
            )
            return None

    @staticmethod
    def _fetch_model_file(repo_id: str, filename: str) -> str:
        """Resolve a model file from a LOCAL directory or the HF Hub.

        model.repo_id may be a local path (e.g. a lerobot training checkpoint's
        .../checkpoints/0015000/pretrained_model directory) — hf_hub_download
        rejects filesystem paths, so check for a local file first.
        """
        from pathlib import Path
        local = Path(repo_id) / filename
        if local.is_file():
            return str(local)
        from huggingface_hub import hf_hub_download
        return hf_hub_download(repo_id, filename)

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
        """Zero-pad or truncate projection weights when action/state dims differ from pre-trained weights.

        Prevents PyTorch load_state_dict mismatch crashes which cause LeRobot to fall back
        to fully randomized weights.
        """
        import torch

        def make_patched_fix(orig_fix):
            def _patched_fix(self, state_dict, model_config):
                fixed = orig_fix(self, state_dict, model_config)
                
                # Action dimension remapping
                model_action_dim = self.model.action_in_proj.in_features
                
                # State dimension remapping (PI0 has state_proj, PI05 does not)
                model_state_dim = None
                if hasattr(self.model, 'state_proj'):
                    model_state_dim = self.model.state_proj.in_features

                for key in list(fixed.keys()):
                    val = fixed[key]
                    
                    # action_in_proj.weight: (width, ckpt_dim) → (width, model_dim)
                    if key.endswith('action_in_proj.weight') and val.ndim == 2:
                        if val.shape[1] < model_action_dim:
                            extra = model_action_dim - val.shape[1]
                            pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                            fixed[key] = torch.cat([val, pad], dim=1)
                        elif val.shape[1] > model_action_dim:
                            fixed[key] = val[:, :model_action_dim]
                    
                    # action_out_proj.weight: (ckpt_dim, width) → (model_dim, width)
                    elif key.endswith('action_out_proj.weight') and val.ndim == 2:
                        if val.shape[0] < model_action_dim:
                            extra = model_action_dim - val.shape[0]
                            pad = torch.zeros(extra, val.shape[1], dtype=val.dtype, device=val.device)
                            fixed[key] = torch.cat([val, pad], dim=0)
                        elif val.shape[0] > model_action_dim:
                            fixed[key] = val[:model_action_dim, :]
                    
                    # action_out_proj.bias: (ckpt_dim,) → (model_dim,)
                    elif key.endswith('action_out_proj.bias') and val.ndim == 1:
                        if val.shape[0] < model_action_dim:
                            extra = model_action_dim - val.shape[0]
                            pad = torch.zeros(extra, dtype=val.dtype, device=val.device)
                            fixed[key] = torch.cat([val, pad], dim=0)
                        elif val.shape[0] > model_action_dim:
                            fixed[key] = val[:model_action_dim]
                            
                    # state_proj.weight: (width, ckpt_dim) → (width, model_dim)
                    elif key.endswith('state_proj.weight') and val.ndim == 2 and model_state_dim is not None:
                        if val.shape[1] < model_state_dim:
                            extra = model_state_dim - val.shape[1]
                            pad = torch.zeros(val.shape[0], extra, dtype=val.dtype, device=val.device)
                            fixed[key] = torch.cat([val, pad], dim=1)
                        elif val.shape[1] > model_state_dim:
                            fixed[key] = val[:, :model_state_dim]
                            
                return fixed
            return _patched_fix

        # Intercept PI05 Policy if available
        try:
            from lerobot.policies.pi05.modeling_pi05 import PI05Policy
            orig_fix_pi05 = PI05Policy._fix_pytorch_state_dict_keys
            PI05Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi05)
        except ImportError:
            pass

        # Intercept PI0 Policy if available
        try:
            from lerobot.policies.pi0.modeling_pi0 import PI0Policy
            orig_fix_pi = PI0Policy._fix_pytorch_state_dict_keys
            PI0Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi)
        except ImportError:
            pass

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
            adapter_cfg_path = self._fetch_model_file(self._model_repo_id, 'adapter_config.json')
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
                adapter_policy_json_path = self._fetch_model_file(self._model_repo_id, 'config.json')
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
                _img_res_raw = adapter_policy_dict.get('image_resolution', [224, 224])
                _img_res = tuple(_img_res_raw) if not isinstance(_img_res_raw, tuple) else _img_res_raw
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
                    image_resolution=_img_res,
                    dtype=adapter_policy_dict.get('dtype', 'bfloat16'),
                )
                if 'action_feature_names' in adapter_policy_dict:
                    adapter_policy_cfg.action_feature_names = adapter_policy_dict['action_feature_names']
                if 'use_relative_actions' in adapter_policy_dict:
                    adapter_policy_cfg.use_relative_actions = adapter_policy_dict['use_relative_actions']
                if 'relative_exclude_joints' in adapter_policy_dict:
                    adapter_policy_cfg.relative_exclude_joints = adapter_policy_dict['relative_exclude_joints']
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
            # FULL-model repo: the config must come from the repo's
            # config.json (input/output features, action_feature_names,
            # use_relative_actions, ...) — the bare ROS-built cfg has empty
            # features and would blind the policy to its cameras.
            repo_cfg = self._build_cfg_from_repo_json(policy_cls)
            load_cfg = repo_cfg if repo_cfg is not None else cfg
            load_kwargs: Dict[str, Any] = {'strict': False}
            if _registry_flag(self._policy_class_path, 4, default=True):
                load_kwargs['torch_dtype'] = torch.bfloat16
            if load_cfg is not None:
                load_kwargs['config'] = load_cfg
            self._policy = policy_cls.from_pretrained(self._model_repo_id, **load_kwargs)

        self._policy.eval()

        cast_bf16 = _registry_flag(self._policy_class_path, 4, default=True)
        if load_device != 'cpu' and not cast_bf16:
            # Small float32 policies (e.g. ACT ~80M): plain move, no bf16 dance.
            self._policy.to(torch.device(load_device))
            self.get_logger().info('Model moved to GPU (float32).')

        if load_device != 'cpu' and cast_bf16:
            import gc as _gc
            _gc.collect()
            torch.cuda.empty_cache()

            # Enforce pure bfloat16 casting to override selected float32 params and save VRAM
            self._policy.to(dtype=torch.bfloat16)

            gpu_device = torch.device(load_device)
            self.get_logger().info('Moving bfloat16 model to {}...'.format(load_device))
            # nn.Module.to() already moves parameter-by-parameter, freeing each
            # CPU tensor as its GPU copy is made — a recursive per-child move
            # with gc/empty_cache between children has the identical memory
            # profile and only adds startup latency.
            self._policy.to(gpu_device)
            _gc.collect()
            torch.cuda.empty_cache()
            self.get_logger().info('Model moved to GPU.')

            # Restore the model's own selective-precision layout. PI05/PI0
            # training keeps the vision tower, multi-modal projector and all
            # layernorms in float32 (to_bfloat16_for_selected_params,
            # modeling_pi05.py:397-418); the blanket bf16 cast above flattened
            # them. embed_image feeds float32 images to the vision tower on
            # the assumption it stayed float32 — without this restore the
            # first inference dtype-mismatches, and bf16 layernorms deviate
            # from training numerics. Costs ~0.5-0.8 GB VRAM (vision path
            # back to fp32), which is the trained configuration.
            _pwe = getattr(getattr(self._policy, 'model', None),
                           'paligemma_with_expert', None)
            if _pwe is not None and hasattr(_pwe, 'to_bfloat16_for_selected_params'):
                _pwe.to_bfloat16_for_selected_params('bfloat16')
                self.get_logger().info(
                    'Re-applied selective precision: vision tower + norms float32.'
                )

            # Restore the model-level projections to float32, completing the
            # TRAINING dtype layout. At training time PI05Pytorch.__init__
            # constructs action_in_proj / action_out_proj / time_mlp_in /
            # time_mlp_out (pi0: action_time_mlp_* / state_proj) as plain
            # nn.Linear — float32, never cast; only paligemma_with_expert is
            # bf16 (with the fp32 keep-list restored above). The full fp32
            # action/time path (fp32 noise → fp32 action_in_proj; fp32
            # timestep → fp32 time MLPs → fp32 adarms_cond → fp32 AdaRMS
            # dense) is upstream's design — earlier patches that forced this
            # path to bf16 (sample_noise/embed_suffix) collided with the fp32
            # AdaRMS weights ("mat1 BFloat16 and mat2 Float") and are removed.
            _mdl = getattr(self._policy, 'model', None)
            if _mdl is not None:
                _restored = []
                for _attr in ('action_in_proj', 'action_out_proj',
                              'time_mlp_in', 'time_mlp_out',
                              'action_time_mlp_in', 'action_time_mlp_out',
                              'state_proj'):
                    _sub = getattr(_mdl, _attr, None)
                    if _sub is not None:
                        _sub.to(dtype=torch.float32)
                        _restored.append(_attr)
                if _restored:
                    self.get_logger().info(
                        'Restored float32 model-level projections (training '
                        'layout): {}'.format(_restored)
                    )

        if hasattr(self._policy, 'reset'):
            self._policy.reset()

        # Ensure RTC processor is initialized on the policy (Bug 3 fix).
        # _build_policy_config() injects rtc_config into the config object, but
        # PI05Policy.__init__ only calls init_rtc_processor() when constructed from
        # scratch — from_pretrained() bypasses that path, leaving rtc_processor=None.
        # For the PEFT adapter path the config was built without rtc_config at all,
        # so we also inject it here before initialising.
        if self._rtc_enabled and _RTC_AVAILABLE:
            rtc_cfg_for_init = self._build_rtc_config()
            if rtc_cfg_for_init is not None and hasattr(self._policy, 'config'):
                if getattr(self._policy.config, 'rtc_config', None) is None:
                    self._policy.config.rtc_config = rtc_cfg_for_init
            if hasattr(self._policy, 'init_rtc_processor'):
                self._policy.init_rtc_processor()
                self.get_logger().info('RTC processor initialized on policy.')
            else:
                self.get_logger().warn(
                    'Policy does not support init_rtc_processor(). RTC disabled.'
                )
                self._rtc_enabled = False

        # Read action_feature_names from the model config so _to_action_steps can map
        # by name instead of by position. Falls back to None (position-based) if absent.
        self._model_action_feature_names: Optional[List[str]] = getattr(
            self._policy.config, 'action_feature_names', None
        )

        # model config is authoritative for use_relative_actions
        model_relative = False
        if hasattr(self, '_policy') and hasattr(self._policy, 'config') and self._policy.config is not None:
            model_relative = getattr(self._policy.config, 'use_relative_actions', False)

        yaml_relative = bool(self.get_parameter('model.use_relative_actions').value)
        if yaml_relative != model_relative:
            self.get_logger().warn(
                f'use_relative_actions mismatch: robot_config YAML={yaml_relative}, '
                f'model config (baked into weights)={model_relative}. '
                f'Prioritising model config ({model_relative}). '
                f'Update use_relative_actions in your robot_config YAML to match how the model was trained.'
            )
        self._model_use_relative_actions = model_relative
        self.get_logger().info(
            'Relative actions mode: {}'.format(self._model_use_relative_actions)
        )

        # Warn if relative_exclude_joints is still the unmatched default
        _deploy_exclude = list(
            getattr(getattr(self, '_policy', None), 'config', None) and
            getattr(self._policy.config, 'relative_exclude_joints', None)
            or []
        )
        # policy.config may return a falsy empty list — retrieve directly if so
        if hasattr(self, '_policy') and hasattr(self._policy, 'config') and self._policy.config is not None:
            _deploy_exclude = list(getattr(self._policy.config, 'relative_exclude_joints', []) or [])
        if self._model_use_relative_actions and _deploy_exclude == ['gripper']:
            self.get_logger().warn(
                'use_relative_actions=true but relative_exclude_joints is the default [\"gripper\"], '
                'which matches no joint in SOBIT HOME (fingers are hand_left_finger_*). '
                'Velocity joints (base_x, base_y, base_theta) may be incorrectly delta-converted. '
                'Set relative_exclude_joints in your adapter policy config or robot_config YAML.'
            )

        # cross-check deploy joint names against model_action_feature_names
        if self._model_action_feature_names:
            self.get_logger().info(
                'Model action_feature_names: {}'.format(self._model_action_feature_names)
            )
            yaml_features = self._joint_features + self._mobile_base_features
            _alias_rev = {v: k for k, v in self._BASE_KEY_ALIASES.items()}
            model_names_set = set(self._model_action_feature_names)
            missing = [
                f for f in yaml_features
                if f not in model_names_set and _alias_rev.get(f) not in model_names_set
            ]
            wired = set(yaml_features) | {_alias_rev.get(f, f) for f in yaml_features}
            unknown = [f for f in self._model_action_feature_names if f not in wired]
            if missing:
                # Mismatch between deploy joint names and model's training joint names.
                # Raise an error rather than silently running wrong.
                raise RuntimeError(
                    'Deploy joint names do not match model action_feature_names — '
                    'zeros would be inserted at wrong positions causing bad actions. '
                    'Missing from model: {}. '
                    'Update your robot_config YAML joints to match the trained model, '
                    'or retrain with the current joint set.'.format(missing)
                )
            if unknown:
                self.get_logger().warn(
                    'Model outputs joints not wired to any controller (ignored): '
                    '{}'.format(unknown)
                )

        # pre-flight check max_action_dim / max_state_dim
        if hasattr(self, '_policy') and hasattr(self._policy, 'config') and self._policy.config is not None:
            _policy_cfg = self._policy.config
            _max_action_dim = getattr(_policy_cfg, 'max_action_dim', None)
            _max_state_dim = getattr(_policy_cfg, 'max_state_dim', None)
            _actual_action_dim = len(self._joint_features + self._mobile_base_features)
            _actual_state_dim = _actual_action_dim
            if _max_action_dim is not None and _max_action_dim < _actual_action_dim:
                raise RuntimeError(
                    'max_action_dim={} < actual joint count={} — joints would be silently truncated. '
                    'Set max_action_dim >= {} in your adapter policy config.'.format(
                        _max_action_dim, _actual_action_dim, _actual_action_dim
                    )
                )
            if _max_state_dim is not None and _max_state_dim < _actual_state_dim:
                raise RuntimeError(
                    'max_state_dim={} < actual joint count={} — state would be silently truncated. '
                    'Set max_state_dim >= {} in your adapter policy config.'.format(
                        _max_state_dim, _actual_state_dim, _actual_state_dim
                    )
                )
            if _max_action_dim is not None:
                self.get_logger().info(
                    'max_action_dim={} >= actual={} OK'.format(_max_action_dim, _actual_action_dim)
                )

        # Build pre/post processor pipelines for v0.5.1 predict_action API.
        self._preprocessor = None
        self._postprocessor = None
        if _LEROBOT_AVAILABLE:
            processor_kwargs: Dict[str, Any] = {}
            is_groot = 'groot' in self._policy_class_path.lower()
            if self._model_dataset_repo_id:
                try:
                    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
                    _ds_meta = LeRobotDatasetMetadata(self._model_dataset_repo_id)
                    processor_kwargs['dataset_stats'] = _ds_meta.stats
                    self.get_logger().info(
                        'Loaded dataset stats from {!r} for processor build.'.format(
                            self._model_dataset_repo_id
                        )
                    )
                except Exception as exc:
                    self.get_logger().warn(
                        'Could not load dataset stats from {!r}: {}'.format(
                            self._model_dataset_repo_id, exc
                        )
                    )
            elif is_groot:
                self.get_logger().warn(
                    'GR00T policy without model.dataset_repo_id — the groot '
                    'processor will be built with stats=None and normalization '
                    'will be BROKEN. Set model.dataset_repo_id in the robot config.'
                )
            try:
                self._preprocessor, self._postprocessor = make_pre_post_processors(
                    self._policy.config, self._model_repo_id, **processor_kwargs
                )
            except Exception as exc:
                self.get_logger().warn(
                    'Could not build pre/post processors: {}. '
                    'Direct policy.select_action will be used.'.format(exc)
                )

        # cross-check postprocessor against use_relative_actions
        if self._postprocessor is not None:
            try:
                from lerobot.processor.relative_action_processor import AbsoluteActionsProcessorStep
                _abs_steps = [
                    s for s in self._postprocessor.steps
                    if isinstance(s, AbsoluteActionsProcessorStep)
                ]
                # A step with enabled=False is a no-op — only count active steps.
                _has_abs_step_active = any(
                    getattr(s, 'enabled', True) for s in _abs_steps
                )
                _has_abs_step = bool(_abs_steps)
                self.get_logger().info(
                    'Postprocessor AbsoluteActionsProcessorStep present: {} (active: {})'.format(
                        _has_abs_step, _has_abs_step_active
                    )
                )
                if self._model_use_relative_actions and not _has_abs_step_active:
                    self.get_logger().warn(
                        'use_relative_actions=true but postprocessor has no active AbsoluteActionsProcessorStep. '
                        'Delta→absolute conversion will be applied manually in _predict_actions. '
                        'This may indicate a stale cached postprocessor from a different training run.'
                    )
                elif not self._model_use_relative_actions and _has_abs_step_active:
                    self.get_logger().warn(
                        'use_relative_actions=false but postprocessor contains an active AbsoluteActionsProcessorStep. '
                        'Absolute actions will be treated as deltas and then un-deltified — likely a '
                        'stale cached postprocessor. Delete the HF cache and reload.'
                    )
            except ImportError:
                pass

        # Determine expected observation.state dim from the preprocessor normalizer stats
        self._expected_state_dim: Optional[int] = None
        _state_dim_source: str = 'disabled'
        try:
            from pathlib import Path as _Path
            from safetensors.torch import load_file as _st_load
            from huggingface_hub import list_repo_files as _list_files

            if _Path(self._model_repo_id).is_dir():
                repo_files = [p.name for p in _Path(self._model_repo_id).iterdir()]
            else:
                repo_files = _list_files(self._model_repo_id)
            normalizer_file = None
            for filename in repo_files:
                if filename.startswith('policy_preprocessor_step_') and filename.endswith('_normalizer_processor.safetensors'):
                    normalizer_file = filename
                    break

            if normalizer_file is not None:
                stats_path = self._fetch_model_file(self._model_repo_id, normalizer_file)
                stats = _st_load(stats_path)
                q01 = stats.get('observation.state.q01')
                if q01 is not None:
                    self._expected_state_dim = int(q01.shape[-1])
                    _state_dim_source = 'normalizer_stats ({})'.format(normalizer_file)
                else:
                    _state_dim_source = 'normalizer_stats_loaded_but_no_q01_key'
            else:
                _state_dim_source = 'no_normalizer_file_in_repo'
        except Exception as exc:
            _state_dim_source = 'exception ({})'.format(exc)

        # Fall back to model_action_feature_names length if normalizer stats unavailable
        if self._expected_state_dim is None and self._model_action_feature_names is not None:
            self._expected_state_dim = len(self._model_action_feature_names)
            _state_dim_source = 'action_feature_names_len (fallback)'

        self.get_logger().info(
            'expected_state_dim={} (source: {})'.format(
                self._expected_state_dim if self._expected_state_dim is not None else 'disabled',
                _state_dim_source,
            )
        )

    def _on_set_parameters(self, params: List[Any]) -> Any:
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'runtime.chunk_size_threshold':
                self._chunk_size_threshold = float(min(max(p.value, 0.0), 1.0))
                self.get_logger().info('chunk_size_threshold → {}'.format(self._chunk_size_threshold))
            elif p.name == 'runtime.aggregate_fn_name':
                self._aggregate_fn_name = str(p.value)
                self._chunk_buffer._aggregate_fn_name = self._aggregate_fn_name
                self.get_logger().info('aggregate_fn_name → {}'.format(self._aggregate_fn_name))
            elif p.name == 'runtime.async_enabled':
                self._async_enabled = bool(p.value)
                self.get_logger().info('async_enabled → {}'.format(self._async_enabled))
            elif p.name == 'runtime.control_hz':
                hz = float(max(p.value, 1.0))
                self._control_hz = hz
                self._control_timer.timer_period_ns = int((1.0 / hz) * 1e9)
                self._step_duration = Duration(sec=0, nanosec=int((1.0 / hz) * 1e9))
                self.get_logger().info('control_hz → {}'.format(hz))
            elif p.name == 'runtime.actions_per_chunk':
                self._actions_per_chunk = max(int(p.value), 1)
                self.get_logger().info('actions_per_chunk → {}'.format(self._actions_per_chunk))
            elif p.name == 'runtime.single_step_mode':
                self._single_step_mode = bool(p.value)
                self.get_logger().info('single_step_mode → {}'.format(self._single_step_mode))
            elif p.name == 'logging.enabled':
                self._logging_enabled = bool(p.value)
                self._episode_logger.enabled = self._logging_enabled
                self.get_logger().info('logging.enabled → {}'.format(self._logging_enabled))
            elif p.name == 'logging.tilt_threshold_deg':
                import math as _math
                self._log_tilt_deg = float(p.value)
                self._episode_logger._tilt_rad = _math.radians(self._log_tilt_deg)
                self.get_logger().info('logging.tilt_threshold_deg → {}'.format(self._log_tilt_deg))
        return SetParametersResult(successful=True)

    def _reset_episode_state(self) -> None:
        """Reset all per-episode model state after a stop/world-reset."""
        self._chunk_buffer.clear()
        if hasattr(self._policy, 'reset'):
            self._policy.reset()
        # Reset any stateful processor steps
        for pipeline in (self._preprocessor, self._postprocessor):
            if pipeline is not None:
                for step in pipeline.steps:
                    if hasattr(step, 'reset'):
                        step.reset()
        with self._lock:
            self._cmd_vector.clear()
        # Reset EE-pose delta tracking
        self._prev_ee_pose_left = None
        # Drop any action computed from the previous episode's last observation
        with self._single_step_lock:
            self._single_step_result = None
        self.get_logger().info('Episode model state reset.')
        Thread(target=self._do_world_reset, daemon=True).start()

    def _do_world_reset(self) -> None:
        """Teleport robot+block, then move robot to detecting_pose via action.

        Runs in a daemon thread so it never blocks the ROS spin.
        Always executes regardless of logging.enabled.
        """
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
        # Move to detecting_pose so next episode starts from the correct configuration
        goal = (
            "pose_name: 'detecting_pose'\n"
            "time_allowance:\n"
            "  sec: 1\n"
            "  nanosec: 500000000"
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
            self.get_logger().info('move_to_pose detecting_pose: {}'.format(
                'OK' if ok_pose else 'FAIL (rc={})'.format(result.returncode)
            ))
            if not ok_pose and result.stderr:
                self.get_logger().warn('move_to_pose stderr: {}'.format(
                    result.stderr.strip()[:200]
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
        self.get_logger().info('Task label updated to {!r}.'.format(request.label))
        response.success = True
        response.message = 'succeeded'
        return response

    def _on_joy(self, msg: Joy) -> None:
        if self._any_button_pressed(msg, self._play_buttons):
            if not self._play_enabled:
                self.get_logger().info('VLA execution started by gamepad play button.')
                with self._lock:
                    self._cmd_vector = dict(self._state_vector)
                self._play_enabled = True
                self._episode_logger.begin_episode()
                with self._inference_cond:
                    self._inference_cond.notify_all()

        if self._any_button_pressed(msg, self._stop_buttons):
            if self._play_enabled:
                self.get_logger().info('VLA execution stopped by gamepad stop button.')
                self._episode_logger.end_episode()
                self._play_enabled = False
                self._reset_episode_state()

    def _on_play(self, msg: Bool) -> None:
        if msg.data and not self._play_enabled:
            self.get_logger().info('VLA execution started via /vla/play topic.')
            with self._lock:
                self._cmd_vector = dict(self._state_vector)
            self._play_enabled = True
            self._episode_logger.begin_episode()
            with self._inference_cond:
                self._inference_cond.notify_all()
        elif not msg.data and self._play_enabled:
            self.get_logger().info('VLA execution stopped via /vla/play topic.')
            self._episode_logger.end_episode()
            self._play_enabled = False
            self._reset_episode_state()

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
            # Build dataset feature spec: float for joints+base, (H,W,C) tuple for cameras
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

        # Add features not produced by hw_to_dataset_features but required by model
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

        # All joint states are fresh
        state_dim = (
            frame['observation.state'].shape[-1]
            if 'observation.state' in frame and hasattr(frame['observation.state'], 'shape')
            else len(self._joint_features) + len(self._mobile_base_features)
        )
        frame['observation.state.is_fresh'] = np.ones(state_dim, dtype=np.float32)

        # Pad observation.state to the expected_state_dim set during _load_policy
        import torch
        expected_state_dim = getattr(self, '_expected_state_dim', None)
        model_names = getattr(self, '_model_action_feature_names', None)
        if expected_state_dim is None and model_names is not None:
            expected_state_dim = len(model_names)

        if expected_state_dim is not None and 'observation.state' in frame:
            state_arr = frame['observation.state']
            current_dim = state_arr.shape[-1] if hasattr(state_arr, 'shape') else len(state_arr)
            if model_names is not None or current_dim != expected_state_dim:
                yaml_index = {
                    name: idx for idx, name in enumerate(self._joint_features + self._mobile_base_features)
                }
                state_tensor = (
                    torch.from_numpy(state_arr)
                    if isinstance(state_arr, np.ndarray)
                    else torch.tensor(state_arr, dtype=torch.float32)
                )
                padded = torch.zeros(expected_state_dim, dtype=state_tensor.dtype)
                if model_names is not None:
                    for i, name in enumerate(model_names):
                        if i >= expected_state_dim:
                            break
                        resolved = name
                        if resolved not in yaml_index:
                            aliased = self._BASE_KEY_ALIASES.get(name)
                            if aliased and aliased in yaml_index:
                                resolved = aliased
                        
                        if resolved in yaml_index:
                            padded[i] = state_tensor[yaml_index[resolved]]
                else:
                    copy_len = min(current_dim, expected_state_dim)
                    padded[:copy_len] = state_tensor[:copy_len]
                frame['observation.state'] = padded.numpy()

        return frame

    def _predict_actions(self, obs_frame: Dict[str, Any]):
        import torch
        from lerobot.policies.utils import prepare_observation_for_inference

        device = torch.device(self._model_device)
        model_dtype = next(self._policy.parameters()).dtype

        # Cast all values to tensors on device
        def _to_device(v: Any) -> Any:
            if isinstance(v, torch.Tensor):
                return v.to(device=device, dtype=model_dtype)
            if isinstance(v, np.ndarray):
                t = torch.from_numpy(v.copy())
                return t.to(device=device, dtype=model_dtype)
            return v

        steps = []
        manually_add_delta = False
        # Holds the model-space (pre-postprocessor) tensor for RTC replace()
        rtc_raw_model_chunk: Optional[Any] = None
        # The delay actually used for this inference call (dynamic when latency tracker active)
        inference_delay: int = self._rtc_inference_delay

        if self._rtc_enabled and hasattr(self._policy, 'predict_action_chunk'):
            obs_numpy: Dict[str, Any] = {}
            for k, v in obs_frame.items():
                if isinstance(v, torch.Tensor):
                    obs_numpy[k] = v.detach().cpu().numpy()
                else:
                    obs_numpy[k] = v
            try:
                # compute inference delay dynamically from a running LatencyTracker rather than using a static config parameter
                time_per_step = 1.0 / max(self._control_hz, 1.0)
                if self._latency_tracker is not None:
                    max_latency = self._latency_tracker.max() or 0.0
                    dynamic_delay = math.ceil(max_latency / time_per_step)
                    # Never go below the configured minimum; clamp to chunk size.
                    inference_delay = max(self._rtc_inference_delay, dynamic_delay)
                else:
                    inference_delay = self._rtc_inference_delay

                prev_left_over = self._chunk_buffer.left_over(inference_delay)
                if prev_left_over is not None:
                    prev_left_over = prev_left_over.to(device=device)
                t0 = monotonic()
                # Must use no_grad (not inference_mode) here: RTC guidance in
                # denoise_step calls torch.autograd.grad() inside enable_grad(),
                # which inference_mode permanently blocks for any tensor it created.
                with torch.no_grad():
                    obs_tensor = prepare_observation_for_inference(
                        obs_numpy,
                        device,
                        task=self._task_label or None,
                    )
                    if self._preprocessor is not None:
                        obs_tensor = self._preprocessor(obs_tensor)
                    raw_chunk = self._policy.predict_action_chunk(
                        obs_tensor,
                        inference_delay=inference_delay,
                        prev_chunk_left_over=prev_left_over,
                    )
                    rtc_raw_model_chunk = raw_chunk[0].cpu() if raw_chunk.dim() == 3 else raw_chunk.cpu()
                    if self._postprocessor is not None:
                        raw_chunk = self._postprocessor(raw_chunk)
                elapsed = monotonic() - t0

                if self._latency_tracker is not None:
                    self._latency_tracker.add(elapsed)

                if elapsed > 0.2:
                    self.get_logger().warn(
                        'RTC inference {:.0f}ms (delay={}) > 200ms threshold.'.format(
                            elapsed * 1000, inference_delay
                        )
                    )
                steps = self._to_action_steps(raw_chunk)
                if steps:
                    manually_add_delta = self._model_use_relative_actions
            except Exception as exc:
                self.get_logger().warn(
                    'RTC chunk inference failed, falling back: {}'.format(exc)
                )
                rtc_raw_model_chunk = None

        if not steps:
            if _LEROBOT_AVAILABLE and self._preprocessor is not None:
                obs_numpy: Dict[str, Any] = {}
                for k, v in obs_frame.items():
                    if isinstance(v, torch.Tensor):
                        obs_numpy[k] = v.detach().cpu().numpy()
                    else:
                        obs_numpy[k] = v
                t0 = monotonic()
                if hasattr(self._policy, 'predict_action_chunk'):
                    with torch.inference_mode():
                        obs_tensor = prepare_observation_for_inference(
                            obs_numpy,
                            device,
                            task=self._task_label or None,
                        )
                        obs_tensor = self._preprocessor(obs_tensor)
                        raw_chunk = self._policy.predict_action_chunk(obs_tensor)
                        if self._postprocessor is not None:
                            raw_chunk = self._postprocessor(raw_chunk)
                    elapsed = monotonic() - t0
                    if elapsed > 0.2:
                        self.get_logger().warn(
                            'Chunk Inference {:.0f}ms > 200ms threshold.'.format(elapsed * 1000)
                        )
                    steps = self._to_action_steps(raw_chunk)
                    if steps:
                        manually_add_delta = self._model_use_relative_actions
                else:
                    with torch.inference_mode():
                        raw_action = predict_action(
                            obs_numpy,
                            self._policy,
                            device,
                            self._preprocessor,
                            self._postprocessor,
                            self._model_use_amp,
                            task=self._task_label or None,
                        )
                    elapsed = monotonic() - t0
                    if elapsed > 0.2:
                        self.get_logger().warn(
                            'Inference {:.0f}ms > 200ms threshold.'.format(elapsed * 1000)
                        )
                    steps = self._to_action_steps(raw_action)
                    if steps:
                        manually_add_delta = self._model_use_relative_actions
            else:
                obs_tensor = {k: _to_device(v) for k, v in obs_frame.items()}
                with torch.inference_mode():
                    raw_action = self._policy.select_action(obs_tensor)
                steps = self._to_action_steps(raw_action)
                if steps:
                    manually_add_delta = self._model_use_relative_actions

        if steps and len(steps) > self._actions_per_chunk:
            steps = steps[:self._actions_per_chunk]

        if manually_add_delta and steps:
            has_absolute_step = False
            try:
                from lerobot.processor.relative_action_processor import AbsoluteActionsProcessorStep
                if self._postprocessor is not None:
                    has_absolute_step = any(isinstance(s, AbsoluteActionsProcessorStep) for s in self._postprocessor.steps)
            except Exception:
                pass

            if not has_absolute_step:
                for step in steps:
                    for key in list(step.keys()):
                        if key in self._state_vector and key not in self._mobile_base_features:
                            step[key] = self._state_vector[key] + step[key]

        try:
            state_in = [float(v) for v in obs_frame.get('observation.state', [])]
            action_out = [float(steps[0].get(k, 0.0)) for k in self._joint_features] if steps else []
            names = self._joint_features + self._mobile_base_features
            state_dict = {names[i]: round(state_in[i], 4) for i in range(min(len(names), len(state_in)))}
            action_dict = {names[i]: round(action_out[i], 4) for i in range(min(len(names), len(action_out)))}
            self.get_logger().info('VLA DBG: state_in={}'.format(state_dict))
            self.get_logger().info('VLA DBG: action_out={}'.format(action_dict))
        except Exception as e:
            pass

        return steps, rtc_raw_model_chunk, inference_delay

    # Dataset conversion stores base dims as base_x/base_y/base_theta.
    # Deploy node publishes via x.vel/y.vel/theta.vel. Map between the two conventions.
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
            # Use model's action_feature_names for named lookup when available.
            # Applies _BASE_KEY_ALIASES to translate dataset names (base_x/base_y/base_theta)
            # to deploy convention (x.vel/y.vel/theta.vel).
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

    def _inference_worker(self) -> None:
        # single_step_mode: publish timer pops _single_step_result directly
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
                    steps, _raw_model, _delay = self._predict_actions(obs_frame)
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

            q_len_at_obs = self._chunk_buffer.size()
            obs_frame = self._snapshot_observation()
            if obs_frame is None:
                with self._inference_cond:
                    self._inference_cond.wait(timeout=0.1)
                continue

            try:
                chunk, raw_model_chunk, used_delay = self._predict_actions(obs_frame)
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
                if self._rtc_enabled:
                    self._chunk_buffer.replace(raw_model_chunk, chunk, used_delay)
                else:
                    self._chunk_buffer.merge_aligned(chunk, q_len_at_obs)

    def _publish_next_action(self) -> None:
        if not self._play_enabled:
            if self._base_pub is not None:
                cmd = Twist()
                self._base_pub.publish(cmd)
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
                if self._base_pub is not None:
                    cmd = Twist()
                    self._base_pub.publish(cmd)
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
                if self._base_pub is not None:
                    cmd = Twist()
                    self._base_pub.publish(cmd)
                return

        now = self.get_clock().now().to_msg()
        joint_log_parts = []
        for group in self._joint_groups:
            msg = JointTrajectory()
            msg.header.stamp = now
            msg.joint_names = group.joints_ros
            point = JointTrajectoryPoint()
            raw_positions = [
                float(step.get(feature, self._state_vector.get(feature, 0.0)))
                for feature in group.features
            ]
            delta = group.max_joint_delta
            if delta > 0.0:
                # Clamp goal vs MEASURED position
                point.positions = []
                for feat, raw in zip(group.features, raw_positions):
                    present = self._state_vector.get(feat, raw)
                    clamped = float(max(present - delta, min(present + delta, raw)))
                    point.positions.append(clamped)
                    self._cmd_vector[feat] = clamped
            else:
                point.positions = raw_positions
                for feat, pos in zip(group.features, raw_positions):
                    self._cmd_vector[feat] = pos
            point.time_from_start = self._step_duration
            msg.points = [point]
            self._group_publishers[group.name].publish(msg)
            
            # Format and save joint positions for logging
            pos_strs = ['{:.3f}'.format(p) for p in point.positions]
            joint_log_parts.append('{}: [{}]'.format(group.name, ', '.join(pos_strs)))

        base_log = ""
        if self._base_pub is not None:
            cmd = Twist()
            vx = float(step.get('x.vel', 0.0))
            vy = float(step.get('y.vel', 0.0))
            vth = float(step.get('theta.vel', 0.0))
            if self._max_vel_x > 0.0:
                vx = max(-self._max_vel_x, min(self._max_vel_x, vx))
            if self._max_vel_y > 0.0:
                vy = max(-self._max_vel_y, min(self._max_vel_y, vy))
            if self._max_vel_theta > 0.0:
                vth = max(-self._max_vel_theta, min(self._max_vel_theta, vth))
            cmd.linear.x = 0.0 if abs(vx) < 0.015 else vx
            cmd.linear.y = 0.0 if abs(vy) < 0.015 else vy
            cmd.linear.z = float(step.get('z.vel', 0.0))
            cmd.angular.z = 0.0 if abs(vth) < 0.005 else vth
            self._base_pub.publish(cmd)
            base_log = ' | BASE: x={:.3f} y={:.3f} th={:.3f}'.format(
                cmd.linear.x, cmd.linear.y, cmd.angular.z
            )

        # Episode logging: joints, base vel, EE pose (TF lookup)
        if self._logging_enabled:
            log_joints: Dict[str, float] = {}
            for group in self._joint_groups:
                for feat in group.features:
                    log_joints[feat] = float(self._cmd_vector.get(feat, 0.0))
            log_base = {
                'x': float(step.get('x.vel', 0.0)) if step else 0.0,
                'y': float(step.get('y.vel', 0.0)) if step else 0.0,
                'theta': float(step.get('theta.vel', 0.0)) if step else 0.0,
            }
            ee = self._get_ee_pose_left()
            self._episode_logger.log_step(
                joints=log_joints,
                base_vel=log_base,
                ee_pose=ee.tolist() if ee is not None else None,
            )

        # Print consolidated status line
        joint_log = ' | '.join(joint_log_parts)
        self.get_logger().info('CMD -> {}{}'.format(joint_log, base_log))


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
