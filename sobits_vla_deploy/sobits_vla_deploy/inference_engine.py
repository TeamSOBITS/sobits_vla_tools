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

import math
from threading import Condition, Lock, Thread
from time import monotonic
from typing import Any, Dict, List, Optional

import numpy as np
from sobits_vla_common.robot_descriptor import BASE_KEY_ALIASES
import torch

try:
    from sobits_vla_common.lerobot_adapter import predict_action, prepare_observation_for_inference

    _LEROBOT_AVAILABLE = True
except ImportError:
    _LEROBOT_AVAILABLE = False

try:
    from sobits_vla_common.lerobot_adapter import LatencyTracker

    _RTC_AVAILABLE = True
except ImportError:
    LatencyTracker = None
    _RTC_AVAILABLE = False


class InferenceEngine:
    def __init__(
        self,
        policy: Any,
        model_device: str,
        model_use_amp: bool,
        control_hz: float,
        actions_per_chunk: int,
        chunk_size_threshold: float,
        async_enabled: bool,
        single_step_mode: bool,
        rtc_enabled: bool,
        rtc_inference_delay: int,
        preprocessor: Optional[Any],
        postprocessor: Optional[Any],
        expected_state_dim: Optional[int],
        model_action_feature_names: Optional[List[str]],
        model_use_relative_actions: bool,
        joint_features: List[str],
        mobile_base_features: List[str],
        relative_exclude_features: Optional[List[str]] = None,
        logger=None,
    ):
        self.policy = policy
        self.model_device = model_device
        self.model_use_amp = model_use_amp
        self.control_hz = control_hz
        self.actions_per_chunk = actions_per_chunk
        self.chunk_size_threshold = chunk_size_threshold
        self.async_enabled = async_enabled
        self.single_step_mode = single_step_mode
        self.rtc_enabled = rtc_enabled
        self.rtc_inference_delay = rtc_inference_delay
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.expected_state_dim = expected_state_dim
        self.model_action_feature_names = model_action_feature_names
        self.model_use_relative_actions = model_use_relative_actions
        self.joint_features = joint_features
        self.mobile_base_features = mobile_base_features
        self.relative_exclude_features = set(relative_exclude_features or [])
        self.logger = logger

        self.task_label = ''
        self.play_enabled = False
        self.shutdown_inference = False

        self.inference_cond = Condition()
        self.single_step_result: Optional[Dict[str, float]] = None
        self.single_step_lock = Lock()
        # Bumped on every play toggle so results from a prior session's
        # in-flight predict (e.g. raced by a STOP+reset) get discarded.
        self.session_gen = 0

        self.latency_tracker = None
        if self.rtc_enabled and _RTC_AVAILABLE and LatencyTracker is not None:
            self.latency_tracker = LatencyTracker(maxlen=20)
            seed_latency = self.rtc_inference_delay / max(self.control_hz, 1.0)
            self.latency_tracker.add(seed_latency)

        self._BASE_KEY_ALIASES: Dict[str, str] = dict(BASE_KEY_ALIASES)

    def log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f'[INFO] {msg}')

    def log_debug(self, msg: str):
        if self.logger:
            self.logger.debug(msg)

    def log_warn(self, msg: str):
        if self.logger:
            self.logger.warning(msg)
        else:
            print(f'[WARN] {msg}')

    def log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    def update_task_label(self, label: str):
        self.task_label = label

    def update_play_enabled(self, enabled: bool):
        with self.inference_cond:
            # Bump only on a real transition: refill calls this every tick, and
            # unconditional bumping livelocked (100ms tick < ~130ms inference).
            if enabled != self.play_enabled:
                self.session_gen += 1
            self.play_enabled = enabled
            self.inference_cond.notify_all()

    def get_single_step_result(self) -> Optional[Dict[str, float]]:
        with self.single_step_lock:
            return self.single_step_result

    def clear_single_step_result(self):
        with self.single_step_lock:
            self.single_step_result = None

    def pop_single_step_result(self) -> Optional[Dict[str, float]]:
        """Atomically get-and-clear so a result can't be cleared unexecuted."""
        with self.single_step_lock:
            result = self.single_step_result
            self.single_step_result = None
            return result

    def start(
        self,
        obs_builder,
        chunk_buffer,
        tf_buffer,
        ee_poses,
    ):
        self.chunk_buffer = chunk_buffer
        self.thread = Thread(
            target=self._inference_worker,
            args=(
                obs_builder,
                chunk_buffer,
                tf_buffer,
                ee_poses,
            ),
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        self.shutdown_inference = True
        with self.inference_cond:
            self.inference_cond.notify_all()
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def _inference_worker(
        self,
        obs_builder,
        chunk_buffer,
        tf_buffer,
        ee_poses,
    ) -> None:
        import rclpy

        if self.single_step_mode:
            while rclpy.ok() and not self.shutdown_inference:
                with self.inference_cond:
                    if not self.play_enabled:
                        self.inference_cond.wait(timeout=0.5)
                        continue
                    gen = self.session_gen

                obs_frame = obs_builder.snapshot_observation(
                    tf_buffer,
                    ee_poses,
                    self.expected_state_dim,
                    self.model_action_feature_names,
                )
                if obs_frame is None:
                    with self.inference_cond:
                        self.inference_cond.wait(timeout=0.05)
                    continue

                try:
                    steps, _raw_model, _delay = self._predict_actions(
                        obs_frame, obs_builder.state_vector
                    )
                except Exception as exc:
                    import traceback as _tb

                    self.log_error(
                        'Inference error: {}. Retrying.\n{}'.format(
                            exc, _tb.format_exc()
                        )
                    )
                    continue

                if steps:
                    with self.inference_cond:
                        if gen != self.session_gen:
                            self.log_info(
                                'Discarding stale single-step result from a '
                                'previous play session.'
                            )
                            continue
                    with self.single_step_lock:
                        self.single_step_result = steps[0]
            return

        while rclpy.ok() and not self.shutdown_inference:
            with self.inference_cond:
                play = self.play_enabled
                queue_len = chunk_buffer.size()
                threshold_len = max(
                    1, int(self.actions_per_chunk * self.chunk_size_threshold)
                )
                need_infer = play and (
                    (self.async_enabled and queue_len <= threshold_len)
                    or (not self.async_enabled and queue_len == 0)
                )
                gen = self.session_gen
                if not need_infer:
                    self.inference_cond.wait(timeout=0.5)
                    continue

            q_len_at_obs = chunk_buffer.size()
            obs_frame = obs_builder.snapshot_observation(
                tf_buffer,
                ee_poses,
                self.expected_state_dim,
                self.model_action_feature_names,
            )
            if obs_frame is None:
                with self.inference_cond:
                    self.inference_cond.wait(timeout=0.1)
                continue

            try:
                chunk, raw_model_chunk, used_delay = self._predict_actions(
                    obs_frame, obs_builder.state_vector
                )
            except Exception as exc:
                import traceback as _tb

                self.log_error(
                    'Inference error: {}. Retrying.\n{}'.format(
                        exc, _tb.format_exc()
                    )
                )
                with self.inference_cond:
                    self.inference_cond.wait(timeout=1.0)
                continue

            if chunk:
                with self.inference_cond:
                    if gen != self.session_gen:
                        self.log_info(
                            'Discarding stale chunk from a previous play '
                            'session (episode was reset mid-inference).'
                        )
                        continue
                if self.rtc_enabled:
                    chunk_buffer.replace(raw_model_chunk, chunk, used_delay)
                else:
                    chunk_buffer.merge_aligned(chunk, q_len_at_obs)

    def _predict_actions(self, obs_frame: Dict[str, Any], state_vector: Dict[str, float]):
        device = torch.device(self.model_device)
        model_dtype = next(self.policy.parameters()).dtype

        def _to_device(v: Any) -> Any:
            if isinstance(v, torch.Tensor):
                return v.to(device=device, dtype=model_dtype)
            if isinstance(v, np.ndarray):
                t = torch.from_numpy(v.copy())
                return t.to(device=device, dtype=model_dtype)
            return v

        steps = []
        manually_add_delta = False
        rtc_raw_model_chunk = None
        inference_delay: int = self.rtc_inference_delay

        if self.rtc_enabled and hasattr(self.policy, 'predict_action_chunk'):
            obs_numpy: Dict[str, Any] = {}
            for k, v in obs_frame.items():
                if isinstance(v, torch.Tensor):
                    obs_numpy[k] = v.detach().cpu().numpy()
                else:
                    obs_numpy[k] = v
            try:
                time_per_step = 1.0 / max(self.control_hz, 1.0)
                if self.latency_tracker is not None:
                    max_latency = self.latency_tracker.max() or 0.0
                    dynamic_delay = math.ceil(max_latency / time_per_step)
                    inference_delay = max(self.rtc_inference_delay, dynamic_delay)
                else:
                    inference_delay = self.rtc_inference_delay

                prev_left_over = self.chunk_buffer.left_over(inference_delay)
                if prev_left_over is not None:
                    prev_left_over = prev_left_over.to(device=device)
                t0 = monotonic()
                with torch.no_grad():
                    obs_tensor = prepare_observation_for_inference(
                        obs_numpy,
                        device,
                        task=self.task_label or None,
                    )
                    if self.preprocessor is not None:
                        obs_tensor = self.preprocessor(obs_tensor)
                    raw_chunk = self.policy.predict_action_chunk(
                        obs_tensor,
                        inference_delay=inference_delay,
                        prev_chunk_left_over=prev_left_over,
                    )
                    rtc_raw_model_chunk = (
                        raw_chunk[0].cpu()
                        if raw_chunk.dim() == 3
                        else raw_chunk.cpu()
                    )
                    if self.postprocessor is not None:
                        raw_chunk = self.postprocessor(raw_chunk)
                elapsed = monotonic() - t0

                if self.latency_tracker is not None:
                    self.latency_tracker.add(elapsed)

                if elapsed > 0.2:
                    self.log_warn(
                        'RTC inference {:.0f}ms (delay={}) > 200ms threshold.'.format(
                            elapsed * 1000, inference_delay
                        )
                    )
                steps = self._to_action_steps(raw_chunk)
                if steps:
                    manually_add_delta = self.model_use_relative_actions
            except Exception as exc:
                self.log_warn(
                    'RTC chunk inference failed, falling back: {}'.format(exc)
                )
                rtc_raw_model_chunk = None

        if not steps:
            if _LEROBOT_AVAILABLE and self.preprocessor is not None:
                obs_numpy: Dict[str, Any] = {}
                for k, v in obs_frame.items():
                    if isinstance(v, torch.Tensor):
                        obs_numpy[k] = v.detach().cpu().numpy()
                    else:
                        obs_numpy[k] = v
                t0 = monotonic()
                if hasattr(self.policy, 'predict_action_chunk'):
                    with torch.inference_mode():
                        obs_tensor = prepare_observation_for_inference(
                            obs_numpy,
                            device,
                            task=self.task_label or None,
                        )
                        obs_tensor = self.preprocessor(obs_tensor)
                        raw_chunk = self.policy.predict_action_chunk(obs_tensor)
                        if self.postprocessor is not None:
                            raw_chunk = self.postprocessor(raw_chunk)
                    elapsed = monotonic() - t0
                    if elapsed > 0.2:
                        self.log_warn(
                            'Chunk Inference {:.0f}ms > 200ms threshold.'.format(
                                elapsed * 1000
                            )
                        )
                    steps = self._to_action_steps(raw_chunk)
                    if steps:
                        manually_add_delta = self.model_use_relative_actions
                else:
                    with torch.inference_mode():
                        raw_action = predict_action(
                            obs_numpy,
                            self.policy,
                            device,
                            self.preprocessor,
                            self.postprocessor,
                            self.model_use_amp,
                            task=self.task_label or None,
                        )
                    elapsed = monotonic() - t0
                    if elapsed > 0.2:
                        self.log_warn(
                            'Inference {:.0f}ms > 200ms threshold.'.format(
                                elapsed * 1000
                            )
                        )
                    steps = self._to_action_steps(raw_action)
                    if steps:
                        manually_add_delta = self.model_use_relative_actions
            else:
                obs_tensor = {k: _to_device(v) for k, v in obs_frame.items()}
                with torch.inference_mode():
                    raw_action = self.policy.select_action(obs_tensor)
                steps = self._to_action_steps(raw_action)
                if steps:
                    manually_add_delta = self.model_use_relative_actions

        if steps and len(steps) > self.actions_per_chunk:
            steps = steps[: self.actions_per_chunk]

        if manually_add_delta and steps:
            self._apply_manual_delta(steps, state_vector)

        try:
            state_in = [float(v) for v in obs_frame.get('observation.state', [])]
            action_out = (
                [float(steps[0].get(k, 0.0)) for k in self.joint_features]
                if steps
                else []
            )
            names = self.joint_features + self.mobile_base_features
            state_dict = {
                names[i]: round(state_in[i], 4)
                for i in range(min(len(names), len(state_in)))
            }
            action_dict = {
                names[i]: round(action_out[i], 4)
                for i in range(min(len(names), len(action_out)))
            }
            self.log_debug('VLA DBG: state_in={}'.format(state_dict))
            self.log_debug('VLA DBG: action_out={}'.format(action_dict))
        except Exception:
            pass

        return steps, rtc_raw_model_chunk, inference_delay

    def _apply_manual_delta(
        self, steps: List[Dict[str, float]], state_vector: Dict[str, float]
    ) -> None:
        """
        Add current state to model output in place.

        For policies that predict deltas but ship no
        AbsoluteActionsProcessorStep to do it themselves.

        Skips mobile-base and relative_exclude features (e.g. a gripper),
        which must stay absolute regardless of the policy's delta mode.
        """
        has_absolute_step = False
        try:
            from sobits_vla_common.lerobot_adapter import AbsoluteActionsProcessorStep

            if self.postprocessor is not None:
                has_absolute_step = any(
                    isinstance(s, AbsoluteActionsProcessorStep)
                    for s in self.postprocessor.steps
                )
        except Exception:
            pass

        if has_absolute_step:
            return

        for step in steps:
            for key in list(step.keys()):
                if (
                    key in state_vector
                    and key not in self.mobile_base_features
                    and key not in self.relative_exclude_features
                ):
                    step[key] = state_vector[key] + step[key]

    def _to_action_steps(self, raw_actions: Any) -> List[Dict[str, float]]:
        action_keys = self.joint_features + self.mobile_base_features
        if not action_keys:
            return []

        if isinstance(raw_actions, dict):
            values = list(raw_actions.values())
            if values and isinstance(
                values[0], (list, tuple, np.ndarray, torch.Tensor)
            ):
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
            return [
                {k: float(v) for k, v in raw_actions.items() if k in action_keys}
            ]

        if isinstance(raw_actions, torch.Tensor):
            raw_actions = raw_actions.detach().cpu().numpy()

        if isinstance(raw_actions, np.ndarray):
            if raw_actions.ndim == 3 and raw_actions.shape[0] == 1:
                raw_actions = raw_actions.squeeze(0)
            model_names = self.model_action_feature_names
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
