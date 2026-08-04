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

import bisect
import io

import cv2
import numpy as np
from sobits_vla_rosbag_conversion.tf_buffer import mat_to_pose6d, OfflineTFTree
import torch

try:
    from PIL import Image as PILImage
except Exception:
    PILImage = None


def decode_image_message(msg) -> np.ndarray | None:
    """Decode ROS image/compressed-image message into RGB uint8 HWC array."""
    # CompressedImage-like messages usually expose a `format` field.
    if hasattr(msg, 'format'):
        data = msg.data
        if isinstance(data, memoryview):
            data = data.tobytes()
        if isinstance(data, (bytes, bytearray)):
            encoded = np.frombuffer(data, dtype=np.uint8)
        else:
            # rosbags can surface sequence payloads as Python lists/arrays.
            encoded = np.asarray(data, dtype=np.uint8)

        if encoded.size == 0:
            return None

        # Preferred path: OpenCV decode (fast). Some environments can fail due
        # OpenCV/Numpy ABI mismatch, so we fall back to Pillow decode.
        try:
            img = cv2.imdecode(np.ascontiguousarray(encoded), cv2.IMREAD_UNCHANGED)
        except Exception:
            img = None

        if img is not None:
            if img.ndim == 2:
                return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            if img.shape[2] == 4:
                return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        if PILImage is None:
            return None
        try:
            pil_img = PILImage.open(io.BytesIO(encoded.tobytes())).convert('RGB')
            return np.asarray(pil_img, dtype=np.uint8)
        except Exception:
            return None

    # Raw Image message path — decode manually
    encoding = getattr(msg, 'encoding', '')
    data = msg.data
    if isinstance(data, memoryview):
        data = bytes(data)
    raw = np.frombuffer(data, dtype=np.uint8)
    h, w = msg.height, msg.width
    if encoding in ('mono8', '8UC1'):
        img = raw.reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if encoding in ('mono16', '16UC1'):
        img = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        img8 = (img >> 8).astype(np.uint8)
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2RGB)
    if encoding in ('rgb8',):
        return raw.reshape(h, w, 3).copy()
    if encoding in ('bgr8',):
        img = raw.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if encoding in ('rgba8',):
        img = raw.reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    if encoding in ('bgra8',):
        img = raw.reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    # Fallback: try to reshape as BGR and convert
    channels = len(raw) // (h * w) if h * w > 0 else 3
    img = raw.reshape(h, w, channels)
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    return img


class FrameSynthesizer:
    def __init__(
        self,
        fps: int,
        sync_threshold: float,
        downsample_tolerance: float,
        use_relative_actions: bool,
        skip_static_threshold: float,
        action_features: list,
        has_mobile_base: bool,
        base_keys: list,
        ee_pose_enabled: bool,
        ee_configs: list,
        skip_cameras: bool,
        primary_camera: str,
        camera_topics: dict,
        subtask_label_to_idx: dict,
        logger=None,
    ):
        self.fps = fps
        self.sync_threshold = sync_threshold
        self.downsample_tolerance = downsample_tolerance
        self.use_relative_actions = use_relative_actions
        self.skip_static_threshold = skip_static_threshold
        self.action_features = action_features
        self.has_mobile_base = has_mobile_base
        self.base_keys = base_keys
        self.ee_pose_enabled = ee_pose_enabled
        self.ee_configs = ee_configs
        self.skip_cameras = skip_cameras
        self.primary_camera = primary_camera
        self.camera_topics = camera_topics
        self.subtask_label_to_idx = subtask_label_to_idx
        self.logger = logger

    def log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f'[WARN] {msg}')

    def log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    def _interpolate_vector(self, series, target_time, dim):
        """Linearly interpolates a time-series list of (t, list_of_floats) at target_time."""
        if not series:
            return [0.0] * dim
        if len(series) == 1:
            return series[0][1]

        times = [s[0] for s in series]
        idx = bisect.bisect_right(times, target_time)

        if idx == 0:
            return series[0][1]
        if idx == len(series):
            return series[-1][1]

        t_prev, val_prev = series[idx - 1]
        t_next, val_next = series[idx]

        dt = t_next - t_prev
        if dt <= 0.0:
            return val_prev

        alpha = (target_time - t_prev) / dt
        return [(1.0 - alpha) * val_prev[j] + alpha * val_next[j] for j in range(dim)]

    def _interpolate_dict(self, series, target_time, keys):
        """Linearly interpolates a time-series list of (t, dict_of_floats) at target_time."""
        if not series:
            return {k: 0.0 for k in keys}
        if len(series) == 1:
            return {k: series[0][1].get(k, 0.0) for k in keys}

        times = [s[0] for s in series]
        idx = bisect.bisect_right(times, target_time)

        if idx == 0:
            return {k: series[0][1].get(k, 0.0) for k in keys}
        if idx == len(series):
            return {k: series[-1][1].get(k, 0.0) for k in keys}

        t_prev, dict_prev = series[idx - 1]
        t_next, dict_next = series[idx]

        dt = t_next - t_prev
        if dt <= 0.0:
            return {k: dict_prev.get(k, 0.0) for k in keys}

        alpha = (target_time - t_prev) / dt
        res = {}
        for k in keys:
            v_prev = dict_prev.get(k, 0.0)
            v_next = dict_next.get(k, 0.0)
            res[k] = (1.0 - alpha) * v_prev + alpha * v_next
        return res

    def _hold_dict(self, series, target_time, keys):
        """Zero-order hold of a (t, dict_of_floats) series at target_time.

        Commanded positions are discrete set-points, not samples of a
        continuous signal: a trajectory point stays in force until the next
        command arrives. Interpolating between two commands invents motion
        that was never commanded — e.g. a grasp held closed for 11 s reads
        back as the hand slowly reopening across the whole hold.

        Returns the most recent command at or before *target_time*.
        """
        if not series:
            return {k: 0.0 for k in keys}

        times = [s[0] for s in series]
        idx = bisect.bisect_right(times, target_time) - 1
        if idx < 0:
            # target_time precedes every command; the first one is the best
            # available estimate of the set-point in force.
            idx = 0
        return {k: series[idx][1].get(k, 0.0) for k in keys}

    def _get_nearest_image(self, series, target_time):
        """Get the nearest image (msg, rawdata, connection, t) to target_time in the series."""
        if not series:
            return None, None, None, 0.0
        if len(series) == 1:
            # entry is (t_sec, msg, rawdata, connection)
            return series[0][1], series[0][2], series[0][3], series[0][0]

        times = [s[0] for s in series]
        idx = bisect.bisect_right(times, target_time)

        if idx == 0:
            return series[0][1], series[0][2], series[0][3], series[0][0]
        if idx == len(series):
            return series[-1][1], series[-1][2], series[-1][3], series[-1][0]

        t_prev, msg_prev, raw_prev, conn_prev = series[idx - 1]
        t_next, msg_next, raw_next, conn_next = series[idx]

        if abs(target_time - t_prev) < abs(target_time - t_next):
            return msg_prev, raw_prev, conn_prev, t_prev
        else:
            return msg_next, raw_next, conn_next, t_next

    def _any_arrived_in_interval(self, series, t_start, t_end):
        """Check if any message in the series arrived in the interval (t_start, t_end]."""
        if not series:
            return False
        times = [s[0] for s in series]
        idx = bisect.bisect_right(times, t_start)
        if idx < len(times) and times[idx] <= t_end:
            return True
        return False

    def synthesize(self, bag_series: dict, subtasks_map: dict, instruction: str):
        """Process the sorted series extracted by BagReader into synchronized frames."""
        joint_states_series = bag_series['joint_states']
        cmd_vel_series = bag_series['cmd_vel']
        odom_series = bag_series['odom']
        cmd_joints_series = bag_series['cmd_joints']
        cam_series = bag_series['cam_series']
        tf_messages = bag_series['tf_messages']

        tf_tree = OfflineTFTree() if self.ee_pose_enabled else None
        if tf_tree is not None:
            for _, msg, topic in tf_messages:
                tf_tree.ingest(msg, is_static=(topic == '/tf_static'))

        if not cam_series[self.primary_camera]:
            self.log_error('Bag contains no primary camera images.')
            return None
        if not joint_states_series:
            self.log_error('Bag contains no joint states.')
            return None

        # Separate pos and vel for joints to simplify interpolation dict helper
        joint_pos_series = [(s[0], s[1]) for s in joint_states_series]
        joint_vel_series = [(s[0], s[2]) for s in joint_states_series]

        frames = []
        skipped_static = 0
        skipped_tf = 0
        skipped_img_decode = 0
        skipped_downsample = 0
        sync_deltas = []

        last_frame_time = 0.0
        min_frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0

        # EE pose tracking — one prev_pose entry per configured EE
        prev_ee_poses = {name: None for name, _, _ in self.ee_configs}

        for t_sec, msg_prim, primary_raw, primary_conn in cam_series[self.primary_camera]:
            # Downsample: enforce minimum interval between frames (with tolerance for jitter)
            if min_frame_interval > 0.0 and last_frame_time > 0.0:
                if (t_sec - last_frame_time) < (min_frame_interval - self.downsample_tolerance):
                    skipped_downsample += 1
                    continue

            # Fetch secondary camera images at nearest time
            images = {}
            image_times = {self.primary_camera: t_sec}
            decoding_failed = False

            # Decode primary camera
            try:
                img_prim = decode_image_message(msg_prim)
                if img_prim is None:
                    decoding_failed = True
                else:
                    images[self.primary_camera] = img_prim
            except Exception:
                decoding_failed = True

            if decoding_failed:
                skipped_img_decode += 1
                continue

            for cam_name in self.camera_topics.keys():
                if cam_name == self.primary_camera:
                    continue
                msg_img, raw_img, conn_img, t_img = (
                    self._get_nearest_image(cam_series[cam_name], t_sec)
                )
                if msg_img is None:
                    decoding_failed = True
                    break
                try:
                    img = decode_image_message(msg_img)
                    if img is None:
                        decoding_failed = True
                        break
                    images[cam_name] = img
                    image_times[cam_name] = t_img
                except Exception:
                    decoding_failed = True
                    break

            if decoding_failed:
                skipped_img_decode += 1
                continue

            # Check sync difference for other cameras
            img_times = list(image_times.values())
            max_camera_diff = max(img_times) - min(img_times)

            # Interpolate joint state
            joint_pos = self._interpolate_dict(joint_pos_series, t_sec, self.action_features)
            joint_vel = self._interpolate_dict(joint_vel_series, t_sec, self.action_features)

            # Check if we have active base
            if self.has_mobile_base:
                cmd_vel = self._interpolate_vector(cmd_vel_series, t_sec, len(self.base_keys))
                odom_vel = self._interpolate_vector(odom_series, t_sec, len(self.base_keys))
            else:
                cmd_vel = None
                odom_vel = None

            # Check sync differences
            closest_joint_t = self._get_closest_t(joint_states_series, t_sec)
            joint_diff = abs(t_sec - closest_joint_t)

            cmd_vel_diff = 0.0
            if self.has_mobile_base:
                closest_cmd_vel_t = self._get_closest_t(cmd_vel_series, t_sec)
                cmd_vel_diff = abs(t_sec - closest_cmd_vel_t)

            odom_diff = 0.0
            if self.has_mobile_base:
                closest_odom_t = self._get_closest_t(odom_series, t_sec)
                odom_diff = abs(t_sec - closest_odom_t)

            max_sync = max(max_camera_diff, joint_diff, cmd_vel_diff, odom_diff)
            if max_sync > self.sync_threshold:
                self.log_warn(
                    f'Sync threshold exceeded at t={t_sec:.3f}s: cam_diff={max_camera_diff:.3f}s, '
                    f'joint_diff={joint_diff:.3f}s, cmd_vel_diff={cmd_vel_diff:.3f}s, '
                    f'odom_diff={odom_diff:.3f}s'
                )
            sync_deltas.append(max_sync)

            # Skip static frames: skip if no joint velocity exceeds threshold
            if self.skip_static_threshold > 0.0:
                joint_vel_vals = [joint_vel[f] for f in self.action_features]
                if not np.any(np.abs(joint_vel_vals) > self.skip_static_threshold):
                    skipped_static += 1
                    continue

            # State: measured joint positions
            state = [joint_pos[feat] for feat in self.action_features]

            # Action: commanded joint positions.
            # Zero-order hold, not interpolation: a command holds until the
            # next one arrives (see _hold_dict).
            action = []
            for i, feat in enumerate(self.action_features):
                cmd_series_for_feat = [(t, d) for t, d in cmd_joints_series if feat in d]
                if cmd_series_for_feat and t_sec >= cmd_series_for_feat[0][0]:
                    cmd_val = self._hold_dict(cmd_series_for_feat, t_sec, [feat])[feat]
                    action.append(cmd_val)
                else:
                    # Fall back to future measured state
                    t_next = t_sec + (1.0 / self.fps if self.fps > 0 else 0.1)
                    next_joint_pos = self._interpolate_dict(joint_pos_series, t_next, [feat])
                    action.append(next_joint_pos[feat])

            # If use_relative_actions is True, subtract joint state from
            # joint action to get the delta command
            if self.use_relative_actions:
                action = [act_val - st_val for act_val, st_val in zip(action, state)]

            if self.has_mobile_base:
                state = state + list(odom_vel)
                action = action + list(cmd_vel)

            frame = {
                'task': instruction,
                'action': torch.tensor(action, dtype=torch.float32),
                'observation.state': torch.tensor(state, dtype=torch.float32),
            }

            # End-effector pose via TF chain (supports multiple EEs)
            if tf_tree is not None:
                stamp_ns = int(t_sec * 1e9)
                ee_failed = False
                for ee_name, ee_src, ee_tgt in self.ee_configs:
                    ee_mat = tf_tree.resolve(ee_tgt, ee_src, stamp_ns)
                    if ee_mat is None:
                        skipped_tf += 1
                        if skipped_tf <= 5:
                            self.log_warn(
                                f"TF lookup failed: '{ee_src}' → '{ee_tgt}' "
                                f'at t={t_sec:.3f}s. Skipping frame.'
                            )
                        ee_failed = True
                        break
                    ee_abs = mat_to_pose6d(ee_mat)
                    prev = prev_ee_poses[ee_name]
                    if prev is not None:
                        for ax in range(3, 6):
                            diff = ee_abs[ax] - prev[ax]
                            if diff > np.pi:
                                ee_abs[ax] -= 2 * np.pi
                            elif diff < -np.pi:
                                ee_abs[ax] += 2 * np.pi
                            ee_rel = ee_abs - prev
                    else:
                        ee_rel = np.zeros(6, dtype=np.float32)
                    key = (
                        f'observation.ee_pose.{ee_name}'
                        if ee_name
                        else 'observation.ee_pose'
                    )
                    frame[key] = torch.from_numpy(ee_abs.copy())
                    frame[f'{key}.delta'] = torch.from_numpy(ee_rel)
                    prev_ee_poses[ee_name] = ee_abs.copy()
                if ee_failed:
                    continue

            if not self.skip_cameras:
                for c_name in self.camera_topics.keys():
                    img_arr = np.ascontiguousarray(
                        images[c_name].transpose(2, 0, 1)
                    )
                    frame[f'observation.images.{c_name}'] = torch.from_numpy(
                        img_arr
                    )

            # Subtask annotation
            if self.subtask_label_to_idx and subtasks_map:
                current_subtask_idx = 0
                for _, st_info in (
                    subtasks_map.items()
                    if isinstance(subtasks_map, dict)
                    else {}
                ):
                    start_t = (
                        st_info.get('start_timestamp', -1.0)
                        if isinstance(st_info, dict)
                        else -1.0
                    )
                    end_t = (
                        st_info.get('end_timestamp', float('inf'))
                        if isinstance(st_info, dict)
                        else float('inf')
                    )
                    if end_t == 0.0:
                        end_t = float('inf')
                    if start_t <= t_sec <= end_t:
                        label = (
                            st_info.get('label', '')
                            if isinstance(st_info, dict)
                            else ''
                        )
                        current_subtask_idx = self.subtask_label_to_idx.get(
                            label, 0
                        )
                        break
                frame['subtask_index'] = torch.tensor(
                    [current_subtask_idx], dtype=torch.int64
                )

            frames.append((t_sec, frame))
            last_frame_time = t_sec

        if skipped_tf > 5:
            self.log_warn(
                f'TF lookup failed {skipped_tf} times total (suppressed after 5).'
            )
        if skipped_img_decode > 0:
            self.log_warn(f'Image decode failed {skipped_img_decode} times.')

        return (
            frames,
            sync_deltas,
            skipped_static,
            skipped_tf,
            skipped_img_decode,
            skipped_downsample,
        )

    def _get_closest_t(self, series, target):
        if not series:
            return target
        times = [s[0] for s in series]
        idx = bisect.bisect_left(times, target)
        if idx == 0:
            return times[0]
        if idx == len(times):
            return times[-1]
        if abs(times[idx] - target) < abs(times[idx - 1] - target):
            return times[idx]
        return times[idx - 1]
