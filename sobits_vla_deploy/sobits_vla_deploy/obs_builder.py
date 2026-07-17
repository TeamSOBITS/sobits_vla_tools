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

from threading import Lock
from typing import Any, Dict, List, Optional

import numpy as np
import torch

try:
    from sobits_vla_common.lerobot_adapter import build_dataset_frame, hw_to_dataset_features

    _LEROBOT_AVAILABLE = True
except ImportError:
    _LEROBOT_AVAILABLE = False


class ObsBuilder:
    def __init__(
        self,
        joint_features: List[str],
        mobile_base_features: List[str],
        camera_names: List[str],
    ):
        self.joint_features = joint_features
        self.mobile_base_features = mobile_base_features
        self.camera_names = camera_names

        self.lock = Lock()
        self.state_vector: Dict[str, float] = {
            feature: 0.0 for feature in self.joint_features
        }
        # Include base features if present
        for feat in self.mobile_base_features:
            self.state_vector[feat] = 0.0

        self.images: Dict[str, Optional[np.ndarray]] = {
            cam_name: None for cam_name in self.camera_names
        }
        self.obs_features = None
        self.prev_ee_pose_left = None

        self._BASE_KEY_ALIASES: Dict[str, str] = {
            'base_x': 'x.vel',
            'base_y': 'y.vel',
            'base_z': 'z.vel',
            'base_theta': 'theta.vel',
        }

    def update_joint_state(self, feature: str, value: float):
        with self.lock:
            if feature in self.state_vector:
                self.state_vector[feature] = value

    def update_odom(self, feature: str, value: float):
        with self.lock:
            if feature in self.state_vector:
                self.state_vector[feature] = value

    def update_image(self, cam_name: str, image: np.ndarray):
        with self.lock:
            self.images[cam_name] = image

    def clear_prev_ee_pose(self):
        with self.lock:
            self.prev_ee_pose_left = None

    def _get_ee_pose_left(
        self, tf_buffer, ee_left_base_frame, ee_left_target_frame
    ) -> Optional[np.ndarray]:
        """Return left EE pose as [x, y, z, roll, pitch, yaw] in base_footprint frame."""
        import rclpy.duration
        import rclpy.time

        try:
            t = tf_buffer.lookup_transform(
                ee_left_base_frame,
                ee_left_target_frame,
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
            return np.array(
                [tx.x, tx.y, tx.z, rpy[0], rpy[1], rpy[2]], dtype=np.float32
            )
        except Exception:
            return None

    def snapshot_observation(
        self,
        tf_buffer,
        ee_left_base_frame: str,
        ee_left_target_frame: str,
        expected_state_dim: Optional[int],
        model_action_feature_names: Optional[List[str]],
    ) -> Optional[Dict[str, Any]]:
        with self.lock:
            if any(self.images[c] is None for c in self.camera_names):
                return None
            obs = dict(self.state_vector)
            for cam_name, image in self.images.items():
                obs[cam_name] = image.copy()

        if self.obs_features is None and _LEROBOT_AVAILABLE:
            hw_features: Dict[str, Any] = {
                feature: float for feature in self.joint_features
            }
            for base_feat in self.mobile_base_features:
                hw_features[base_feat] = float
            for cam_name in self.camera_names:
                img = obs[cam_name]
                hw_features[cam_name] = img.shape
            self.obs_features = hw_to_dataset_features(
                hw_features, 'observation'
            )

        if not _LEROBOT_AVAILABLE or self.obs_features is None:
            return obs

        frame = build_dataset_frame(self.obs_features, obs, 'observation')

        ee_pose = self._get_ee_pose_left(
            tf_buffer, ee_left_base_frame, ee_left_target_frame
        )
        if ee_pose is not None:
            frame['observation.ee_pose.left'] = ee_pose
            frame['observation.ee_pose.left.delta'] = (
                ee_pose - self.prev_ee_pose_left
                if self.prev_ee_pose_left is not None
                else np.zeros(6, dtype=np.float32)
            )
            self.prev_ee_pose_left = ee_pose.copy()
        else:
            frame['observation.ee_pose.left'] = np.zeros(6, dtype=np.float32)
            frame['observation.ee_pose.left.delta'] = np.zeros(
                6, dtype=np.float32
            )

        state_dim = (
            frame['observation.state'].shape[-1]
            if 'observation.state' in frame
            and hasattr(frame['observation.state'], 'shape')
            else len(self.joint_features) + len(self.mobile_base_features)
        )
        frame['observation.state.is_fresh'] = np.ones(
            state_dim, dtype=np.float32
        )

        expected_dim = expected_state_dim
        if expected_dim is None and model_action_feature_names is not None:
            expected_dim = len(model_action_feature_names)

        if expected_dim is not None and 'observation.state' in frame:
            state_arr = frame['observation.state']
            current_dim = (
                state_arr.shape[-1]
                if hasattr(state_arr, 'shape')
                else len(state_arr)
            )
            if (
                model_action_feature_names is not None
                or current_dim != expected_dim
            ):
                yaml_index = {
                    name: idx
                    for idx, name in enumerate(
                        self.joint_features + self.mobile_base_features
                    )
                }
                state_tensor = (
                    torch.from_numpy(state_arr)
                    if isinstance(state_arr, np.ndarray)
                    else torch.tensor(state_arr, dtype=torch.float32)
                )
                padded = torch.zeros(expected_dim, dtype=state_tensor.dtype)
                if model_action_feature_names is not None:
                    for i, name in enumerate(model_action_feature_names):
                        if i >= expected_dim:
                            break
                        resolved = name
                        if resolved not in yaml_index:
                            aliased = self._BASE_KEY_ALIASES.get(name)
                            if aliased and aliased in yaml_index:
                                resolved = aliased

                        if resolved in yaml_index:
                            padded[i] = state_tensor[yaml_index[resolved]]
                else:
                    copy_len = min(current_dim, expected_dim)
                    padded[:copy_len] = state_tensor[:copy_len]
                frame['observation.state'] = padded.numpy()

        return frame
