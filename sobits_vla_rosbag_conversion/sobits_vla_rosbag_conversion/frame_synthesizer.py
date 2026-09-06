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

import numpy as np
from sobits_vla_common.geometry import (
    quat_relative, quat_rotate_vec, quat_shortest_arc, quat_to_rpy, rpy_to_quat,
)
from sobits_vla_rosbag_conversion.offline_tf_tree import OfflineTFTree
from sobits_vla_rosbag_conversion.sync import images as sync_images
from sobits_vla_rosbag_conversion.sync import joints as sync_joints
from sobits_vla_rosbag_conversion.sync import poses as sync_poses
from sobits_vla_rosbag_conversion.sync.core import get_closest_t, should_downsample
import torch
from tqdm import tqdm


def _body_translation(state_q, state_xyz, action_xyz):
    """R(t)^T . (p(t+1) - p(t)): translation delta in the EE's own axes at t."""
    p_diff = (action_xyz - state_xyz).tolist()
    conj = (-state_q[0], -state_q[1], -state_q[2], state_q[3])
    return quat_rotate_vec(conj, p_diff)


class FrameSynthesizer:
    """Facade: composes sync/core+images+joints+poses into synchronized frames."""

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
        depth_camera_topics: dict | None = None,
        ee_action_specs: list = [],
        ee_rotation: str = 'rpy',
        ee_frame: str = 'base',
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
        self.depth_camera_topics = depth_camera_topics or {}
        self.subtask_label_to_idx = subtask_label_to_idx
        self.ee_action_specs = ee_action_specs
        self.ee_rotation = ee_rotation
        self.ee_frame = ee_frame
        self.logger = logger

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

    def synthesize(self, bag_series: dict, subtasks_map: dict, instruction: str):
        """Process the sorted series extracted by BagReader into synchronized frames."""
        ctx = self._prepare(bag_series)
        if ctx is None:
            return None

        frames = []
        counters = {'static': 0, 'tf': 0, 'img_decode': 0, 'downsample': 0}
        sync_deltas = []
        last_frame_time = 0.0
        min_frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
        prev_ee_poses = {name: None for name, _, _ in self.ee_configs}
        prev_ee_action_poses = {name: None for name, _, _ in self.ee_action_specs}
        tf_tree = ctx['tf_tree']

        # Inner bar: frames within the current episode; leave=False keeps the
        # episode-level bar as the only persistent line. Auto-off on non-TTY.
        frame_iter = tqdm(
            ctx['primary_series'], desc='  frames', unit='f',
            disable=None, leave=False,
        )
        for t_sec, msg_prim, _primary_raw, _primary_conn in frame_iter:
            if should_downsample(
                t_sec, last_frame_time, min_frame_interval, self.downsample_tolerance
            ):
                counters['downsample'] += 1
                continue

            images, image_times, decoding_failed = self._decode_all_images(
                msg_prim, t_sec, ctx
            )
            if decoding_failed:
                counters['img_decode'] += 1
                continue
            depth_images, depth_times, decoding_failed = sync_images.decode_depth_images(
                self.depth_camera_topics, ctx['cam_series'], ctx['cam_series_times'], t_sec
            )
            if decoding_failed:
                counters['img_decode'] += 1
                continue
            image_times.update(depth_times)

            frame_data = self._build_frame_core(
                t_sec, image_times, ctx, sync_deltas, instruction,
                tf_tree, prev_ee_action_poses, counters,
            )
            if frame_data is None:
                continue

            state, action, frame = frame_data
            if tf_tree is not None:
                ok = self._attach_ee_poses(frame, tf_tree, t_sec, prev_ee_poses, counters)
                if not ok:
                    continue

            self._attach_images(frame, images, depth_images)
            self._attach_subtask(frame, subtasks_map, t_sec)

            frames.append((t_sec, frame))
            last_frame_time = t_sec

        self._log_summary(counters)
        return (
            frames, sync_deltas, counters['static'], counters['tf'],
            counters['img_decode'], counters['downsample'],
        )

    def _prepare(self, bag_series: dict):
        joint_states_series = bag_series['joint_states']
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

        return self._build_context(bag_series, tf_tree)

    def _build_context(self, bag_series: dict, tf_tree) -> dict:
        joint_states_series = bag_series['joint_states']
        cam_series = bag_series['cam_series']

        # Separate pos and vel for joints to simplify interpolation dict helper
        joint_pos_series = [(s[0], s[1]) for s in joint_states_series]
        joint_vel_series = [(s[0], s[2]) for s in joint_states_series]

        cmd_series_by_feature = {feat: [] for feat in self.action_features}
        for t, d in bag_series['cmd_joints']:
            for feat in self.action_features:
                if feat in d:
                    cmd_series_by_feature[feat].append((t, d))

        return {
            'tf_tree': tf_tree,
            'primary_series': cam_series[self.primary_camera],
            'cam_series': cam_series,
            'cam_series_times': {
                cam_name: [s[0] for s in series] for cam_name, series in cam_series.items()
            },
            'joint_pos_series': joint_pos_series,
            'joint_vel_series': joint_vel_series,
            'joint_pos_times': [s[0] for s in joint_pos_series],
            'joint_vel_times': [s[0] for s in joint_vel_series],
            'joint_states_series': joint_states_series,
            'joint_states_times': [s[0] for s in joint_states_series],
            'cmd_vel_series': bag_series['cmd_vel'],
            'cmd_vel_times': [s[0] for s in bag_series['cmd_vel']],
            'odom_series': bag_series['odom'],
            'odom_times': [s[0] for s in bag_series['odom']],
            'cmd_series_by_feature': cmd_series_by_feature,
            'cmd_series_by_feature_times': {
                feat: [t for t, _ in series] for feat, series in cmd_series_by_feature.items()
            },
        }

    def _decode_all_images(self, msg_prim, t_sec, ctx):
        img_prim = sync_images.decode_primary(msg_prim)
        if img_prim is None:
            return {}, {}, True
        images = {self.primary_camera: img_prim}
        image_times = {self.primary_camera: t_sec}

        secondary_images, secondary_times, failed = sync_images.decode_secondary_images(
            self.camera_topics, self.primary_camera, ctx['cam_series'],
            ctx['cam_series_times'], t_sec,
        )
        if failed:
            return images, image_times, True
        images.update(secondary_images)
        image_times.update(secondary_times)
        return images, image_times, False

    def _build_frame_core(
        self, t_sec, image_times, ctx, sync_deltas, instruction,
        tf_tree=None, prev_ee_action_poses=None, counters=None,
    ):
        img_times = list(image_times.values())
        max_camera_diff = max(img_times) - min(img_times)

        joint_pos, joint_vel = sync_joints.interpolate_joint_state(
            ctx['joint_pos_series'], ctx['joint_vel_series'], t_sec, self.action_features,
            ctx['joint_pos_times'], ctx['joint_vel_times'],
        )

        if self.has_mobile_base:
            cmd_vel, odom_vel = sync_joints.interpolate_base_velocity(
                ctx['cmd_vel_series'], ctx['odom_series'], t_sec, self.base_keys,
                ctx['cmd_vel_times'], ctx['odom_times'],
            )
        else:
            cmd_vel, odom_vel = None, None

        joint_diff, cmd_vel_diff, odom_diff = self._sync_component_diffs(t_sec, ctx)
        max_sync = max(max_camera_diff, joint_diff, cmd_vel_diff, odom_diff)
        if max_sync > self.sync_threshold:
            self.log_warn(
                f'Sync threshold exceeded at t={t_sec:.3f}s: cam_diff={max_camera_diff:.3f}s, '
                f'joint_diff={joint_diff:.3f}s, cmd_vel_diff={cmd_vel_diff:.3f}s, '
                f'odom_diff={odom_diff:.3f}s'
            )
        sync_deltas.append(max_sync)

        if self.skip_static_threshold > 0.0:
            joint_vel_vals = [joint_vel[f] for f in self.action_features]
            if not np.any(np.abs(joint_vel_vals) > self.skip_static_threshold):
                counters['static'] += 1
                return None

        state = [joint_pos[feat] for feat in self.action_features]
        action = sync_joints.synthesize_action(
            t_sec, self.fps, self.action_features, ctx['cmd_series_by_feature'],
            ctx['cmd_series_by_feature_times'], ctx['joint_pos_series'], ctx['joint_pos_times'],
        )
        if self.use_relative_actions:
            action = sync_joints.to_relative_action(action, state)

        if self.ee_action_specs:
            ok = self._append_ee_channels(
                state, action, tf_tree, t_sec, prev_ee_action_poses, counters
            )
            if not ok:
                return None

        if self.has_mobile_base:
            state = state + list(odom_vel)
            action = action + list(cmd_vel)

        frame = {
            'task': instruction,
            'action': torch.tensor(action, dtype=torch.float32),
            'observation.state': torch.tensor(state, dtype=torch.float32),
        }
        return state, action, frame

    def _append_ee_channels(
        self, state, action, tf_tree, t_sec, prev_ee_action_poses, counters
    ) -> bool:
        """Extend state/action in place with EE state/action pose per spec; False = skip frame."""
        t_ns = int(t_sec * 1e9)
        synth_fn = (
            sync_poses.synthesize_ee_action_quat if self.ee_rotation == 'quat'
            else sync_poses.synthesize_ee_action
        )
        for spec in self.ee_action_specs:
            name, ee_src, ee_tgt = spec
            result = synth_fn(
                tf_tree, ee_src, ee_tgt, t_ns, self.fps, prev_ee_action_poses[name]
            )
            if result is None:
                counters['tf'] += 1
                if counters['tf'] <= 5:
                    self.log_warn(
                        f"EE action TF lookup failed: '{ee_src}' -> '{ee_tgt}' "
                        f'at t={t_sec:.3f}s. Skipping frame.'
                    )
                return False
            state_pose, action_pose = result
            state.extend(state_pose.tolist())
            # Joints get delta-converted by to_relative_action above; EE deltas
            # are formed here from the already-unwrap/shortest-arc-consistent
            # (state, action) pair.
            if self.use_relative_actions:
                if self.ee_rotation == 'quat':
                    action.extend(self._relative_ee_pose7(state_pose, action_pose, self.ee_frame))
                else:
                    action.extend(self._relative_ee_pose6(state_pose, action_pose, self.ee_frame))
            else:
                action.extend(action_pose.tolist())
            prev_ee_action_poses[name] = state_pose
        return True

    @staticmethod
    def _relative_ee_pose6(state_pose, action_pose, ee_frame: str) -> list:
        """Relative 6D EE action: [dx,dy,dz, droll,dpitch,dyaw] per ee_frame."""
        if ee_frame == 'base':
            return (action_pose - state_pose).tolist()
        state_q = rpy_to_quat(*state_pose[3:6])
        delta_xyz = _body_translation(state_q, state_pose[:3], action_pose[:3])
        action_q = rpy_to_quat(*action_pose[3:6])
        delta_rpy = quat_to_rpy(*quat_relative(state_q, action_q))
        return list(delta_xyz) + list(delta_rpy)

    @staticmethod
    def _relative_ee_pose7(state_pose, action_pose, ee_frame: str) -> list:
        """Relative 7D EE action: [dx,dy,dz, dqx,dqy,dqz,dqw] per ee_frame."""
        state_q = tuple(state_pose[3:7])
        # state/action quats already come out shortest-arc-continuous from
        # synthesize_ee_action_quat; re-align action against state here too
        # since quat_relative's result is only meaningful for the shorter arc.
        action_q = quat_shortest_arc(action_pose[3:7], state_q)
        delta_q = quat_relative(state_q, action_q)
        if ee_frame == 'base':
            delta_xyz = (action_pose[:3] - state_pose[:3]).tolist()
        else:
            delta_xyz = list(_body_translation(state_q, state_pose[:3], action_pose[:3]))
        return delta_xyz + list(delta_q)

    def _sync_component_diffs(self, t_sec, ctx):
        joint_diff = abs(t_sec - get_closest_t(
            ctx['joint_states_series'], t_sec, times=ctx['joint_states_times']
        ))
        cmd_vel_diff = 0.0
        odom_diff = 0.0
        if self.has_mobile_base:
            cmd_vel_diff = abs(t_sec - get_closest_t(
                ctx['cmd_vel_series'], t_sec, times=ctx['cmd_vel_times']
            ))
            odom_diff = abs(t_sec - get_closest_t(
                ctx['odom_series'], t_sec, times=ctx['odom_times']
            ))
        return joint_diff, cmd_vel_diff, odom_diff

    def _attach_ee_poses(self, frame, tf_tree, t_sec, prev_ee_poses, counters) -> bool:
        stamp_ns = int(t_sec * 1e9)
        for ee_name, ee_src, ee_tgt in self.ee_configs:
            ee_mat = sync_poses.resolve_ee_pose(tf_tree, ee_src, ee_tgt, stamp_ns)
            if ee_mat is None:
                counters['tf'] += 1
                if counters['tf'] <= 5:
                    self.log_warn(
                        f"TF lookup failed: '{ee_src}' → '{ee_tgt}' "
                        f'at t={t_sec:.3f}s. Skipping frame.'
                    )
                return False
            ee_abs, ee_rel = sync_poses.compute_ee_pose_and_delta(ee_mat, prev_ee_poses[ee_name])
            key = f'observation.ee_pose.{ee_name}' if ee_name else 'observation.ee_pose'
            frame[key] = torch.from_numpy(ee_abs.copy())
            frame[f'{key}.delta'] = torch.from_numpy(ee_rel)
            prev_ee_poses[ee_name] = ee_abs.copy()
        return True

    def _attach_images(self, frame, images, depth_images) -> None:
        if not self.skip_cameras:
            for c_name in self.camera_topics.keys():
                img_arr = np.ascontiguousarray(images[c_name].transpose(2, 0, 1))
                frame[f'observation.images.{c_name}'] = torch.from_numpy(img_arr)

        for c_name in self.depth_camera_topics.keys():
            depth_arr = np.ascontiguousarray(depth_images[c_name][..., np.newaxis])
            frame[f'observation.images.{c_name}'] = torch.from_numpy(depth_arr)

    def _attach_subtask(self, frame, subtasks_map, t_sec) -> None:
        # Index 0 ("No Subtask") covers episodes with no subtasks_map.
        if not self.subtask_label_to_idx:
            return
        current_subtask_idx = 0
        for _, st_info in (subtasks_map.items() if isinstance(subtasks_map, dict) else {}):
            start_t = st_info.get('start_timestamp', -1.0) if isinstance(st_info, dict) else -1.0
            end_t = (
                st_info.get('end_timestamp', float('inf')) if isinstance(st_info, dict)
                else float('inf')
            )
            if end_t == 0.0:
                end_t = float('inf')
            if start_t <= t_sec <= end_t:
                label = st_info.get('label', '') if isinstance(st_info, dict) else ''
                current_subtask_idx = self.subtask_label_to_idx.get(label, 0)
                break
        frame['subtask_index'] = torch.tensor([current_subtask_idx], dtype=torch.int64)

    def _log_summary(self, counters) -> None:
        if counters['tf'] > 5:
            self.log_warn(f"TF lookup failed {counters['tf']} times total (suppressed after 5).")
        if counters['img_decode'] > 0:
            self.log_warn(f"Image decode failed {counters['img_decode']} times.")
