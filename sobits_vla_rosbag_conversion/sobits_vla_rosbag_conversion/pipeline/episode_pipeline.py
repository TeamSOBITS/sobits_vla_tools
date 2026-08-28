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


"""
Per-episode conversion: read one bag, synthesize frames, write them out.

No rclpy import. EpisodePipeline.run() is convert()'s inner-loop body,
factored out so the node just orchestrates discovery -> pipeline -> stats.
"""

from dataclasses import dataclass, field
import os
import traceback
from typing import Any, Dict, Optional

import numpy as np
from sobits_vla_rosbag_conversion.bag_reader import BagReader


@dataclass
class EpisodeResult:
    """Outcome of converting one episode bag; either a skip or written frames."""

    bag: str
    skipped: bool = False
    skip_reason: str = ''
    skip_detail: Dict[str, Any] = field(default_factory=dict)
    task: str = ''
    frames: int = 0
    skipped_downsample: int = 0
    skipped_static: int = 0
    skipped_tf: int = 0
    skipped_img_decode: int = 0
    sync_avg_ms: Optional[float] = None
    sync_max_ms: Optional[float] = None
    sync_min_ms: Optional[float] = None
    fps_warning: Optional[Dict[str, Any]] = None


class EpisodePipeline:
    """Converts one episode bag into frames written to a shared DatasetWriter."""

    def __init__(
        self,
        camera_topics: Dict[str, str],
        depth_camera_topics: Dict[str, str],
        topic_to_cam: Dict[str, str],
        depth_topic_to_cam: Dict[str, str],
        joint_states_topic: str,
        part_command_topics: set,
        cmd_vel_topic: str,
        odom_topic: str,
        has_mobile_base: bool,
        has_cmd_vel_y: bool,
        has_cmd_vel_z: bool,
        ee_pose_enabled: bool,
        fps: float,
        synthesizer,
        writer,
        logger=None,
    ) -> None:
        self._camera_topics = camera_topics
        self._depth_camera_topics = depth_camera_topics
        self._topic_to_cam = topic_to_cam
        self._depth_topic_to_cam = depth_topic_to_cam
        self._joint_states_topic = joint_states_topic
        self._part_command_topics = part_command_topics
        self._cmd_vel_topic = cmd_vel_topic
        self._odom_topic = odom_topic
        self._has_mobile_base = has_mobile_base
        self._has_cmd_vel_y = has_cmd_vel_y
        self._has_cmd_vel_z = has_cmd_vel_z
        self._ee_pose_enabled = ee_pose_enabled
        self._fps = fps
        self._synthesizer = synthesizer
        self._writer = writer
        self._logger = logger

    def _log(self, level: str, msg: str) -> None:
        if self._logger is not None:
            getattr(self._logger, level)(msg)

    def _wanted_topics(self) -> set:
        wanted = set(self._camera_topics.values()) | {self._joint_states_topic}
        wanted |= set(self._depth_camera_topics.values())
        wanted |= self._part_command_topics
        if self._has_mobile_base and self._cmd_vel_topic:
            wanted.add(self._cmd_vel_topic)
        if self._has_mobile_base and self._odom_topic:
            wanted.add(self._odom_topic)
        if self._ee_pose_enabled:
            wanted |= {'/tf', '/tf_static'}
        return wanted

    def _missing_topic_reason(self, topic: str) -> str:
        if topic == self._joint_states_topic:
            return 'joint state observation'
        if topic in self._part_command_topics:
            return 'joint command (action)'
        if topic == self._cmd_vel_topic:
            return 'base velocity command (action.base)'
        if topic in self._topic_to_cam:
            return f'camera image ({self._topic_to_cam[topic]})'
        if topic in self._depth_topic_to_cam:
            return f'depth camera image ({self._depth_topic_to_cam[topic]})'
        if topic in ('/tf', '/tf_static'):
            return 'TF transforms (ee_pose)'
        return 'unknown'

    def _missing_topic_code(self, topic: str) -> str:
        if topic == self._joint_states_topic:
            return 'joint_state'
        if topic in self._part_command_topics:
            return 'joint_command'
        if topic == self._cmd_vel_topic:
            return 'base_velocity'
        if topic in self._topic_to_cam:
            return f'camera:{self._topic_to_cam[topic]}'
        if topic in self._depth_topic_to_cam:
            return f'depth_camera:{self._depth_topic_to_cam[topic]}'
        if topic in ('/tf', '/tf_static'):
            return 'tf'
        return 'unknown'

    def run(
        self, ep_path: str, bagfile: str, instruction: str, subtasks: dict
    ) -> Optional[EpisodeResult]:
        """Convert one episode; None means the synthesizer found nothing to write."""
        bag_dir = os.path.dirname(bagfile)
        wanted = self._wanted_topics()

        bag_reader = BagReader(bag_dir, logger=self._logger)
        missing = bag_reader.check_missing_topics(wanted)
        if missing:
            return self._skip_missing_topics(bag_dir, missing)

        try:
            result = self._read_and_synthesize(bag_reader, wanted, subtasks, instruction)
        except Exception as e:
            self._log('error', f'Failed to read/process bag {bagfile}: {e}')
            self._log('error', traceback.format_exc())
            return EpisodeResult(
                bag=bag_dir, skipped=True, skip_reason='read_error',
                skip_detail={'error': str(e)},
            )

        if result is None:
            return None

        frames, sync_deltas, skipped_static, skipped_tf, skipped_img, skipped_ds = result
        if not frames:
            self._log(
                'warning',
                f'No frames extracted from {bagfile} '
                f'(downsample={skipped_ds}, static={skipped_static}, '
                f'tf={skipped_tf}, img_decode={skipped_img})',
            )
            return EpisodeResult(
                bag=bag_dir, skipped=True, skip_reason='no_frames',
                skip_detail={
                    'skipped_downsample': skipped_ds, 'skipped_static': skipped_static,
                    'skipped_tf': skipped_tf, 'skipped_img_decode': skipped_img,
                },
            )

        return self._write_frames(
            ep_path, bagfile, instruction, frames, sync_deltas,
            skipped_static, skipped_tf, skipped_img, skipped_ds,
        )

    def _skip_missing_topics(self, bag_dir: str, missing: set) -> EpisodeResult:
        self._log(
            'error',
            f"Bag '{bag_dir}' is missing {len(missing)} required topic(s). Skipping episode.",
        )
        missing_report = {}
        for m in sorted(missing):
            self._log('error', f'  missing: {m}  ({self._missing_topic_reason(m)})')
            missing_report[m] = self._missing_topic_code(m)
        return EpisodeResult(
            bag=bag_dir, skipped=True, skip_reason='missing_topics',
            skip_detail={'missing': missing_report},
        )

    def _read_and_synthesize(self, bag_reader, wanted, subtasks, instruction):
        # Merged for cam_series bucketing only, not decode routing.
        read_topic_to_cam = {**self._topic_to_cam, **self._depth_topic_to_cam}
        bag_series = bag_reader.read_topic_series(
            wanted_topics=wanted,
            topic_to_cam=read_topic_to_cam,
            part_command_topics=self._part_command_topics,
            cmd_vel_topic=self._cmd_vel_topic,
            odom_topic=self._odom_topic,
            joint_states_topic=self._joint_states_topic,
            ee_pose_enabled=self._ee_pose_enabled,
            has_mobile_base=self._has_mobile_base,
            has_cmd_vel_y=self._has_cmd_vel_y,
            has_cmd_vel_z=self._has_cmd_vel_z,
        )
        return self._synthesizer.synthesize(
            bag_series=bag_series, subtasks_map=subtasks, instruction=instruction,
        )

    def _write_frames(
        self, ep_path, bagfile, instruction, frames, sync_deltas,
        skipped_static, skipped_tf, skipped_img, skipped_ds,
    ) -> EpisodeResult:
        fps_warning = None
        if len(frames) > 1:
            duration = frames[-1][0] - frames[0][0]
            actual_fps = (len(frames) - 1) / duration if duration > 0 else 0.0
            if actual_fps < self._fps * 0.8:
                self._log(
                    'warning',
                    f'Proceeding with {bagfile}: actual fps ({actual_fps:.1f}) is below '
                    f'configured fps ({self._fps}) threshold (80%).',
                )
                fps_warning = {
                    'bag': os.path.dirname(bagfile),
                    'reason': 'fps_below_target',
                    'actual_fps': round(actual_fps, 1),
                    'required_fps': self._fps,
                }

        for _, frame in frames:
            self._writer.add_frame(frame)
        self._writer.save_episode()

        result = EpisodeResult(
            bag=os.path.basename(ep_path), task=instruction, frames=len(frames),
            skipped_downsample=skipped_ds, skipped_static=skipped_static,
            skipped_tf=skipped_tf, skipped_img_decode=skipped_img, fps_warning=fps_warning,
        )
        if sync_deltas:
            result.sync_avg_ms = round(float(np.mean(sync_deltas) * 1000), 2)
            result.sync_max_ms = round(float(np.max(sync_deltas) * 1000), 2)
            result.sync_min_ms = round(float(np.min(sync_deltas) * 1000), 2)
        self._log('info', f'  → Saved {len(frames)} frames.')
        return result
