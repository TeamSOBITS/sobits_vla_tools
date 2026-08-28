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
JSONL episode log: per-step record building, file lifecycle, summary stats.

Owns the aggregate accumulators (max_block_lift, max_robot_tilt, ...) that
both log_step() and TerminationLogic.evaluate() update -- the latter folds
poses in on the terminal tick, before log_step runs one last time.
"""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import threading
from time import monotonic
from typing import Any, Dict, List, Optional

from sobits_vla_common.gz_utils import wrap_pi


def known_fields(**values) -> Dict[str, float]:
    """Drop unknown (None) components so metadata never records a guess."""
    return {k: v for k, v in values.items() if v is not None}


def sobits_vla_tools_rev() -> str:
    """
    Return the sobits_vla_tools revision for provenance, or 'unknown'.

    Tries SOBITS_VLA_TOOLS_REV (for installed/CI environments), then
    `git describe --always --dirty` from this file's directory — the latter
    only works when running from the source space, since the colcon install
    space is not a git checkout.
    """
    env_rev = os.environ.get('SOBITS_VLA_TOOLS_REV', '').strip()
    if env_rev:
        return env_rev
    try:
        result = subprocess.run(
            ['git', 'describe', '--always', '--dirty'],
            cwd=Path(__file__).resolve().parents[1],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return result.stdout.strip() or 'unknown'
    except Exception:
        pass
    return 'unknown'


def lerobot_version_str() -> str:
    """Return the installed lerobot version as 'major.minor.patch', or 'unknown'."""
    try:
        from sobits_vla_common.lerobot_adapter import LEROBOT_VERSION
        return '.'.join(str(p) for p in LEROBOT_VERSION)
    except Exception:
        return 'unknown'


class StepLog:
    """JSONL file lifecycle + per-step record building + summary accumulators."""

    def __init__(self, log_dir: str, joint_groups: Optional[Dict[str, List[str]]]) -> None:
        self.log_dir = Path(log_dir)
        self.joint_groups: Dict[str, List[str]] = dict(joint_groups or {})
        self.lock = threading.Lock()
        self.file = None
        self.episode_idx = 0
        self.step_idx = 0
        self.t0: Optional[float] = None
        self.active = False
        self.prev_joints: Optional[Dict[str, float]] = None
        self.prev_joint_delta: Optional[Dict[str, float]] = None
        self.reset_accumulators()

    def ensure_log_dir(self) -> None:
        if not str(self.log_dir):
            # An empty log_dir would resolve to '.' and silently write
            # episode files into the process's current working directory.
            self.log_dir = Path('/tmp/vla_logs')
            print(f"[WARN] logging.log_dir is empty -- defaulting to '{self.log_dir}'.")
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def reset_accumulators(self) -> None:
        """Reset per-episode aggregate metrics. Called from begin_episode."""
        self.min_ee_block_dist = float('inf')
        self.max_block_lift = 0.0
        self.final_block_lift = 0.0
        self.max_robot_tilt = 0.0
        self.min_robot_z = float('inf')
        self.joint_path_length = 0.0
        self.max_joint_jerk = 0.0
        self.base_speed_sum = 0.0
        self.base_speed_n = 0
        self.track_err_sum = 0.0
        self.track_err_n = 0
        self.track_err_group_sum: Dict[str, float] = {}
        self.track_err_group_n: Dict[str, int] = {}
        self.time_to_success: Optional[float] = None

    def open_episode(self, meta: Dict[str, Any]) -> None:
        """Bump episode_idx, open a fresh file, write the meta header line."""
        with self.lock:
            if self.file is not None:
                # Defense in depth: unreachable while a file is open, but
                # don't leak the fd if it ever happens.
                print('[WARN] begin_episode: closing already-open episode file.')
                self.file.close()
            self.episode_idx += 1
            self.step_idx = 0
            self.t0 = monotonic()
            self.active = True
            ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
            fname = self.log_dir / f'episode_{self.episode_idx:04d}_{ts}.jsonl'
            self.file = open(fname, 'w')  # buffered; flushed on close in close_episode
            self._write({'type': 'meta', 'episode': self.episode_idx, **meta})

    def next_step(self) -> Optional[tuple]:
        """(t, step_idx) for this tick, or None if the episode isn't active."""
        with self.lock:
            if not self.active or self.file is None:
                return None
            t = monotonic() - self.t0
            step = self.step_idx
            self.step_idx += 1
            return t, step

    def write_step(self, row: Dict[str, Any]) -> None:
        with self.lock:
            self._write(row)

    def build_step_row(
        self, t: float, step: int, joints: Dict[str, float], base_vel: Dict[str, float],
        ee_pose: Optional[List[float]], joints_measured: Optional[Dict[str, float]],
        robot_pose: Optional[Dict[str, float]], block_pose: Optional[Dict[str, float]],
        baseline_block_z: float, baseline_robot_z: float,
        tilt_rad: float, fall_z_drop_m: float,
    ) -> Dict[str, Any]:
        """Build one step record and fold its values into the summary accumulators."""
        joint_delta, max_jerk = self._joint_deltas(joints)
        tracking_error, track_abs_mean, track_by_group = self._tracking_error(
            joints, joints_measured
        )
        base_speed = self._base_speed(base_vel)
        block_lift = self._block_lift(block_pose, baseline_block_z)
        robot_z_drop = self._robot_z_drop(robot_pose, baseline_robot_z)
        ee_world, ee_error = self._ee_world_and_error(ee_pose, robot_pose, block_pose)
        fallen = self._fallen(robot_pose, robot_z_drop, tilt_rad, fall_z_drop_m)

        return {
            'type': 'step',
            't': round(t, 4),
            'step': step,
            'joints': {k: round(v, 5) for k, v in joints.items()},
            'joints_measured': (
                {k: round(v, 5) for k, v in joints_measured.items()}
                if joints_measured else None
            ),
            'joint_delta': (
                {k: round(v, 5) for k, v in joint_delta.items()}
                if joint_delta is not None else None
            ),
            'max_jerk': round(max_jerk, 6),
            'tracking_error': (
                {k: round(v, 5) for k, v in tracking_error.items()}
                if tracking_error is not None else None
            ),
            'track_abs_mean': (
                round(track_abs_mean, 5) if track_abs_mean is not None else None
            ),
            'track_abs_mean_by_group': (
                {k: round(v, 5) for k, v in track_by_group.items()}
                if track_by_group else None
            ),
            'base_vel': {k: round(v, 5) for k, v in base_vel.items()},
            'base_speed': round(base_speed, 5),
            'ee_pose': ee_world,
            'ee_pose_base': (
                {k: round(v, 4) for k, v in zip(
                    ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], ee_pose
                )} if ee_pose is not None else None
            ),
            'ee_error': ee_error,
            'robot_pose': (
                {k: round(robot_pose[k], 4) for k in robot_pose}
                if robot_pose is not None else None
            ),
            'robot_z_drop': round(robot_z_drop, 4) if robot_z_drop is not None else None,
            'block_pose': (
                {'x': round(block_pose['x'], 4),
                 'y': round(block_pose['y'], 4),
                 'z': round(block_pose['z'], 4)}
                if block_pose is not None else None
            ),
            'block_lift': round(block_lift, 4) if block_lift is not None else None,
            'fallen': fallen,
        }

    def _joint_deltas(self, joints: Dict[str, float]) -> tuple:
        joint_delta = None
        max_jerk = 0.0
        if self.prev_joints is not None:
            joint_delta = {
                k: joints[k] - self.prev_joints.get(k, joints[k]) for k in joints
            }
            self.joint_path_length += sum(abs(d) for d in joint_delta.values())
            if self.prev_joint_delta is not None:
                for k, d in joint_delta.items():
                    jerk = abs(d - self.prev_joint_delta.get(k, d))
                    if jerk > max_jerk:
                        max_jerk = jerk
                if max_jerk > self.max_joint_jerk:
                    self.max_joint_jerk = max_jerk
            self.prev_joint_delta = joint_delta
        self.prev_joints = dict(joints)
        return joint_delta, max_jerk

    def _tracking_error(self, joints, joints_measured) -> tuple:
        if not joints_measured:
            return None, None, None
        te = {k: joints[k] - joints_measured[k] for k in joints if k in joints_measured}
        if not te:
            return None, None, None
        track_abs_mean = sum(abs(v) for v in te.values()) / len(te)
        self.track_err_sum += track_abs_mean
        self.track_err_n += 1
        # Per-group means, so downstream analysis selects a group by name
        # instead of guessing joint-name prefixes.
        track_by_group = {}
        for group, feats in self.joint_groups.items():
            vals = [abs(te[f]) for f in feats if f in te]
            if vals:
                track_by_group[group] = sum(vals) / len(vals)
                self.track_err_group_sum[group] = (
                    self.track_err_group_sum.get(group, 0.0) + track_by_group[group]
                )
                self.track_err_group_n[group] = self.track_err_group_n.get(group, 0) + 1
        return te, track_abs_mean, track_by_group

    def _base_speed(self, base_vel: Dict[str, float]) -> float:
        speed = math.hypot(float(base_vel.get('x', 0.0)), float(base_vel.get('y', 0.0)))
        self.base_speed_sum += speed
        self.base_speed_n += 1
        return speed

    def _block_lift(self, block_pose, baseline_block_z) -> Optional[float]:
        if block_pose is None:
            return None
        lift = block_pose['z'] - baseline_block_z
        if lift > self.max_block_lift:
            self.max_block_lift = lift
        self.final_block_lift = lift
        return lift

    def _robot_z_drop(self, robot_pose, baseline_robot_z) -> Optional[float]:
        if robot_pose is None:
            return None
        z_drop = baseline_robot_z - robot_pose['z']
        if robot_pose['z'] < self.min_robot_z:
            self.min_robot_z = robot_pose['z']
        tilt = max(abs(robot_pose['roll']), abs(robot_pose['pitch']))
        if tilt > self.max_robot_tilt:
            self.max_robot_tilt = tilt
        return z_drop

    def _ee_world_and_error(self, ee_pose, robot_pose, block_pose) -> tuple:
        if ee_pose is None or robot_pose is None:
            return None, None
        # base_footprint -> world: robot_xy + R(yaw) * ee_xy, z offset.
        yaw = robot_pose['yaw']
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ee_x_w = robot_pose['x'] + cos_y * ee_pose[0] - sin_y * ee_pose[1]
        ee_y_w = robot_pose['y'] + sin_y * ee_pose[0] + cos_y * ee_pose[1]
        ee_z_w = robot_pose['z'] + ee_pose[2]
        ee_world = {
            'x': round(ee_x_w, 4), 'y': round(ee_y_w, 4), 'z': round(ee_z_w, 4),
            'roll': round(ee_pose[3], 4), 'pitch': round(ee_pose[4], 4),
            'yaw': round(wrap_pi(ee_pose[5] + yaw), 4),
        }
        ee_error = None
        if block_pose is not None:
            dx = block_pose['x'] - ee_x_w
            dy = block_pose['y'] - ee_y_w
            dz = block_pose['z'] - ee_z_w
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.min_ee_block_dist:
                self.min_ee_block_dist = dist
            ee_error = {
                'dx': round(dx, 4), 'dy': round(dy, 4), 'dz': round(dz, 4),
                'dist': round(dist, 4),
            }
        return ee_world, ee_error

    def _fallen(self, robot_pose, robot_z_drop, tilt_rad, fall_z_drop_m) -> bool:
        if robot_pose is None:
            return False
        return (
            abs(robot_pose['roll']) > tilt_rad
            or abs(robot_pose['pitch']) > tilt_rad
            or (robot_z_drop is not None and robot_z_drop > fall_z_drop_m)
        )

    def close_episode(self, outcome: str) -> None:
        """Write the summary line and close the file. False if not active."""
        with self.lock:
            if not self.active:
                return False
            self.active = False
            duration = monotonic() - self.t0 if self.t0 else 0.0
            self._write(self._build_summary(outcome, duration))
            if self.file:
                self.file.close()
                self.file = None
            return True

    def _build_summary(self, outcome: str, duration: float) -> Dict[str, Any]:
        mean_base_speed = self.base_speed_sum / self.base_speed_n if self.base_speed_n else 0.0
        mean_track_err = (
            self.track_err_sum / self.track_err_n if self.track_err_n else None
        )
        mean_track_err_by_group = {
            g: round(s / self.track_err_group_n[g], 5)
            for g, s in self.track_err_group_sum.items()
            if self.track_err_group_n.get(g)
        } or None
        return {
            'type': 'summary',
            'outcome': outcome,
            'success': outcome == 'success_lift',
            'end_time_iso': datetime.now(timezone.utc).isoformat(),
            'duration_s': round(duration, 3),
            'total_steps': self.step_idx,
            'time_to_success_s': (
                round(self.time_to_success, 3) if self.time_to_success is not None else None
            ),
            'min_ee_block_dist': (
                round(self.min_ee_block_dist, 4)
                if math.isfinite(self.min_ee_block_dist) else None
            ),
            'max_block_lift': round(self.max_block_lift, 4),
            'final_block_lift': round(self.final_block_lift, 4),
            'max_robot_tilt_rad': round(self.max_robot_tilt, 4),
            'max_robot_tilt_deg': round(math.degrees(self.max_robot_tilt), 2),
            'min_robot_z': (
                round(self.min_robot_z, 4) if math.isfinite(self.min_robot_z) else None
            ),
            'joint_path_length': round(self.joint_path_length, 4),
            'max_joint_jerk': round(self.max_joint_jerk, 6),
            'mean_base_speed': round(mean_base_speed, 5),
            'mean_abs_tracking_error': (
                round(mean_track_err, 5) if mean_track_err is not None else None
            ),
            'mean_abs_tracking_error_by_group': mean_track_err_by_group,
        }

    def _write(self, row: Dict[str, Any]) -> None:
        """Write one JSON line. Must be called under self.lock."""
        if self.file:
            self.file.write(json.dumps(row, separators=(',', ':')) + '\n')
