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
Logger for the VLA deployment node.

Writes one JSON-Lines file per episode:
  <log_dir>/episode_<N>_<timestamp>.jsonl

Each line is one timestep dict with:
  t              — float, seconds since episode start
  step           — int, step index within episode
  joints         — dict[joint_name -> rad]  (commanded)
  joints_measured— dict[joint_name -> rad]  (measured, None if unavailable)
  joint_delta    — dict[joint_name -> rad]  (commanded delta vs previous step)
  max_jerk       — float, max |2nd difference| over joints this step
  tracking_error — dict[joint_name -> rad]  (commanded - measured)
  track_abs_mean — float, mean |tracking_error| this step
  track_abs_mean_by_group — dict[group -> mean |tracking_error|] using the
                   descriptor's joint groups, so analysis needs no name prefixes
  base_vel       — {x, y, theta}
  base_speed     — float, sqrt(x^2 + y^2)
  ee_pose        — {x, y, z, roll, pitch, yaw} in the WORLD frame, directly
                   comparable with block_pose/robot_pose (None if TF miss)
  ee_pose_base   — the same pose in base_footprint, for controller debugging
  ee_error       — {dx, dy, dz, dist} relative to block position (None if gz query fails)
  robot_pose     — {x, y, z, roll, pitch, yaw} in world frame from Gazebo
  robot_z_drop   — float, baseline_robot_z - robot_pose.z
  block_pose     — {x, y, z} world frame
  block_lift     — float, block_pose.z - baseline_block_z
  fallen         — bool, |roll|/|pitch| > tilt_threshold or robot_z_drop > fall_z_drop_m

Episode metadata header (first line, type=="meta"):
  episode, start_time_iso, robot_name, world_name, spawn_pose, block_reset_pose,
  baseline_block_z, baseline_robot_z, thresholds

After stop, writes a final line type=="summary" with the episode outcome and
aggregate metrics: outcome, success, end_time_iso, duration_s, total_steps, time_to_success_s,
min_ee_block_dist, max_block_lift, final_block_lift, max_robot_tilt_*,
min_robot_z, joint_path_length, max_joint_jerk, mean_base_speed,
mean_abs_tracking_error.

Automatic termination (evaluate_termination, polled by the deploy node):
  - success_lift: block lifted > lift_success_m (default 0.05)
  - fallen:       robot tilt > tilt_threshold or world-z drop > fall_z_drop_m
  - timeout:      elapsed sim time >= episode_timeout_s (default 60)
  - dropped:      place mode only -- object left low and away from the
                  goal for drop_abort_s (0 = disabled)

World reset between episodes is performed by the shared world_reset_node
(sobits_vla_common); the spawn/block poses here only seed the lift and fall
baselines below.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import threading
from time import monotonic
from typing import Any, Dict, List, Optional

from sobits_vla_common.gz_utils import gz_get_pose, wrap_pi


def _known_fields(**values) -> Dict[str, float]:
    """Drop unknown (None) components so metadata never records a guess."""
    return {k: v for k, v in values.items() if v is not None}


def _sobits_vla_tools_rev() -> str:
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
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return result.stdout.strip() or 'unknown'
    except Exception:
        pass
    return 'unknown'


def _lerobot_version_str() -> str:
    """Return the installed lerobot version as 'major.minor.patch', or 'unknown'."""
    try:
        from sobits_vla_common.lerobot_adapter import LEROBOT_VERSION
        return '.'.join(str(p) for p in LEROBOT_VERSION)
    except Exception:
        return 'unknown'


def _gz_get_pose_fast(world_name: str, model_name: str) -> Optional[Dict[str, float]]:
    """
    Like gz_get_pose but uses a shorter timeout for lower latency.

    Falls back to pose/info on any error.
    """
    return gz_get_pose(world_name, model_name, timeout=1.5)


class EpisodeLogger:
    """
    Logs one episode to a JSON-Lines file.

    Thread-safe: log_step() may be called from the control timer thread.
    begin_episode() and end_episode() are called from the joy/play callbacks.
    reset_world() is called from end_episode() in a background thread so it
    does not block the ROS spin.

    Parameters:
      log_dir          — output directory (default /tmp/vla_logs)
      world_name       — Gazebo world name ('' = unset)
      robot_name       — Gazebo model name of the robot ('' = fall detection off)
      block_name       — Gazebo model name of the tracked object ('' = lift scoring off)
      spawn_z/block_z  — reset heights; the lift and fall baselines
      spawn_x/y, spawn_qx/y/z/w, block_x/y — recorded-only. None (the default)
                          omits the field rather than inventing a pose.
      tilt_threshold_deg — |roll| or |pitch| above this → fallen=True (default 30°)
      episode_timeout_s — auto-terminate after this many sim seconds (default 60)
      lift_success_m    — block lift above this → success_lift (default 0.05)
      place_z_max_m     — for place mode, the object must also be below
                          this world z (0 = no height condition)
      success_settle_s  — ignore lift crossings for this long after PLAY, so
                          world-reset settling cannot score a success
                          (default 2.0)
      fall_z_drop_m     — robot world-z drop above this → fallen (default 0.15)
      enabled          — master switch; if False all methods are no-ops
    """

    def __init__(
        self,
        log_dir: str = '/tmp/vla_logs',
        world_name: str = '',
        robot_name: str = '',
        block_name: str = '',
        # Only the z components are used (lift/fall baselines); x/y/orientation
        # are recorded-only. None means unknown -- omitted, never invented.
        spawn_z: float = 0.0,
        block_z: float = 0.0,
        spawn_x: Optional[float] = None,
        spawn_y: Optional[float] = None,
        spawn_qx: Optional[float] = None,
        spawn_qy: Optional[float] = None,
        spawn_qz: Optional[float] = None,
        spawn_qw: Optional[float] = None,
        block_x: Optional[float] = None,
        block_y: Optional[float] = None,
        tilt_threshold_deg: float = 30.0,
        episode_timeout_s: float = 60.0,
        lift_success_m: float = 0.05,
        fall_z_drop_m: float = 0.15,
        success_settle_s: float = 2.0,
        goal_name: str = '',
        place_radius_m: float = 0.12,
        place_settle_s: float = 1.0,
        place_z_max_m: float = 0.0,
        drop_abort_s: float = 0.0,
        drop_abort_z_max_m: float = 0.0,
        enabled: bool = True,
        model_repo_id: str = '',
        sim_enabled: bool = True,
        joint_groups: Optional[Dict[str, List[str]]] = None,
    ) -> None:
        self._model_repo_id = model_repo_id
        # {group_name: [joint feature names]} from the robot descriptor; lets
        # tracking error be reported per group instead of by name prefix.
        self._joint_groups: Dict[str, List[str]] = dict(joint_groups or {})
        self._log_dir = Path(log_dir)
        self._world_name = world_name
        self._robot_name = robot_name
        self._block_name = block_name
        # Pick-and-place goal. Empty = lift-only scoring (plain pickup task).
        self._goal_name = goal_name
        self._place_radius_m = place_radius_m
        self._place_settle_s = place_settle_s
        # Object must also sit below this world z to count as placed
        # (0 = no height condition). Separates 'in the bin' from
        # 'on the floor next to the bin'.
        self._place_z_max_m = place_z_max_m
        # Early abort: if the object sits below drop_abort_z_max_m and
        # farther than place_radius_m from the goal continuously for
        # drop_abort_s, the episode is unrecoverable -- stop waiting.
        # 0 disables. Measured separation on the can task: true drops
        # dwell 170-188 s, the worst success only 4.7 s.
        self._drop_abort_s = drop_abort_s
        self._drop_abort_z_max_m = drop_abort_z_max_m
        self._dropped_since: Optional[float] = None
        self._goal_xy: Optional[tuple] = None
        self._goal_reached_at: Optional[float] = None
        self._lifted = False
        self._spawn = (spawn_x, spawn_y, spawn_z, spawn_qx, spawn_qy, spawn_qz, spawn_qw)
        self._block_reset = (block_x, block_y, block_z)
        self._tilt_rad = math.radians(tilt_threshold_deg)
        # Automatic termination thresholds.
        self._episode_timeout_s = episode_timeout_s
        self._lift_success_m = lift_success_m
        self._fall_z_drop_m = fall_z_drop_m
        # Grace period after PLAY during which a lift-threshold crossing is
        # ignored (world-reset settling, not a real pick).
        self._success_settle_s = success_settle_s
        self.enabled = enabled

        self._lock = threading.Lock()
        self._file = None
        self._episode_idx = 0
        self._step_idx = 0
        self._t0: Optional[float] = None
        self._active = False

        # Per-episode baselines (sampled at begin_episode) and aggregate
        # accumulators (summarised at end_episode). All reset per episode.
        self._baseline_block_z: float = block_z
        self._baseline_robot_z: float = spawn_z
        self._prev_joints: Optional[Dict[str, float]] = None
        self._prev_joint_delta: Optional[Dict[str, float]] = None
        self._reset_accumulators()

        # Cached gz poses — updated by background poller at ~5 Hz so log_step()
        # never blocks the 10 Hz control loop with subprocess calls.
        self._cached_robot_pose: Optional[Dict[str, float]] = None
        self._cached_block_pose: Optional[Dict[str, float]] = None
        self._pose_lock = threading.Lock()
        self._poller_stop = threading.Event()
        self._poller_thread: Optional[threading.Thread] = None

        # Real robot (sim_enabled=False): no Gazebo to poll, so poses stay None
        # (lift/fall auto-term unavailable, timeout still works); skip poller.
        self._sim_enabled = sim_enabled
        if self.enabled:
            if not log_dir:
                # An empty log_dir would resolve to '.' and silently write
                # episode files into the process's current working directory.
                self._log_dir = Path('/tmp/vla_logs')
                print(
                    "[WARN] logging.log_dir is empty -- defaulting to '{}'.".format(
                        self._log_dir
                    )
                )
            self._log_dir.mkdir(parents=True, exist_ok=True)
            if self._sim_enabled:
                self._start_poller()

    def begin_episode(self) -> None:
        if not self.enabled:
            return
        # Fresh blocking read, not the 200 ms-lagged poller cache: a stale
        # pre-teleport pose makes the reset itself read as a +0.4 m lift.
        block_pose = _gz_get_pose_fast(self._world_name, self._block_name) \
            if (self._sim_enabled and self._block_name) else None
        robot_pose = _gz_get_pose_fast(self._world_name, self._robot_name) \
            if (self._sim_enabled and self._robot_name) else None

        # Deviating further than this from the reset height means the read
        # caught the block mid-teleport -- use the configured value instead.
        max_baseline_dev_m = self._lift_success_m

        if block_pose is not None and abs(
            float(block_pose['z']) - self._block_reset[2]
        ) <= max_baseline_dev_m:
            self._baseline_block_z = float(block_pose['z'])
        else:
            if block_pose is not None:
                print(
                    '[WARN] begin_episode: block z={:.4f} deviates >{:.3f} m '
                    'from the reset height {:.4f} (stale/incomplete teleport?) '
                    '-- using the configured baseline.'.format(
                        float(block_pose['z']),
                        max_baseline_dev_m,
                        self._block_reset[2],
                    )
                )
            self._baseline_block_z = self._block_reset[2]

        if robot_pose is not None:
            self._baseline_robot_z = float(robot_pose['z'])
        else:
            self._baseline_robot_z = self._spawn[2]
        # Goal xy is re-read per episode: randomize may have moved it.
        self._goal_xy = None
        self._goal_reached_at = None
        self._dropped_since = None
        self._lifted = False

        # Seed the cache so the first tick uses post-reset poses.
        with self._pose_lock:
            if block_pose is not None:
                self._cached_block_pose = block_pose
            if robot_pose is not None:
                self._cached_robot_pose = robot_pose
        self._prev_joints = None
        self._prev_joint_delta = None
        self._reset_accumulators()
        with self._lock:
            if self._file is not None:
                # Defense in depth: begin_episode should be unreachable while a
                # file is open, but don't leak the fd if it ever happens.
                print('[WARN] begin_episode: closing already-open episode file.')
                self._file.close()
            self._episode_idx += 1
            self._step_idx = 0
            self._t0 = monotonic()
            self._active = True
            ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
            fname = self._log_dir / 'episode_{:04d}_{}.jsonl'.format(
                self._episode_idx, ts
            )
            self._file = open(fname, 'w')  # buffered; flushed on close in end_episode
            sx, sy, sz, sqx, sqy, sqz, sqw = self._spawn
            bx, by, bz = self._block_reset
            spawn_pose = _known_fields(
                x=sx, y=sy, z=sz, qx=sqx, qy=sqy, qz=sqz, qw=sqw
            )
            block_reset_pose = _known_fields(x=bx, y=by, z=bz)
            meta = {
                'type': 'meta',
                'episode': self._episode_idx,
                'start_time_iso': datetime.now(timezone.utc).isoformat(),
                'lerobot_version': _lerobot_version_str(),
                'sobits_vla_tools_rev': _sobits_vla_tools_rev(),
                'model_repo_id': self._model_repo_id,
                'world_name': self._world_name,
                'robot_name': self._robot_name,
                'block_name': self._block_name,
                'spawn_pose': spawn_pose,
                'block_reset_pose': block_reset_pose,
                'joint_groups': self._joint_groups or None,
                'baseline_block_z': round(self._baseline_block_z, 4),
                'baseline_robot_z': round(self._baseline_robot_z, 4),
                'thresholds': {
                    'episode_timeout_s': self._episode_timeout_s,
                    'lift_success_m': self._lift_success_m,
                    'fall_z_drop_m': self._fall_z_drop_m,
                    'tilt_threshold_deg': round(math.degrees(self._tilt_rad), 3),
                },
            }
            self._write(meta)

    def log_step(
        self,
        joints: Dict[str, float],
        base_vel: Dict[str, float],
        ee_pose: Optional[List[float]],
        joints_measured: Optional[Dict[str, float]] = None,
    ) -> None:
        """
        Call once per control tick while episode is active.

        joints:          commanded joint positions (rad).
        joints_measured: measured joint positions (rad) for tracking error;
                         may be None when unavailable.
        base_vel:        {x, y, theta} commanded base velocity.
        ee_pose:         [x, y, z, roll, pitch, yaw] in base_footprint, or None.

        Gazebo poses are read from cache updated by background poller — zero
        blocking latency on the 10 Hz control loop.
        """
        if not self.enabled:
            return
        with self._lock:
            if not self._active or self._file is None:
                return
            t = monotonic() - self._t0
            step = self._step_idx
            self._step_idx += 1

        # Read cached poses — never blocks.
        with self._pose_lock:
            robot_pose = dict(self._cached_robot_pose) if self._cached_robot_pose else None
            block_pose = dict(self._cached_block_pose) if self._cached_block_pose else None

        # ---- Per-joint deltas (smoothness) and jerk (2nd difference) --------
        joint_delta: Optional[Dict[str, float]] = None
        max_jerk_this_step = 0.0
        if self._prev_joints is not None:
            joint_delta = {
                k: joints[k] - self._prev_joints.get(k, joints[k]) for k in joints
            }
            step_path = sum(abs(d) for d in joint_delta.values())
            self._joint_path_length += step_path
            if self._prev_joint_delta is not None:
                for k, d in joint_delta.items():
                    jerk = abs(d - self._prev_joint_delta.get(k, d))
                    if jerk > max_jerk_this_step:
                        max_jerk_this_step = jerk
                if max_jerk_this_step > self._max_joint_jerk:
                    self._max_joint_jerk = max_jerk_this_step
            self._prev_joint_delta = joint_delta
        self._prev_joints = dict(joints)

        # ---- Tracking error (commanded - measured) -------------------------
        tracking_error: Optional[Dict[str, float]] = None
        track_abs_mean: Optional[float] = None
        track_by_group: Optional[Dict[str, float]] = None
        if joints_measured:
            te = {
                k: joints[k] - joints_measured[k]
                for k in joints if k in joints_measured
            }
            if te:
                tracking_error = te
                track_abs_mean = sum(abs(v) for v in te.values()) / len(te)
                self._track_err_sum += track_abs_mean
                self._track_err_n += 1
                # Per-group means, so downstream analysis selects a group by
                # name instead of guessing joint-name prefixes.
                track_by_group = {}
                for group, feats in self._joint_groups.items():
                    vals = [abs(te[f]) for f in feats if f in te]
                    if vals:
                        track_by_group[group] = sum(vals) / len(vals)
                        self._track_err_group_sum[group] = (
                            self._track_err_group_sum.get(group, 0.0)
                            + track_by_group[group]
                        )
                        self._track_err_group_n[group] = (
                            self._track_err_group_n.get(group, 0) + 1
                        )

        # ---- Base speed ----------------------------------------------------
        base_speed = math.hypot(
            float(base_vel.get('x', 0.0)), float(base_vel.get('y', 0.0))
        )
        self._base_speed_sum += base_speed
        self._base_speed_n += 1

        # ---- Block lift relative to baseline -------------------------------
        block_lift = None
        if block_pose is not None:
            block_lift = block_pose['z'] - self._baseline_block_z
            if block_lift > self._max_block_lift:
                self._max_block_lift = block_lift
            self._final_block_lift = block_lift

        # ---- Robot world-z drop relative to baseline -----------------------
        robot_z_drop = None
        if robot_pose is not None:
            robot_z_drop = self._baseline_robot_z - robot_pose['z']
            if robot_pose['z'] < self._min_robot_z:
                self._min_robot_z = robot_pose['z']
            tilt = max(abs(robot_pose['roll']), abs(robot_pose['pitch']))
            if tilt > self._max_robot_tilt:
                self._max_robot_tilt = tilt

        # EE in world frame, and its error relative to the block.
        ee_world = None
        ee_error = None
        if ee_pose is not None and robot_pose is not None:
            # base_footprint -> world: robot_xy + R(yaw) * ee_xy, z offset.
            yaw = robot_pose['yaw']
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            ee_x_w = robot_pose['x'] + cos_y * ee_pose[0] - sin_y * ee_pose[1]
            ee_y_w = robot_pose['y'] + sin_y * ee_pose[0] + cos_y * ee_pose[1]
            ee_z_w = robot_pose['z'] + ee_pose[2]
            ee_world = {
                'x': round(ee_x_w, 4),
                'y': round(ee_y_w, 4),
                'z': round(ee_z_w, 4),
                'roll': round(ee_pose[3], 4),
                'pitch': round(ee_pose[4], 4),
                'yaw': round(wrap_pi(ee_pose[5] + yaw), 4),
            }
        if ee_world is not None and block_pose is not None:
            dx = block_pose['x'] - ee_x_w
            dy = block_pose['y'] - ee_y_w
            dz = block_pose['z'] - ee_z_w
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self._min_ee_block_dist:
                self._min_ee_block_dist = dist
            ee_error = {
                'dx': round(dx, 4),
                'dy': round(dy, 4),
                'dz': round(dz, 4),
                'dist': round(dist, 4),
            }

        # Collision / fallen detection from robot world pose
        fallen = False
        if robot_pose is not None:
            fallen = (
                abs(robot_pose['roll']) > self._tilt_rad
                or abs(robot_pose['pitch']) > self._tilt_rad
                or (robot_z_drop is not None and robot_z_drop > self._fall_z_drop_m)
            )

        row: Dict[str, Any] = {
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
            'max_jerk': round(max_jerk_this_step, 6),
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
            # World frame -- directly comparable with block_pose/robot_pose.
            'ee_pose': ee_world,
            # Raw base_footprint frame, kept for controller-side debugging.
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

        with self._lock:
            self._write(row)

    def evaluate_termination(self, elapsed_sim_s: float) -> Optional[str]:
        """
        Check the automatic termination conditions.

        Evaluates the latest cached Gazebo poses and the elapsed simulation
        time. Returns the outcome reason or None if the episode should continue:
          "success_lift" — block lifted more than lift_success_m
          "fallen"       — robot tilt > threshold or world-z drop > fall_z_drop_m
          "timeout"      — elapsed_sim_s >= episode_timeout_s
        """
        if not self.enabled or not self._active:
            return None
        with self._pose_lock:
            robot_pose = dict(self._cached_robot_pose) if self._cached_robot_pose else None
            block_pose = dict(self._cached_block_pose) if self._cached_block_pose else None

        # Fold poses into accumulators here: this tick returns before log_step
        # runs, so the summary would otherwise miss the terminal instant.
        if block_pose is not None:
            # Success: block lifted clear of its start height.
            lift = block_pose['z'] - self._baseline_block_z
            if lift > self._max_block_lift:
                self._max_block_lift = lift
            self._final_block_lift = lift
            # A crossing this early is reset settling, not a pick: real picks
            # took 30-45 s, the observed false positive fired at 0.84 s.
            settled = elapsed_sim_s >= self._success_settle_s
            # Use the running MAX lift: a shelf -> floor-bin placement ends
            # far below its start height, so the instantaneous lift is
            # negative by the time the object reaches the goal.
            if self._max_block_lift > self._lift_success_m and settled:
                self._lifted = True
                # Pickup-only task: the lift itself is the success criterion.
                if not self._goal_name:
                    if self._time_to_success is None:
                        self._time_to_success = elapsed_sim_s
                    return 'success_lift'

            # Pick-and-place: the object must reach the goal AND stay there,
            # so a fly-through on the way past does not score.
            if self._goal_name and self._lifted and settled:
                if self._goal_xy is None:
                    goal = gz_get_pose(self._world_name, self._goal_name)
                    if goal is not None:
                        self._goal_xy = (goal['x'], goal['y'])
                if self._goal_xy is not None:
                    dist = math.hypot(
                        block_pose['x'] - self._goal_xy[0],
                        block_pose['y'] - self._goal_xy[1],
                    )
                    low_enough = (self._place_z_max_m <= 0.0
                                  or block_pose['z'] <= self._place_z_max_m)
                    if dist <= self._place_radius_m and low_enough:
                        if self._goal_reached_at is None:
                            self._goal_reached_at = elapsed_sim_s
                        elif elapsed_sim_s - self._goal_reached_at >= self._place_settle_s:
                            if self._time_to_success is None:
                                self._time_to_success = elapsed_sim_s
                            return 'success_place'
                    else:
                        self._goal_reached_at = None

                    # Early abort: object at rest low and away from the goal.
                    if self._drop_abort_s > 0.0 and self._lifted:
                        low = (self._drop_abort_z_max_m <= 0.0
                               or block_pose['z'] <= self._drop_abort_z_max_m)
                        if low and dist > self._place_radius_m:
                            if self._dropped_since is None:
                                self._dropped_since = elapsed_sim_s
                            elif (elapsed_sim_s - self._dropped_since
                                    >= self._drop_abort_s):
                                return 'dropped'
                        else:
                            self._dropped_since = None

        # Failure: the robot tipped over or dropped in world-z.
        if robot_pose is not None:
            tilt = max(abs(robot_pose['roll']), abs(robot_pose['pitch']))
            z_drop = self._baseline_robot_z - robot_pose['z']
            if tilt > self._max_robot_tilt:
                self._max_robot_tilt = tilt
            if robot_pose['z'] < self._min_robot_z:
                self._min_robot_z = robot_pose['z']
            if tilt > self._tilt_rad or z_drop > self._fall_z_drop_m:
                return 'fallen'

        # Time budget exhausted.
        if elapsed_sim_s >= self._episode_timeout_s:
            return 'timeout'

        return None

    def end_episode(self, outcome: str = 'manual_stop') -> None:
        """Write the per-episode summary and close the episode file."""
        if not self.enabled:
            return
        with self._lock:
            if not self._active:
                return
            self._active = False
            duration = monotonic() - self._t0 if self._t0 else 0.0
            total_steps = self._step_idx
            mean_base_speed = (
                self._base_speed_sum / self._base_speed_n
                if self._base_speed_n else 0.0
            )
            mean_track_err = (
                self._track_err_sum / self._track_err_n
                if self._track_err_n else None
            )
            mean_track_err_by_group = {
                g: round(s / self._track_err_group_n[g], 5)
                for g, s in self._track_err_group_sum.items()
                if self._track_err_group_n.get(g)
            } or None
            summary = {
                'type': 'summary',
                'outcome': outcome,
                'success': outcome == 'success_lift',
                'end_time_iso': datetime.now(timezone.utc).isoformat(),
                'duration_s': round(duration, 3),
                'total_steps': total_steps,
                'time_to_success_s': (
                    round(self._time_to_success, 3)
                    if self._time_to_success is not None else None
                ),
                'min_ee_block_dist': (
                    round(self._min_ee_block_dist, 4)
                    if math.isfinite(self._min_ee_block_dist) else None
                ),
                'max_block_lift': round(self._max_block_lift, 4),
                'final_block_lift': round(self._final_block_lift, 4),
                'max_robot_tilt_rad': round(self._max_robot_tilt, 4),
                'max_robot_tilt_deg': round(math.degrees(self._max_robot_tilt), 2),
                'min_robot_z': (
                    round(self._min_robot_z, 4)
                    if math.isfinite(self._min_robot_z) else None
                ),
                'joint_path_length': round(self._joint_path_length, 4),
                'max_joint_jerk': round(self._max_joint_jerk, 6),
                'mean_base_speed': round(mean_base_speed, 5),
                'mean_abs_tracking_error': (
                    round(mean_track_err, 5) if mean_track_err is not None else None
                ),
                'mean_abs_tracking_error_by_group': mean_track_err_by_group,
            }
            self._write(summary)
            if self._file:
                self._file.close()
                self._file = None

    def shutdown(self) -> None:
        """Stop background poller. Call from deploy node destroy_node()."""
        self._poller_stop.set()
        if self._poller_thread is not None:
            self._poller_thread.join(timeout=2.0)

    def _reset_accumulators(self) -> None:
        """Reset per-episode aggregate metrics. Called from begin_episode."""
        self._min_ee_block_dist = float('inf')
        self._max_block_lift = 0.0
        self._final_block_lift = 0.0
        self._max_robot_tilt = 0.0
        self._min_robot_z = float('inf')
        self._joint_path_length = 0.0
        self._max_joint_jerk = 0.0
        self._base_speed_sum = 0.0
        self._base_speed_n = 0
        self._track_err_sum = 0.0
        self._track_err_n = 0
        self._track_err_group_sum: Dict[str, float] = {}
        self._track_err_group_n: Dict[str, int] = {}
        self._time_to_success: Optional[float] = None

    def _start_poller(self) -> None:
        self._poller_stop.clear()
        self._poller_thread = threading.Thread(
            target=self._pose_poller_loop, daemon=True
        )
        self._poller_thread.start()

    def _pose_poller_loop(self) -> None:
        """Poll Gazebo for robot+block poses at ~5 Hz in background."""
        while not self._poller_stop.is_set():
            try:
                robot = _gz_get_pose_fast(self._world_name, self._robot_name)
                block = _gz_get_pose_fast(self._world_name, self._block_name)
                with self._pose_lock:
                    if robot:
                        self._cached_robot_pose = robot
                    if block:
                        self._cached_block_pose = block
            except Exception:
                pass
            self._poller_stop.wait(timeout=0.2)  # 5 Hz

    def _write(self, row: Dict[str, Any]) -> None:
        """Write one JSON line. Must be called under self._lock."""
        if self._file:
            self._file.write(json.dumps(row, separators=(',', ':')) + '\n')
