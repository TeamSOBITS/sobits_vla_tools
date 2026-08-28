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
import math
from typing import Dict, List, Optional

from sobits_vla_common.gz_utils import gz_get_pose
from sobits_vla_deploy.logging.scene_probe import ScenePoller
from sobits_vla_deploy.logging.step_log import (
    known_fields, lerobot_version_str, sobits_vla_tools_rev, StepLog,
)
from sobits_vla_deploy.logging.termination import TerminationLogic


class EpisodeLogger:
    """
    Logs one episode to a JSON-Lines file.

    Facade over three collaborators split by concern:
      StepLog        — JSONL file lifecycle, per-step record, summary stats.
      TerminationLogic — pick/place scoring, settle windows, drop-abort.
      ScenePoller     — gz pose polling thread, cache, goal lookup.

    Thread-safe: log_step() may be called from the control timer thread.
    begin_episode() and end_episode() are called from the joy/play callbacks.
    reset_world() is called from end_episode() in a background thread so it
    does not block the ROS spin.

    Parameters
    ----------
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
        self._world_name = world_name
        self._robot_name = robot_name
        self._block_name = block_name
        self._goal_name = goal_name
        self._spawn = (spawn_x, spawn_y, spawn_z, spawn_qx, spawn_qy, spawn_qz, spawn_qw)
        self._block_reset = (block_x, block_y, block_z)
        self._baseline_block_z: float = block_z
        self._baseline_robot_z: float = spawn_z
        self.enabled = enabled
        self._sim_enabled = sim_enabled

        self._log = StepLog(log_dir, joint_groups)
        self._scene = ScenePoller(world_name, robot_name, block_name, sim_enabled)
        self._term = TerminationLogic(
            self._scene, self._log, self._goal_lookup,
            tilt_threshold_deg=tilt_threshold_deg,
            episode_timeout_s=episode_timeout_s,
            lift_success_m=lift_success_m,
            fall_z_drop_m=fall_z_drop_m,
            success_settle_s=success_settle_s,
            goal_name=goal_name,
            place_radius_m=place_radius_m,
            place_settle_s=place_settle_s,
            place_z_max_m=place_z_max_m,
            drop_abort_s=drop_abort_s,
            drop_abort_z_max_m=drop_abort_z_max_m,
        )

        if self.enabled:
            self._log.ensure_log_dir()
            self._scene.start()

    def _goal_lookup(self, goal_name: str):
        # Own method (not ScenePoller's) so tests can monkeypatch this
        # module's gz_get_pose and have evaluate_termination see the patch.
        return gz_get_pose(self._world_name, goal_name)

    # ---- Seams preserved for tests that inject cached poses directly -------

    @property
    def _pose_lock(self):
        return self._scene.lock

    @property
    def _cached_block_pose(self):
        return self._scene.cached_block_pose

    @_cached_block_pose.setter
    def _cached_block_pose(self, value):
        self._scene.cached_block_pose = value

    @property
    def _cached_robot_pose(self):
        return self._scene.cached_robot_pose

    @_cached_robot_pose.setter
    def _cached_robot_pose(self, value):
        self._scene.cached_robot_pose = value

    # ---- Live-tunable thresholds (node's _on_set_parameters writes these) -

    @property
    def _tilt_rad(self):
        return self._term._tilt_rad

    @_tilt_rad.setter
    def _tilt_rad(self, value):
        self._term._tilt_rad = value

    @property
    def _episode_timeout_s(self):
        return self._term._episode_timeout_s

    @_episode_timeout_s.setter
    def _episode_timeout_s(self, value):
        self._term._episode_timeout_s = value

    @property
    def _lift_success_m(self):
        return self._term._lift_success_m

    @_lift_success_m.setter
    def _lift_success_m(self, value):
        self._term._lift_success_m = value

    @property
    def _success_settle_s(self):
        return self._term._success_settle_s

    @_success_settle_s.setter
    def _success_settle_s(self, value):
        self._term._success_settle_s = value

    @property
    def _fall_z_drop_m(self):
        return self._term._fall_z_drop_m

    @_fall_z_drop_m.setter
    def _fall_z_drop_m(self, value):
        self._term._fall_z_drop_m = value

    @property
    def _active(self):
        return self._log.active

    @property
    def _lifted(self):
        return self._term._lifted

    @property
    def _max_block_lift(self):
        return self._log.max_block_lift

    def begin_episode(self) -> None:
        if not self.enabled:
            return
        # Fresh blocking read, not the 200 ms-lagged poller cache: a stale
        # pre-teleport pose makes the reset itself read as a +0.4 m lift.
        robot_pose, block_pose = self._scene.fresh_block_and_robot()
        self._baseline_block_z = self._resolve_block_baseline(block_pose)
        self._baseline_robot_z = float(robot_pose['z']) if robot_pose is not None \
            else self._spawn[2]

        self._term.reset_episode_state()
        self._scene.seed(robot_pose, block_pose)
        self._log.prev_joints = None
        self._log.prev_joint_delta = None
        self._log.reset_accumulators()
        self._log.open_episode(self._build_meta())

    def _resolve_block_baseline(self, block_pose) -> float:
        # Deviating further than this from the reset height means the read
        # caught the block mid-teleport -- use the configured value instead.
        max_dev = self._lift_success_m
        dev = abs(float(block_pose['z']) - self._block_reset[2]) if block_pose else None
        if block_pose is not None and dev <= max_dev:
            return float(block_pose['z'])
        if block_pose is not None:
            print(
                '[WARN] begin_episode: block z={:.4f} deviates >{:.3f} m '
                'from the reset height {:.4f} (stale/incomplete teleport?) '
                '-- using the configured baseline.'.format(
                    float(block_pose['z']), max_dev, self._block_reset[2],
                )
            )
        return self._block_reset[2]

    def _build_meta(self) -> dict:
        sx, sy, sz, sqx, sqy, sqz, sqw = self._spawn
        bx, by, bz = self._block_reset
        return {
            'start_time_iso': datetime.now(timezone.utc).isoformat(),
            'lerobot_version': lerobot_version_str(),
            'sobits_vla_tools_rev': sobits_vla_tools_rev(),
            'model_repo_id': self._model_repo_id,
            'world_name': self._world_name,
            'robot_name': self._robot_name,
            'block_name': self._block_name,
            'spawn_pose': known_fields(x=sx, y=sy, z=sz, qx=sqx, qy=sqy, qz=sqz, qw=sqw),
            'block_reset_pose': known_fields(x=bx, y=by, z=bz),
            'joint_groups': self._log.joint_groups or None,
            'baseline_block_z': round(self._baseline_block_z, 4),
            'baseline_robot_z': round(self._baseline_robot_z, 4),
            'thresholds': {
                'episode_timeout_s': self._episode_timeout_s,
                'lift_success_m': self._lift_success_m,
                'fall_z_drop_m': self._fall_z_drop_m,
                'tilt_threshold_deg': round(math.degrees(self._tilt_rad), 3),
            },
        }

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
        step_info = self._log.next_step()
        if step_info is None:
            return
        t, step = step_info

        robot_pose, block_pose = self._scene.read_cached()
        row = self._log.build_step_row(
            t, step, joints, base_vel, ee_pose, joints_measured,
            robot_pose, block_pose, self._baseline_block_z, self._baseline_robot_z,
            self._tilt_rad, self._fall_z_drop_m,
        )
        self._log.write_step(row)

    def evaluate_termination(self, elapsed_sim_s: float) -> Optional[str]:
        """
        Check the automatic termination conditions.

        Evaluates the latest cached Gazebo poses and the elapsed simulation
        time. Returns the outcome reason or None if the episode should continue:
          "success_lift" — block lifted more than lift_success_m
          "fallen"       — robot tilt > threshold or world-z drop > fall_z_drop_m
          "timeout"      — elapsed_sim_s >= episode_timeout_s
        """
        if not self.enabled or not self._log.active:
            return None
        return self._term.evaluate(elapsed_sim_s, self._baseline_block_z, self._baseline_robot_z)

    def end_episode(self, outcome: str = 'manual_stop') -> None:
        """Write the per-episode summary and close the episode file."""
        if not self.enabled:
            return
        self._log.close_episode(outcome)

    def shutdown(self) -> None:
        """Stop background poller. Call from deploy node destroy_node()."""
        self._scene.shutdown()
