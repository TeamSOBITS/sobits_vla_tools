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
  t          — float, seconds since episode start
  step       — int, step index within episode
  joints     — dict[joint_name -> rad]
  base_vel   — {x, y, theta}
  ee_pose    — {x, y, z, roll, pitch, yaw} in base_footprint (None if TF miss)
  ee_error   — {dx, dy, dz, dist} relative to block position (None if gz query fails)
  robot_pose — {x, y, z, roll, pitch, yaw} in world frame from Gazebo
  block_pose — {x, y, z} world frame
  fallen     — bool, True if |robot roll| or |robot pitch| > tilt_threshold_deg

Episode metadata header (first line, type=="meta"):
  episode, start_time_iso, robot_name, world_name, spawn_pose, block_reset_pose

After stop, writes a final line type=="summary" with duration_s and total_steps.

World reset (called between episodes):
  - Teleports robot to spawn_pose via gz service
  - Teleports block to block_reset_pose via gz service
  Both use the UserCommands plugin which is already in simple_data_collection.world.xacro.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import threading
from time import monotonic
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _rpy_from_quat(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion → (roll, pitch, yaw) in radians."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _gz_set_pose(
    world_name: str,
    model_name: str,
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
    timeout: float = 3.0,
) -> bool:
    """
    Teleport a Gazebo model via the /world/.../set_pose service.

    Uses gz-transport CLI so no Python gz bindings are required.
    The UserCommands plugin must be loaded in the world.
    """
    # gz.msgs.Pose identifies the entity via its own `name` field — there is
    # no `entity` wrapper. The old wrapped format failed to parse, and
    # `gz service` STILL exited 0, so this function reported success while
    # the model never moved.
    req = (
        'name: "{name}" '
        'position: {{x: {x} y: {y} z: {z}}} '
        'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}'
    ).format(name=model_name, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)

    cmd = [
        'gz', 'service',
        '-s', '/world/{}/set_pose'.format(world_name),
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(int(timeout * 1000)),
        '--req', req,
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout + 1.0
        )
        # Exit code is unreliable (0 even on request-parse failure) — trust
        # only the service's Boolean reply on stdout.
        return result.returncode == 0 and 'data: true' in result.stdout
    except Exception:
        return False


def _gz_get_pose(
    world_name: str,
    model_name: str,
    timeout: float = 2.0,
) -> Optional[Dict[str, float]]:
    """
    Query a model's world pose via the `gz model` CLI.

    Returns dict {x, y, z, roll, pitch, yaw} or None on failure.

    NOTE: the previous implementation called `gz service -s
    /world/.../pose/info`, but pose/info is a TOPIC, not a service, and the
    parser took the first x/y/z in a Pose_V of ALL models without filtering
    by name — it could never return this model's pose. `gz model -m <name>
    -p` is name-filtered and prints XYZ + RPY directly. (world_name is kept
    for signature compatibility; gz model uses the running world.)
    """
    del world_name  # gz model resolves the active world itself
    cmd = ['gz', 'model', '-m', model_name, '-p']
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout + 1.0
        )
        if result.returncode != 0:
            return None
        # Output format:
        #   - Pose [ XYZ (m) ] [ RPY (rad) ]:
        #     [x y z]
        #     [r p y]
        lines = [ln.strip() for ln in result.stdout.splitlines()]
        for i, ln in enumerate(lines):
            if 'Pose [ XYZ' in ln and i + 2 < len(lines):
                xyz = lines[i + 1].strip('[]').split()
                rpy = lines[i + 2].strip('[]').split()
                if len(xyz) == 3 and len(rpy) == 3:
                    x_, y_, z_ = (float(v) for v in xyz)
                    roll, pitch, yaw = (float(v) for v in rpy)
                    return {'x': x_, 'y': y_, 'z': z_,
                            'roll': roll, 'pitch': pitch, 'yaw': yaw}
        return None
    except Exception:
        return None


def _gz_get_pose_fast(world_name: str, model_name: str) -> Optional[Dict[str, float]]:
    """
    Like _gz_get_pose but uses /world/…/state_async for lower latency.

    Falls back to pose/info on any error.
    """
    return _gz_get_pose(world_name, model_name, timeout=1.5)


# ---------------------------------------------------------------------------
# EpisodeLogger
# ---------------------------------------------------------------------------

class EpisodeLogger:
    """
    Logs one episode to a JSON-Lines file.

    Thread-safe: log_step() may be called from the control timer thread.
    begin_episode() and end_episode() are called from the joy/play callbacks.
    reset_world() is called from end_episode() in a background thread so it
    does not block the ROS spin.

    Parameters (all optional, sensible defaults for simple_data_collection):
      log_dir          — output directory (default /tmp/vla_logs)
      world_name       — Gazebo world name (default simple_data_collection)
      robot_name       — Gazebo model name (default sobit_home)
      block_name       — Gazebo model name of the blue block (default box_to_pick)
      spawn_x/y/z      — robot reset position (default 2.0/-1.5/0.0)
      spawn_qx/y/z/w   — robot reset orientation (default yaw=π/2)
      block_x/y/z      — block reset position (default 2.0/-0.5/0.45)
      tilt_threshold_deg — |roll| or |pitch| above this → fallen=True (default 30°)
      enabled          — master switch; if False all methods are no-ops
    """

    def __init__(
        self,
        log_dir: str = '/tmp/vla_logs',
        world_name: str = 'simple_data_collection',
        robot_name: str = 'sobit_home',
        block_name: str = 'box_to_pick',
        spawn_x: float = 2.0,
        spawn_y: float = -1.5,
        spawn_z: float = 0.0,
        spawn_qx: float = 0.0,
        spawn_qy: float = 0.0,
        spawn_qz: float = 0.7071,
        spawn_qw: float = 0.7071,
        block_x: float = 2.0,
        block_y: float = -0.5,
        block_z: float = 0.45,
        tilt_threshold_deg: float = 30.0,
        enabled: bool = True,
    ) -> None:
        self._log_dir = Path(log_dir)
        self._world_name = world_name
        self._robot_name = robot_name
        self._block_name = block_name
        self._spawn = (spawn_x, spawn_y, spawn_z, spawn_qx, spawn_qy, spawn_qz, spawn_qw)
        self._block_reset = (block_x, block_y, block_z)
        self._tilt_rad = math.radians(tilt_threshold_deg)
        self.enabled = enabled

        self._lock = threading.Lock()
        self._file = None
        self._episode_idx = 0
        self._step_idx = 0
        self._t0: Optional[float] = None
        self._active = False

        # Cached gz poses — updated by background poller at ~5 Hz so log_step()
        # never blocks the 10 Hz control loop with subprocess calls.
        self._cached_robot_pose: Optional[Dict[str, float]] = None
        self._cached_block_pose: Optional[Dict[str, float]] = None
        self._pose_lock = threading.Lock()
        self._poller_stop = threading.Event()
        self._poller_thread: Optional[threading.Thread] = None

        if self.enabled:
            self._log_dir.mkdir(parents=True, exist_ok=True)
            self._start_poller()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def begin_episode(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._episode_idx += 1
            self._step_idx = 0
            self._t0 = monotonic()
            self._active = True
            ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
            fname = self._log_dir / 'episode_{:04d}_{}.jsonl'.format(
                self._episode_idx, ts
            )
            self._file = open(fname, 'w', buffering=1)  # line-buffered
            sx, sy, sz, sqx, sqy, sqz, sqw = self._spawn
            bx, by, bz = self._block_reset
            meta = {
                'type': 'meta',
                'episode': self._episode_idx,
                'start_time_iso': datetime.now(timezone.utc).isoformat(),
                'world_name': self._world_name,
                'robot_name': self._robot_name,
                'block_name': self._block_name,
                'spawn_pose': {
                    'x': sx, 'y': sy, 'z': sz,
                    'qx': sqx, 'qy': sqy, 'qz': sqz, 'qw': sqw,
                },
                'block_reset_pose': {'x': bx, 'y': by, 'z': bz},
            }
            self._write(meta)

    def log_step(
        self,
        joints: Dict[str, float],
        base_vel: Dict[str, float],
        ee_pose: Optional[List[float]],
    ) -> None:
        """
        Call once per control tick while episode is active.

        ee_pose: [x, y, z, roll, pitch, yaw] in base_footprint, or None.
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

        # EE error relative to block (world-frame approximation via robot pose + EE pose)
        ee_error = None
        if ee_pose is not None and robot_pose is not None and block_pose is not None:
            # Transform EE from base_footprint to world:
            # world_pos = robot_xy + R(yaw) * ee_xy + z offset
            yaw = robot_pose['yaw']
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            ee_x_w = robot_pose['x'] + cos_y * ee_pose[0] - sin_y * ee_pose[1]
            ee_y_w = robot_pose['y'] + sin_y * ee_pose[0] + cos_y * ee_pose[1]
            ee_z_w = robot_pose['z'] + ee_pose[2]
            dx = block_pose['x'] - ee_x_w
            dy = block_pose['y'] - ee_y_w
            dz = block_pose['z'] - ee_z_w
            ee_error = {
                'dx': round(dx, 4),
                'dy': round(dy, 4),
                'dz': round(dz, 4),
                'dist': round(math.sqrt(dx * dx + dy * dy + dz * dz), 4),
            }

        # Collision / fallen detection from robot world pose
        fallen = False
        if robot_pose is not None:
            fallen = (
                abs(robot_pose['roll']) > self._tilt_rad
                or abs(robot_pose['pitch']) > self._tilt_rad
            )

        row: Dict[str, Any] = {
            'type': 'step',
            't': round(t, 4),
            'step': step,
            'joints': {k: round(v, 5) for k, v in joints.items()},
            'base_vel': {k: round(v, 5) for k, v in base_vel.items()},
            'ee_pose': (
                {k: round(v, 4) for k, v in zip(
                    ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], ee_pose
                )} if ee_pose is not None else None
            ),
            'ee_error': ee_error,
            'robot_pose': (
                {k: round(robot_pose[k], 4) for k in robot_pose}
                if robot_pose is not None else None
            ),
            'block_pose': (
                {'x': round(block_pose['x'], 4),
                 'y': round(block_pose['y'], 4),
                 'z': round(block_pose['z'], 4)}
                if block_pose is not None else None
            ),
            'fallen': fallen,
        }

        with self._lock:
            self._write(row)

    def end_episode(self) -> None:
        """Close the current episode file."""
        if not self.enabled:
            return
        with self._lock:
            if not self._active:
                return
            self._active = False
            duration = monotonic() - self._t0 if self._t0 else 0.0
            total_steps = self._step_idx
            summary = {
                'type': 'summary',
                'duration_s': round(duration, 3),
                'total_steps': total_steps,
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

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

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
