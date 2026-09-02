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
Shared Gazebo transport helpers (CLI / gz-transport text I/O, no gz Python bindings).

Moved from sobits_vla_deploy.vla_episode_logger so sobits_vla_common.world_reset and
the C++ collection package's reset client share one implementation. The parsing and
fallback logic below were hard-won bug fixes -- kept verbatim, not "improved".
"""

from __future__ import annotations

import math
import re
import subprocess
import threading
from typing import Dict, List, Optional

from sobits_vla_common.geometry import quat_to_rpy


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def gz_set_pose(
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
    # gz.msgs.Pose identifies the entity via its own `name` field, no `entity` wrapper —
    # the old wrapped format failed to parse while `gz service` still exited 0.
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


def gz_get_pose_dynamic(
    world_name: str,
    model_name: str,
    timeout: float = 2.0,
) -> Optional[Dict[str, float]]:
    """
    Read a model's LIVE world pose from the dynamic_pose/info topic.

    `gz model -m <name> -p` reports the model's static/spawn pose, which never
    changes once the simulation is running -- a block picked up and lifted
    still reports its table position. dynamic_pose/info carries the per-entity
    poses the physics engine actually updates, so it is the only source that
    reflects motion.

    Returns {x, y, z, roll, pitch, yaw} (RPY converted from the quaternion),
    or None if the entity is absent from the message (static entities are not
    published here -- the caller falls back to the static query).
    """
    topic = '/world/{}/dynamic_pose/info'.format(world_name)
    cmd = ['gz', 'topic', '-e', '-t', topic, '-n', '1']
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout + 1.0
        )
        if result.returncode != 0:
            return None
        # Pose_V text format: repeated `pose { name: "x" position {...}
        # orientation {...} }` blocks. Walk to the block whose name matches.
        blocks, cur, depth, inside = [], [], 0, False
        for ln in result.stdout.splitlines():
            if not inside and ln.strip().startswith('pose {'):
                inside, cur, depth = True, [], 0
            if inside:
                cur.append(ln)
                depth += ln.count('{') - ln.count('}')
                if depth == 0:
                    blocks.append('\n'.join(cur))
                    inside = False
        for blk in blocks:
            name = re.search(r'name:\s*"([^"]*)"', blk)
            if not name or name.group(1) != model_name:
                continue
            pos = re.search(
                r'position\s*{([^}]*)}', blk, re.S)
            ori = re.search(
                r'orientation\s*{([^}]*)}', blk, re.S)
            if not pos:
                continue

            def _f(body, key):
                m = re.search(r'%s:\s*(-?[\d.eE+-]+)' % key, body)
                return float(m.group(1)) if m else 0.0

            px, py, pz = (_f(pos.group(1), k) for k in ('x', 'y', 'z'))
            if ori:
                qx, qy, qz = (_f(ori.group(1), k) for k in ('x', 'y', 'z'))
                qw = _f(ori.group(1), 'w')
            else:
                qx = qy = qz = 0.0
                qw = 1.0
            # Quaternion -> RPY (ZYX convention), matching gz's own output.
            roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
            return {'x': px, 'y': py, 'z': pz,
                    'roll': roll, 'pitch': pitch, 'yaw': yaw}
        return None
    except Exception:
        return None


def gz_get_pose(
    world_name: str,
    model_name: str,
    timeout: float = 2.0,
) -> Optional[Dict[str, float]]:
    """
    Query a model's world pose, preferring the live (dynamic) pose.

    Tries dynamic_pose/info first -- the only source that reflects motion --
    and falls back to `gz model -m <name> -p` for entities the physics engine
    never moves (which are absent from the dynamic topic).
    """
    live = gz_get_pose_dynamic(world_name, model_name, timeout=timeout)
    if live is not None:
        return live
    cmd = ['gz', 'model', '-m', model_name, '-p']
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout + 1.0
        )
        if result.returncode != 0:
            return None
        # Output format: "- Pose [ XYZ (m) ] [ RPY (rad) ]:" then an [x y z] line, then [r p y].
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


class GzPoseCache:
    """
    Background poller caching the live world pose of N Gazebo models.

    Generalizes the episode logger's original two-field (robot/block) cache to
    an arbitrary set of model names, since the scene YAML can list any number
    of them. get() never blocks -- callers on a control loop read the cache.
    """

    def __init__(self, world: str, model_names: List[str], hz: float = 5.0) -> None:
        self._world = world
        self._model_names = list(model_names)
        self._period_s = 1.0 / hz if hz > 0 else 0.2
        self._poses: Dict[str, Dict[str, float]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def get(self, name: str) -> Optional[Dict[str, float]]:
        """Return the last-cached pose for ``name``, or None if not yet seen."""
        with self._lock:
            pose = self._poses.get(name)
            return dict(pose) if pose is not None else None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            for name in self._model_names:
                try:
                    pose = gz_get_pose(self._world, name, timeout=1.5)
                except Exception:
                    pose = None
                if pose is not None:
                    with self._lock:
                        self._poses[name] = pose
            self._stop.wait(timeout=self._period_s)
