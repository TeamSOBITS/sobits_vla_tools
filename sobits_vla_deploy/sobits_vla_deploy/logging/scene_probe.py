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
Gazebo pose polling for EpisodeLogger: background poller, cache, goal lookup.

Wraps sobits_vla_common.gz_utils so log_step()/evaluate_termination() never
block the control loop on a `gz` subprocess call.
"""

import threading
from typing import Dict, Optional

from sobits_vla_common.gz_utils import gz_get_pose


def gz_get_pose_fast(world_name: str, model_name: str) -> Optional[Dict[str, float]]:
    """Like gz_get_pose but with a shorter timeout for lower latency."""
    return gz_get_pose(world_name, model_name, timeout=1.5)


class ScenePoller:
    """
    Owns the cached robot/block poses and the ~5 Hz background poll thread.

    sim_enabled=False (real robot) skips the poller entirely -- poses stay
    None, matching the pre-split behaviour (lift/fall auto-term unavailable,
    timeout still works).
    """

    def __init__(self, world_name: str, robot_name: str, block_name: str,
                 sim_enabled: bool) -> None:
        self.world_name = world_name
        self.robot_name = robot_name
        self.block_name = block_name
        self.sim_enabled = sim_enabled

        self.cached_robot_pose: Optional[Dict[str, float]] = None
        self.cached_block_pose: Optional[Dict[str, float]] = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not self.sim_enabled:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def read_cached(self) -> tuple:
        """Return (robot_pose, block_pose) copies -- never blocks."""
        with self.lock:
            robot = dict(self.cached_robot_pose) if self.cached_robot_pose else None
            block = dict(self.cached_block_pose) if self.cached_block_pose else None
        return robot, block

    def seed(
        self, robot_pose: Optional[Dict[str, float]], block_pose: Optional[Dict[str, float]],
    ) -> None:
        """Overwrite the cache with a fresh blocking read (begin_episode)."""
        with self.lock:
            if block_pose is not None:
                self.cached_block_pose = block_pose
            if robot_pose is not None:
                self.cached_robot_pose = robot_pose

    def fresh_block_and_robot(self) -> tuple:
        """Blocking read of both poses now, bypassing the poller cache."""
        block = (
            gz_get_pose_fast(self.world_name, self.block_name)
            if (self.sim_enabled and self.block_name) else None
        )
        robot = (
            gz_get_pose_fast(self.world_name, self.robot_name)
            if (self.sim_enabled and self.robot_name) else None
        )
        return robot, block

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                robot = gz_get_pose_fast(self.world_name, self.robot_name)
                block = gz_get_pose_fast(self.world_name, self.block_name)
                with self.lock:
                    if robot:
                        self.cached_robot_pose = robot
                    if block:
                        self.cached_block_pose = block
            except Exception:
                pass
            self._stop.wait(timeout=0.2)  # 5 Hz
