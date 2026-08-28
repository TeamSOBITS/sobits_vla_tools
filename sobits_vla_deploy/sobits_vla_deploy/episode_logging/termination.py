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
Automatic episode termination: pick/place scoring, settle windows, drop-abort.

No gz calls of its own -- reads poses from an injected ScenePoller and folds
them into the injected StepLog's accumulators (the same numbers log_step()
updates), since a terminal tick returns before log_step runs one more time.
"""

import math
from typing import Optional


class TerminationLogic:
    """Threshold state + evaluate(): the pre-split EpisodeLogger.evaluate_termination body."""

    def __init__(
        self,
        scene_probe,
        step_log,
        goal_lookup_fn,
        tilt_threshold_deg: float,
        episode_timeout_s: float,
        lift_success_m: float,
        fall_z_drop_m: float,
        success_settle_s: float,
        goal_name: str,
        place_radius_m: float,
        place_settle_s: float,
        place_z_max_m: float,
        drop_abort_s: float,
        drop_abort_z_max_m: float,
    ) -> None:
        self._scene = scene_probe
        self._log = step_log
        # Injected so tests can monkeypatch the facade's module-level
        # gz_get_pose and have this call see the patch.
        self._goal_lookup_fn = goal_lookup_fn
        self._tilt_rad = math.radians(tilt_threshold_deg)
        self._episode_timeout_s = episode_timeout_s
        self._lift_success_m = lift_success_m
        self._fall_z_drop_m = fall_z_drop_m
        self._success_settle_s = success_settle_s
        self._goal_name = goal_name
        self._place_radius_m = place_radius_m
        self._place_settle_s = place_settle_s
        self._place_z_max_m = place_z_max_m
        self._drop_abort_s = drop_abort_s
        self._drop_abort_z_max_m = drop_abort_z_max_m
        self.reset_episode_state()

    def reset_episode_state(self) -> None:
        """Reset per-episode goal/settle/drop state; called from begin_episode."""
        self._goal_xy: Optional[tuple] = None
        self._goal_reached_at: Optional[float] = None
        self._dropped_since: Optional[float] = None
        self._lifted = False

    def evaluate(
        self, elapsed_sim_s: float, baseline_block_z: float, baseline_robot_z: float,
    ) -> Optional[str]:
        """
        Check the automatic termination conditions.

        Returns the outcome reason or None if the episode should continue:
          "success_lift" — block lifted more than lift_success_m
          "success_place" — pick-and-place goal reached and settled
          "fallen"       — robot tilt > threshold or world-z drop > fall_z_drop_m
          "timeout"      — elapsed_sim_s >= episode_timeout_s
          "dropped"      — place mode: object left low, away from goal, too long
        """
        robot_pose, block_pose = self._scene.read_cached()

        if block_pose is not None:
            outcome = self._evaluate_block(elapsed_sim_s, block_pose, baseline_block_z)
            if outcome is not None:
                return outcome

        if robot_pose is not None and self._evaluate_fall(robot_pose, baseline_robot_z):
            return 'fallen'

        if elapsed_sim_s >= self._episode_timeout_s:
            return 'timeout'
        return None

    def _evaluate_block(self, elapsed_sim_s, block_pose, baseline_block_z) -> Optional[str]:
        lift = block_pose['z'] - baseline_block_z
        if lift > self._log.max_block_lift:
            self._log.max_block_lift = lift
        self._log.final_block_lift = lift
        # A crossing this early is reset settling, not a pick: real picks
        # took 30-45 s, the observed false positive fired at 0.84 s.
        settled = elapsed_sim_s >= self._success_settle_s
        # Use the running MAX lift: a shelf -> floor-bin placement ends far
        # below its start height, so instantaneous lift is negative by goal.
        if self._log.max_block_lift > self._lift_success_m and settled:
            self._lifted = True
            if not self._goal_name:
                if self._log.time_to_success is None:
                    self._log.time_to_success = elapsed_sim_s
                return 'success_lift'

        if self._goal_name and self._lifted and settled:
            return self._evaluate_place(elapsed_sim_s, block_pose)
        return None

    def _evaluate_place(self, elapsed_sim_s, block_pose) -> Optional[str]:
        if self._goal_xy is None:
            goal = self._goal_lookup_fn(self._goal_name)
            if goal is not None:
                self._goal_xy = (goal['x'], goal['y'])
        if self._goal_xy is None:
            return None

        dist = math.hypot(block_pose['x'] - self._goal_xy[0], block_pose['y'] - self._goal_xy[1])
        low_enough = self._place_z_max_m <= 0.0 or block_pose['z'] <= self._place_z_max_m
        if dist <= self._place_radius_m and low_enough:
            if self._goal_reached_at is None:
                self._goal_reached_at = elapsed_sim_s
            elif elapsed_sim_s - self._goal_reached_at >= self._place_settle_s:
                if self._log.time_to_success is None:
                    self._log.time_to_success = elapsed_sim_s
                return 'success_place'
        else:
            self._goal_reached_at = None

        # Early abort: object at rest low and away from the goal.
        if self._drop_abort_s > 0.0 and self._lifted:
            low = self._drop_abort_z_max_m <= 0.0 or block_pose['z'] <= self._drop_abort_z_max_m
            if low and dist > self._place_radius_m:
                if self._dropped_since is None:
                    self._dropped_since = elapsed_sim_s
                elif elapsed_sim_s - self._dropped_since >= self._drop_abort_s:
                    return 'dropped'
            else:
                self._dropped_since = None
        return None

    def _evaluate_fall(self, robot_pose, baseline_robot_z) -> bool:
        tilt = max(abs(robot_pose['roll']), abs(robot_pose['pitch']))
        z_drop = baseline_robot_z - robot_pose['z']
        if tilt > self._log.max_robot_tilt:
            self._log.max_robot_tilt = tilt
        if robot_pose['z'] < self._log.min_robot_z:
            self._log.min_robot_z = robot_pose['z']
        return tilt > self._tilt_rad or z_drop > self._fall_z_drop_m
