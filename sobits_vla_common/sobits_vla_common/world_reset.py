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
Pure scene-reset logic, no rclpy import -- unit-testable without a ROS graph.

WorldResetter walks a scene dict (built from ROS params by the caller) and
teleports each listed model via an injected set_pose_fn. Poses carry RPY;
the quaternion Gazebo wants is derived here. Each pose field may be given a
randomize [lo, hi] range, sampled per reset.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Any, Callable, Dict, List, Optional

from sobits_vla_common.geometry import rpy_to_quat


SetPoseFn = Callable[[str, float, float, float, float, float, float, float], bool]

POSE_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


@dataclass
class ResetResult:
    success: bool
    message: str
    models_reset: List[str] = field(default_factory=list)


class WorldResetter:
    """
    Applies a scene preset's model poses via an injected teleport function.

    ``set_pose_fn`` is injected so the node can pass the ros_gz service client
    (with its CLI fallback) and tests can pass a recording stub instead.
    """

    def __init__(
        self,
        scene: Dict[str, Any],
        set_pose_fn: SetPoseFn,
        logger: Optional[Any] = None,
        max_placement_tries: int = 100,
    ) -> None:
        self._scene = scene
        self._set_pose_fn = set_pose_fn
        self._logger = logger
        self._max_tries = max(1, int(max_placement_tries))

    def reset(self, preset: str = '') -> ResetResult:
        """
        Apply the given preset's model poses (or the scene's active preset).

        An empty ``preset`` falls back to the scene's ``active_preset``, then
        to 'default'. Callers that do not set the field therefore follow the
        config rather than being pinned to 'default'.

        Every listed model is teleported with both position and orientation.
        Returns a ResetResult; never raises on a bad preset, a missing model
        field, or a per-model teleport failure.
        """
        presets = self._scene.get('presets')
        if not isinstance(presets, dict) or not presets:
            return ResetResult(False, 'Scene has no presets defined.')

        preset_name = preset or self._scene.get('active_preset') or 'default'
        preset_data = presets.get(preset_name)
        if preset_data is None:
            return ResetResult(
                False, 'Unknown preset {!r}.'.format(preset_name)
            )

        models = preset_data.get('models') if isinstance(preset_data, dict) else None
        if not isinstance(models, list):
            return ResetResult(
                False, 'Preset {!r} has no model list.'.format(preset_name)
            )

        # Sample every pose first so overlaps can be rejected before anything
        # is teleported -- a mid-list clash would otherwise need a rollback.
        placed: List[tuple] = []
        plan: List[tuple] = []
        for entry in models:
            if not isinstance(entry, dict) or 'name' not in entry or 'pose' not in entry:
                self._warn('Skipping malformed model entry: {!r}'.format(entry))
                continue
            name = str(entry['name'])
            pose = entry['pose']
            if not isinstance(pose, dict):
                self._warn('Skipping {!r}: pose is not a mapping.'.format(name))
                continue

            sampled = self._sample_clear_pose(name, entry, placed)
            placed.append((sampled['x'], sampled['y'], self._radius_of(entry)))
            plan.append((name, sampled))

        reset_names: List[str] = []
        failed_names: List[str] = []
        for name, sampled in plan:
            x, y, z = sampled['x'], sampled['y'], sampled['z']
            qx, qy, qz, qw = rpy_to_quat(
                sampled['roll'], sampled['pitch'], sampled['yaw']
            )

            try:
                ok = self._set_pose_fn(name, x, y, z, qx, qy, qz, qw)
            except Exception as exc:
                self._warn('set_pose_fn raised for {!r}: {}'.format(name, exc))
                ok = False

            if ok:
                reset_names.append(name)
            else:
                failed_names.append(name)

        if failed_names:
            return ResetResult(
                False,
                'Failed to reset: {}'.format(', '.join(failed_names)),
                models_reset=reset_names,
            )
        return ResetResult(
            True,
            'Reset {} model(s) in preset {!r}.'.format(len(reset_names), preset_name),
            models_reset=reset_names,
        )

    @staticmethod
    def _radius_of(entry: Dict[str, Any]) -> float:
        """Clearance radius (m) for overlap checks. 0 = never blocks anything."""
        try:
            return max(0.0, float(entry.get('radius', 0.0)))
        except (TypeError, ValueError):
            return 0.0

    def _sample_clear_pose(
        self, name: str, entry: Dict[str, Any], placed: List[tuple]
    ) -> Dict[str, float]:
        """
        Sample a pose whose radius does not overlap an already-placed model.

        Redraws up to max_placement_tries, then gives up and returns the last
        sample: a slightly overlapping scene beats refusing to reset at all.
        Models with radius 0, or with nothing to randomize, pass through.
        """
        pose = entry['pose']
        randomize = entry.get('randomize')
        radius = self._radius_of(entry)
        if not isinstance(randomize, dict) or radius <= 0.0:
            return self._sample_pose(pose, randomize)

        sampled = self._sample_pose(pose, randomize)
        for attempt in range(1, self._max_tries + 1):
            if self._is_clear(sampled, radius, placed):
                if attempt > 1:
                    self._warn(
                        '{!r} placed after {} redraw(s).'.format(name, attempt - 1)
                    )
                return sampled
            sampled = self._sample_pose(pose, randomize)
        self._warn(
            '{!r}: no clear placement in {} tries -- accepting an overlap. '
            'Widen its randomize range or shrink radius.'.format(
                name, self._max_tries
            )
        )
        return sampled

    @staticmethod
    def _is_clear(
        sampled: Dict[str, float], radius: float, placed: List[tuple]
    ) -> bool:
        """Return True when this sample clears every placed model's radius."""
        for px, py, pradius in placed:
            if pradius <= 0.0:
                continue
            gap = math.hypot(sampled['x'] - px, sampled['y'] - py)
            if gap < radius + pradius:
                return False
        return True

    @staticmethod
    def _sample_pose(
        pose: Dict[str, Any], randomize: Optional[Dict[str, Any]]
    ) -> Dict[str, float]:
        """Nominal pose plus a per-field uniform offset. Orientation is RPY."""
        sampled = {f: float(pose.get(f, 0.0)) for f in POSE_FIELDS}
        if not isinstance(randomize, dict):
            return sampled
        for key in POSE_FIELDS:
            if key in randomize:
                sampled[key] += WorldResetter._sample_offset(randomize[key])
        return sampled

    @staticmethod
    def _sample_offset(bounds: Any) -> float:
        lo, hi = float(bounds[0]), float(bounds[1])
        if lo > hi:
            lo, hi = hi, lo
        return random.uniform(lo, hi)

    def _warn(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.warning(msg)
