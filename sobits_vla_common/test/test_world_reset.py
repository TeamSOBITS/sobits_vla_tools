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

"""Unit tests for WorldResetter -- no rclpy, no ROS graph required."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.world_reset import WorldResetter  # noqa: E402


def _scene():
    return {
        'world_name': 'simple_data_collection',
        'settle_s': 0.5,
        'presets': {
            'default': {
                'models': [
                    {
                        'name': 'sobit_home',
                        'pose': {
                            'x': 2.0, 'y': -1.5, 'z': 0.0,
                            'roll': 0.0, 'pitch': 0.0, 'yaw': math.pi / 2,
                        },
                    },
                    {
                        'name': 'box_to_pick',
                        'pose': {'x': 2.0, 'y': -0.5, 'z': 0.45},
                        'randomize': {'x': [-0.05, 0.05], 'y': [-0.05, 0.05]},
                    },
                ],
            },
            'empty': {'models': []},
        },
    }


class _RecordingSetPose:

    def __init__(self, fail_names=frozenset()):
        self.calls = []
        self._fail_names = fail_names

    def __call__(self, name, x, y, z, qx, qy, qz, qw):
        self.calls.append((name, x, y, z, qx, qy, qz, qw))
        return name not in self._fail_names


class TestPresetSelection:

    def test_default_preset_resets_all_models(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        result = resetter.reset()
        assert result.success
        assert set(result.models_reset) == {'sobit_home', 'box_to_pick'}
        assert len(set_pose.calls) == 2

    def test_empty_preset_resets_nothing(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        result = resetter.reset(preset='empty')
        assert result.success
        assert result.models_reset == []
        assert set_pose.calls == []

    def test_unknown_preset_fails(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        result = resetter.reset(preset='does_not_exist')
        assert not result.success
        assert set_pose.calls == []

    def test_no_presets_in_scene_fails(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter({}, set_pose)
        result = resetter.reset()
        assert not result.success


class TestRandomizeBounds:

    def test_randomize_offset_within_bounds(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        for _ in range(50):
            resetter.reset()
        block_calls = [c for c in set_pose.calls if c[0] == 'box_to_pick']
        for _, x, y, _z, *_rest in block_calls:
            assert 1.95 <= x <= 2.05
            assert -0.55 <= y <= -0.45

    def test_no_randomize_key_is_exact(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        resetter.reset()
        robot_call = next(c for c in set_pose.calls if c[0] == 'sobit_home')
        assert robot_call[1] == 2.0
        assert robot_call[2] == -1.5

    def test_swapped_bounds_still_samples_in_range(self):
        scene = _scene()
        scene['presets']['default']['models'][1]['randomize'] = {'x': [0.05, -0.05]}
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        for _ in range(20):
            resetter.reset()
        block_calls = [c for c in set_pose.calls if c[0] == 'box_to_pick']
        for _, x, *_rest in block_calls:
            assert 1.95 <= x <= 2.05


class TestMissingModelHandling:

    def test_missing_name_field_is_skipped(self):
        scene = _scene()
        scene['presets']['default']['models'].append({'pose': {'x': 0.0}})
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert result.success
        assert len(set_pose.calls) == 2

    def test_missing_pose_field_is_skipped(self):
        scene = _scene()
        scene['presets']['default']['models'].append({'name': 'ghost'})
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert result.success
        assert 'ghost' not in result.models_reset

    def test_set_pose_failure_is_reported(self):
        set_pose = _RecordingSetPose(fail_names={'box_to_pick'})
        resetter = WorldResetter(_scene(), set_pose)
        result = resetter.reset()
        assert not result.success
        assert 'box_to_pick' not in result.models_reset
        assert 'sobit_home' in result.models_reset

    def test_set_pose_fn_exception_counts_as_failure(self):
        def _raising(*_args):
            raise RuntimeError('boom')

        resetter = WorldResetter(_scene(), _raising)
        result = resetter.reset()
        assert not result.success
        assert result.models_reset == []


class TestOrientation:

    def test_yaw_is_converted_to_quaternion(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        resetter.reset()
        robot_call = next(c for c in set_pose.calls if c[0] == 'sobit_home')
        for actual, want in zip(robot_call[4:], (0.0, 0.0, 0.7071, 0.7071)):
            assert abs(actual - want) < 1e-4

    def test_missing_rpy_defaults_to_identity(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        resetter.reset()
        block_call = next(c for c in set_pose.calls if c[0] == 'box_to_pick')
        for actual, want in zip(block_call[4:], (0.0, 0.0, 0.0, 1.0)):
            assert abs(actual - want) < 1e-9

    def test_randomized_yaw_varies_and_stays_normalised(self):
        scene = _scene()
        scene['presets']['default']['models'][1]['randomize'] = {
            'yaw': [-math.pi, math.pi]
        }
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        for _ in range(20):
            resetter.reset()
        yaws = set()
        for call in (c for c in set_pose.calls if c[0] == 'box_to_pick'):
            qx, qy, qz, qw = call[4:]
            assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-9
            yaws.add(round(qz, 6))
        assert len(yaws) > 1

    def test_every_model_is_teleported(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(_scene(), set_pose)
        result = resetter.reset()
        assert set(result.models_reset) == {'sobit_home', 'box_to_pick'}


class TestCollisionAvoidance:

    def _two_models(self, radius, spread=0.05):
        return {'presets': {'default': {'models': [
            {'name': 'a', 'radius': radius,
             'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
            {'name': 'b', 'radius': radius,
             'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'randomize': {'x': [-spread, spread], 'y': [-spread, spread]}},
        ]}}}

    def test_radius_keeps_models_apart(self):
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(self._two_models(0.02), set_pose)
        for _ in range(50):
            set_pose.calls.clear()
            resetter.reset()
            pos = {c[0]: (c[1], c[2]) for c in set_pose.calls}
            gap = math.hypot(pos['a'][0] - pos['b'][0], pos['a'][1] - pos['b'][1])
            assert gap >= 0.04 - 1e-9

    def test_zero_radius_never_blocks(self):
        scene = self._two_models(0.0)
        set_pose = _RecordingSetPose()
        result = WorldResetter(scene, set_pose).reset()
        assert result.success
        assert len(set_pose.calls) == 2

    def test_unsatisfiable_accepts_overlap_rather_than_failing(self):
        # Radii demand 0.30 m of separation the randomize range cannot give.
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(
            self._two_models(0.15, spread=0.02), set_pose, max_placement_tries=5
        )
        result = resetter.reset()
        assert result.success
        assert set(result.models_reset) == {'a', 'b'}


class TestMalformedYaml:

    def test_preset_without_models_key_fails(self):
        scene = {'presets': {'default': {}}}
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert not result.success

    def test_model_entry_not_a_dict_is_skipped(self):
        scene = {'presets': {'default': {'models': ['not_a_dict']}}}
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert result.success
        assert result.models_reset == []

    def test_pose_not_a_dict_is_skipped(self):
        scene = {'presets': {'default': {'models': [
            {'name': 'sobit_home', 'pose': 'not_a_dict'},
        ]}}}
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert result.success
        assert result.models_reset == []

    def test_presets_not_a_dict_fails(self):
        scene = {'presets': ['default']}
        set_pose = _RecordingSetPose()
        resetter = WorldResetter(scene, set_pose)
        result = resetter.reset()
        assert not result.success
