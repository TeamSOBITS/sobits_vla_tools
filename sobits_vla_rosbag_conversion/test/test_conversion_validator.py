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


"""Unit tests for conversion_validator -- pure dict-in/bool-out checks."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.pipeline.validator import (  # noqa: E402
    morphologies_match, resolve_cameras, sensors_match,
)


def _morph(**overrides):
    base = {
        'type': 'mobile_manipulator',
        'has_mobile_base': False,
        'has_cmd_vel_y': False,
        'joint_states_topic': '/joint_states',
        'parts': ['head'],
        'head': {'is_actionable': True, 'joint_names': ['head_yaw_joint']},
    }
    base.update(overrides)
    return base


class TestMorphologiesMatch:

    def test_identical_dicts_match(self):
        assert morphologies_match(_morph(), _morph())

    def test_different_type_does_not_match(self):
        assert not morphologies_match(_morph(), _morph(type='manipulator'))

    def test_different_parts_order_does_not_match(self):
        a = _morph(parts=['head', 'arm'])
        b = _morph(parts=['arm', 'head'])
        assert not morphologies_match(a, b)

    def test_different_is_actionable_does_not_match(self):
        a = _morph()
        b = _morph(head={'is_actionable': False, 'joint_names': ['head_yaw_joint']})
        assert not morphologies_match(a, b)

    def test_different_joint_names_does_not_match(self):
        a = _morph()
        b = _morph(head={'is_actionable': True, 'joint_names': ['other_joint']})
        assert not morphologies_match(a, b)

    def test_missing_part_block_defaults_empty(self):
        # A part listed but with no block: both sides default identically.
        a = {**_morph(), 'parts': ['head', 'ghost']}
        b = {**_morph(), 'parts': ['head', 'ghost']}
        assert morphologies_match(a, b)


class TestSensorsMatch:

    def _sensors(self, **overrides):
        base = {
            'types': ['camera'],
            'camera': {
                'names': ['head_camera'],
                'topics': ['/head/image_raw'],
                'compressed_topics': [''],
                'info_topics': ['/head/camera_info'],
                'properties': {'head_camera': {'width': 64, 'height': 48}},
            },
        }
        base.update(overrides)
        return base

    def test_identical_dicts_match(self):
        assert sensors_match(self._sensors(), self._sensors())

    def test_types_compared_order_independent(self):
        a = self._sensors(types=['camera', 'lidar'])
        b = self._sensors(types=['lidar', 'camera'])
        # Only 'camera' block populated on both -- types match, no lidar-key mismatch.
        assert sensors_match(a, b)

    def test_different_topics_does_not_match(self):
        a = self._sensors()
        b = self._sensors(camera={**self._sensors()['camera'], 'topics': ['/other/image_raw']})
        assert not sensors_match(a, b)

    def test_different_properties_does_not_match(self):
        a = self._sensors()
        cam = {**self._sensors()['camera'],
               'properties': {'head_camera': {'width': 32, 'height': 24}}}
        b = self._sensors(camera=cam)
        assert not sensors_match(a, b)


class TestResolveCameras:

    def _maps(self):
        return (
            {'head_camera': '/head/image_raw'},        # all_cam_raw
            {'head_camera': '/head/image_raw/compressed'},  # all_cam_compressed
            {'head_camera': '/head/camera_info'},       # all_cam_info
            {'head_camera': {'width': 64, 'height': 48}},  # all_cam_props
        )

    def test_resolves_raw_topic_with_known_shape(self):
        raw, compressed, info, props = self._maps()
        result = resolve_cameras(
            ['head_camera'], [False], {'head_camera'}, raw, compressed, info, props,
            [], decode_fn=None, label='Camera',
        )
        assert result is not None
        topics, info_topics, shapes = result
        assert topics == {'head_camera': '/head/image_raw'}
        assert shapes == {'head_camera': (64, 48)}

    def test_unknown_camera_name_returns_none(self):
        raw, compressed, info, props = self._maps()
        result = resolve_cameras(
            ['ghost_camera'], [False], {'head_camera'}, raw, compressed, info, props,
            [], decode_fn=None, label='Camera',
        )
        assert result is None

    def test_compressed_requested_but_missing_falls_back_to_raw(self):
        raw, _, info, props = self._maps()
        result = resolve_cameras(
            ['head_camera'], [True], {'head_camera'}, raw, {}, info, props,
            [], decode_fn=None, label='Camera',
        )
        assert result is not None
        topics, _, _ = result
        assert topics == {'head_camera': '/head/image_raw'}

    def test_no_topic_available_returns_none(self):
        result = resolve_cameras(
            ['head_camera'], [False], {'head_camera'}, {}, {},
            {'head_camera': '/head/camera_info'}, {'head_camera': {}},
            [], decode_fn=None, label='Camera',
        )
        assert result is None

    def test_unresolved_shape_with_no_candidates_returns_none(self):
        raw = {'head_camera': '/head/image_raw'}
        result = resolve_cameras(
            ['head_camera'], [False], {'head_camera'}, raw, {},
            {'head_camera': ''}, {'head_camera': {}},
            [], decode_fn=None, label='Camera',
        )
        assert result is None

    def test_logger_receives_error_on_unknown_camera(self):
        events = []

        class _Logger:
            def error(self, msg):
                events.append(('error', msg))

            def warning(self, msg):
                events.append(('warning', msg))

        raw, compressed, info, props = self._maps()
        resolve_cameras(
            ['ghost'], [False], {'head_camera'}, raw, compressed, info, props,
            [], decode_fn=None, label='Camera', logger=_Logger(),
        )
        assert any(level == 'error' for level, _ in events)


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))


class _Conn:
    def __init__(self, topic):
        self.topic = topic
        self.msgtype = 'x'


class _FakeReader:
    """AnyReader stand-in: rosbags scans every message when connections is empty."""

    def __init__(self, topics, calls):
        self.connections = [_Conn(t) for t in topics]
        self._calls = calls

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def messages(self, connections=()):
        self._calls.append(list(connections))
        if not connections:
            raise AssertionError('empty connection filter would scan the whole bag')
        return iter(())


class TestSniffSkipsBagsWithoutTopic:

    def _patch_reader(self, monkeypatch, topics, calls):
        from sobits_vla_rosbag_conversion.pipeline import validator
        monkeypatch.setattr(
            validator, 'AnyReader', lambda paths: _FakeReader(topics, calls))
        return validator

    def test_camera_info_pass_skips_bags_lacking_the_topic(self, monkeypatch):
        calls = []
        v = self._patch_reader(monkeypatch, ['/cam/image_raw'], calls)
        shapes, unresolved = {}, {'cam'}
        v._fill_shapes_from_camera_info(
            shapes, unresolved, {'cam': '/cam/camera_info'}, ['/b1', '/b2'])
        assert calls == []  # never asked to iterate messages
        assert unresolved == {'cam'} and shapes == {}

    def test_image_pass_skips_bags_lacking_the_topic(self, monkeypatch):
        calls = []
        v = self._patch_reader(monkeypatch, ['/other'], calls)
        shapes, unresolved = {}, {'cam'}
        v._fill_shapes_from_images(
            shapes, unresolved, {'cam': '/cam/image_raw'}, ['/b1'], lambda m: None)
        assert calls == []
