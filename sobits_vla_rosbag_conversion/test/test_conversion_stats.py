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


"""Unit tests for conversion_stats -- feed synthetic EpisodeResults, assert exact dict."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.pipeline.episode_pipeline import EpisodeResult  # noqa: E402
from sobits_vla_rosbag_conversion.pipeline.stats import (  # noqa: E402
    build_stats_report, episode_stat_dict,
)


def _result(**overrides):
    base = {
        'bag': 'episode_1', 'task': 'Pick up the pear', 'frames': 30,
        'skipped_downsample': 5, 'skipped_static': 0, 'skipped_tf': 0, 'skipped_img_decode': 0,
    }
    base.update(overrides)
    return EpisodeResult(**base)


class TestEpisodeStatDict:

    def test_basic_fields(self):
        d = episode_stat_dict(_result())
        assert d == {
            'bag': 'episode_1', 'task': 'Pick up the pear', 'frames': 30,
            'skipped_downsample': 5, 'skipped_static': 0, 'skipped_tf': 0,
            'skipped_img_decode': 0,
        }

    def test_key_order_matches_original_dump_shape(self):
        d = episode_stat_dict(_result())
        assert list(d.keys()) == [
            'bag', 'task', 'frames', 'skipped_downsample',
            'skipped_static', 'skipped_tf', 'skipped_img_decode',
        ]

    def test_sync_stats_included_only_when_present(self):
        no_sync = episode_stat_dict(_result())
        assert 'sync_avg_ms' not in no_sync

        with_sync = episode_stat_dict(
            _result(sync_avg_ms=6.97, sync_max_ms=16.0, sync_min_ms=0.0)
        )
        assert with_sync['sync_avg_ms'] == 6.97
        assert with_sync['sync_max_ms'] == 16.0
        assert with_sync['sync_min_ms'] == 0.0
        assert list(with_sync.keys())[-3:] == ['sync_avg_ms', 'sync_max_ms', 'sync_min_ms']


class TestBuildStatsReport:

    def test_totals_and_key_order(self):
        results = [_result(bag='ep1', frames=152), _result(bag='ep2', frames=142)]
        params = {
            'rosbag_directory': '/bags', 'fps': 10, 'vcodec': 'h264',
            'sync_threshold': 0.1, 'downsample_tolerance': 0.015,
            'skip_static_threshold': 0.0, 'use_relative_actions': False,
            'ee_pose_enabled': False, 'cameras_skip': False,
        }
        report = build_stats_report('my_dataset', params, results, [], [])

        assert list(report.keys()) == [
            'dataset_name', 'rosbag_directory', 'fps', 'vcodec', 'sync_threshold',
            'downsample_tolerance', 'skip_static_threshold', 'use_relative_actions',
            'ee_pose_enabled', 'cameras_skip', 'total_episodes', 'total_frames',
            'skipped_bags', 'fps_warnings', 'episodes',
        ]
        assert report['dataset_name'] == 'my_dataset'
        assert report['total_episodes'] == 2
        assert report['total_frames'] == 294
        assert report['skipped_bags'] == []
        assert report['fps_warnings'] == []
        assert [e['bag'] for e in report['episodes']] == ['ep1', 'ep2']

    def test_empty_results_zero_totals(self):
        report = build_stats_report('empty_ds', {}, [], [], [])
        assert report['total_episodes'] == 0
        assert report['total_frames'] == 0
        assert report['episodes'] == []

    def test_skipped_bags_and_fps_warnings_passed_through(self):
        skipped = [{'bag': 'bad_ep', 'reason': 'missing_topics', 'missing': {}}]
        warnings = [{'bag': 'slow_ep', 'reason': 'fps_below_target', 'actual_fps': 7.0}]
        report = build_stats_report('ds', {}, [], skipped, warnings)
        assert report['skipped_bags'] == skipped
        assert report['fps_warnings'] == warnings

    def test_conversion_params_spliced_after_dataset_name(self):
        params = {'fps': 10, 'vcodec': 'h264'}
        report = build_stats_report('ds', params, [], [], [])
        keys = list(report.keys())
        assert keys[0] == 'dataset_name'
        assert keys[1:3] == ['fps', 'vcodec']


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
