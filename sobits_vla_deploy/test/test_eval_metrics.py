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
Unit tests for sobits_vla_deploy.eval.metrics against a synthetic episode log.

Numbers here feed paper tables -- assert actual values, not just shapes.
"""

import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.eval.metrics import (  # noqa: E402
    aggregate_table, per_episode_table, task_stages,
)


def _steps_df(rows):
    return pd.DataFrame(rows) if rows else pd.DataFrame()


def _episode(path, meta, summary, step_rows):
    return {'path': path, 'meta': meta, 'summary': summary, 'steps': _steps_df(step_rows)}


def _model(label, episodes, pick_only=False, is_sim=False):
    return {'label': label, 'dir': '/fake', 'episodes': episodes,
            'pick_only': pick_only, 'is_sim': is_sim}


class TestPerEpisodeTable:

    def _known_episode(self):
        # 3 steps, deterministic values chosen so every derived column has a
        # hand-checkable answer.
        steps = [
            {'t': 0.0, 'max_jerk': 0.10, 'track_arm_abs_mean': 0.01,
             'track_arm_abs_max': 0.02, 'ee_x': 0.0, 'ee_y': 0.0, 'ee_z': 0.0},
            {'t': 1.0, 'max_jerk': 0.30, 'track_arm_abs_mean': 0.03,
             'track_arm_abs_max': 0.05, 'ee_x': 3.0, 'ee_y': 4.0, 'ee_z': 0.0},
            {'t': 2.0, 'max_jerk': 0.20, 'track_arm_abs_mean': 0.02,
             'track_arm_abs_max': 0.04, 'ee_x': 3.0, 'ee_y': 4.0, 'ee_z': 3.0},
        ]
        meta = {'episode': 7, 'source_run': 'run1', 'model_repo_id': 'org/model'}
        summary = {
            'outcome': 'success_lift', 'duration_s': 2.0, 'total_steps': 20,
            'joint_path_length': 1.5, 'max_joint_jerk': 0.30,
            'mean_abs_tracking_error': 0.02, 'min_ee_block_dist': 0.05,
            'max_block_lift': 0.08, 'time_to_success_s': 1.8,
        }
        return _episode('/fake/episode_0007.jsonl', meta, summary, steps)

    def test_known_episode_columns_and_values(self):
        model = _model('smolvla', [self._known_episode()], is_sim=True)
        df = per_episode_table(model)
        assert len(df) == 1
        row = df.iloc[0]

        assert row['model'] == 'smolvla'
        assert row['episode'] == 7
        assert row['file'] == 'episode_0007.jsonl'
        assert bool(row['is_sim']) is True
        assert bool(row['pick_only']) is False
        assert row['outcome'] == 'success_lift'
        assert row['duration_s'] == 2.0
        assert row['total_steps'] == 20
        # control_hz = total_steps / duration_s = 20 / 2.0
        assert row['control_hz'] == 10.0
        assert row['joint_path_length'] == 1.5
        assert row['max_joint_jerk'] == 0.30
        assert row['max_block_lift'] == 0.08
        assert row['time_to_success_s'] == 1.8

        # arm_track_mean = mean(0.01, 0.03, 0.02) = 0.02
        assert row['arm_track_mean'] == pytest.approx(0.02)
        # arm_track_max = max(0.02, 0.05, 0.04) = 0.05
        assert row['arm_track_max'] == pytest.approx(0.05)
        # jerk_p95 = 95th percentile of [0.10, 0.30, 0.20] (linear interp)
        expected_p95 = float(np.quantile([0.10, 0.30, 0.20], 0.95))
        assert row['jerk_p95'] == pytest.approx(expected_p95)

        # EE path: (0,0,0)->(3,4,0) dist 5.0, then (3,4,0)->(3,4,3) dist 3.0
        assert row['ee_path_len_m'] == pytest.approx(8.0)

    def test_missing_summary_fields_become_none(self):
        ep = _episode('/fake/episode_0001.jsonl', {}, {}, [])
        model = _model('m', [ep])
        df = per_episode_table(model)
        row = df.iloc[0]
        assert row['outcome'] == 'incomplete'
        assert row['duration_s'] is None
        assert row['control_hz'] is None
        assert 'ee_path_len_m' in df.columns
        assert row['ee_path_len_m'] is None

    def test_control_hz_none_when_duration_zero(self):
        ep = _episode('/fake/ep.jsonl', {}, {'duration_s': 0.0, 'total_steps': 5}, [])
        df = per_episode_table(_model('m', [ep]))
        assert df.iloc[0]['control_hz'] is None


class TestAggregateTable:

    def _episode_with_score(self, score, outcome='timeout', duration=10.0):
        summary = {
            'outcome': outcome, 'duration_s': duration, 'total_steps': 100,
            'joint_path_length': 2.0, 'max_joint_jerk': 0.1,
        }
        return summary

    def test_known_scores_produce_known_success_rate(self):
        # 4 episodes scored 2, 3, 5, 5 -> success_rate (>=5) = 2/4 = 50%.
        rows = []
        for i, score in enumerate([2, 3, 5, 5]):
            rows.append({
                'model': 'smolvla', 'episode': i, 'file': f'ep{i}.jsonl',
                'pick_only': False, 'is_sim': False, 'outcome': 'timeout',
                'duration_s': 10.0, 'total_steps': 100,
                'joint_path_length': 2.0, 'max_joint_jerk': 0.1,
                'mean_abs_tracking_error': 0.02, 'min_ee_block_dist': None,
                'max_block_lift': None, 'time_to_success_s': None,
                'ee_path_len_m': 1.0, 'score': score,
            })
        per_ep = pd.DataFrame(rows)
        agg = aggregate_table(per_ep)

        assert list(agg.index) == ['smolvla']
        row = agg.loc['smolvla']
        assert row['episodes'] == 4
        assert row['scored'] == 4
        assert row['task'] == 'pick+place'
        assert row['success_rate_%'] == 50.0
        # stage 1/2 rates: all 4 scored >=1 and >=2 -> 100%.
        assert row['reach_%'] == 100.0
        assert row['grasp_%'] == 100.0
        # stage 3 (lift): scores >=3 are [3,5,5] -> 3/4 = 75%.
        assert row['lift_%'] == 75.0
        # stage 5 (place) == success_rate.
        assert row['place_%'] == 50.0
        assert row['stages'] == '5/5'
        assert row['mean_stage'] == '{:.2f} ± {:.2f}'.format(
            np.mean([2, 3, 5, 5]), np.std([2, 3, 5, 5], ddof=0))

    def test_no_scores_yields_dashes(self):
        rows = [{
            'model': 'm', 'episode': 0, 'file': 'ep0.jsonl', 'pick_only': False,
            'is_sim': False, 'outcome': 'timeout', 'duration_s': 5.0,
            'total_steps': 50, 'joint_path_length': 1.0, 'max_joint_jerk': 0.05,
            'mean_abs_tracking_error': 0.01, 'min_ee_block_dist': None,
            'max_block_lift': None, 'time_to_success_s': None,
            'ee_path_len_m': 0.5, 'score': np.nan,
        }]
        agg = aggregate_table(pd.DataFrame(rows))
        row = agg.loc['m']
        assert row['scored'] == 0
        assert row['mean_stage'] == '--'
        assert row['success_rate_%'] == '--'
        assert row['reach_%'] == '--'

    def test_timeout_rate_percent(self):
        rows = []
        for i, outcome in enumerate(['timeout', 'timeout', 'manual_stop', 'fallen']):
            rows.append({
                'model': 'm', 'episode': i, 'file': f'ep{i}.jsonl', 'pick_only': False,
                'is_sim': False, 'outcome': outcome, 'duration_s': 1.0,
                'total_steps': 10, 'joint_path_length': 0.1, 'max_joint_jerk': 0.01,
                'mean_abs_tracking_error': 0.001, 'min_ee_block_dist': None,
                'max_block_lift': None, 'time_to_success_s': None,
                'ee_path_len_m': 0.1, 'score': np.nan,
            })
        agg = aggregate_table(pd.DataFrame(rows))
        # 2/4 timeouts = 50%.
        assert agg.loc['m']['timeout_rate_%'] == 50.0


class TestTaskStages:

    def test_pick_only_returns_three_stages(self):
        names, success = task_stages(pick_only=True)
        assert names == ['reach', 'grasp', 'lift']
        assert success == 3

    def test_full_task_returns_five_stages(self):
        names, success = task_stages(pick_only=False)
        assert names == ['reach', 'grasp', 'lift', 'move', 'place']
        assert success == 5


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
