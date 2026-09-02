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
Per-episode and aggregate metric tables, and time-series resampling.

Pure pandas/numpy; the numbers here land directly in the paper tables.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

# Score = number of CUMULATIVE pick-and-place stages completed (score N
# implies every stage below N also succeeded). Only 5 = completed task.
STAGE_LABELS = ['reach', 'grasp', 'lift', 'move', 'place']
SUCCESS_SCORE = 5  # full task completion — the final stage

# A pick-only task ends at the lift (3 stages, success == 3); scoring it out
# of 5 would understate it and isn't comparable to a pick-and-place task.
PICK_ONLY_STAGES = 3


def task_stages(pick_only: bool) -> Tuple[List[str], int]:
    """Stage names and the success score for a pick-only / full task."""
    if pick_only:
        return STAGE_LABELS[:PICK_ONLY_STAGES], PICK_ONLY_STAGES
    return STAGE_LABELS, SUCCESS_SCORE


def per_episode_table(model: Dict) -> pd.DataFrame:
    """One row per episode, from the summary line plus step-derived columns."""
    recs = []
    for i, ep in enumerate(model['episodes']):
        s = ep['summary']
        df = ep['steps']
        outcome = s.get('outcome', 'incomplete')
        dur = s.get('duration_s')
        steps = s.get('total_steps')
        rec = {
            'model': model['label'],
            'episode': ep['meta'].get('episode', i + 1),
            'file': os.path.basename(ep['path']),
            'source_run': ep['meta'].get('source_run', ''),
            'source_file': ep['meta'].get('source_file', ''),
            'model_repo_id': ep['meta'].get('model_repo_id', ''),
            'pick_only': bool(model.get('pick_only')),
            'is_sim': bool(model.get('is_sim')),
            'outcome': outcome,
            'duration_s': dur,
            'total_steps': steps,
            'control_hz': (round(steps / dur, 2)
                           if isinstance(dur, (int, float)) and dur
                           and isinstance(steps, (int, float)) else None),
            'joint_path_length': s.get('joint_path_length'),
            'max_joint_jerk': s.get('max_joint_jerk'),
            'mean_abs_tracking_error': s.get('mean_abs_tracking_error'),
            # Gazebo-only ground truth; None on real-robot runs.
            'min_ee_block_dist': s.get('min_ee_block_dist'),
            'max_block_lift': s.get('max_block_lift'),
            'time_to_success_s': s.get('time_to_success_s'),
        }
        # Step-derived, arm-only motion quality.
        for col, out in (('track_arm_abs_mean', 'arm_track_mean'),
                         ('track_arm_abs_max', 'arm_track_max'),
                         ('max_jerk', 'jerk_p95')):
            if not df.empty and col in df.columns:
                v = pd.to_numeric(df[col], errors='coerce').dropna()
                if not v.empty:
                    rec[out] = (float(v.quantile(0.95)) if out == 'jerk_p95'
                                else float(v.mean()) if out.endswith('mean')
                                else float(v.max()))
        rec['ee_path_len_m'] = _ee_path_length(df)
        recs.append(rec)
    return pd.DataFrame(recs)


def _ee_path_length(df: pd.DataFrame) -> Optional[float]:
    """Cartesian path length of the end-effector (motion economy proxy)."""
    if df.empty or not {'ee_x', 'ee_y', 'ee_z'} <= set(df.columns):
        return None
    xyz = df[['ee_x', 'ee_y', 'ee_z']].apply(pd.to_numeric, errors='coerce')
    xyz = xyz.dropna()
    if len(xyz) < 2:
        return None
    return float(np.linalg.norm(np.diff(xyz.to_numpy(), axis=0), axis=1).sum())


def _mean_std(series: pd.Series, fmt: str = '{:.3f}') -> str:
    s = pd.to_numeric(series, errors='coerce').dropna()
    if s.empty:
        return '--'
    return (fmt + ' ± ' + fmt).format(s.mean(), s.std(ddof=0))


def aggregate_table(per_ep: pd.DataFrame) -> pd.DataFrame:
    """Aggregate per-task metrics for the comparison table."""
    rows = []
    for model, g in per_ep.groupby('model', sort=False):
        n = len(g)
        sc = pd.to_numeric(g['score'], errors='coerce').dropna()
        outcomes = g['outcome'].value_counts()
        pick_only = bool(g['pick_only'].iloc[0])
        stage_names, success_score = task_stages(pick_only)
        row = {
            'model': model,
            'task': 'pick' if pick_only else 'pick+place',
            'episodes': n,
            'scored': int(sc.size),
        }
        # Stages cumulative: score N means stages 1..N succeeded, so stage N's
        # rate is the share scoring >= N. Only the final stage counts as success.
        if sc.size:
            row['mean_stage'] = '{:.2f} ± {:.2f}'.format(sc.mean(),
                                                         sc.std(ddof=0))
            row['stages'] = '{}/{}'.format(int(sc.max()), success_score)
            row['success_rate_%'] = round(
                100.0 * (sc >= success_score).sum() / sc.size, 1)
            # NB: not `n` — that is the episode count, used again below.
            for stage, name in enumerate(STAGE_LABELS, start=1):
                # Stages the task does not have stay blank rather than 0: a
                # pick-only run did not "fail to place", it was never asked to.
                row['{}_%'.format(name)] = (
                    round(100.0 * (sc >= stage).sum() / sc.size, 1)
                    if name in stage_names else '--')
        else:
            row['mean_stage'] = '--'
            row['stages'] = '--'
            row['success_rate_%'] = '--'
            for name in STAGE_LABELS:
                row['{}_%'.format(name)] = '--'
        row.update({
            'timeout_rate_%': (round(100.0 * outcomes.get('timeout', 0) / n, 1)
                               if n else 0.0),
            'duration_s': _mean_std(g['duration_s'], '{:.1f}'),
            # Blank, not all-joint mean: substituting it would silently report
            # a different metric under the arm-only label.
            'arm_track_err_rad': (_mean_std(g['arm_track_mean'])
                                  if 'arm_track_mean' in g else '--'),
            'jerk_p95_rad': _mean_std(g.get('jerk_p95', g['max_joint_jerk'])),
            'joint_path_rad': _mean_std(g['joint_path_length'], '{:.1f}'),
            'ee_path_m': _mean_std(g['ee_path_len_m'], '{:.2f}'),
        })
        rows.append(row)
    return pd.DataFrame(rows).set_index('model')


def resample(episodes: List[Dict], column: str, grid: np.ndarray) -> np.ndarray:
    """
    Interpolate ``column`` of each episode onto a common time grid.

    Returns an (n_episodes, n_grid) array with NaN where the grid time falls
    outside an episode's recorded span.
    """
    out = np.full((len(episodes), grid.size), np.nan)
    for i, ep in enumerate(episodes):
        df = ep['steps']
        if df.empty or column not in df.columns or 't' not in df.columns:
            continue
        t = pd.to_numeric(df['t'], errors='coerce').to_numpy()
        v = pd.to_numeric(df[column], errors='coerce').to_numpy()
        mask = np.isfinite(t) & np.isfinite(v)
        if mask.sum() < 2:
            continue
        t, v = t[mask], v[mask]
        vals = np.interp(grid, t, v, left=np.nan, right=np.nan)
        # np.interp clamps outside-range to endpoints; force NaN beyond span.
        vals[(grid < t[0]) | (grid > t[-1])] = np.nan
        out[i] = vals
    return out


def _smooth(v: np.ndarray, win: int) -> np.ndarray:
    """Centred rolling mean that tolerates NaN, for legible trend lines."""
    if win <= 1:
        return v
    return (pd.Series(v).rolling(win, center=True, min_periods=1)
            .mean().to_numpy())


def _representative_episode(model: Dict,
                            per_ep: pd.DataFrame) -> Optional[Dict]:
    """Best-scored episode if scored, else the median-duration one."""
    pool = [e for e in model['episodes']
            if not e['steps'].empty and 't' in e['steps'].columns]
    if not pool:
        return None
    g = per_ep[per_ep['model'] == model['label']].dropna(subset=['score'])
    if not g.empty:
        best = g.sort_values('score', ascending=False).iloc[0]['file']
        for e in pool:
            if os.path.basename(e['path']) == best:
                return e
    pool = sorted(pool, key=lambda e: e['summary'].get('duration_s') or 0.0)
    return pool[len(pool) // 2]
