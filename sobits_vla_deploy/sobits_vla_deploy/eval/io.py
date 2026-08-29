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
Episode-log loading, operator score sheets, and step flattening.

Pure pandas/numpy, no ROS/torch — offline analysis of already-written JSONL.
"""

from __future__ import annotations

import glob
import json
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.context import EvalContext
from tqdm import tqdm

# Displacement (m) marking stage-2 "moved but not lifted" from sim ground
# truth. Sits clear of contact noise: jostling <= 0.006 m, real lift >= 0.076 m.
GRASP_LIFT_M = 0.01

SCORE_COLUMNS = ['model', 'episode', 'file', 'score', 'note']


def parse_logs_arg(items: List[str]) -> List[Tuple[str, str]]:
    """Parse ``label:dir`` items into (label, dir) tuples."""
    out: List[Tuple[str, str]] = []
    for item in items:
        if ':' not in item:
            raise ValueError(
                'Expected label:dir, got {!r}. '
                'Example: ball_bowl:/tmp/vla_logs/smolvla_pnp_ball_bowl'.format(item)
            )
        label, path = item.split(':', 1)
        out.append((label.strip(), os.path.expanduser(path.strip())))
    return out


def derive_sim_scores(model: Dict) -> Dict[str, int]:
    """
    Infer stage scores from Gazebo ground truth, keyed by episode filename.

    Simulation logs carry the observer fields the real robot lacks, so the
    pick stages can be read off directly instead of being scored by hand:

      3 (lift)  — ``success``/``success_lift``: block raised past the
                  lift threshold recorded in the episode's own meta,
      2 (grasp) — block left the table (lift clearly above sensor noise)
                  without reaching the success threshold,
      1 (reach) — the gripper got close to the block but never moved it.

    Episodes that never approach the block stay at 1: the real-robot rubric
    has no "no progress" code, so 1 is the floor in both cases.
    """
    scores: Dict[str, int] = {}
    for ep in model['episodes']:
        s = ep['summary']
        name = os.path.basename(ep['path'])
        lift = s.get('max_block_lift')
        if s.get('success') or s.get('outcome') == 'success_lift':
            scores[name] = 3
        elif isinstance(lift, (int, float)) and lift >= GRASP_LIFT_M:
            scores[name] = 2
        else:
            scores[name] = 1
    return scores


def load_episode_file(path: str, ctx: EvalContext) -> Optional[Dict]:
    """Parse one episode_*.jsonl into {meta, summary, steps(DataFrame)}."""
    meta: Dict = {}
    summary: Dict = {}
    rows: List[Dict] = []
    # Groups whose tracking error counts as "arm". Resolved from the meta line
    # (written first), so steps are flattened with the grouping already known.
    arm_groups: Optional[set] = None
    with open(path, 'r') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = obj.get('type')
            if kind == 'meta':
                meta = obj
                arm_groups = ctx.resolve_arm_groups(meta.get('joint_groups'))
            elif kind == 'summary':
                summary = obj
            elif kind == 'step':
                rows.append(_flatten_step(obj, arm_groups))
    # Keep zero-step episodes with a meta/summary — an immediate abort logs
    # no steps, and dropping it would bias the termination-reason counts.
    if not rows and not meta and not summary:
        return None
    steps = (pd.DataFrame(rows).sort_values('t').reset_index(drop=True)
             if rows else pd.DataFrame())
    return {'path': path, 'meta': meta, 'summary': summary, 'steps': steps}


def _flatten_step(obj: Dict, arm_groups: Optional[set] = None) -> Dict:
    """
    Flatten one step into scalar columns.

    Only fields the real robot can actually observe are kept: the Gazebo-only
    ground truth (ee_error, block_*, robot_pose, robot_z_drop) is null in every
    real-robot step and is deliberately not carried into the DataFrame.
    """
    ee = obj.get('ee_pose') or {}
    row = {
        't': obj.get('t'),
        'step': obj.get('step'),
        'max_jerk': obj.get('max_jerk'),
        'track_abs_mean': obj.get('track_abs_mean'),
        'base_speed': obj.get('base_speed'),
        'ee_x': ee.get('x'), 'ee_y': ee.get('y'), 'ee_z': ee.get('z'),
        'ee_roll': ee.get('roll'), 'ee_pitch': ee.get('pitch'),
        'ee_yaw': ee.get('yaw'),
    }
    # Arm-only tracking error from per-group means — hand/head joints skew
    # the all-joint mean and aren't what the policy is judged on.
    by_group = obj.get('track_abs_mean_by_group') or {}
    arm = [abs(v) for g, v in by_group.items()
           if (arm_groups is None or g in arm_groups) and isinstance(v, (int, float))]
    row['track_arm_abs_mean'] = float(np.mean(arm)) if arm else None
    row['track_arm_abs_max'] = float(np.max(arm)) if arm else None
    return row


def load_model(label: str, directory: str, ctx: EvalContext, pick_only: bool = False) -> Dict:
    """Load all episodes for one task/model directory."""
    files = sorted(glob.glob(os.path.join(directory, 'episode_*.jsonl')))
    episodes: List[Dict] = []
    for path in tqdm(files, desc='loading {}'.format(label), unit='ep',
                     disable=None, leave=False):
        ep = load_episode_file(path, ctx)
        if ep is not None:
            episodes.append(ep)
    model = {'label': label, 'dir': directory, 'episodes': episodes,
             'pick_only': pick_only}
    # "Simulated" = Gazebo reported a block pose. Test `min_ee_block_dist`, not
    # `max_block_lift`: it defaults to 0.0 on hardware too ("never observed").
    model['is_sim'] = any(
        ep['summary'].get('min_ee_block_dist') is not None
        for ep in episodes)
    return model


def load_scores(path: Optional[str]) -> Optional[pd.DataFrame]:
    """Load the operator score sheet, keyed by (model, file)."""
    if not path or not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    missing = {'model', 'score'} - set(df.columns)
    if missing:
        raise SystemExit(
            'Score CSV {} is missing column(s): {}. Expected {}.'.format(
                path, sorted(missing), SCORE_COLUMNS))
    key = 'file' if 'file' in df.columns else 'episode'
    df['score'] = pd.to_numeric(df['score'], errors='coerce')
    bad = df['score'].dropna()
    bad = bad[(bad < 1) | (bad > 5) | (bad != bad.round())]
    if not bad.empty:
        raise SystemExit(
            'Score CSV {} has out-of-range scores (expected integers 1-5): '
            '{}'.format(path, sorted(bad.unique().tolist())))
    return df.set_index(['model', key])['score']
