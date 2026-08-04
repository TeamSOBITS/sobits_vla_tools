#!/usr/bin/env python3
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
Real-robot evaluation of VLA policies from episode JSON-Lines logs.

Unlike the Gazebo case, a real-robot run has **no ground-truth observer**: the
deploy node cannot measure block pose, end-effector-to-object distance, block
lift, robot world-z or tilt, so ``ee_error``, ``block_lift``, ``robot_pose``,
``robot_z_drop`` and ``block_pose`` are ``null`` in every step, and the logger
can never set ``outcome='success_lift'``. Task success is therefore **scored by
a human operator on a 1-5 scale**, supplied via ``--scores``; everything else
reported here is derived only from signals the robot itself observes:

  * commanded vs measured joint states (tracking error),
  * per-step max joint jerk (motion smoothness),
  * joint path length (motion economy),
  * end-effector pose in base_footprint (kinematic, from measured joints),
  * episode duration / step count / termination reason.

Outputs
-------
  * ``per_episode.csv``     — one row per episode,
  * ``aggregate.csv/.md/.tex`` — per-task comparison table,
  * ``scores_template.csv`` — operator scoring sheet (written if absent),
  * figures (PNG + SVG each by default, see ``--formats``, plus one combined
    multi-page PDF):
      1. furthest stage reached per episode + mean stage,
      2. pick-and-place stage funnel (share clearing each stage),
      3. termination reason breakdown,
      4. episode duration distribution,
      5. tracking error vs time (mean +/- std),
      6. joint jerk distribution,
      7. joint path length vs duration (motion economy),
      8. EE 6-DOF trajectory grid for a representative episode.

Scoring rubric, recorded per episode in the scores CSV. The score is the number
of **cumulative** pick-and-place stages the policy completed, so a score of N
means every stage up to N succeeded:

  1 = moved the hand close over the target object
  2 = closed the hand and grasped the target object
  3 = lifted the target object
  4 = moved the target object towards its final destination
  5 = placed the target object over its final destination

Only a 5 is a completed task; ``success_rate`` counts score == 5. Because the
stages are cumulative, the per-stage completion rates form a funnel (every
episode scoring >= N cleared stage N), which is the ``stage_funnel`` figure.

Usage:
  ros2 run sobits_vla_visualization vla_eval \\
    --logs ball_bowl:<pkg>/logs/smolvla_pnp_ball_bowl \\
           bottle_bin:<pkg>/logs/smolvla_pnp_bottle_bin \\
    --scores /tmp/vla_eval/scores.csv \\
    --out /tmp/vla_eval
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import warnings
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')  # headless rendering
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


# ---------------------------------------------------------------------------
# Style — validated palette (see sobits_vla_visualization docs)
# ---------------------------------------------------------------------------
# Categorical slots for the compared tasks/models. Slots 1-2 of the reference
# categorical theme; validated all-pairs on the light surface (worst CVD
# dE 24.7, normal-vision dE 33.6, both >= 3:1 contrast) so the two series stay
# distinguishable for colourblind readers and in greyscale print.
SERIES_COLORS = ['#2a78d6', '#eb6834', '#1baf7a']
# Ordinal ramp for the 1-5 stage score: ONE hue, light -> dark, because the
# score is an ordered scale, not an identity. Validated: monotone lightness,
# adjacent dL >= 0.093, lightest step 2.06:1 vs the light surface.
#
# Five steps is more than one hue's usable band can separate cleanly: adjacent
# pairs land at dE ~9.8-10.1, under the 15 floor for telling categories apart
# (widening the ramp pushes the pale end below the 2:1 ordinal floor instead —
# there is no 5-step blue that clears both). Since each segment here is a
# discrete category rather than a point on a continuous scale, the ramp carries
# a second channel: alternating segments are hatched, so neighbours differ by
# texture as well as lightness. That also survives greyscale print and full CVD.
SCORE_COLORS = ['#86b6ef', '#5598e7', '#2a78d6', '#1c5cab', '#0d366b']
SCORE_HATCH = ['', '///', '', '///', '']
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SOFT = '#52514e'
GRID = '#e6e5e1'
NEUTRAL = '#9a9892'

# The operator score is the number of CUMULATIVE pick-and-place stages the
# policy completed, so score N implies every stage below N also succeeded.
# Only a 5 (object placed at its destination) is a completed task.
SCORE_LABELS = {
    1: '1 reached over object',
    2: '2 grasped',
    3: '3 lifted',
    4: '4 moved to destination',
    5: '5 placed',
}
# Short stage names for the funnel axis.
STAGE_LABELS = ['reach', 'grasp', 'lift', 'move', 'place']
SUCCESS_SCORE = 5  # full task completion — the final stage

# A pick-only task ("Pick up the block") ends at the lift: there is no
# destination to move to, so it has 3 stages and success == 3. Scoring it out
# of 5 would understate it, and comparing its success rate against a
# pick-and-place task's is comparing different tasks — see PICK_ONLY below.
PICK_ONLY_STAGES = 3

# Block displacement (m) that counts as "moved the block but did not lift it"
# when deriving stage 2 from simulation ground truth. Sits in the empty gap
# between jostling (<= 0.006 m observed) and a real lift (>= 0.076 m observed),
# so it is well clear of contact noise on either side.
GRASP_LIFT_M = 0.01


def task_stages(pick_only: bool) -> Tuple[List[str], int]:
    """Stage names and the success score for a pick-only / full task."""
    if pick_only:
        return STAGE_LABELS[:PICK_ONLY_STAGES], PICK_ONLY_STAGES
    return STAGE_LABELS, SUCCESS_SCORE


# Termination reasons the logger can actually emit on the real robot.
OUTCOME_ORDER = ['manual_stop', 'timeout', 'fallen', 'success_lift', 'incomplete']
OUTCOME_LABELS = {
    'manual_stop': 'operator stop',
    'timeout': 'timeout',
    'fallen': 'fallen',
    'success_lift': 'auto success',
    'incomplete': 'incomplete',
}
# Termination reason is a nominal state, not a quality ranking: keep it in one
# hue + neutrals so it is never mistaken for the score encoding.
OUTCOME_COLORS = {
    'manual_stop': '#2a78d6',
    'timeout': '#86b6ef',
    'fallen': '#e34948',
    'success_lift': '#1baf7a',
    'incomplete': '#c9c7c0',
}

ARM_PREFIXES = ('arm_', 'body_lift')


def _set_hatch_color(patch, color: str, alpha: float = 0.55) -> None:
    """
    Tint one patch's hatch, keeping its surface-coloured edge.

    ``set_hatch_color`` only exists from matplotlib 3.10; on older versions the
    private ``_hatch_color`` is the sole per-patch control (the ``hatch.color``
    rcParam is global and read at draw time, so it cannot vary per segment).
    Falls back to leaving the default hatch colour rather than raising.
    """
    rgba = matplotlib.colors.to_rgba(color, alpha)
    setter = getattr(patch, 'set_hatch_color', None)
    if callable(setter):
        setter(rgba)
    elif hasattr(patch, '_hatch_color'):
        patch._hatch_color = rgba


def _on_fill(hex_color: str) -> str:
    """Ink or white for a label set inside a coloured fill, by luminance."""
    h = hex_color.lstrip('#')
    r, g, b = (int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))
    lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
           for c in (r, g, b)]
    lum = 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2]
    return INK if lum > 0.42 else '#ffffff'


def apply_style() -> None:
    """Recessive axes, hairline solid grid, ink-token text."""
    plt.rcParams.update({
        'figure.facecolor': SURFACE,
        'axes.facecolor': SURFACE,
        'savefig.facecolor': SURFACE,
        'axes.edgecolor': GRID,
        'axes.labelcolor': INK_SOFT,
        'axes.titlecolor': INK,
        'axes.titlesize': 11,
        'axes.titleweight': 'semibold',
        'axes.titlelocation': 'left',
        'axes.labelsize': 9,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'xtick.color': INK_SOFT,
        'ytick.color': INK_SOFT,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'grid.color': GRID,
        'grid.linewidth': 0.8,
        'grid.linestyle': '-',
        'legend.frameon': False,
        'legend.fontsize': 9,
        'lines.linewidth': 2.0,
        'lines.solid_capstyle': 'round',
        'font.size': 10,
        # Keep text as text in SVG/PDF (selectable, searchable, re-styleable)
        # instead of converting glyphs to paths.
        'svg.fonttype': 'none',
        'pdf.fonttype': 42,
    })


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

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


def load_episode_file(path: str) -> Optional[Dict]:
    """Parse one episode_*.jsonl into {meta, summary, steps(DataFrame)}."""
    meta: Dict = {}
    summary: Dict = {}
    rows: List[Dict] = []
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
            elif kind == 'summary':
                summary = obj
            elif kind == 'step':
                rows.append(_flatten_step(obj))
    # Keep episodes that have a meta or summary even with zero steps — an
    # immediate abort can terminate before any step is logged, and dropping
    # those would bias the termination-reason counts.
    if not rows and not meta and not summary:
        return None
    steps = (pd.DataFrame(rows).sort_values('t').reset_index(drop=True)
             if rows else pd.DataFrame())
    return {'path': path, 'meta': meta, 'summary': summary, 'steps': steps}


def _flatten_step(obj: Dict) -> Dict:
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
    # Arm-only tracking error: the hand/head joints dominate the all-joint mean
    # and are not what the policy is being judged on.
    track = obj.get('tracking_error') or {}
    arm = [abs(v) for k, v in track.items()
           if k.startswith(ARM_PREFIXES) and isinstance(v, (int, float))]
    row['track_arm_abs_mean'] = float(np.mean(arm)) if arm else None
    row['track_arm_abs_max'] = float(np.max(arm)) if arm else None
    return row


def load_model(label: str, directory: str, pick_only: bool = False) -> Dict:
    """Load all episodes for one task/model directory."""
    files = sorted(glob.glob(os.path.join(directory, 'episode_*.jsonl')))
    episodes: List[Dict] = []
    for path in files:
        ep = load_episode_file(path)
        if ep is not None:
            episodes.append(ep)
    model = {'label': label, 'dir': directory, 'episodes': episodes,
             'pick_only': pick_only}
    # A run is "simulated" when the Gazebo observer actually reported a block
    # pose; that is also what makes its stage scores derivable without an
    # operator. Test `min_ee_block_dist`, not `max_block_lift`: the logger
    # defaults the latter to 0.0 even on hardware, where it means "never
    # observed" rather than "did not move", so it is not a sim/real signal.
    model['is_sim'] = any(
        ep['summary'].get('min_ee_block_dist') is not None
        for ep in episodes)
    return model


# ---------------------------------------------------------------------------
# Operator scores
# ---------------------------------------------------------------------------

SCORE_COLUMNS = ['model', 'episode', 'file', 'score', 'note']


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


def write_scores_template(per_ep: pd.DataFrame, path: str) -> None:
    """Write an empty scoring sheet for the operator to fill in."""
    tmpl = per_ep[['model', 'episode', 'file']].copy()
    tmpl['score'] = ''
    tmpl['note'] = ''
    tmpl.to_csv(path, index=False)


def attach_scores(per_ep: pd.DataFrame,
                  scores: Optional[pd.Series]) -> pd.DataFrame:
    """Join operator scores onto the per-episode table."""
    per_ep = per_ep.copy()
    if scores is None:
        per_ep['score'] = np.nan
        return per_ep
    by_file = scores.index.names[1] == 'file'
    keys = list(zip(per_ep['model'],
                    per_ep['file'] if by_file else per_ep['episode']))
    per_ep['score'] = [scores.get(k, np.nan) for k in keys]
    return per_ep


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

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
        # Stages are cumulative: score N means stages 1..N all succeeded, so
        # the rate for stage N is simply the share of episodes scoring >= N.
        # Only the final stage (place) counts as task success.
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
            'arm_track_err_rad': _mean_std(g.get('arm_track_mean',
                                                 g['mean_abs_tracking_error'])),
            'jerk_p95_rad': _mean_std(g.get('jerk_p95', g['max_joint_jerk'])),
            'joint_path_rad': _mean_std(g['joint_path_length'], '{:.1f}'),
            'ee_path_m': _mean_std(g['ee_path_len_m'], '{:.2f}'),
        })
        rows.append(row)
    return pd.DataFrame(rows).set_index('model')


def write_markdown(df: pd.DataFrame, path: str, note: str = '') -> None:
    """Write the aggregate table as Markdown, with an optional caveat note."""
    cols = list(df.columns)
    lines = ['| model | ' + ' | '.join(cols) + ' |',
             '| --- | ' + ' | '.join(['---'] * len(cols)) + ' |']
    for idx, row in df.iterrows():
        lines.append('| {} | '.format(idx)
                     + ' | '.join(str(row[c]) for c in cols) + ' |')
    if note:
        lines += ['', note]
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')


def _tex_escape(s: str) -> str:
    for a, b in (('\\', r'\textbackslash{}'), ('_', r'\_'), ('%', r'\%'),
                 ('&', r'\&'), ('#', r'\#'), ('±', r'$\pm$')):
        s = s.replace(a, b)
    return s


def write_latex(df: pd.DataFrame, path: str) -> None:
    """Write a booktabs-style LaTeX tabular (no jinja2 / Styler dependency)."""
    cols = list(df.columns)
    align = 'l' + 'r' * len(cols)
    out = [
        r'\begin{table}[t]',
        r'\centering',
        r'\caption{Real-robot VLA evaluation. The operator score is the number '
        r'of cumulative pick-and-place stages completed (reach, grasp, lift, '
        r'move, place); success requires all five. The deploy node has no '
        r'ground-truth observer on hardware.}',
        r'\label{tab:vla_real}',
        r'\begin{tabular}{' + align + '}',
        r'\toprule',
        'model & ' + ' & '.join(_tex_escape(c) for c in cols) + r' \\',
        r'\midrule',
    ]
    for idx, row in df.iterrows():
        out.append(_tex_escape(str(idx)) + ' & '
                   + ' & '.join(_tex_escape(str(row[c])) for c in cols) + r' \\')
    out += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
    with open(path, 'w') as fh:
        fh.write('\n'.join(out) + '\n')


# ---------------------------------------------------------------------------
# Time-series resampling for mean +/- std bands
# ---------------------------------------------------------------------------

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


def plot_band(ax, models: List[Dict], column: str, dt: float = 0.1,
              smooth_s: float = 0.0):
    """Mean +/- std band per model on a common time grid."""
    t_max = 0.0
    for m in models:
        for ep in m['episodes']:
            df = ep['steps']
            if df.empty or 't' not in df.columns:
                continue
            tt = pd.to_numeric(df['t'], errors='coerce')
            if tt.notna().any():
                t_max = max(t_max, float(tt.max()))
    if t_max <= 0:
        return
    grid = np.arange(0.0, t_max + dt, dt)
    for mi, m in enumerate(models):
        arr = resample(m['episodes'], column, grid)
        if np.isnan(arr).all():
            continue
        # Columns past the longest episode are all-NaN; ignore the resulting
        # "mean of empty slice" warnings.
        with np.errstate(invalid='ignore'), warnings.catch_warnings():
            warnings.simplefilter('ignore', category=RuntimeWarning)
            mean = np.nanmean(arr, axis=0)
            std = np.nanstd(arr, axis=0)
            n = np.sum(np.isfinite(arr), axis=0)
        # Past the point where only a couple of episodes remain, the band is
        # noise dressed up as a trend — stop drawing it.
        keep = n >= max(2, 0.3 * len(m['episodes']))
        c = SERIES_COLORS[mi % len(SERIES_COLORS)]
        win = int(round(smooth_s / dt)) if smooth_s else 0
        line = _smooth(mean, win) if win > 1 else mean
        ax.plot(grid[keep], line[keep], color=c, label=m['label'])
        ax.fill_between(grid[keep], (mean - std)[keep], (mean + std)[keep],
                        color=c, alpha=0.10, linewidth=0)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_scores(per_ep: pd.DataFrame):
    """Operator score distribution (1-5) per task — the headline result."""
    scored = per_ep.dropna(subset=['score'])
    if scored.empty:
        return None
    models = list(per_ep['model'].unique())
    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(11, 2.2 + 0.8 * len(models)),
        gridspec_kw={'width_ratios': [2.1, 1]})

    # Stacked share-of-episodes by score, one bar per task.
    y = np.arange(len(models))
    left = np.zeros(len(models))
    for si, s in enumerate([1, 2, 3, 4, 5]):
        vals = []
        for m in models:
            g = scored[scored['model'] == m]
            vals.append(100.0 * (g['score'] == s).sum() / len(g) if len(g) else 0.0)
        vals = np.array(vals)
        if vals.sum() == 0:
            continue
        # Hatch rides in the label ink so it reads on both light and dark
        # fills; the 2px surface edge still separates touching segments.
        bars = ax.barh(y, vals, left=left, height=0.5, color=SCORE_COLORS[si],
                       label=SCORE_LABELS[s], edgecolor=SURFACE, linewidth=2,
                       hatch=SCORE_HATCH[si] or None)
        if SCORE_HATCH[si]:
            for b in bars:
                _set_hatch_color(b, _on_fill(SCORE_COLORS[si]), 0.55)
        # Label only segments with room — never clip text inside a mark.
        for yi, v in zip(y, vals):
            if v >= 9:
                ax.text(left[yi] + v / 2, yi, '{:.0f}'.format(v),
                        ha='center', va='center', fontsize=8, zorder=5,
                        color=_on_fill(SCORE_COLORS[si]))
        left += vals
    ax.set_yticks(y)
    ax.set_yticklabels(models)
    ax.invert_yaxis()
    ax.set_xlabel('share of episodes (%)')
    ax.set_xlim(0, 100)
    ax.set_title('Furthest stage reached per episode')
    # All five stage keys on one row: anchored to the figure, not the left
    # axes, so the long labels have the full width to sit in.
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=len(labels),
               bbox_to_anchor=(0.5, -0.02), columnspacing=1.4,
               handlelength=1.6, handletextpad=0.5)
    ax.grid(True, axis='x', alpha=0.6)
    ax.set_axisbelow(True)

    # Mean score with std, direct-labelled.
    for mi, m in enumerate(models):
        g = scored[scored['model'] == m]
        if g.empty:
            continue
        mu, sd = g['score'].mean(), g['score'].std(ddof=0)
        c = SERIES_COLORS[mi % len(SERIES_COLORS)]
        ax2.errorbar(mu, mi, xerr=sd, fmt='o', color=c, markersize=9,
                     capsize=4, markeredgecolor=SURFACE, markeredgewidth=2)
        ax2.text(mu, mi + 0.16, '{:.2f}'.format(mu), ha='center',
                 va='top', fontsize=9, color=INK)
    ax2.set_yticks(np.arange(len(models)))
    ax2.set_yticklabels(models)
    # Headroom so the direct labels never ride into the title/axis.
    ax2.set_ylim(len(models) - 0.5, -0.5)
    ax2.set_xlim(0.5, 5.5)
    ax2.set_xticks([1, 2, 3, 4, 5])
    ax2.set_xlabel('mean stage reached (1-5)')
    ax2.set_title('Mean stage ± std')
    ax2.grid(True, axis='x', alpha=0.6)
    ax2.set_axisbelow(True)
    # Leave a strip at the bottom for the figure-level legend; tight_layout
    # only accounts for axes-level artists.
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    return fig


def fig_stage_funnel(per_ep: pd.DataFrame):
    """Share of episodes clearing each cumulative pick-and-place stage."""
    scored = per_ep.dropna(subset=['score'])
    if scored.empty:
        return None
    models = list(per_ep['model'].unique())
    x = np.arange(len(STAGE_LABELS))
    fig, ax = plt.subplots(figsize=(8.5, 4.4))
    n_m = len(models)
    width = min(0.34, 0.8 / max(n_m, 1))
    for mi, m in enumerate(models):
        g = scored[scored['model'] == m]
        if g.empty:
            continue
        n_stages = (PICK_ONLY_STAGES if bool(g['pick_only'].iloc[0])
                    else len(STAGE_LABELS))
        n_ep = len(g)
        counts = [int((g['score'] >= n).sum())
                  for n in range(1, n_stages + 1)]
        rates = [100.0 * k / n_ep for k in counts]
        # Centre the group of bars on each stage tick.
        off = (mi - (n_m - 1) / 2) * width
        c = SERIES_COLORS[mi % len(SERIES_COLORS)]
        ax.bar(x[:n_stages] + off, rates, width=width * 0.92, color=c,
               label='{} (n={})'.format(m, n_ep), edgecolor=SURFACE,
               linewidth=2)
        # Percentage over the raw count: a rate alone hides how few episodes
        # it rests on.
        for xi, r, k in zip(x[:n_stages], rates, counts):
            ax.text(xi + off, r + 2.5, '{:.0f}%'.format(r), ha='center',
                    va='bottom', fontsize=8, color=INK_SOFT)
            ax.text(xi + off, r + 9.0, '{}/{}'.format(k, n_ep), ha='center',
                    va='bottom', fontsize=7, color=NEUTRAL)
        # Mark EVERY stage the task does not have, so the absent bars are not
        # misread as zeros — a gap at one stage and nothing at the next reads
        # as "failed here", which is the opposite of "never attempted".
        for xi in x[n_stages:]:
            ax.text(xi + off, 3, 'n/a', ha='center', va='bottom',
                    fontsize=8, color=NEUTRAL, rotation=90)
    ax.set_xticks(x)
    ax.set_xticklabels(['{}. {}'.format(i + 1, s)
                        for i, s in enumerate(STAGE_LABELS)])
    # Room above a 100% bar for the stacked percent + count labels.
    ax.set_ylim(0, 120)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_ylabel('episodes clearing stage (%)')
    ax.set_title('Task stage funnel (stages are cumulative; n/a = task has '
                 'no such stage)')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def fig_outcomes(per_ep: pd.DataFrame):
    """How episodes terminated — a logging fact, not a quality measure."""
    models = list(per_ep['model'].unique())
    fig, ax = plt.subplots(figsize=(7, 1.1 + 0.75 * len(models)))
    y = np.arange(len(models))
    left = np.zeros(len(models))
    for o in OUTCOME_ORDER:
        vals = np.array([int((per_ep[per_ep['model'] == m]['outcome'] == o).sum())
                         for m in models])
        if vals.sum() == 0:
            continue
        ax.barh(y, vals, left=left, height=0.34, color=OUTCOME_COLORS[o],
                label=OUTCOME_LABELS[o], edgecolor=SURFACE, linewidth=2)
        for yi, v in zip(y, vals):
            if v >= 2:
                ax.text(left[yi] + v / 2, yi, str(v), ha='center', va='center',
                        fontsize=8, color=_on_fill(OUTCOME_COLORS[o]))
        left += vals
    ax.set_yticks(y)
    ax.set_yticklabels(models)
    ax.set_ylim(len(models) - 0.5, -0.5)
    ax.set_xlabel('episodes')
    # Episode counts are integers — no 2.5-episode ticks.
    ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.set_title('Episode termination reason')
    # Top-right, beside the left-aligned title: below the plot it collides
    # with the x-axis label.
    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=4,
              borderaxespad=0.0)
    ax.grid(True, axis='x', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def fig_duration(per_ep: pd.DataFrame):
    """Episode duration per task — strip plot over a mean marker."""
    models = list(per_ep['model'].unique())
    fig, ax = plt.subplots(figsize=(7, 1.4 + 0.95 * len(models)))
    rng = np.random.default_rng(0)  # deterministic jitter
    any_data = False
    for mi, m in enumerate(models):
        v = pd.to_numeric(per_ep[per_ep['model'] == m]['duration_s'],
                          errors='coerce').dropna()
        if v.empty:
            continue
        any_data = True
        c = SERIES_COLORS[mi % len(SERIES_COLORS)]
        ax.scatter(v, mi + rng.uniform(-0.10, 0.10, v.size), s=42, color=c,
                   alpha=0.75, edgecolor=SURFACE, linewidth=1.5, zorder=3)
        ax.scatter([v.mean()], [mi], marker='|', s=520, color=INK,
                   linewidth=2, zorder=4)
        ax.text(v.mean(), mi - 0.22, '{:.1f} s'.format(v.mean()),
                ha='center', fontsize=9, color=INK)
    if not any_data:
        plt.close(fig)
        return None
    ax.set_yticks(np.arange(len(models)))
    ax.set_yticklabels(models)
    ax.set_ylim(len(models) - 0.5, -0.45)
    ax.set_xlabel('episode duration (s)')
    ax.set_title('Episode duration (each dot = one episode, bar = mean)')
    ax.grid(True, axis='x', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def fig_timeseries(models, column, ylabel, title, smooth_s: float = 0.0):
    """Mean +/- std of ``column`` vs time, one band per task."""
    fig, ax = plt.subplots(figsize=(7, 3.8))
    plot_band(ax, models, column, smooth_s=smooth_s)
    if not ax.lines:
        plt.close(fig)
        return None
    ax.set_xlabel('time (s)')
    ax.set_ylabel(ylabel)
    if smooth_s:
        title += '  ({:.0f} s rolling mean, band = ±1 std)'.format(smooth_s)
    ax.set_title(title)
    if ax.get_legend_handles_labels()[0]:
        ax.legend()
    ax.grid(True, alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def fig_jerk(models):
    """Per-step max joint jerk distribution — motion smoothness."""
    data, labels, used = [], [], []
    for mi, m in enumerate(models):
        pooled = []
        for ep in m['episodes']:
            df = ep['steps']
            if df.empty or 'max_jerk' not in df.columns:
                continue
            pooled.extend(pd.to_numeric(df['max_jerk'],
                                        errors='coerce').dropna().tolist())
        if pooled:
            data.append(pooled)
            labels.append(m['label'])
            used.append(mi)
    if not data:
        return None
    # Width follows the box count so two boxes don't sprawl across 7 inches.
    fig, ax = plt.subplots(figsize=(2.0 + 1.5 * len(data), 3.8))
    bp = ax.boxplot(data, labels=labels, showfliers=False, patch_artist=True,
                    widths=0.4, medianprops={'color': INK, 'linewidth': 2})
    for box, mi in zip(bp['boxes'], used):
        box.set_facecolor(SERIES_COLORS[mi % len(SERIES_COLORS)])
        box.set_alpha(0.30)
        box.set_edgecolor(SERIES_COLORS[mi % len(SERIES_COLORS)])
        box.set_linewidth(1.5)
    for part in ('whiskers', 'caps'):
        for art in bp[part]:
            art.set_color(NEUTRAL)
    ax.set_ylabel('per-step max joint jerk (rad)')
    ax.set_title('Motion smoothness — joint jerk distribution')
    ax.grid(True, axis='y', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def fig_economy(per_ep: pd.DataFrame):
    """Joint path length vs duration: how much motion buys how much time."""
    fig, ax = plt.subplots(figsize=(7, 4))
    models = list(per_ep['model'].unique())
    plotted = False
    for mi, m in enumerate(models):
        g = per_ep[per_ep['model'] == m]
        x = pd.to_numeric(g['duration_s'], errors='coerce')
        y = pd.to_numeric(g['joint_path_length'], errors='coerce')
        ok = x.notna() & y.notna()
        if not ok.any():
            continue
        plotted = True
        c = SERIES_COLORS[mi % len(SERIES_COLORS)]
        sc = pd.to_numeric(g['score'], errors='coerce')
        # Marker size carries the operator score when available (composite
        # encoding: hue = task, size = score) — never a second colour scale.
        sizes = (30 + 26 * (sc[ok].fillna(2) - 1)) if sc.notna().any() else 55
        ax.scatter(x[ok], y[ok], s=sizes, color=c, alpha=0.75,
                   edgecolor=SURFACE, linewidth=1.5, label=m, zorder=3)
    if not plotted:
        plt.close(fig)
        return None
    ax.set_xlabel('episode duration (s)')
    ax.set_ylabel('joint path length (rad)')
    ax.set_title('Motion economy (marker size = operator score, when scored)')
    ax.legend()
    ax.grid(True, alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


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


def fig_ee_trajectory(models, per_ep: pd.DataFrame):
    """EE 6-DOF trajectory for a representative episode per task."""
    dofs = [('ee_x', 'x (m)'), ('ee_y', 'y (m)'), ('ee_z', 'z (m)'),
            ('ee_roll', 'roll (rad)'), ('ee_pitch', 'pitch (rad)'),
            ('ee_yaw', 'yaw (rad)')]
    fig, axes = plt.subplots(2, 3, figsize=(12, 5.6), sharex=True)
    axes = axes.ravel()
    plotted = False
    for mi, m in enumerate(models):
        ep = _representative_episode(m, per_ep)
        if ep is None:
            continue
        df = ep['steps']
        t = pd.to_numeric(df['t'], errors='coerce')
        for ax, (col, _) in zip(axes, dofs):
            if col in df:
                ax.plot(t, pd.to_numeric(df[col], errors='coerce'),
                        color=SERIES_COLORS[mi % len(SERIES_COLORS)],
                        label=m['label'], linewidth=1.6)
                plotted = True
    if not plotted:
        plt.close(fig)
        return None
    for ax, (_, ylab) in zip(axes, dofs):
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.6)
        ax.set_axisbelow(True)
    for ax in axes[3:]:
        ax.set_xlabel('time (s)')
    if axes[0].get_legend_handles_labels()[0]:
        axes[0].legend()
    fig.suptitle('Left end-effector pose (base_footprint) — '
                 'representative episode per task',
                 x=0.01, ha='left', fontsize=11, fontweight='semibold',
                 color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Load episode logs, write tables and figures, print the summary."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--logs', nargs='+', required=True, metavar='label:dir',
        help='One or more task/model log dirs, e.g. '
             'ball_bowl:/tmp/vla_logs/smolvla_pnp_ball_bowl',
    )
    parser.add_argument('--out', default='./vla_eval_out',
                        help='Output directory for tables and figures.')
    parser.add_argument('--pick-only', nargs='*', default=[], metavar='LABEL',
                        help='Labels whose task ends at the lift ("Pick up the '
                             'block") and so has 3 stages, not 5. Their '
                             'success rate means "lifted" and is NOT '
                             'comparable with a pick-and-place success rate.')
    parser.add_argument('--scores', default=None,
                        help='CSV of operator scores (model,episode,file,'
                             'score,note) with score the number of cumulative '
                             'stages completed, an integer 1-5. If the '
                             'file does not exist, a blank template is '
                             'written there for you to fill in.')
    parser.add_argument('--dpi', type=int, default=200,
                        help='Raster resolution; ignored for vector formats.')
    parser.add_argument('--formats', nargs='+', default=['png', 'svg'],
                        metavar='EXT',
                        help='Figure formats to write, e.g. png svg pdf. '
                             'SVG/PDF are vector and scale losslessly for '
                             'print (default: png svg).')
    args = parser.parse_args()

    supported = matplotlib.figure.Figure().canvas.get_supported_filetypes()
    fmts = [f.lower().lstrip('.') for f in args.formats]
    bad = [f for f in fmts if f not in supported]
    if bad:
        raise SystemExit(
            'Unsupported figure format(s): {}. Supported: {}'.format(
                bad, ' '.join(sorted(supported))))

    os.makedirs(args.out, exist_ok=True)
    apply_style()

    pairs = parse_logs_arg(args.logs)
    pick_only = set(args.pick_only)
    unknown = pick_only - {label for label, _ in pairs}
    if unknown:
        raise SystemExit('--pick-only names unknown label(s): {}. Known: '
                         '{}'.format(sorted(unknown),
                                     [label for label, _ in pairs]))
    models = [load_model(label, d, label in pick_only) for label, d in pairs]
    models = [m for m in models if m['episodes']]
    if not models:
        raise SystemExit('No episodes found in any of: {}'.format(
            [d for _, d in pairs]))
    for m in models:
        print('Loaded {} episodes for {!r} from {}{}{}'.format(
            len(m['episodes']), m['label'], m['dir'],
            ' [sim]' if m['is_sim'] else '',
            ' [pick-only, {} stages]'.format(PICK_ONLY_STAGES)
            if m['pick_only'] else ''))

    # ---- Tables -------------------------------------------------------------
    per_ep = pd.concat([per_episode_table(m) for m in models],
                       ignore_index=True)
    scores = load_scores(args.scores)
    per_ep = attach_scores(per_ep, scores)

    # Simulated runs carry the ground truth the operator sheet exists to
    # replace, so fill their stages automatically — but never overwrite a score
    # that was supplied by hand.
    per_ep['score_source'] = np.where(per_ep['score'].notna(), 'operator', '')
    for m in models:
        if not m['is_sim']:
            continue
        derived = derive_sim_scores(m)
        sel = (per_ep['model'] == m['label']) & per_ep['score'].isna()
        if not sel.any():
            continue
        per_ep.loc[sel, 'score'] = per_ep.loc[sel, 'file'].map(derived)
        per_ep.loc[sel, 'score_source'] = 'ground_truth'
        print('Derived {} stage scores for {!r} from simulation ground '
              'truth'.format(int(sel.sum()), m['label']))

    per_ep.to_csv(os.path.join(args.out, 'per_episode.csv'), index=False)

    # Leave a scoring sheet for the runs that still need a human — the
    # ground-truth ones do not.
    needs_scoring = per_ep[per_ep['score'].isna()]
    tmpl_path = args.scores or os.path.join(args.out, 'scores_template.csv')
    if not needs_scoring.empty and not os.path.exists(tmpl_path):
        write_scores_template(needs_scoring, tmpl_path)
        print('\nNo operator scores yet — wrote a blank scoring sheet to\n  {}\n'
              'Fill in the "score" column (stages completed, 1-5; see the '
              'rubric in --help) and '
              're-run with --scores {}'.format(tmpl_path, tmpl_path))

    n_scored = int(per_ep['score'].notna().sum())
    note_parts = [
        '*The score is the number of cumulative stages completed '
        '(1 reach, 2 grasp, 3 lift, 4 move, 5 place); a stage rate is the '
        'share of episodes scoring >= that stage.',
        'The deploy node has no ground-truth observer on the real robot, so '
        'block pose, lift, EE-to-object distance and base balance are '
        'unavailable there and the automatic `success_lift` outcome can never '
        'fire; those runs are scored by a human operator. Simulated runs have '
        'the observer, so their stages are derived from ground truth '
        '(`score_source` in per_episode.csv).',
    ]
    if any(m['pick_only'] for m in models):
        note_parts.append(
            '**Success rates are not comparable across the `task` column.** A '
            'pick-only task ends at the lift (3 stages), so its "success" is '
            'stage 3 — the same event that counts as only partial progress '
            'for a pick+place task. Compare those tasks at the shared '
            'reach/grasp/lift stages instead.')
    note = ' '.join(note_parts) + (
        ' `timeout_rate` and the termination reasons are logger facts; all '
        'motion metrics are robot-observed.*')
    if not n_scored:
        note = ('*No operator scores supplied — success/score columns are '
                'blank. ' + note.lstrip('*'))

    agg = aggregate_table(per_ep)
    agg.to_csv(os.path.join(args.out, 'aggregate.csv'))
    write_markdown(agg, os.path.join(args.out, 'aggregate.md'), note)
    write_latex(agg, os.path.join(args.out, 'aggregate.tex'))

    # ---- Figures ------------------------------------------------------------
    figs = [
        ('operator_scores', fig_scores(per_ep)),
        ('stage_funnel', fig_stage_funnel(per_ep)),
        ('outcomes', fig_outcomes(per_ep)),
        ('duration', fig_duration(per_ep)),
        ('tracking_error', fig_timeseries(
            models, 'track_arm_abs_mean',
            'mean |commanded - measured| (rad)',
            'Arm joint tracking error over time', smooth_s=2.0)),
        ('joint_jerk', fig_jerk(models)),
        ('motion_economy', fig_economy(per_ep)),
        ('ee_trajectory', fig_ee_trajectory(models, per_ep)),
    ]

    for name, fig in figs:
        if fig is None:
            continue
        for ext in fmts:
            fig.savefig(os.path.join(args.out, '{}.{}'.format(name, ext)),
                        dpi=args.dpi, bbox_inches='tight')

    pdf_path = os.path.join(args.out, 'vla_real_eval.pdf')
    with PdfPages(pdf_path) as pdf:
        for _, fig in figs:
            if fig is not None:
                pdf.savefig(fig, bbox_inches='tight')
    for _, fig in figs:
        if fig is not None:
            plt.close(fig)

    print('\nAggregate comparison ({}/{} episodes scored):\n'.format(
        n_scored, len(per_ep)))
    print(agg.to_string())
    print('\nWrote tables + figures to {}'.format(os.path.abspath(args.out)))


if __name__ == '__main__':
    main()
