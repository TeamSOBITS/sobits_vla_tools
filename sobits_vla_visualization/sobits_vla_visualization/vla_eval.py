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
Research-grade comparison of VLA models from episode JSON-Lines logs.

Loads the per-episode logs written by sobits_vla_deploy's EpisodeLogger
(``episode_*.jsonl`` under a per-model directory) and emits, for an
IEEE-style evaluation:

  * a per-episode summary CSV,
  * an aggregate per-model table (CSV + Markdown + LaTeX tabular), and
  * comparison figures (PNG each, plus one combined multi-page PDF):
      1. outcome breakdown bar chart,
      2. EE -> block distance vs time (mean +/- std),
      3. block lift vs time (mean +/- std, with success line),
      4. robot world-z vs time (balance / collision),
      5. joint jerk distribution (box plot),
      6. EE 6-DOF trajectory grid for a representative success per model.

Usage:
  ros2 run sobits_vla_visualization vla_eval \\
    --logs smolvla:/tmp/vla_logs/smolvla pi05:/tmp/vla_logs/pi05 \\
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
# Loading
# ---------------------------------------------------------------------------

def parse_logs_arg(items: List[str]) -> List[Tuple[str, str]]:
    """Parse ``label:dir`` items into (label, dir) tuples."""
    out: List[Tuple[str, str]] = []
    for item in items:
        if ':' not in item:
            raise ValueError(
                'Expected label:dir, got {!r}. '
                'Example: smolvla:/tmp/vla_logs/smolvla'.format(item)
            )
        label, path = item.split(':', 1)
        out.append((label.strip(), os.path.expanduser(path.strip())))
    return out


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
    # Keep episodes that have a meta or summary even with zero steps — a fast
    # fall can terminate before any step is logged, and dropping those would
    # bias the outcome counts (understating the fall rate).
    if not rows and not meta and not summary:
        return None
    steps = (pd.DataFrame(rows).sort_values('t').reset_index(drop=True)
             if rows else pd.DataFrame())
    return {'path': path, 'meta': meta, 'summary': summary, 'steps': steps}


def _flatten_step(obj: Dict) -> Dict:
    """Flatten the nested step dict into scalar columns for a DataFrame."""
    ee_err = obj.get('ee_error') or {}
    robot = obj.get('robot_pose') or {}
    ee = obj.get('ee_pose') or {}
    return {
        't': obj.get('t'),
        'step': obj.get('step'),
        'ee_dist': ee_err.get('dist'),
        'block_lift': obj.get('block_lift'),
        'robot_z': robot.get('z'),
        'robot_z_drop': obj.get('robot_z_drop'),
        'robot_roll': robot.get('roll'),
        'robot_pitch': robot.get('pitch'),
        'max_jerk': obj.get('max_jerk'),
        'base_speed': obj.get('base_speed'),
        'track_abs_mean': obj.get('track_abs_mean'),
        'ee_x': ee.get('x'), 'ee_y': ee.get('y'), 'ee_z': ee.get('z'),
        'ee_roll': ee.get('roll'), 'ee_pitch': ee.get('pitch'),
        'ee_yaw': ee.get('yaw'),
        'fallen': obj.get('fallen'),
    }


def load_model(label: str, directory: str) -> Dict:
    """Load all episodes for one model directory."""
    files = sorted(glob.glob(os.path.join(directory, 'episode_*.jsonl')))
    episodes: List[Dict] = []
    for path in files:
        ep = load_episode_file(path)
        if ep is not None:
            episodes.append(ep)
    return {'label': label, 'dir': directory, 'episodes': episodes}


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

def per_episode_table(model: Dict) -> pd.DataFrame:
    """One row per episode from the summary lines."""
    recs = []
    for i, ep in enumerate(model['episodes']):
        s = ep['summary']
        outcome = s.get('outcome', 'incomplete')
        recs.append({
            'model': model['label'],
            'episode': ep['meta'].get('episode', i + 1),
            'file': os.path.basename(ep['path']),
            'outcome': outcome,
            'success': bool(s.get('success', outcome == 'success_lift')),
            'duration_s': s.get('duration_s'),
            'total_steps': s.get('total_steps'),
            'time_to_success_s': s.get('time_to_success_s'),
            'min_ee_block_dist': s.get('min_ee_block_dist'),
            'max_block_lift': s.get('max_block_lift'),
            'final_block_lift': s.get('final_block_lift'),
            'max_robot_tilt_deg': s.get('max_robot_tilt_deg'),
            'min_robot_z': s.get('min_robot_z'),
            'joint_path_length': s.get('joint_path_length'),
            'max_joint_jerk': s.get('max_joint_jerk'),
            'mean_base_speed': s.get('mean_base_speed'),
            'mean_abs_tracking_error': s.get('mean_abs_tracking_error'),
        })
    return pd.DataFrame(recs)


def _mean_std(series: pd.Series) -> str:
    s = pd.to_numeric(series, errors='coerce').dropna()
    if s.empty:
        return '--'
    return '{:.3f} ± {:.3f}'.format(s.mean(), s.std(ddof=0))


def aggregate_table(per_ep: pd.DataFrame) -> pd.DataFrame:
    """Aggregate per-model metrics for the comparison table."""
    rows = []
    for model, g in per_ep.groupby('model', sort=False):
        n = len(g)
        outcomes = g['outcome'].value_counts()
        n_success = int(g['success'].sum())
        rows.append({
            'model': model,
            'episodes': n,
            'success_rate_%': round(100.0 * n_success / n, 1) if n else 0.0,
            'fall_rate_%': round(100.0 * outcomes.get('fallen', 0) / n, 1) if n else 0.0,
            'timeout_rate_%': round(100.0 * outcomes.get('timeout', 0) / n, 1) if n else 0.0,
            'time_to_success_s': _mean_std(g.loc[g['success'], 'time_to_success_s']),
            'min_ee_block_dist_m': _mean_std(g['min_ee_block_dist']),
            'max_block_lift_m': _mean_std(g['max_block_lift']),
            'joint_path_length_rad': _mean_std(g['joint_path_length']),
            'max_joint_jerk_rad': _mean_std(g['max_joint_jerk']),
            'tracking_err_rad': _mean_std(g['mean_abs_tracking_error']),
            'base_speed_mps': _mean_std(g['mean_base_speed']),
        })
    return pd.DataFrame(rows).set_index('model')


def write_markdown(df: pd.DataFrame, path: str) -> None:
    cols = list(df.columns)
    lines = ['| model | ' + ' | '.join(cols) + ' |',
             '| --- | ' + ' | '.join(['---'] * len(cols)) + ' |']
    for idx, row in df.iterrows():
        lines.append('| {} | '.format(idx)
                     + ' | '.join(str(row[c]) for c in cols) + ' |')
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
        r'\caption{VLA model comparison.}',
        r'\label{tab:vla}',
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


def plot_band(ax, models: List[Dict], column: str, colors, dt: float = 0.1):
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
        c = colors(mi)
        ax.plot(grid, mean, color=c, linewidth=1.4, label=m['label'])
        ax.fill_between(grid, mean - std, mean + std, color=c, alpha=0.18)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

OUTCOME_ORDER = ['success_lift', 'timeout', 'fallen', 'manual_stop', 'incomplete']
OUTCOME_COLORS = {
    'success_lift': '#2ca02c', 'timeout': '#ff7f0e',
    'fallen': '#d62728', 'manual_stop': '#7f7f7f', 'incomplete': '#bcbd22',
}


def fig_outcomes(per_ep: pd.DataFrame):
    fig, ax = plt.subplots(figsize=(7, 4))
    models = list(per_ep['model'].unique())
    counts = {o: [] for o in OUTCOME_ORDER}
    for m in models:
        g = per_ep[per_ep['model'] == m]
        for o in OUTCOME_ORDER:
            counts[o].append(int((g['outcome'] == o).sum()))
    bottom = np.zeros(len(models))
    x = np.arange(len(models))
    for o in OUTCOME_ORDER:
        vals = np.array(counts[o])
        if vals.sum() == 0:
            continue
        ax.bar(x, vals, bottom=bottom, label=o, color=OUTCOME_COLORS[o])
        bottom += vals
    ax.set_xticks(x)
    ax.set_xticklabels(models)
    ax.set_ylabel('episodes')
    ax.set_title('Episode outcomes by model')
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def fig_timeseries(models, column, ylabel, title, colors, hline=None):
    fig, ax = plt.subplots(figsize=(7, 4))
    plot_band(ax, models, column, colors)
    if hline is not None:
        ax.axhline(hline, color='k', linestyle='--', linewidth=0.8,
                   label='{:.2f} m'.format(hline))
    ax.set_xlabel('time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def fig_jerk(models, colors):
    fig, ax = plt.subplots(figsize=(7, 4))
    data, labels = [], []
    for m in models:
        pooled = []
        for ep in m['episodes']:
            df = ep['steps']
            if df.empty or 'max_jerk' not in df.columns:
                continue
            v = pd.to_numeric(df['max_jerk'], errors='coerce').dropna()
            pooled.extend(v.tolist())
        if pooled:
            data.append(pooled)
            labels.append(m['label'])
    if not data:
        plt.close(fig)
        return None
    bp = ax.boxplot(data, labels=labels, showfliers=False, patch_artist=True)
    for i, box in enumerate(bp['boxes']):
        box.set_facecolor(colors(i))
        box.set_alpha(0.6)
    ax.set_ylabel('per-step max joint jerk (rad)')
    ax.set_title('Motion smoothness — joint jerk distribution')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def _representative_success(model: Dict) -> Optional[Dict]:
    succ = [ep for ep in model['episodes']
            if ep['summary'].get('success')]
    pool = succ if succ else model['episodes']
    # Only episodes that actually have a trajectory to draw.
    pool = [e for e in pool if not e['steps'].empty and 't' in e['steps'].columns]
    if not pool:
        return None
    # Pick the median-duration episode for representativeness.
    pool = sorted(pool, key=lambda e: e['summary'].get('duration_s', 0.0))
    return pool[len(pool) // 2]


def fig_ee_trajectory(models, colors):
    dofs = [('ee_x', 'x (m)'), ('ee_y', 'y (m)'), ('ee_z', 'z (m)'),
            ('ee_roll', 'roll (rad)'), ('ee_pitch', 'pitch (rad)'),
            ('ee_yaw', 'yaw (rad)')]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6))
    axes = axes.ravel()
    plotted = False
    for mi, m in enumerate(models):
        ep = _representative_success(m)
        if ep is None:
            continue
        df = ep['steps']
        t = pd.to_numeric(df['t'], errors='coerce')
        for ax, (col, ylab) in zip(axes, dofs):
            if col in df:
                ax.plot(t, pd.to_numeric(df[col], errors='coerce'),
                        color=colors(mi), linewidth=1.0,
                        label=m['label'], alpha=0.85)
                plotted = True
    if not plotted:
        plt.close(fig)
        return None
    for ax, (_, ylab) in zip(axes, dofs):
        ax.set_xlabel('time (s)')
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    if axes[0].get_legend_handles_labels()[0]:
        axes[0].legend(fontsize=8)
    fig.suptitle('Left EE pose (base_footprint) — representative successful episode')
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--logs', nargs='+', required=True, metavar='label:dir',
        help='One or more model log dirs, e.g. smolvla:/tmp/vla_logs/smolvla',
    )
    parser.add_argument('--out', default='./vla_eval_out',
                        help='Output directory for tables and figures.')
    parser.add_argument('--lift-success', type=float, default=0.05,
                        help='Lift success threshold (m) for the reference line.')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    pairs = parse_logs_arg(args.logs)
    models = [load_model(label, d) for label, d in pairs]
    models = [m for m in models if m['episodes']]
    if not models:
        raise SystemExit('No episodes found in any of: {}'.format(
            [d for _, d in pairs]))
    for m in models:
        print('Loaded {} episodes for {!r} from {}'.format(
            len(m['episodes']), m['label'], m['dir']))

    colors = plt.get_cmap('tab10')

    # ---- Tables -------------------------------------------------------------
    per_ep = pd.concat([per_episode_table(m) for m in models], ignore_index=True)
    per_ep_path = os.path.join(args.out, 'per_episode.csv')
    per_ep.to_csv(per_ep_path, index=False)

    agg = aggregate_table(per_ep)
    agg.to_csv(os.path.join(args.out, 'aggregate.csv'))
    write_markdown(agg, os.path.join(args.out, 'aggregate.md'))
    write_latex(agg, os.path.join(args.out, 'aggregate.tex'))

    # ---- Figures ------------------------------------------------------------
    figs = []
    figs.append(('outcomes', fig_outcomes(per_ep)))
    figs.append(('ee_block_distance', fig_timeseries(
        models, 'ee_dist', 'EE → block distance (m)',
        'End-effector to block distance', colors)))
    figs.append(('block_lift', fig_timeseries(
        models, 'block_lift', 'block lift (m)', 'Block lift over time',
        colors, hline=args.lift_success)))
    figs.append(('robot_z', fig_timeseries(
        models, 'robot_z', 'robot world-z (m)',
        'Robot base height (balance / collision)', colors)))
    figs.append(('joint_jerk', fig_jerk(models, colors)))
    figs.append(('ee_trajectory', fig_ee_trajectory(models, colors)))

    for name, fig in figs:
        if fig is None:
            continue
        fig.savefig(os.path.join(args.out, '{}.png'.format(name)),
                    dpi=args.dpi, bbox_inches='tight')

    pdf_path = os.path.join(args.out, 'vla_comparison.pdf')
    with PdfPages(pdf_path) as pdf:
        for _, fig in figs:
            if fig is not None:
                pdf.savefig(fig, bbox_inches='tight')
    for _, fig in figs:
        if fig is not None:
            plt.close(fig)

    print('\nAggregate comparison:\n')
    print(agg.to_string())
    print('\nWrote tables + figures to {}'.format(os.path.abspath(args.out)))


if __name__ == '__main__':
    main()
