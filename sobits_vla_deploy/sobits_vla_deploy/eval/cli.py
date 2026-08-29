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
r"""
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
  ros2 run sobits_vla_deploy vla_eval \\
    --logs ball_bowl:<pkg>/logs/smolvla_pnp_ball_bowl \\
           bottle_bin:<pkg>/logs/smolvla_pnp_bottle_bin \\
    --scores /tmp/vla_eval/scores.csv \\
    --out /tmp/vla_eval

--out is optional; it defaults to <sobits_vla_deploy logs root>/eval.
"""

from __future__ import annotations

import argparse
import os

import matplotlib
matplotlib.use('Agg')  # headless rendering
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from sobits_vla_common.output_root import output_root  # noqa: E402
from sobits_vla_deploy.eval.context import EvalContext  # noqa: E402
from sobits_vla_deploy.eval.export import (  # noqa: E402
    attach_scores, write_latex, write_markdown, write_scores_template,
)
from sobits_vla_deploy.eval.figures import FIGURES  # noqa: E402
from sobits_vla_deploy.eval.io import (  # noqa: E402
    derive_sim_scores, load_model, load_scores, parse_logs_arg,
)
from sobits_vla_deploy.eval.metrics import (  # noqa: E402
    aggregate_table, per_episode_table, PICK_ONLY_STAGES,
)
from sobits_vla_deploy.eval.style import apply_style  # noqa: E402
from tqdm import tqdm  # noqa: E402


def _parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--logs', nargs='+', required=True, metavar='label:dir',
        help='One or more task/model log dirs, e.g. '
             'ball_bowl:/tmp/vla_logs/smolvla_pnp_ball_bowl',
    )
    parser.add_argument('--out', default=None,
                        help='Output directory for tables and figures. '
                             'Default: <sobits_vla_deploy logs root>/eval.')
    parser.add_argument('--pick-only', nargs='*', default=[], metavar='LABEL',
                        help='Labels whose task ends at the lift ("Pick up the '
                             'block") and so has 3 stages, not 5. Their '
                             'success rate means "lifted" and is NOT '
                             'comparable with a pick-and-place success rate.')
    parser.add_argument('--arm-groups', nargs='+', default=None, metavar='GROUP',
                        help='Descriptor joint groups counted as the arm for '
                             'tracking error. Default: every group in the '
                             "episode's joint_groups except hand/gripper/head.")
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
    return parser.parse_args()


def _load_models(args, ctx: EvalContext):
    pairs = parse_logs_arg(args.logs)
    pick_only = set(args.pick_only)
    unknown = pick_only - {label for label, _ in pairs}
    if unknown:
        raise SystemExit('--pick-only names unknown label(s): {}. Known: '
                         '{}'.format(sorted(unknown),
                                     [label for label, _ in pairs]))
    models = [load_model(label, d, ctx, label in pick_only) for label, d in pairs]
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
    return models


def _build_per_episode_table(args, models):
    per_ep = pd.concat([per_episode_table(m) for m in models],
                       ignore_index=True)
    scores = load_scores(args.scores)
    per_ep = attach_scores(per_ep, scores)

    # Sim runs have ground truth, so auto-fill stages — never overwrite an
    # operator-supplied score.
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
    return per_ep


def _write_scoring_sheet(args, per_ep):
    needs_scoring = per_ep[per_ep['score'].isna()]
    tmpl_path = args.scores or os.path.join(args.out, 'scores_template.csv')
    if not needs_scoring.empty and not os.path.exists(tmpl_path):
        write_scores_template(needs_scoring, tmpl_path)
        print('\nNo operator scores yet — wrote a blank scoring sheet to\n  {}\n'
              'Fill in the "score" column (stages completed, 1-5; see the '
              'rubric in --help) and '
              're-run with --scores {}'.format(tmpl_path, tmpl_path))


def _aggregate_note(models, n_scored) -> str:
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
    return note


def _render_figures(args, models, per_ep, fmts):
    # Lazy specs so the bar advances per figure actually rendered.
    specs = [
        ('operator_scores', lambda: FIGURES['operator_scores'](per_ep)),
        ('stage_funnel', lambda: FIGURES['stage_funnel'](per_ep)),
        ('outcomes', lambda: FIGURES['outcomes'](per_ep)),
        ('duration', lambda: FIGURES['duration'](per_ep)),
        ('tracking_error', lambda: FIGURES['tracking_error'](
            models, 'track_arm_abs_mean',
            'mean |commanded - measured| (rad)',
            'Arm joint tracking error over time', smooth_s=2.0)),
        ('joint_jerk', lambda: FIGURES['joint_jerk'](models)),
        ('motion_economy', lambda: FIGURES['motion_economy'](per_ep)),
        ('ee_trajectory', lambda: FIGURES['ee_trajectory'](models, per_ep)),
    ]
    figs = [
        (name, build())
        for name, build in tqdm(specs, desc='figures', unit='fig', disable=None)
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


def main() -> None:
    """Load episode logs, write tables and figures, print the summary."""
    args = _parse_args()
    if args.out is None:
        args.out = str(output_root('sobits_vla_deploy', 'logs') / 'eval')

    supported = matplotlib.figure.Figure().canvas.get_supported_filetypes()
    fmts = [f.lower().lstrip('.') for f in args.formats]
    bad = [f for f in fmts if f not in supported]
    if bad:
        raise SystemExit(
            'Unsupported figure format(s): {}. Supported: {}'.format(
                bad, ' '.join(sorted(supported))))

    os.makedirs(args.out, exist_ok=True)
    apply_style()
    # Mirrors the old set_arm_groups: filters falsy items, empty -> None.
    arm_groups = {g for g in (args.arm_groups or []) if g} or None
    ctx = EvalContext(arm_groups=arm_groups)

    models = _load_models(args, ctx)

    per_ep = _build_per_episode_table(args, models)
    per_ep.to_csv(os.path.join(args.out, 'per_episode.csv'), index=False)
    _write_scoring_sheet(args, per_ep)

    n_scored = int(per_ep['score'].notna().sum())
    note = _aggregate_note(models, n_scored)

    agg = aggregate_table(per_ep)
    agg.to_csv(os.path.join(args.out, 'aggregate.csv'))
    write_markdown(agg, os.path.join(args.out, 'aggregate.md'), note)
    write_latex(agg, os.path.join(args.out, 'aggregate.tex'))

    _render_figures(args, models, per_ep, fmts)

    print('\nAggregate comparison ({}/{} episodes scored):\n'.format(
        n_scored, len(per_ep)))
    print(agg.to_string())
    print('\nWrote tables + figures to {}'.format(os.path.abspath(args.out)))


if __name__ == '__main__':
    main()
