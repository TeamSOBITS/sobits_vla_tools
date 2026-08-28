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

"""Figure: operator score distribution (1-5) per task — the headline result."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.style import (
    _on_fill, _set_hatch_color, INK, SCORE_COLORS, SCORE_HATCH, SCORE_LABELS,
    SERIES_COLORS, SURFACE,
)


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
