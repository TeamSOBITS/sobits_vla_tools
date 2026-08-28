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

"""Figure: pick-and-place stage funnel (share of episodes clearing each stage)."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.metrics import PICK_ONLY_STAGES, STAGE_LABELS
from sobits_vla_deploy.eval.style import INK_SOFT, NEUTRAL, SERIES_COLORS, SURFACE


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
        # Mark every missing stage explicitly — a blank bar next to real ones
        # would misread as "failed here" instead of "never attempted".
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
