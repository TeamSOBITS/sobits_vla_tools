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

"""Figure: episode termination reason breakdown."""

from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.style import (
    _on_fill, OUTCOME_COLORS, OUTCOME_LABELS, OUTCOME_ORDER, SURFACE,
)


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
