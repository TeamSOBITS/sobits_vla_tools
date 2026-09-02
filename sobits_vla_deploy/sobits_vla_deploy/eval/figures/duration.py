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

"""Figure: episode duration per task — strip plot over a mean marker."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.style import INK, SERIES_COLORS, SURFACE


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
