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

"""Figure: joint path length vs duration — motion economy."""

from __future__ import annotations

import matplotlib.pyplot as plt
import pandas as pd

from sobits_vla_deploy.eval.style import SERIES_COLORS, SURFACE


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
