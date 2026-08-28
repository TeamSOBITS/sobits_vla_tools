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

"""Figure: per-step max joint jerk distribution — motion smoothness."""

from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

from sobits_vla_deploy.eval.style import INK, NEUTRAL, SERIES_COLORS


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
    # 'labels' was renamed 'tick_labels' in matplotlib 3.9 and removed in 3.11.
    label_kw = ('tick_labels'
                if tuple(int(p) for p in matplotlib.__version__.split('.')[:2]) >= (3, 9)
                else 'labels')
    bp = ax.boxplot(data, showfliers=False, patch_artist=True, widths=0.4,
                    medianprops={'color': INK, 'linewidth': 2},
                    **{label_kw: labels})
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
