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

"""Figure: end-effector 6-DOF trajectory grid for a representative episode."""

from __future__ import annotations

import matplotlib.pyplot as plt
import pandas as pd

from sobits_vla_deploy.eval.metrics import _representative_episode
from sobits_vla_deploy.eval.style import INK, SERIES_COLORS


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
