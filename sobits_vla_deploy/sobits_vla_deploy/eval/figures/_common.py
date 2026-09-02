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

"""Shared plotting helper: mean +/- std band on a common time grid."""

from __future__ import annotations

from typing import Dict, List
import warnings

import numpy as np
import pandas as pd

from sobits_vla_deploy.eval.metrics import _smooth, resample
from sobits_vla_deploy.eval.style import SERIES_COLORS


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
