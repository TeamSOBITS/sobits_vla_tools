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

"""Palette constants and matplotlib rcParams shared by every figure module."""

from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt

# --- Style — validated palette (see sobits_vla_visualization docs) ---

# Categorical slots 1-2 of the reference theme; validated all-pairs on the
# light surface (worst CVD dE 24.7, both >= 3:1 contrast) for CVD/greyscale.
SERIES_COLORS = ['#2a78d6', '#eb6834', '#1baf7a']

# Ordinal ramp: ONE hue, light->dark (score is ordered). Alternating segments
# also hatched — hue alone can't separate 5 steps (adjacent dE ~9.8-10.1).
SCORE_COLORS = ['#86b6ef', '#5598e7', '#2a78d6', '#1c5cab', '#0d366b']
SCORE_HATCH = ['', '///', '', '///', '']
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SOFT = '#52514e'
GRID = '#e6e5e1'
NEUTRAL = '#9a9892'

SCORE_LABELS = {
    1: '1 reached over object',
    2: '2 grasped',
    3: '3 lifted',
    4: '4 moved to destination',
    5: '5 placed',
}

# Termination reasons the logger can actually emit on the real robot.
OUTCOME_ORDER = ['manual_stop', 'timeout', 'fallen', 'success_lift', 'incomplete']
OUTCOME_LABELS = {
    'manual_stop': 'operator stop',
    'timeout': 'timeout',
    'fallen': 'fallen',
    'success_lift': 'auto success',
    'incomplete': 'incomplete',
}
# Termination reason is a nominal state, not a quality ranking: keep it in one
# hue + neutrals so it is never mistaken for the score encoding.
OUTCOME_COLORS = {
    'manual_stop': '#2a78d6',
    'timeout': '#86b6ef',
    'fallen': '#e34948',
    'success_lift': '#1baf7a',
    'incomplete': '#c9c7c0',
}


def _set_hatch_color(patch, color: str, alpha: float = 0.55) -> None:
    """
    Tint one patch's hatch, keeping its surface-coloured edge.

    ``set_hatch_color`` only exists from matplotlib 3.10; on older versions the
    private ``_hatch_color`` is the sole per-patch control (the ``hatch.color``
    rcParam is global and read at draw time, so it cannot vary per segment).
    Falls back to leaving the default hatch colour rather than raising.
    """
    rgba = matplotlib.colors.to_rgba(color, alpha)
    setter = getattr(patch, 'set_hatch_color', None)
    if callable(setter):
        setter(rgba)
    elif hasattr(patch, '_hatch_color'):
        patch._hatch_color = rgba


def _on_fill(hex_color: str) -> str:
    """Ink or white for a label set inside a coloured fill, by luminance."""
    h = hex_color.lstrip('#')
    r, g, b = (int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))
    lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
           for c in (r, g, b)]
    lum = 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2]
    return INK if lum > 0.42 else '#ffffff'


def apply_style() -> None:
    """Recessive axes, hairline solid grid, ink-token text."""
    plt.rcParams.update({
        'figure.facecolor': SURFACE,
        'axes.facecolor': SURFACE,
        'savefig.facecolor': SURFACE,
        'axes.edgecolor': GRID,
        'axes.labelcolor': INK_SOFT,
        'axes.titlecolor': INK,
        'axes.titlesize': 11,
        'axes.titleweight': 'semibold',
        'axes.titlelocation': 'left',
        'axes.labelsize': 9,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'xtick.color': INK_SOFT,
        'ytick.color': INK_SOFT,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'grid.color': GRID,
        'grid.linewidth': 0.8,
        'grid.linestyle': '-',
        'legend.frameon': False,
        'legend.fontsize': 9,
        'lines.linewidth': 2.0,
        'lines.solid_capstyle': 'round',
        'font.size': 10,
        # Keep text as text in SVG/PDF (selectable, searchable, re-styleable)
        # instead of converting glyphs to paths.
        'svg.fonttype': 'none',
        'pdf.fonttype': 42,
    })
