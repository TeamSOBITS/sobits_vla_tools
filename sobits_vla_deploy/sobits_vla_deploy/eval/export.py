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

"""Table/score-sheet writers: Markdown, LaTeX, operator scoring CSV."""

from __future__ import annotations

from typing import Optional

import numpy as np
import pandas as pd


def write_scores_template(per_ep: pd.DataFrame, path: str) -> None:
    """Write an empty scoring sheet for the operator to fill in."""
    tmpl = per_ep[['model', 'episode', 'file']].copy()
    tmpl['score'] = ''
    tmpl['note'] = ''
    tmpl.to_csv(path, index=False)


def attach_scores(per_ep: pd.DataFrame,
                  scores: Optional[pd.Series]) -> pd.DataFrame:
    """Join operator scores onto the per-episode table."""
    per_ep = per_ep.copy()
    if scores is None:
        per_ep['score'] = np.nan
        return per_ep
    by_file = scores.index.names[1] == 'file'
    keys = list(zip(per_ep['model'],
                    per_ep['file'] if by_file else per_ep['episode']))
    per_ep['score'] = [scores.get(k, np.nan) for k in keys]
    return per_ep


def write_markdown(df: pd.DataFrame, path: str, note: str = '') -> None:
    """Write the aggregate table as Markdown, with an optional caveat note."""
    cols = list(df.columns)
    lines = ['| model | ' + ' | '.join(cols) + ' |',
             '| --- | ' + ' | '.join(['---'] * len(cols)) + ' |']
    for idx, row in df.iterrows():
        lines.append('| {} | '.format(idx)
                     + ' | '.join(str(row[c]) for c in cols) + ' |')
    if note:
        lines += ['', note]
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')


def _tex_escape(s: str) -> str:
    for a, b in (('\\', r'\textbackslash{}'), ('_', r'\_'), ('%', r'\%'),
                 ('&', r'\&'), ('#', r'\#'), ('±', r'$\pm$')):
        s = s.replace(a, b)
    return s


def write_latex(df: pd.DataFrame, path: str) -> None:
    """Write a booktabs-style LaTeX tabular (no jinja2 / Styler dependency)."""
    cols = list(df.columns)
    align = 'l' + 'r' * len(cols)
    out = [
        r'\begin{table}[t]',
        r'\centering',
        r'\caption{Real-robot VLA evaluation. The operator score is the number '
        r'of cumulative pick-and-place stages completed (reach, grasp, lift, '
        r'move, place); success requires all five. The deploy node has no '
        r'ground-truth observer on hardware.}',
        r'\label{tab:vla_real}',
        r'\begin{tabular}{' + align + '}',
        r'\toprule',
        'model & ' + ' & '.join(_tex_escape(c) for c in cols) + r' \\',
        r'\midrule',
    ]
    for idx, row in df.iterrows():
        out.append(_tex_escape(str(idx)) + ' & '
                   + ' & '.join(_tex_escape(str(row[c])) for c in cols) + r' \\')
    out += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
    with open(path, 'w') as fh:
        fh.write('\n'.join(out) + '\n')
