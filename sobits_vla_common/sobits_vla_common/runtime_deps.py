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

"""
Fail-fast runtime dependency checks shared by conversion/training/deploy entry points.

lerobot 0.6.0's slim base install (`pip install lerobot` with no extras)
drops dataset/training/evaluation deps entirely — the #1 foreseeable field
failure after the port. Error messages here always name the extra so the
fix is a copy-pasteable pip command, not a guessing game.
"""

from __future__ import annotations

import importlib
import sys


def ensure(pkgs: dict[str, str]) -> None:
    """
    Fail fast with a clear message when required Python packages are missing.

    Parameters
    ----------
    pkgs : dict[str, str]
        Maps an importable module name (e.g. ``'lerobot'``, ``'rosbags'``) to
        a human-readable install hint (e.g.
        ``'pip install lerobot[training]~=0.6.0'``). The hint should name the
        concrete extra needed, not just the bare package name.

    Raises
    ------
    RuntimeError
        If any module in ``pkgs`` fails to import. Lists every missing
        module and its hint in one message (not just the first failure).

    """
    missing: list[str] = []
    for module_name, hint in pkgs.items():
        try:
            importlib.import_module(module_name)
        except Exception:
            missing.append(f'{module_name} ({hint})')

    if missing:
        missing_str = '; '.join(missing)
        raise RuntimeError(
            'Missing required Python runtime dependencies: '
            f'{missing_str}. '
            f'Current interpreter: {sys.executable}. '
            'Install missing packages in this same environment.'
        )
