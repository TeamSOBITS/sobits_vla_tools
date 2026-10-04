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
CONTRIBUTING size caps: no function over 80 lines, no file over 600.

An over-cap def or file needs a `refactor-exempt: <reason>` marker (on the
def line or the line above; anywhere in a file's first 80 lines), so every
exception is visible and greppable instead of silently accumulating.
"""

import ast
from pathlib import Path

FUNCTION_CAP = 80
FILE_CAP = 600
MARKER = 'refactor-exempt'
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SKIP_DIRS = {'.git', '.pixi', 'build', 'install', 'log', '__pycache__', '.pytest_cache'}


def _repo_files(suffixes):
    for path in sorted(_REPO_ROOT.rglob('*')):
        if path.suffix not in suffixes or not path.is_file():
            continue
        if _SKIP_DIRS & set(path.relative_to(_REPO_ROOT).parts):
            continue
        yield path


def _over_cap_functions():
    offenders = []
    for path in _repo_files({'.py'}):
        lines = path.read_text().splitlines()
        try:
            tree = ast.parse('\n'.join(lines))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            length = node.end_lineno - node.lineno + 1
            if length <= FUNCTION_CAP:
                continue
            marked = MARKER in lines[node.lineno - 1] or (
                node.lineno >= 2 and MARKER in lines[node.lineno - 2])
            if not marked:
                rel = path.relative_to(_REPO_ROOT)
                offenders.append(f'{rel}:{node.lineno} {node.name} ({length} lines)')
    return offenders


def _over_cap_files():
    offenders = []
    for path in _repo_files({'.py', '.cpp', '.hpp'}):
        lines = path.read_text().splitlines()
        if len(lines) <= FILE_CAP:
            continue
        if not any(MARKER in line for line in lines[:80]):
            offenders.append(f'{path.relative_to(_REPO_ROOT)} ({len(lines)} lines)')
    return offenders


def test_functions_over_80_lines_carry_refactor_exempt():
    assert _over_cap_functions() == []


def test_files_over_600_lines_carry_refactor_exempt():
    assert _over_cap_files() == []
