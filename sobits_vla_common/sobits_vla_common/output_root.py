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
Shared resolver for "find <pkg_src>/<dirname>" output roots.

One implementation of the source-tree / colcon-install / share-dir walk-up
that used to be triplicated across conversion, training, and launch utils.
"""

from pathlib import Path
from typing import Callable, Optional


def _colcon_ignored(pkg_dir, src_root) -> bool:
    """
    Return True when any ancestor up to src_root carries a COLCON_IGNORE marker.

    Excludes trees colcon itself skips (e.g. a read-only backup copy of the
    repo) so they can never shadow the real package in resolution.
    """
    node = pkg_dir
    while node != src_root and node != node.parent:
        if (node / 'COLCON_IGNORE').exists():
            return True
        node = node.parent
    return False


def output_root(
    package: str,
    dirname: str,
    anchor_file: Optional[str] = None,
    *,
    recursive: bool = True,
    final_fallback: Optional[Callable[[], Path]] = None,
) -> Path:
    """
    Resolve <package_src>/<dirname>, walking up from anchor_file (default: this file).

    Works from the colcon install space by walking up to the workspace root
    and locating the package under src/, and from source/symlink-install runs
    by finding the package root directly. recursive=False restricts the src/
    search to the shallow */<package> and */*/<package> glob patterns used
    historically by the conversion package; recursive=True (default) walks
    the whole src/ tree, matching training's and the launch helpers' prior
    behavior. final_fallback overrides the last-resort ament-index share-dir
    lookup for callers (training) that had a different last resort.
    """
    start = Path(anchor_file).resolve() if anchor_file else Path(__file__).resolve()
    candidate = start.parent

    for _ in range(8):
        # share/<pkg> in an install space carries package.xml AND the output
        # dir (its installed .gitignore) -- a false direct hit; skip it so
        # resolution keeps walking to the real source tree.
        in_share = candidate.parent.name == 'share'
        if (not in_share and (candidate / 'package.xml').exists()
                and (candidate / dirname).is_dir()):
            return candidate / dirname

        src_root = candidate / 'src'
        if src_root.is_dir():
            if recursive:
                for path in src_root.rglob('package.xml'):
                    if (path.parent.name == package and (path.parent / dirname).is_dir()
                            and not _colcon_ignored(path.parent, src_root)):
                        return path.parent / dirname
            else:
                for pattern in (f'*/{package}', f'*/*/{package}'):
                    for pkg_dir in src_root.glob(pattern):
                        if ((pkg_dir / dirname).is_dir()
                                and not _colcon_ignored(pkg_dir, src_root)):
                            return pkg_dir / dirname

        candidate = candidate.parent

    if final_fallback is not None:
        return final_fallback()

    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package)) / dirname
