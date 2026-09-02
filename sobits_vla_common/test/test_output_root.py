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

"""Unit tests for output_root: source-tree, colcon-install src/ layout, fallback."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.output_root import output_root  # noqa: E402


def _make_source_tree(tmp_path, package='sobits_vla_demo', dirname='outputs'):
    pkg_dir = tmp_path / package / package
    pkg_dir.mkdir(parents=True)
    (tmp_path / package / 'package.xml').write_text('<package/>')
    (tmp_path / package / dirname).mkdir()
    return pkg_dir / 'anchor.py'


def test_source_tree_layout_shallow(tmp_path):
    anchor = _make_source_tree(tmp_path)
    result = output_root('sobits_vla_demo', 'outputs', anchor_file=str(anchor), recursive=False)
    assert result == tmp_path / 'sobits_vla_demo' / 'outputs'


def test_source_tree_layout_recursive(tmp_path):
    anchor = _make_source_tree(tmp_path)
    result = output_root('sobits_vla_demo', 'outputs', anchor_file=str(anchor), recursive=True)
    assert result == tmp_path / 'sobits_vla_demo' / 'outputs'


def test_workspace_src_layout_shallow(tmp_path):
    # colcon install space: <ws>/install/<pkg>/.../anchor.py, source one repo
    # level under src/ (<ws>/src/<repo>/<pkg>/) -- the shallow glob's "*/<pkg>" shape.
    ws = tmp_path / 'colcon_ws'
    install_anchor = ws / 'install' / 'sobits_vla_demo' / 'lib' / 'python3' / 'anchor.py'
    install_anchor.parent.mkdir(parents=True)
    install_anchor.write_text('')
    src_pkg = ws / 'src' / 'sobits_vla_tools' / 'sobits_vla_demo'
    src_pkg.mkdir(parents=True)
    (src_pkg / 'package.xml').write_text('<package/>')
    (src_pkg / 'outputs').mkdir()

    result = output_root(
        'sobits_vla_demo', 'outputs', anchor_file=str(install_anchor), recursive=False,
    )
    assert result == src_pkg / 'outputs'


def test_workspace_src_layout_recursive_deep_nesting(tmp_path):
    # A repo checkout deeper than the shallow "*/<pkg>" or "*/*/<pkg>" glob
    # reaches (<ws>/src/<repo>/<group>/<subgroup>/<pkg>) -- only found with
    # recursive=True's full rglob('package.xml') walk.
    ws = tmp_path / 'colcon_ws'
    install_anchor = ws / 'install' / 'sobits_vla_demo' / 'lib' / 'python3' / 'x' / 'anchor.py'
    install_anchor.parent.mkdir(parents=True)
    install_anchor.write_text('')
    src_pkg = ws / 'src' / 'repo' / 'group' / 'subgroup' / 'sobits_vla_demo'
    src_pkg.mkdir(parents=True)
    (src_pkg / 'package.xml').write_text('<package/>')
    (src_pkg / 'outputs').mkdir()

    result = output_root(
        'sobits_vla_demo', 'outputs', anchor_file=str(install_anchor), recursive=True,
    )
    assert result == src_pkg / 'outputs'

    # Shallow mode's fixed-depth glob doesn't reach this nesting; it falls
    # through to the fallback instead of finding src_pkg.
    fallback_hits = []
    shallow_result = output_root(
        'sobits_vla_demo', 'outputs', anchor_file=str(install_anchor), recursive=False,
        final_fallback=lambda: fallback_hits.append(True) or (tmp_path / 'fallback'),
    )
    assert fallback_hits == [True]
    assert shallow_result == tmp_path / 'fallback'


def test_falls_back_when_nothing_found(tmp_path):
    anchor = tmp_path / 'isolated' / 'anchor.py'
    anchor.parent.mkdir(parents=True)
    anchor.write_text('')

    result = output_root(
        'sobits_vla_demo', 'outputs', anchor_file=str(anchor),
        final_fallback=lambda: tmp_path / 'fallback_root',
    )
    assert result == tmp_path / 'fallback_root'


def test_default_final_fallback_uses_ament_index(tmp_path, monkeypatch):
    anchor = tmp_path / 'isolated' / 'anchor.py'
    anchor.parent.mkdir(parents=True)
    anchor.write_text('')

    calls = []

    def _fake_get_package_share_directory(name):
        calls.append(name)
        return str(tmp_path / 'share' / name)

    import ament_index_python.packages as ament_pkg
    monkeypatch.setattr(
        ament_pkg, 'get_package_share_directory', _fake_get_package_share_directory,
    )

    result = output_root('sobits_vla_demo', 'outputs', anchor_file=str(anchor))
    assert calls == ['sobits_vla_demo']
    assert result == tmp_path / 'share' / 'sobits_vla_demo' / 'outputs'


class TestInstallShareSkip:

    def test_share_dir_with_installed_gitignore_is_not_a_direct_hit(self, tmp_path):
        # Reproduces the live bug: install/<pkg>/share/<pkg> holds package.xml
        # and <dirname>/ (the installed .gitignore), shadowing the source tree.
        ws = tmp_path
        share_pkg = ws / 'install' / 'pkg_a' / 'share' / 'pkg_a'
        (share_pkg / 'logs').mkdir(parents=True)
        (share_pkg / 'package.xml').write_text('<package/>')
        src_pkg = ws / 'src' / 'repo' / 'pkg_a'
        (src_pkg / 'logs').mkdir(parents=True)
        (src_pkg / 'package.xml').write_text('<package/>')
        anchor = share_pkg / 'launch' / 'f.launch.py'
        anchor.parent.mkdir()
        anchor.write_text('')
        result = output_root('pkg_a', 'logs', anchor_file=str(anchor))
        assert result == src_pkg / 'logs'

    def test_colcon_ignored_sibling_copy_never_shadows(self, tmp_path):
        # A backup copy with COLCON_IGNORE must lose to the real package even
        # when rglob encounters it first.
        ws = tmp_path
        for repo, ignored in (('aaa_backup', True), ('repo', False)):
            pkg = ws / 'src' / repo / 'pkg_a'
            (pkg / 'logs').mkdir(parents=True)
            (pkg / 'package.xml').write_text('<package/>')
            if ignored:
                (ws / 'src' / repo / 'COLCON_IGNORE').write_text('')
        anchor = ws / 'install' / 'x' / 'share' / 'x' / 'f.py'
        anchor.parent.mkdir(parents=True)
        anchor.write_text('')
        result = output_root('pkg_a', 'logs', anchor_file=str(anchor))
        assert result == ws / 'src' / 'repo' / 'pkg_a' / 'logs'
