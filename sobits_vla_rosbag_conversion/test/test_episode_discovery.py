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


"""Unit tests for episode_discovery -- pure filesystem walking over tmp dirs."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.pipeline.discovery import (  # noqa: E402
    candidate_bag_dirs, discover_episodes, resolve_group_dir,
)


def _make_episode(root: Path, session: str, episode: str) -> None:
    ep_dir = root / session / episode
    ep_dir.mkdir(parents=True)
    (ep_dir / f'{episode}.mcap').write_bytes(b'')
    (ep_dir / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')


def _task(bag_path: str, meta_src: str = '') -> dict:
    info = {'bag_path': bag_path}
    if meta_src:
        info['_meta_source_dir'] = meta_src
    return info


class TestResolveGroupDir:

    def test_relative_bag_path_resolves_under_meta_src(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep1')
        got = resolve_group_dir(str(tmp_path), 'session_a', _task('session_a'), True)
        assert got == str(tmp_path / 'session_a')

    def test_absolute_bag_path_used_verbatim(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep1')
        abs_path = str(tmp_path / 'session_a')
        got = resolve_group_dir('/irrelevant', 'session_a', _task(abs_path), True)
        assert got == abs_path

    def test_missing_dir_without_retry_returns_none(self, tmp_path):
        got = resolve_group_dir(str(tmp_path), 'ghost', _task('ghost'), False)
        assert got is None

    def test_missing_dir_falls_back_to_task_name_with_retry(self, tmp_path):
        # bag_path is stale, but a dir named after the task itself exists.
        _make_episode(tmp_path, 'session_a', 'ep1')
        got = resolve_group_dir(str(tmp_path), 'session_a', _task('stale_path'), True)
        assert got == str(tmp_path / 'session_a')

    def test_missing_dir_with_retry_still_none_if_task_name_also_missing(self, tmp_path):
        got = resolve_group_dir(str(tmp_path), 'ghost', _task('also_missing'), True)
        assert got is None


class TestDiscoverEpisodes:

    def test_lists_episodes_sorted(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep_b')
        _make_episode(tmp_path, 'session_a', 'ep_a')
        all_tasks = [('session_a', _task('session_a'))]
        result = discover_episodes(all_tasks, str(tmp_path))
        assert len(result) == 1
        names = [Path(p).name for p in result[0].episode_dirs]
        assert names == ['ep_a', 'ep_b']

    def test_skips_task_with_unresolvable_group_dir(self, tmp_path):
        all_tasks = [('ghost_task', _task('ghost_task'))]
        result = discover_episodes(all_tasks, str(tmp_path))
        assert result == []

    def test_ignores_non_bag_subdirectories(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep1')
        (tmp_path / 'session_a' / 'not_an_episode').mkdir()
        all_tasks = [('session_a', _task('session_a'))]
        result = discover_episodes(all_tasks, str(tmp_path))
        assert len(result[0].episode_dirs) == 1

    def test_uses_meta_source_dir_per_task(self, tmp_path):
        other_root = tmp_path / 'other'
        _make_episode(other_root, 'session_b', 'ep1')
        all_tasks = [('session_b', _task('session_b', meta_src=str(other_root)))]
        result = discover_episodes(all_tasks, str(tmp_path))
        assert len(result) == 1
        assert result[0].group_dir == str(other_root / 'session_b')

    def test_multiple_tasks_preserve_order(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep1')
        _make_episode(tmp_path, 'session_b', 'ep1')
        all_tasks = [('session_a', _task('session_a')), ('session_b', _task('session_b'))]
        result = discover_episodes(all_tasks, str(tmp_path))
        assert [r.task_name for r in result] == ['session_a', 'session_b']


class TestCandidateBagDirs:

    def test_collects_across_all_tasks(self, tmp_path):
        _make_episode(tmp_path, 'session_a', 'ep1')
        _make_episode(tmp_path, 'session_b', 'ep1')
        all_tasks = [('session_a', _task('session_a')), ('session_b', _task('session_b'))]
        dirs = candidate_bag_dirs(all_tasks, str(tmp_path))
        assert len(dirs) == 2

    def test_no_task_name_retry_unlike_discover_episodes(self, tmp_path):
        # bag_path is stale and no dir named after the task exists either --
        # discover_episodes would find nothing regardless, but this asserts
        # candidate_bag_dirs never attempts the task_name fallback at all.
        _make_episode(tmp_path, 'real_session', 'ep1')
        all_tasks = [('real_session', _task('stale_bag_path'))]
        dirs = candidate_bag_dirs(all_tasks, str(tmp_path))
        assert dirs == []

    def test_empty_when_no_tasks(self, tmp_path):
        assert candidate_bag_dirs([], str(tmp_path)) == []


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
