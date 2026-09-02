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

"""Unit tests for launch.utils.config_declares -- flat and nested key lookup."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.launch.utils import config_declares  # noqa: E402


def _write(tmp_path, body):
    p = tmp_path / 'cfg.yaml'
    p.write_text('/**:\n  ros__parameters:\n' + body)
    return str(p)


class TestConfigDeclares:

    def test_flat_key_set(self, tmp_path):
        cfg = _write(tmp_path, '    rosbag_directory: "/data/bags"\n')
        assert config_declares(cfg, 'rosbag_directory') is True

    def test_flat_key_empty_is_undeclared(self, tmp_path):
        cfg = _write(tmp_path, '    rosbag_directory: ""\n')
        assert config_declares(cfg, 'rosbag_directory') is False

    def test_nested_dotted_key_set(self, tmp_path):
        cfg = _write(tmp_path, '    rosbag_config:\n      record_directory: "/data"\n')
        assert config_declares(cfg, 'rosbag_config.record_directory') is True

    def test_nested_dotted_key_empty(self, tmp_path):
        cfg = _write(tmp_path, '    rosbag_config:\n      record_directory: ""\n')
        assert config_declares(cfg, 'rosbag_config.record_directory') is False

    def test_missing_key(self, tmp_path):
        cfg = _write(tmp_path, '    other: 1\n')
        assert config_declares(cfg, 'rosbag_directory') is False

    def test_unreadable_file_returns_false(self, tmp_path):
        assert config_declares(str(tmp_path / 'missing.yaml'), 'k') is False
