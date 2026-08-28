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
Unit tests for gz_utils: Pose_V text parsing and quat_to_rpy, no gz binary required.

gz_get_pose_dynamic shells out to `gz topic`; subprocess.run is monkeypatched with a
captured Pose_V fixture so the parser is exercised without a running simulator.
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common import gz_utils  # noqa: E402


# Captured `gz topic -e -t /world/<w>/dynamic_pose/info -n 1` output shape.
_POSE_V_FIXTURE = """header {
  stamp {
    sec: 123
    nsec: 456000000
  }
}
pose {
  name: "sobit_home"
  id: 8
  position {
    x: 2.001
    y: -1.498
    z: 0.0
  }
  orientation {
    x: 0.0
    y: 0.0
    z: 0.7071068
    w: 0.7071068
  }
}
pose {
  name: "box_to_pick"
  id: 12
  position {
    x: 1.97
    y: -0.52
    z: 0.451
  }
  orientation {
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  }
}
"""


class _FakeCompletedProcess:

    def __init__(self, stdout, returncode=0):
        self.stdout = stdout
        self.returncode = returncode


def test_gz_get_pose_dynamic_parses_matching_block(monkeypatch):
    monkeypatch.setattr(
        subprocess, 'run',
        lambda *a, **k: _FakeCompletedProcess(_POSE_V_FIXTURE),
    )
    pose = gz_utils.gz_get_pose_dynamic('simple_data_collection', 'box_to_pick')
    assert pose is not None
    assert math.isclose(pose['x'], 1.97, abs_tol=1e-6)
    assert math.isclose(pose['y'], -0.52, abs_tol=1e-6)
    assert math.isclose(pose['z'], 0.451, abs_tol=1e-6)


def test_gz_get_pose_dynamic_converts_quaternion(monkeypatch):
    monkeypatch.setattr(
        subprocess, 'run',
        lambda *a, **k: _FakeCompletedProcess(_POSE_V_FIXTURE),
    )
    pose = gz_utils.gz_get_pose_dynamic('simple_data_collection', 'sobit_home')
    assert pose is not None
    assert math.isclose(pose['yaw'], math.pi / 2, abs_tol=1e-3)
    assert math.isclose(pose['roll'], 0.0, abs_tol=1e-6)
    assert math.isclose(pose['pitch'], 0.0, abs_tol=1e-6)


def test_gz_get_pose_dynamic_missing_model_returns_none(monkeypatch):
    monkeypatch.setattr(
        subprocess, 'run',
        lambda *a, **k: _FakeCompletedProcess(_POSE_V_FIXTURE),
    )
    assert gz_utils.gz_get_pose_dynamic('simple_data_collection', 'no_such_model') is None


def test_gz_get_pose_dynamic_nonzero_exit_returns_none(monkeypatch):
    monkeypatch.setattr(
        subprocess, 'run',
        lambda *a, **k: _FakeCompletedProcess('', returncode=1),
    )
    assert gz_utils.gz_get_pose_dynamic('simple_data_collection', 'sobit_home') is None


def test_gz_get_pose_dynamic_subprocess_exception_returns_none(monkeypatch):
    def _raise(*_a, **_k):
        raise subprocess.TimeoutExpired(cmd='gz', timeout=1.0)

    monkeypatch.setattr(subprocess, 'run', _raise)
    assert gz_utils.gz_get_pose_dynamic('simple_data_collection', 'sobit_home') is None


class TestQuatToRpy:

    def test_identity_quaternion(self):
        roll, pitch, yaw = gz_utils.quat_to_rpy(0.0, 0.0, 0.0, 1.0)
        assert math.isclose(roll, 0.0, abs_tol=1e-9)
        assert math.isclose(pitch, 0.0, abs_tol=1e-9)
        assert math.isclose(yaw, 0.0, abs_tol=1e-9)

    def test_90deg_yaw(self):
        # qz = qw = sqrt(2)/2 -> 90 deg rotation about Z.
        s = math.sqrt(2.0) / 2.0
        roll, pitch, yaw = gz_utils.quat_to_rpy(0.0, 0.0, s, s)
        assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)
        assert math.isclose(roll, 0.0, abs_tol=1e-6)
        assert math.isclose(pitch, 0.0, abs_tol=1e-6)

    def test_180deg_yaw(self):
        roll, pitch, yaw = gz_utils.quat_to_rpy(0.0, 0.0, 1.0, 0.0)
        assert math.isclose(abs(yaw), math.pi, abs_tol=1e-6)

    def test_90deg_roll(self):
        s = math.sqrt(2.0) / 2.0
        roll, pitch, yaw = gz_utils.quat_to_rpy(s, 0.0, 0.0, s)
        assert math.isclose(roll, math.pi / 2, abs_tol=1e-6)
        assert math.isclose(pitch, 0.0, abs_tol=1e-6)
        assert math.isclose(yaw, 0.0, abs_tol=1e-6)


class TestWrapPi:

    def test_already_in_range(self):
        assert math.isclose(gz_utils.wrap_pi(1.0), 1.0, abs_tol=1e-9)

    def test_wraps_above_pi(self):
        wrapped = gz_utils.wrap_pi(math.pi + 0.1)
        assert -math.pi <= wrapped <= math.pi
        assert math.isclose(wrapped, -math.pi + 0.1, abs_tol=1e-6)

    def test_wraps_below_negative_pi(self):
        wrapped = gz_utils.wrap_pi(-math.pi - 0.1)
        assert -math.pi <= wrapped <= math.pi
        assert math.isclose(wrapped, math.pi - 0.1, abs_tol=1e-6)


class TestGzSetPose:

    def test_success_requires_data_true_in_stdout(self, monkeypatch):
        monkeypatch.setattr(
            subprocess, 'run',
            lambda *a, **k: _FakeCompletedProcess('data: true\n', returncode=0),
        )
        assert gz_utils.gz_set_pose('w', 'm', 0, 0, 0, 0, 0, 0, 1) is True

    def test_zero_exit_without_data_true_is_failure(self, monkeypatch):
        # Exit code is unreliable -- only the Boolean reply on stdout counts.
        monkeypatch.setattr(
            subprocess, 'run',
            lambda *a, **k: _FakeCompletedProcess('data: false\n', returncode=0),
        )
        assert gz_utils.gz_set_pose('w', 'm', 0, 0, 0, 0, 0, 0, 1) is False

    def test_subprocess_exception_is_failure(self, monkeypatch):
        def _raise(*_a, **_k):
            raise subprocess.TimeoutExpired(cmd='gz', timeout=1.0)

        monkeypatch.setattr(subprocess, 'run', _raise)
        assert gz_utils.gz_set_pose('w', 'm', 0, 0, 0, 0, 0, 0, 1) is False
