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

"""Unit tests for the pure-Python quaternion/rotation helpers in geometry.py."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.geometry import (  # noqa: E402
    quat_to_rpy, rpy_to_quat, unwrap_rpy,
)


def test_quat_rpy_roundtrip():
    for roll, pitch, yaw in (
        (0.0, 0.0, 0.0),
        (0.3, -0.4, 1.2),
        (-1.5, 0.7, -2.9),
    ):
        x, y, z, w = rpy_to_quat(roll, pitch, yaw)
        r2, p2, y2 = quat_to_rpy(x, y, z, w)
        assert math.isclose(r2, roll, abs_tol=1e-9)
        assert math.isclose(p2, pitch, abs_tol=1e-9)
        assert math.isclose(y2, yaw, abs_tol=1e-9)


def test_unwrap_rpy_identity_when_within_pi():
    rpy = (0.1, 0.2, 0.3)
    prev = (0.05, 0.15, 0.25)
    assert unwrap_rpy(rpy, prev) == rpy


def test_unwrap_rpy_positive_crossing():
    # prev near +pi, raw sample wrapped to near -pi -> shift up by 2pi.
    prev = (3.0, 0.0, 0.0)
    rpy = (-3.0, 0.0, 0.0)
    out = unwrap_rpy(rpy, prev)
    assert math.isclose(out[0], -3.0 + 2 * math.pi, abs_tol=1e-9)
    assert abs(out[0] - prev[0]) <= math.pi + 1e-9


def test_unwrap_rpy_negative_crossing():
    # prev near -pi, raw sample wrapped to near +pi -> shift down by 2pi.
    prev = (-3.0, 0.0, 0.0)
    rpy = (3.0, 0.0, 0.0)
    out = unwrap_rpy(rpy, prev)
    assert math.isclose(out[0], 3.0 - 2 * math.pi, abs_tol=1e-9)
    assert abs(out[0] - prev[0]) <= math.pi + 1e-9


def test_unwrap_rpy_accumulated_prev_outside_pi_range():
    # prev already accumulated past +pi from earlier unwrap steps.
    prev = (4.5, 0.0, 0.0)
    rpy = (-1.5, 0.0, 0.0)
    out = unwrap_rpy(rpy, prev)
    assert math.isclose(out[0], -1.5 + 2 * math.pi, abs_tol=1e-9)
    assert abs(out[0] - prev[0]) <= math.pi + 1e-9


def test_unwrap_rpy_per_axis_independent():
    prev = (3.0, -3.0, 0.5)
    rpy = (-3.0, 3.0, 0.6)
    out = unwrap_rpy(rpy, prev)
    assert math.isclose(out[0], -3.0 + 2 * math.pi, abs_tol=1e-9)
    assert math.isclose(out[1], 3.0 - 2 * math.pi, abs_tol=1e-9)
    assert math.isclose(out[2], 0.6, abs_tol=1e-9)
