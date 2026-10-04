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

"""Unwrap of ee.<arm>.{roll,pitch,yaw} before blending in the chunk buffer and interpolator."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.action_interpolator import ActionInterpolator  # noqa: E402
from sobits_vla_deploy.deploy_node import ActionChunkBuffer  # noqa: E402


def _rotvec(prefix, rx=0.0, ry=0.0, rz=0.0):
    return {f'{prefix}.rx': rx, f'{prefix}.ry': ry, f'{prefix}.rz': rz}


class TestActionChunkBufferAngleUnwrap:

    def test_ee_angle_key_unwraps_before_blending(self):
        # old 3.1, new -3.1: naive average is ~0, but the values are close
        # across the +-pi seam and should blend to ~+-pi instead.
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([{'ee.left.roll': 3.1}], overlap=0)
        buf.merge([{'ee.left.roll': -3.1}], overlap=1)
        step = buf.pop()
        assert step is not None
        assert abs(abs(step['ee.left.roll']) - math.pi) < 0.05

    def test_non_angle_key_unaffected_by_unwrap(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([{'ee.left.x': 3.1}], overlap=0)
        buf.merge([{'ee.left.x': -3.1}], overlap=1)
        step = buf.pop()
        assert step is not None
        assert abs(step['ee.left.x'] - 0.0) < 1e-9  # plain average, no unwrap

    def test_ee_rotvec_group_blends_on_so3(self):
        # rx 3.1 and -3.1 are 0.08 rad apart as rotations; a per-axis average
        # would give identity, 3.1 rad from both.
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([_rotvec('ee.left', rx=3.1)], overlap=0)
        buf.merge([_rotvec('ee.left', rx=-3.1)], overlap=1)
        step = buf.pop()
        assert step is not None
        assert abs(abs(step['ee.left.rx']) - math.pi) < 0.05
        assert abs(step['ee.left.ry']) < 1e-9 and abs(step['ee.left.rz']) < 1e-9

    def test_partial_rotvec_group_falls_back_to_per_axis(self):
        buf = ActionChunkBuffer('weighted_average')
        buf.merge([{'ee.left.rx': 0.2}], overlap=0)
        buf.merge([{'ee.left.rx': 0.4}], overlap=1)
        assert buf.pop()['ee.left.rx'] == pytest.approx(0.3)

    def test_newest_takes_rotvec_as_is(self):
        buf = ActionChunkBuffer('newest')
        buf.merge([_rotvec('ee.left', rx=3.1)], overlap=0)
        buf.merge([_rotvec('ee.left', rx=-3.1)], overlap=1)
        assert buf.pop()['ee.left.rx'] == pytest.approx(-3.1)


class TestActionInterpolatorAngleUnwrap:

    def test_ee_angle_key_unwraps_before_interpolating(self):
        # old 3.1, new -3.1: interpolated path should stay near +-pi, not
        # sweep back through 0.
        interp = ActionInterpolator(4)
        interp.add({'ee.left.roll': 3.1})
        interp.add({'ee.left.roll': -3.1})
        values = [interp.get()['ee.left.roll'] for _ in range(4)]
        assert all(abs(v) > math.pi - 0.3 for v in values)

    def test_non_angle_key_unaffected_by_unwrap(self):
        interp = ActionInterpolator(2)
        interp.add({'ee.left.x': 3.1})
        interp.add({'ee.left.x': -3.1})
        values = [interp.get()['ee.left.x'] for _ in range(2)]
        assert values == [pytest.approx((3.1 + -3.1) / 2), pytest.approx(-3.1)]

    def test_rotvec_group_interpolates_on_so3(self):
        interp = ActionInterpolator(4)
        interp.add(_rotvec('ee.left', rx=3.1))
        interp.add(_rotvec('ee.left', rx=-3.1))
        values = [interp.get()['ee.left.rx'] for _ in range(4)]
        assert all(abs(v) > math.pi - 0.3 for v in values)
        assert values[-1] == pytest.approx(-3.1)
