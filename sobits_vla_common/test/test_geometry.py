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
    quat_relative, quat_rotate_vec, quat_shortest_arc, quat_slerp, quat_to_rotvec,
    quat_to_rpy, rotvec_to_quat, rpy_to_quat, slerp_rotvec, unwrap_rpy,
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


def test_quat_shortest_arc_flips_antipodal():
    q_ref = rpy_to_quat(0.1, 0.2, 0.3)
    q_antipodal = tuple(-c for c in q_ref)
    out = quat_shortest_arc(q_antipodal, q_ref)
    for a, b in zip(out, q_ref):
        assert math.isclose(a, b, abs_tol=1e-9)


def test_quat_shortest_arc_identity_when_aligned():
    q_ref = rpy_to_quat(0.1, 0.2, 0.3)
    out = quat_shortest_arc(q_ref, q_ref)
    assert out == q_ref


def test_quat_relative_roundtrip():
    for (rf, pf, yf), (rt, pt, yt) in (
        ((0.0, 0.0, 0.0), (0.3, -0.4, 1.2)),
        ((0.3, -0.4, 1.2), (-1.5, 0.7, -2.9)),
        ((-1.0, 0.5, 2.0), (1.0, -0.5, -2.0)),
    ):
        q_from = rpy_to_quat(rf, pf, yf)
        q_to = rpy_to_quat(rt, pt, yt)
        rel = quat_relative(q_from, q_to)
        composed = _quat_mul(q_from, rel)
        composed = quat_shortest_arc(composed, q_to)
        for a, b in zip(composed, q_to):
            assert math.isclose(a, b, abs_tol=1e-6)


def test_quat_rotate_vec_90deg_about_z():
    q = rpy_to_quat(0.0, 0.0, math.pi / 2.0)
    x, y, z = quat_rotate_vec(q, (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)
    assert math.isclose(z, 0.0, abs_tol=1e-9)


def test_quat_rotate_vec_identity():
    q = rpy_to_quat(0.0, 0.0, 0.0)
    v = (0.3, -0.7, 1.5)
    out = quat_rotate_vec(q, v)
    for a, b in zip(out, v):
        assert math.isclose(a, b, abs_tol=1e-9)


def test_quat_rotate_vec_conjugate_is_inverse_roundtrip():
    for roll, pitch, yaw in (
        (0.3, -0.4, 1.2),
        (-1.5, 0.7, -2.9),
        (0.0, math.pi / 2.0, 0.0),
    ):
        q = rpy_to_quat(roll, pitch, yaw)
        conj = (-q[0], -q[1], -q[2], q[3])
        v = (0.5, -1.2, 2.3)
        rotated = quat_rotate_vec(q, v)
        back = quat_rotate_vec(conj, rotated)
        for a, b in zip(back, v):
            assert math.isclose(a, b, abs_tol=1e-9)


def _quat_mul(q1, q2):
    """Hamilton product q1 (x)(x) q2, both (x, y, z, w) -- test-local, matches rpy_to_quat."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rot_angle_between(a, b) -> float:
    """Geodesic angle between two rotvecs, via scipy as the independent reference."""
    from scipy.spatial.transform import Rotation
    return float((Rotation.from_rotvec(a).inv() * Rotation.from_rotvec(b)).magnitude())


def test_rotvec_quat_roundtrip_matches_scipy():
    from scipy.spatial.transform import Rotation
    for rv in ((0.0, 0.0, 0.0), (0.3, -0.4, 1.2), (3.0, 0.1, -0.2), (1e-9, 0.0, 0.0)):
        q = rotvec_to_quat(*rv)
        qs = Rotation.from_rotvec(rv).as_quat()
        q = quat_shortest_arc(q, qs)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(q, qs))
        back = quat_to_rotvec(*q)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(back, rv))


def test_quat_to_rotvec_angle_in_0_pi():
    q = rotvec_to_quat(0.0, 0.0, 4.0)  # 4 rad about z == -2.283 rad about z
    rv = quat_to_rotvec(*q)
    assert math.isclose(rv[2], 4.0 - 2.0 * math.pi, abs_tol=1e-9)


def test_quat_slerp_endpoints_and_midpoint_match_scipy():
    from scipy.spatial.transform import Rotation, Slerp
    a, b = (0.3, -0.4, 1.2), (-1.5, 0.7, -2.9)
    ref = Slerp([0.0, 1.0], Rotation.from_rotvec([a, b]))
    for t in (0.0, 0.25, 0.5, 1.0):
        got = quat_to_rotvec(*quat_slerp(rotvec_to_quat(*a), rotvec_to_quat(*b), t))
        assert _rot_angle_between(got, ref(t).as_rotvec()) < 1e-9


def test_slerp_rotvec_near_pi_does_not_collapse_to_identity():
    # 3.1 rad and -3.1 rad about x are 0.083 rad apart as rotations; a
    # per-axis average gives 0 (identity), 3.1 rad away from both.
    mid = slerp_rotvec((3.1, 0.0, 0.0), (-3.1, 0.0, 0.0), 0.5)
    assert _rot_angle_between(mid, (3.1, 0.0, 0.0)) < 0.05
    assert _rot_angle_between(mid, (-3.1, 0.0, 0.0)) < 0.05
    assert abs(abs(mid[0]) - math.pi) < 0.05


def test_slerp_rotvec_identical_inputs_is_stable():
    rv = (0.2, 0.1, -0.3)
    for t in (0.0, 0.5, 1.0):
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(slerp_rotvec(rv, rv, t), rv))
