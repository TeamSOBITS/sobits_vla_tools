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

import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.robot_descriptor import EEControlSpec  # noqa: E402
from sobits_vla_deploy.servo_target_publisher import ServoTargetPublisher  # noqa: E402


class _FakeBroadcaster:
    def __init__(self):
        self.sent = []

    def sendTransform(self, t):
        self.sent.append(t)


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg.data)


def _ee_axes(prefix, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return {
        f'{prefix}.x': x, f'{prefix}.y': y, f'{prefix}.z': z,
        f'{prefix}.roll': roll, f'{prefix}.pitch': pitch, f'{prefix}.yaw': yaw,
    }


def _make_servo_publisher(
    max_lin_step_m=0.03, max_ang_step_rad=0.15, arms=None, rotation='rpy',
    max_lag_m=0.10, max_lag_rad=0.5,
):
    if arms is None:
        arms = [EEControlSpec(
            ee_pose='left', group='arm_left',
            target_frame='left_target_link',
            enable_topic='arm_left/moveit_track_enabled',
        )]
    broadcaster = _FakeBroadcaster()
    publishers = {spec.ee_pose: _FakePublisher() for spec in arms}
    pub = ServoTargetPublisher(
        arms=arms,
        parent_frame='base_footprint',
        tf_broadcaster=broadcaster,
        enable_publishers=publishers,
        max_lin_step_m=max_lin_step_m,
        max_ang_step_rad=max_ang_step_rad,
        max_lag_m=max_lag_m,
        max_lag_rad=max_lag_rad,
        rotation=rotation,
    )
    return pub, broadcaster, publishers


def _rotvec_axes(prefix, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
    return {
        f'{prefix}.x': x, f'{prefix}.y': y, f'{prefix}.z': z,
        f'{prefix}.rx': rx, f'{prefix}.ry': ry, f'{prefix}.rz': rz,
    }


class TestServoTargetPublisher:

    def test_engage_seeds_from_state_vector_and_latches_true(self):
        pub, _bc, publishers = _make_servo_publisher()
        state = {'ee.left.' + a: v for a, v in zip(
            ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], [0.5, 0.1, 0.3, 0.0, 0.0, 1.0],
        )}
        assert pub.engage(state) is True
        assert pub.engaged
        assert publishers['left'].published == [True]
        assert pub._last_target['left'] == [0.5, 0.1, 0.3, 0.0, 0.0, 1.0]

    def test_engage_skips_arm_with_missing_keys(self):
        pub, _bc, publishers = _make_servo_publisher()
        assert pub.engage({}) is False  # no ee.left.* keys at all
        assert not pub.engaged
        assert 'left' not in pub._last_target
        assert publishers['left'].published == []

    def test_not_engaged_publish_step_is_noop(self):
        pub, bc, _publishers = _make_servo_publisher()
        step = _ee_axes('ee.left', x=1.0)
        pub.publish_step(step, now_msg='t0')
        assert bc.sent == []

    def test_publish_step_missing_key_arm_skipped(self):
        pub, bc, _publishers = _make_servo_publisher()
        pub.engage(_ee_axes('ee.left', z=0.5))
        pub.publish_step({'ee.left.x': 1.0}, now_msg='t0')  # missing other 5 keys
        assert bc.sent == []

    def test_engage_all_zero_state_leaves_arm_disabled(self):
        pub, bc, publishers = _make_servo_publisher()
        # All-zero state: nothing seeded, so engage must report False too.
        assert pub.engage(_ee_axes('ee.left')) is False
        assert not pub.engaged
        assert 'left' not in pub._last_target
        assert publishers['left'].published == []  # enable never latched true
        pub.publish_step(_ee_axes('ee.left', x=0.4, z=0.3), now_msg='t0')
        assert bc.sent == []  # unseeded arm broadcasts nothing

    def test_publish_step_clamps_linear_norm(self):
        pub, bc, _publishers = _make_servo_publisher(max_lin_step_m=0.03)
        pub.engage(_ee_axes('ee.left', yaw=0.1))
        step = _ee_axes('ee.left', x=1.0, yaw=0.1)  # huge translation jump
        pub.publish_step(step, now_msg='t0')
        assert len(bc.sent) == 1
        t = bc.sent[0].transform.translation
        dist = (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5
        assert abs(dist - 0.03) < 1e-9

    def test_publish_step_clamps_per_axis_angular(self):
        pub, bc, _publishers = _make_servo_publisher(max_ang_step_rad=0.15)
        pub.engage(_ee_axes('ee.left', x=0.4))
        step = _ee_axes('ee.left', x=0.4, roll=1.0, pitch=-1.0, yaw=0.05)
        pub.publish_step(step, now_msg='t0')
        roll, pitch, yaw = pub._last_target['left'][3:]
        assert abs(roll - 0.15) < 1e-9
        assert abs(pitch - (-0.15)) < 1e-9
        assert abs(yaw - 0.05) < 1e-9  # under the clamp, passes through

    def test_publish_step_unwraps_pi_crossing(self):
        import math
        pub, _bc, _publishers = _make_servo_publisher(max_ang_step_rad=10.0)
        near_pi = math.pi - 0.05
        pub.engage({
            'ee.left.x': 0.0, 'ee.left.y': 0.0, 'ee.left.z': 0.0,
            'ee.left.roll': near_pi, 'ee.left.pitch': 0.0, 'ee.left.yaw': 0.0,
        })
        # Target commands near -pi: naive delta would be ~2pi (a huge spin);
        # unwrap must recognize this as a small motion across the +-pi seam.
        step = _ee_axes('ee.left', roll=-near_pi)
        pub.publish_step(step, now_msg='t0')
        new_roll = pub._last_target['left'][3]
        assert abs(new_roll - math.pi) < 0.2  # continued past pi, not spun back to 0

    def test_disable_tracking_latches_false_and_reseeds_on_next_engage(self):
        pub, _bc, publishers = _make_servo_publisher()
        state = {'ee.left.' + a: v for a, v in zip(
            ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )}
        pub.engage(state)
        pub.disable_tracking()
        assert not pub.engaged
        assert publishers['left'].published == [True, False]
        assert pub._last_target == {}

        state2 = dict(state)
        state2['ee.left.x'] = 2.0
        pub.engage(state2)
        assert pub._last_target['left'][0] == 2.0
        assert publishers['left'].published == [True, False, True]


def _rotvec_state(prefix, pos, rotvec):
    return {f'{prefix}.{a}': float(v) for a, v in zip(
        ('x', 'y', 'z', 'rx', 'ry', 'rz'), list(pos) + list(rotvec))}


def _tf_quat(t):
    r = t.transform.rotation
    return np.array([r.x, r.y, r.z, r.w])


class TestServoTargetPublisherRotvec:

    def test_rejects_quat_rotation(self):
        with pytest.raises(ValueError, match='rotvec or rpy'):
            _make_servo_publisher(rotation='quat')

    def test_rotvec_target_broadcasts_from_rotvec_quaternion(self):
        pub, bc, _publishers = _make_servo_publisher(max_ang_step_rad=10.0, rotation='rotvec')
        rv0 = [0.1, -0.2, 0.3]
        assert pub.engage(_rotvec_state('ee.left', [0.4, 0.0, 0.3], rv0))
        rv1 = [0.2, -0.1, 0.5]
        pub.publish_step(_rotvec_state('ee.left', [0.4, 0.0, 0.3], rv1), now_msg='t0')
        assert len(bc.sent) == 1
        expected = Rotation.from_rotvec(rv1).as_quat()
        got = _tf_quat(bc.sent[0])
        assert min(np.abs(got - expected).max(), np.abs(got + expected).max()) < 1e-9

    def test_rotvec_clamps_geodesic_angle(self):
        pub, bc, _publishers = _make_servo_publisher(max_ang_step_rad=0.15, rotation='rotvec')
        r0 = Rotation.from_rotvec([0.0, 0.0, 3.0])
        pub.engage(_rotvec_state('ee.left', [0.4, 0.0, 0.3], r0.as_rotvec()))
        # 1 rad about x relative to r0, crossing the rotvec pi seam in absolute terms.
        r1 = r0 * Rotation.from_rotvec([1.0, 0.0, 0.0])
        pub.publish_step(_rotvec_state('ee.left', [0.4, 0.0, 0.3], r1.as_rotvec()),
                         now_msg='t0')
        r_new = Rotation.from_rotvec(pub._last_target['left'][3:])
        assert abs((r0.inv() * r_new).magnitude() - 0.15) < 1e-9
        step_axis = (r0.inv() * r_new).as_rotvec() / 0.15
        assert np.allclose(step_axis, [1.0, 0.0, 0.0], atol=1e-9)
        q = Rotation.from_quat(_tf_quat(bc.sent[0]))
        assert (q.inv() * r_new).magnitude() < 1e-9

    def test_rotvec_small_step_passes_through(self):
        pub, _bc, _publishers = _make_servo_publisher(max_ang_step_rad=0.15, rotation='rotvec')
        pub.engage(_rotvec_state('ee.left', [0.4, 0.0, 0.3], [0.0, 0.0, 1.0]))
        pub.publish_step(_rotvec_state('ee.left', [0.4, 0.0, 0.3], [0.0, 0.05, 1.0]),
                         now_msg='t0')
        assert np.allclose(pub._last_target['left'][3:], [0.0, 0.05, 1.0], atol=1e-12)


class TestServoTargetPublisherSafety:

    def test_engage_skips_non_finite_seed(self):
        pub, _bc, publishers = _make_servo_publisher()
        assert pub.engage(_ee_axes('ee.left', x=float('nan'), z=0.5)) is False
        assert not pub.engaged
        assert publishers['left'].published == []

    def test_publish_step_drops_non_finite_target(self):
        pub, bc, _publishers = _make_servo_publisher()
        pub.engage(_ee_axes('ee.left', x=0.5, z=0.3))
        pub.publish_step(_ee_axes('ee.left', x=float('nan'), z=0.3), now_msg='t0')
        pub.publish_step(_ee_axes('ee.left', x=float('inf'), z=0.3), now_msg='t1')
        assert bc.sent == []
        assert pub._last_target['left'][:3] == [0.5, 0.0, 0.3]
        # A later finite step still works from the untouched seed.
        pub.publish_step(_ee_axes('ee.left', x=0.51, z=0.3), now_msg='t2')
        assert len(bc.sent) == 1
        assert bc.sent[0].transform.translation.x == pytest.approx(0.51)

    def test_target_lead_bounded_by_measured_pose(self):
        pub, bc, _publishers = _make_servo_publisher(max_lin_step_m=0.03, max_lag_m=0.10)
        seed = _ee_axes('ee.left', x=0.5, z=0.3)
        pub.engage(seed)
        # Arm stalled at the seed pose while the policy keeps asking for x=2.0.
        for i in range(10):
            pub.publish_step(_ee_axes('ee.left', x=2.0, z=0.3), now_msg=f't{i}', measured=seed)
        assert bc.sent[-1].transform.translation.x == pytest.approx(0.60, abs=1e-9)

    def test_without_measured_pose_only_step_clamp_applies(self):
        pub, bc, _publishers = _make_servo_publisher(max_lin_step_m=0.03, max_lag_m=0.10)
        pub.engage(_ee_axes('ee.left', x=0.5, z=0.3))
        for i in range(10):
            pub.publish_step(_ee_axes('ee.left', x=2.0, z=0.3), now_msg=f't{i}')
        assert bc.sent[-1].transform.translation.x == pytest.approx(0.80, abs=1e-9)

    def test_rotation_lead_bounded_by_measured_pose_rotvec(self):
        pub, bc, _publishers = _make_servo_publisher(
            rotation='rotvec', max_ang_step_rad=0.15, max_lag_rad=0.5,
        )
        seed = _rotvec_axes('ee.left', x=0.5, z=0.3)
        pub.engage(seed)
        for i in range(10):
            pub.publish_step(
                _rotvec_axes('ee.left', x=0.5, z=0.3, rz=2.0), now_msg=f't{i}', measured=seed,
            )
        q = bc.sent[-1].transform.rotation
        angle = Rotation.from_quat([q.x, q.y, q.z, q.w]).magnitude()
        assert angle == pytest.approx(0.5, abs=1e-9)

    def test_unmeasured_zero_pose_does_not_anchor(self):
        # measured all-zero == never measured: must not yank the target to origin.
        pub, bc, _publishers = _make_servo_publisher(max_lin_step_m=0.03, max_lag_m=0.10)
        pub.engage(_ee_axes('ee.left', x=0.5, z=0.3))
        pub.publish_step(
            _ee_axes('ee.left', x=0.52, z=0.3), now_msg='t0', measured=_ee_axes('ee.left'),
        )
        assert bc.sent[-1].transform.translation.x == pytest.approx(0.52)

    def test_disable_during_publish_is_serialized(self):
        import threading
        pub, bc, publishers = _make_servo_publisher()
        pub.engage(_ee_axes('ee.left', x=0.5, z=0.3))
        stop = threading.Event()

        def hammer():
            i = 0
            while not stop.is_set():
                pub.publish_step(_ee_axes('ee.left', x=0.5 + 0.001 * i, z=0.3), now_msg=i)
                i += 1

        t = threading.Thread(target=hammer)
        t.start()
        pub.disable_tracking()
        stop.set()
        t.join()
        # After disable no seed survives, so nothing can be broadcast afterwards.
        assert pub._last_target == {}
        n = len(bc.sent)
        pub.publish_step(_ee_axes('ee.left', x=0.9, z=0.3), now_msg='after')
        assert len(bc.sent) == n
