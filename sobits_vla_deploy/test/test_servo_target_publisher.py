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


def _make_servo_publisher(max_lin_step_m=0.03, max_ang_step_rad=0.15, arms=None):
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
    )
    return pub, broadcaster, publishers


class TestServoTargetPublisher:

    def test_engage_seeds_from_state_vector_and_latches_true(self):
        pub, _bc, publishers = _make_servo_publisher()
        state = {'ee.left.' + a: v for a, v in zip(
            ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], [0.5, 0.1, 0.3, 0.0, 0.0, 1.0],
        )}
        pub.engage(state)
        assert pub.engaged
        assert publishers['left'].published == [True]
        assert pub._last_target['left'] == [0.5, 0.1, 0.3, 0.0, 0.0, 1.0]

    def test_engage_skips_arm_with_missing_keys(self):
        pub, _bc, publishers = _make_servo_publisher()
        pub.engage({})  # no ee.left.* keys at all
        assert pub.engaged
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
        pub.engage(_ee_axes('ee.left'))  # all-zero: EE state never measured
        assert pub.engaged
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
