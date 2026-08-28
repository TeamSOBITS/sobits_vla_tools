#!/usr/bin/env python3
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
World reset node: the single owner of scene-reset logic for both deploy and collection.

Reads the scene from ROS parameters (world_reset.*) and teleports every model it lists
-- position and orientation -- via ros_gz SetEntityPose, falling back to the gz CLI.
Teleports run only when use_sim_time is true. The robot re-pose is published straight
to the joint controllers from the same scene YAML, so no teleop node is required.
Advertises ~/reset_world (VlaResetWorld). Blocking work runs on a
ReentrantCallbackGroup under a MultiThreadedExecutor so the reset never deadlocks
the executor.

Parameters are flat because ROS cannot express a list of dicts:

  world_reset.world_name       str
  world_reset.settle_s         float
  world_reset.presets          str[]   preset names
  world_reset.active_preset    str     preset used when a call sends ""
  world_reset.<preset>.models  str[]   Gazebo model names in that preset
  world_reset.<preset>.<model>.pose.<field>       float     x/y/z required,
                                                            roll/pitch/yaw
                                                            (radians) default 0
  world_reset.<preset>.<model>.randomize.<field>  float[2]  optional [lo, hi]
                                                            offset bounds
  world_reset.<preset>.<model>.radius             float     clearance for
                                                            overlap rejection
  world_reset.reset_pose.wait                     bool      wait for the arm to
                                                            settle before the
                                                            teleports
  world_reset.reset_pose.wait_s                   float     that wait's timeout
  world_reset.reset_pose.groups                   str[]     joint groups to pose
  world_reset.reset_pose.<group>.topic            str       JointTrajectory topic
  world_reset.reset_pose.<group>.joints           str[]     joint names
  world_reset.reset_pose.<group>.positions        float[]   targets, same order
"""

from __future__ import annotations

from threading import Event, Lock
import time
from typing import Any, Dict, Optional

from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from sobits_interfaces.srv import VlaResetWorld
from sobits_vla_common import gz_utils
from sobits_vla_common.param_schema import declare_from_schema, P, read_schema
from sobits_vla_common.world_reset import POSE_FIELDS, ResetResult, WorldResetter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Fixed-name params only. Presets/models/reset_pose groups are keyed by list
# params resolved at runtime (Template can't nest a dynamic key inside
# another dynamic key), so that scene-loading stays hand-written below.
_SCHEMA = {
    'world_reset': {
        'world_name': P(''),
        'settle_s': P(0.5),
        'presets': P(['default']),
        # Preset applied when a caller sends preset:"" -- which every caller
        # does unless it sets the field. Switches scenes without code changes.
        'active_preset': P(''),
        # Redraw budget when a randomized pose lands inside another model's
        # radius. Exceeding it accepts an overlap rather than skipping a reset.
        'max_placement_tries': P(100),
        # Robot re-pose is published straight to the joint controllers from
        # this node's own scene YAML -- no dependency on a teleop node.
        'reset_pose': {
            'groups': P(['']),
            'time_from_start': P(3.0),
            # Trajectories are fire-and-forget, so wait out the motion before
            # teleporting onto a still-moving base.
            'wait_s': P(4.0),
            # False skips waiting for the arm to settle before teleporting --
            # faster, but a still-moving base gets shoved by arm reaction torque.
            'wait': P(True),
        },
        # Absolute: this node runs un-namespaced, so a relative name would
        # resolve to /joint_states where the robot publishes nothing.
        'joint_states_topic': P('/sobit_home/joint_states'),
        'still_eps_rad': P(0.002),
        'still_hold_s': P(0.4),
    },
}


class WorldResetNode(Node):

    def __init__(self) -> None:
        super().__init__('world_reset_node')
        self._cb_group = ReentrantCallbackGroup()

        declare_from_schema(self, _SCHEMA)
        params = read_schema(self, _SCHEMA).world_reset

        self._world_name = str(params.world_name)
        self._settle_s = float(params.settle_s)
        self._pose_time_from_start = float(params.reset_pose.time_from_start)
        self._pose_wait_s = float(params.reset_pose.wait_s)
        self._pose_wait = bool(params.reset_pose.wait)
        self._joint_states_topic = str(params.joint_states_topic)
        self._still_eps = float(params.still_eps_rad)
        self._still_hold_s = float(params.still_hold_s)
        self._max_placement_tries = int(params.max_placement_tries)
        self._joint_lock = Lock()
        self._reset_lock = Lock()
        self._last_joint_positions: Optional[Dict[str, float]] = None
        self._scene = self._load_scene(params)
        if not self._scene['presets']:
            raise RuntimeError(
                'No world_reset presets resolved -- pass a scene config to this '
                'node (see sobits_vla_common/config/world_reset_<robot>.yaml).'
            )

        # rclpy declares use_sim_time on every node; the launcher sets it.
        self._sim_enabled = bool(self.get_parameter('use_sim_time').value)

        self._resetter = WorldResetter(
            self._scene, self._set_entity_pose, logger=self.get_logger(),
            max_placement_tries=self._max_placement_tries,
        )

        self._pose_groups = self._load_reset_pose(params)
        if self._pose_groups:
            self.create_subscription(
                JointState,
                self._joint_states_topic,
                self._on_joint_states,
                QoSProfile(depth=10),
                callback_group=self._cb_group,
            )

        self._set_pose_client = None
        self._reset_srv = self.create_service(
            VlaResetWorld,
            '~/reset_world',
            self._on_reset_world,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'World reset node ready: world={!r}, sim_enabled={}, presets={}, '
            'reset_pose_groups={}.'.format(
                self._world_name, self._sim_enabled,
                sorted(self._scene['presets']),
                [g['name'] for g in self._pose_groups] or '<none>',
            )
        )

    def _load_reset_pose(self, params: Any) -> list:
        """Read the reset_pose block: one trajectory publisher per joint group."""
        groups = []
        for name in params.reset_pose.groups:
            base = 'world_reset.reset_pose.{}'.format(name)
            self.declare_parameter(base + '.topic', '')
            self._declare_array(base + '.joints')
            self._declare_array(base + '.positions')
            topic = str(self.get_parameter(base + '.topic').value)
            joints = list(self.get_parameter(base + '.joints').value or [])
            positions = [
                float(p) for p in (self.get_parameter(base + '.positions').value or [])
            ]
            if not topic or not joints:
                self.get_logger().error(
                    'reset_pose group {!r}: topic or joints missing -- '
                    'skipping.'.format(name)
                )
                continue
            if len(joints) != len(positions):
                self.get_logger().error(
                    'reset_pose group {!r}: {} joints but {} positions -- '
                    'skipping.'.format(name, len(joints), len(positions))
                )
                continue
            groups.append({
                'name': name,
                'joints': joints,
                'positions': positions,
                'pub': self.create_publisher(JointTrajectory, topic, 10),
            })
        return groups

    def _send_reset_pose(self) -> bool:
        """
        Publish the reset pose to the joint controllers; blocking until still.

        Self-contained: the trajectories come from this node's scene YAML, so
        no teleop node needs to be running. True when nothing is configured.
        """
        if not self._pose_groups:
            return True

        stamp = rclpy.time.Time().to_msg()
        for group in self._pose_groups:
            traj = JointTrajectory()
            traj.header.stamp = stamp
            traj.joint_names = group['joints']
            point = JointTrajectoryPoint()
            point.positions = group['positions']
            point.velocities = [0.0] * len(group['positions'])
            point.time_from_start = Duration(
                sec=int(self._pose_time_from_start),
                nanosec=int(
                    (self._pose_time_from_start % 1.0) * 1e9
                ),
            )
            traj.points.append(point)
            group['pub'].publish(traj)
        self.get_logger().info(
            'Reset pose sent over {} joint group(s), {:.1f}s.'.format(
                len(self._pose_groups), self._pose_time_from_start
            )
        )
        return self._wait_until_still()

    def _on_joint_states(self, msg: JointState) -> None:
        # Keyed by name: several controllers publish this topic with the same
        # joints in different orders, so index-wise comparison is meaningless.
        with self._joint_lock:
            self._last_joint_positions = dict(zip(msg.name, msg.position))

    def _wait_until_still(self) -> bool:
        """
        Block until the arm stops moving, so the teleport lands on a still base.

        The pose service returns as soon as the trajectory is published, and a
        fixed sleep either wastes time or fires mid-motion. Poll joint_states
        and return once every joint has held position across two samples.

        Skipped entirely when reset_pose.wait is False (or the timeout is
        non-positive), so the reset proceeds without settling the arm.
        """
        if not self._pose_wait or self._pose_wait_s <= 0.0:
            return True
        deadline = time.monotonic() + self._pose_wait_s
        prev = None
        still_since = None
        while time.monotonic() < deadline:
            time.sleep(0.1)
            with self._joint_lock:
                cur = dict(self._last_joint_positions or {})
            if not cur:
                continue
            shared = cur.keys() & prev.keys() if prev else set()
            if shared:
                moved = max(abs(cur[j] - prev[j]) for j in shared)
                if moved < self._still_eps:
                    if still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since >= self._still_hold_s:
                        return True
                else:
                    still_since = None
            prev = cur
        self.get_logger().warning(
            'Robot still moving after {:.1f}s -- teleporting anyway.'.format(
                self._pose_wait_s
            )
        )
        return True

    def _declare_array(self, name: str) -> None:
        """Declare an array param that may be absent; [] alone carries no type."""
        self.declare_parameter(
            name, None, ParameterDescriptor(dynamic_typing=True)
        )

    def _load_scene(self, params: Any) -> Dict[str, Any]:
        """Build the WorldResetter scene dict from the flat world_reset.* params."""
        scene: Dict[str, Any] = {
            'presets': {},
            # Used when a caller sends preset:"". Empty -> 'default'.
            'active_preset': str(params.active_preset or ''),
        }
        for preset in params.presets:
            base = 'world_reset.{}'.format(preset)
            self._declare_array(base + '.models')
            models = []
            for model in self.get_parameter(base + '.models').value or []:
                pose = self._load_pose(base + '.' + model)
                if pose is None:
                    continue
                entry = {'name': model, 'pose': pose}
                randomize = self._load_randomize(base + '.' + model)
                if randomize:
                    entry['randomize'] = randomize
                radius_name = '{}.{}.radius'.format(base, model)
                self.declare_parameter(radius_name, 0.0)
                entry['radius'] = float(self.get_parameter(radius_name).value)
                models.append(entry)
            scene['presets'][preset] = {'models': models}
        return scene

    def _load_pose(self, base: str) -> Optional[Dict[str, float]]:
        """Read the pose block: x/y/z required, roll/pitch/yaw default to 0."""
        pose = {}
        for key in ('x', 'y', 'z'):
            name = '{}.pose.{}'.format(base, key)
            self.declare_parameter(name, float('nan'))
            value = float(self.get_parameter(name).value)
            if value != value:
                self.get_logger().error(
                    'Model {!r}: {} is unset -- skipping this model.'.format(
                        base.rsplit('.', 1)[-1], name
                    )
                )
                return None
            pose[key] = value
        for key in ('roll', 'pitch', 'yaw'):
            name = '{}.pose.{}'.format(base, key)
            self.declare_parameter(name, 0.0)
            pose[key] = float(self.get_parameter(name).value)
        return pose

    def _load_randomize(self, base: str) -> Dict[str, Any]:
        """Read the optional randomize block: per-field [lo, hi] bounds."""
        randomize = {}
        for field in POSE_FIELDS:
            name = '{}.randomize.{}'.format(base, field)
            self._declare_array(name)
            bounds = list(self.get_parameter(name).value or [])
            if len(bounds) == 2:
                randomize[field] = bounds
            elif bounds:
                self.get_logger().warning(
                    '{} needs exactly 2 values, got {} -- ignoring.'.format(
                        name, len(bounds)
                    )
                )
        return randomize

    @staticmethod
    def _wait_future(future, timeout_s: float) -> bool:
        """Block a worker thread until an rclpy future resolves (executor spins it)."""
        done = Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout=timeout_s)

    def _set_entity_pose(
        self, name: str,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> bool:
        """
        Teleport a Gazebo entity.

        Prefers the bridged ros_gz SetEntityPose service
        (/world/<world>/set_pose); falls back to the `gz service` CLI when
        the bridge does not expose it.
        """
        try:
            from ros_gz_interfaces.msg import Entity
            from ros_gz_interfaces.srv import SetEntityPose

            if self._set_pose_client is None:
                self._set_pose_client = self.create_client(
                    SetEntityPose,
                    '/world/{}/set_pose'.format(self._world_name),
                    callback_group=self._cb_group,
                )
            if self._set_pose_client.wait_for_service(timeout_sec=1.0):
                req = SetEntityPose.Request()
                req.entity = Entity(name=name, type=Entity.MODEL)
                req.pose.position.x = float(x)
                req.pose.position.y = float(y)
                req.pose.position.z = float(z)
                req.pose.orientation.x = float(qx)
                req.pose.orientation.y = float(qy)
                req.pose.orientation.z = float(qz)
                req.pose.orientation.w = float(qw)
                fut = self._set_pose_client.call_async(req)
                if self._wait_future(fut, timeout_s=3.0) and fut.result() is not None:
                    return bool(fut.result().success)
                self.get_logger().warning(
                    'SetEntityPose service call timed out — falling back to gz CLI.'
                )
        except ImportError:
            pass
        return gz_utils.gz_set_pose(self._world_name, name, x, y, z, qx, qy, qz, qw)

    def _on_reset_world(
        self,
        request: VlaResetWorld.Request,
        response: VlaResetWorld.Response,
    ) -> VlaResetWorld.Response:
        # One reset at a time: concurrent callers would interleave set_pose
        # calls and each would see the other's teleports as failures.
        with self._reset_lock:
            return self._reset_once(request, response)

    def _reset_once(
        self,
        request: VlaResetWorld.Request,
        response: VlaResetWorld.Response,
    ) -> VlaResetWorld.Response:
        # Pose BEFORE the teleports: arm reaction torque shoves the brake-less
        # mobile base, so posing afterwards would push it off the target.
        pose_ok = self._send_reset_pose()

        if self._sim_enabled:
            result = self._resetter.reset(preset=request.preset)
            self.get_logger().info(
                'World reset (preset={!r}): {}'.format(
                    request.preset or 'default', result.message
                )
            )
        else:
            result = ResetResult(True, 'Sim disabled -- no teleports.')

        if self._settle_s > 0.0:
            time.sleep(self._settle_s)

        response.success = result.success and pose_ok
        response.models_reset = result.models_reset
        response.message = result.message
        if not pose_ok:
            response.message += ' Reset pose failed.'
        return response


def main(args: Optional[Any] = None) -> None:
    rclpy.init(args=args)
    node = WorldResetNode()
    # A reset callback blocks for seconds; the spare threads keep joint_states
    # and the pose-service reply flowing while it does.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
