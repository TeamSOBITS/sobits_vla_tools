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

"""Bag reader module for loading ROS 2 bags and extracting topic data."""

from pathlib import Path

from rosbags.highlevel import AnyReader
from tqdm import tqdm


class BagReader:
    """Reader for ROS 2 bags using rosbags library."""

    def __init__(self, bag_path: str, logger=None):
        """Initialize BagReader with bag path and optional logger."""
        self.bag_path = Path(bag_path)
        self.logger = logger

    def check_missing_topics(self, wanted_topics: set) -> set:
        """Return a set of missing topics from the bag."""
        with AnyReader([self.bag_path]) as reader:
            available = {c.topic for c in reader.connections}
            missing = wanted_topics - available
            return missing

    def read_topic_series(
        self,
        wanted_topics: set,
        topic_to_cam: dict,
        part_command_topics: set,
        cmd_vel_topic: str,
        odom_topic: str,
        joint_states_topic: str,
        ee_pose_enabled: bool,
        has_mobile_base: bool,
        has_cmd_vel_y: bool,
        has_cmd_vel_z: bool,
    ):
        """Read and parse topics into sorted lists of events."""
        joint_states_series = []
        cmd_vel_series = []
        odom_series = []
        cmd_joints_series = []
        cam_series = {cam_name: [] for cam_name in topic_to_cam.values()}
        tf_messages = []

        with AnyReader([self.bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in wanted_topics]
            total = sum(getattr(c, 'msgcount', 0) for c in connections) or None
            msg_iter = tqdm(
                reader.messages(connections=connections), total=total,
                desc='  reading bag', unit='msg', disable=None, leave=False,
            )
            for connection, timestamp, rawdata in msg_iter:
                topic = connection.topic
                t_bag = timestamp * 1e-9

                if topic == joint_states_topic:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    joint_pos = dict(zip(msg.name, msg.position))
                    velocity_values = getattr(msg, 'velocity', [])
                    if len(velocity_values) > 0:
                        joint_vel = dict(zip(msg.name, velocity_values))
                    else:
                        joint_vel = {}
                    joint_states_series.append((t_sec, joint_pos, joint_vel))

                elif ee_pose_enabled and topic in ('/tf', '/tf_static'):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    tf_messages.append((t_bag, msg, topic))

                elif topic in part_command_topics:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    if (
                        hasattr(msg, 'joint_names')
                        and hasattr(msg, 'points')
                        and len(msg.points) > 0
                    ):
                        target_point = msg.points[-1]
                        if hasattr(msg, 'header') and msg.header.stamp.sec > 0:
                            t_cmd = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        else:
                            t_cmd = t_bag
                        commanded_joints = dict(zip(msg.joint_names, target_point.positions))
                        cmd_joints_series.append((t_cmd, commanded_joints))

                elif topic == cmd_vel_topic and has_mobile_base:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    if hasattr(msg, 'header'):
                        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    else:
                        t_sec = t_bag
                    if has_cmd_vel_y and has_cmd_vel_z:
                        cmd_vel = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
                    elif has_cmd_vel_y:
                        cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
                    elif has_cmd_vel_z:
                        cmd_vel = [msg.linear.x, msg.linear.z, msg.angular.z]
                    else:
                        cmd_vel = [msg.linear.x, msg.angular.z]
                    cmd_vel_series.append((t_sec, cmd_vel))

                elif topic == odom_topic and has_mobile_base:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    if hasattr(msg, 'header'):
                        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    else:
                        t_sec = t_bag
                    linear_x = msg.twist.twist.linear.x
                    linear_y = msg.twist.twist.linear.y
                    linear_z = msg.twist.twist.linear.z
                    angular_z = msg.twist.twist.angular.z
                    if has_cmd_vel_y and has_cmd_vel_z:
                        odom_vel = [linear_x, linear_y, linear_z, angular_z]
                    elif has_cmd_vel_y:
                        odom_vel = [linear_x, linear_y, angular_z]
                    elif has_cmd_vel_z:
                        odom_vel = [linear_x, linear_z, angular_z]
                    else:
                        odom_vel = [linear_x, angular_z]
                    odom_series.append((t_sec, odom_vel))

                elif topic in topic_to_cam:
                    cam_name = topic_to_cam[topic]
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    # Keep rawdata, connection, t_sec, msg
                    cam_series[cam_name].append((t_sec, msg, rawdata, connection))

        # Sort all buffers by timestamp
        joint_states_series.sort(key=lambda x: x[0])
        cmd_vel_series.sort(key=lambda x: x[0])
        odom_series.sort(key=lambda x: x[0])
        cmd_joints_series.sort(key=lambda x: x[0])
        for cam_name in cam_series:
            cam_series[cam_name].sort(key=lambda x: x[0])

        return {
            'joint_states': joint_states_series,
            'cmd_vel': cmd_vel_series,
            'odom': odom_series,
            'cmd_joints': cmd_joints_series,
            'cam_series': cam_series,
            'tf_messages': tf_messages,
        }
