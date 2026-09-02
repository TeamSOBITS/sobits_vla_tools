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
Deterministic synthetic rosbag2 fixture for the conversion pipeline.

Writes <root>/recorded_bags_meta.yaml + <session>/<episode>/episode.mcap,
using the sobit_light descriptor's head group + head_camera so no new
robot descriptor is needed. Fixed epoch t0 -> byte-identical mcap across runs.
"""

from pathlib import Path

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores
import yaml

ROBOT_ID = 'sobit_light'
JOINT_STATES_TOPIC = '/sobit_light/joint_states'
HEAD_CMD_TOPIC = '/sobit_light/head_joint_controller/joint_trajectory'
HEAD_CAMERA_TOPIC = '/sobit_light/head_camera/color/image_raw'
HEAD_CAMERA_INFO_TOPIC = '/sobit_light/head_camera/camera_info'
JOINT_NAMES = ['head_yaw_joint', 'head_pitch_joint']

FPS = 10
NUM_FRAMES = 30
JOINT_HZ = 30
IMG_W, IMG_H = 64, 48
T0_SEC = 1700000000  # fixed epoch -- never Time.now(); keeps mcap byte-identical
SESSION_NAME = 'fixture_session_20260828_000000'
EPISODE_NAME = 'episode_20260828_000000'
TASK_LABEL = 'Fixture task: nod the head'


def _stamp(ts):
    sec = int(ts)
    nanosec = int(round((ts - sec) * 1e9))
    return sec, nanosec


def _joint_positions(t):
    """Deterministic ramp/sine so state/action differ frame to frame."""
    yaw = 0.3 * np.sin(2 * np.pi * 0.2 * t)
    pitch = 0.1 * t
    return [float(yaw), float(pitch)]


def _gradient_frame(frame_idx):
    """64x48 rgb8 gradient keyed only on frame_idx -- no wall-clock, no RNG."""
    x = np.linspace(0, 255, IMG_W, dtype=np.uint8)
    y = np.linspace(0, 255, IMG_H, dtype=np.uint8)
    r = np.tile(x, (IMG_H, 1))
    g = np.tile(y[:, None], (1, IMG_W))
    b = np.full((IMG_H, IMG_W), (frame_idx * 8) % 256, dtype=np.uint8)
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


def generate(root: Path) -> Path:
    """Write the fixture under root; returns the episode bag directory."""
    root = Path(root)
    ts = get_typestore(Stores.ROS2_JAZZY)
    Header = ts.types['std_msgs/msg/Header']
    Time = ts.types['builtin_interfaces/msg/Time']
    Duration = ts.types['builtin_interfaces/msg/Duration']
    JointState = ts.types['sensor_msgs/msg/JointState']
    Image = ts.types['sensor_msgs/msg/Image']
    JointTrajectory = ts.types['trajectory_msgs/msg/JointTrajectory']
    JointTrajectoryPoint = ts.types['trajectory_msgs/msg/JointTrajectoryPoint']

    bag_dir = root / SESSION_NAME / EPISODE_NAME
    bag_dir.parent.mkdir(parents=True, exist_ok=True)

    with Writer(bag_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conn_js = writer.add_connection(
            JOINT_STATES_TOPIC, JointState.__msgtype__, typestore=ts)
        conn_cmd = writer.add_connection(
            HEAD_CMD_TOPIC, JointTrajectory.__msgtype__, typestore=ts)
        conn_img = writer.add_connection(
            HEAD_CAMERA_TOPIC, Image.__msgtype__, typestore=ts)

        duration_s = NUM_FRAMES / FPS

        # Joint states at JOINT_HZ, well above FPS so interpolation has real data.
        n_js = int(duration_s * JOINT_HZ) + 1
        for i in range(n_js):
            t = i / JOINT_HZ
            sec, nanosec = _stamp(T0_SEC + t)
            pos = _joint_positions(t)
            msg = JointState(
                header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=''),
                name=JOINT_NAMES,
                position=np.array(pos, dtype=np.float64),
                velocity=np.array([0.0, 0.0], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            )
            data = ts.serialize_cdr(msg, JointState.__msgtype__)
            writer.write(conn_js, int((T0_SEC + t) * 1e9), data)

        # Commanded targets every 0.5s -- zero-order-hold source for `action`.
        n_cmd = int(duration_s / 0.5) + 1
        for i in range(n_cmd):
            t = i * 0.5
            sec, nanosec = _stamp(T0_SEC + t)
            pos = _joint_positions(t + 0.1)  # commands lead state slightly
            msg = JointTrajectory(
                header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=''),
                joint_names=JOINT_NAMES,
                points=[JointTrajectoryPoint(
                    positions=np.array(pos, dtype=np.float64),
                    velocities=np.array([], dtype=np.float64),
                    accelerations=np.array([], dtype=np.float64),
                    effort=np.array([], dtype=np.float64),
                    time_from_start=Duration(sec=0, nanosec=0),
                )],
            )
            data = ts.serialize_cdr(msg, JointTrajectory.__msgtype__)
            writer.write(conn_cmd, int((T0_SEC + t) * 1e9), data)

        # Primary camera frames at FPS -- these drive frame count directly.
        for i in range(NUM_FRAMES):
            t = i / FPS
            sec, nanosec = _stamp(T0_SEC + t)
            frame = _gradient_frame(i)
            msg = Image(
                header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=''),
                height=IMG_H,
                width=IMG_W,
                encoding='rgb8',
                is_bigendian=0,
                step=IMG_W * 3,
                data=frame.reshape(-1).astype(np.uint8),
            )
            data = ts.serialize_cdr(msg, Image.__msgtype__)
            writer.write(conn_img, int((T0_SEC + t) * 1e9), data)

    _write_meta(root)
    return bag_dir


def _write_meta(root: Path) -> None:
    meta = {
        'robot_info': {
            'name': ROBOT_ID,
            'version': '1.0.0',
            'morphology': {
                'type': 'mobile_manipulator',
                'joint_states_topic': JOINT_STATES_TOPIC,
                'parts': ['head'],
                'head': {
                    'is_actionable': True,
                    'command_topic': HEAD_CMD_TOPIC,
                    'joint_names': JOINT_NAMES,
                },
            },
            'sensors': {
                'types': ['camera'],
                'camera': {
                    'names': ['head_camera'],
                    'models': [''],
                    'topics': [HEAD_CAMERA_TOPIC],
                    'info_topics': [HEAD_CAMERA_INFO_TOPIC],
                    'compressed_topics': [''],
                    'properties': {
                        'head_camera': {
                            'width': IMG_W, 'height': IMG_H, 'topic': HEAD_CAMERA_INFO_TOPIC,
                        },
                    },
                },
            },
        },
        'user_info': {'name': '', 'email': '', 'location': ''},
        'recorded_bags': {
            'tasks_list': [SESSION_NAME],
            'tasks': {
                SESSION_NAME: {
                    'label': TASK_LABEL,
                    'bag_path': SESSION_NAME,
                    'gamepad': 'fixture',
                    'episodes_list': [EPISODE_NAME],
                    'episodes': {EPISODE_NAME: {'bag_path': f'{SESSION_NAME}/{EPISODE_NAME}'}},
                },
            },
        },
    }
    with open(root / 'recorded_bags_meta.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=False, sort_keys=False)


if __name__ == '__main__':
    import sys
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/tmp/fixture_bag')
    generate(out)
    print(f'Fixture written to {out}')
