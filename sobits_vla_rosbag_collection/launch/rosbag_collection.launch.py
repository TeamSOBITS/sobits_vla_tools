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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from sobits_vla_common.launch.utils import config_declares


def generate_launch_description_impl(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    enable_world_reset = (
        LaunchConfiguration('enable_world_reset').perform(context).lower() == 'true'
    )
    pkg_share = get_package_share_directory('sobits_vla_rosbag_collection')

    rosbag_config = os.path.join(
        pkg_share,
        'config',
        'collection_config_' + robot_name + '.yaml',
    )

    gamepad_config = os.path.join(
        get_package_share_directory('sobits_vla_common'),
        'config',
        'gamepad_config.yaml',
    )

    record_directory = LaunchConfiguration('record_directory').perform(context)
    # If no record directory is specified, try the source tree first, then fall back to pkg_share
    if not record_directory:
        pkg_name = 'sobits_vla_rosbag_collection'
        launch_file_path = os.path.abspath(__file__)
        if '/install/' in launch_file_path:
            ws_root = launch_file_path.split('/install/')[0]
            src_candidate = os.path.join(
                ws_root, 'src', 'robocup_opl_doinglaundry', 'sobits_vla_tools', pkg_name)
            src_candidate_flat = os.path.join(ws_root, 'src', 'sobits_vla_tools', pkg_name)
            if os.path.exists(src_candidate):
                record_directory = os.path.join(src_candidate, 'rosbags')
            elif os.path.exists(src_candidate_flat):
                record_directory = os.path.join(src_candidate_flat, 'rosbags')
        if not record_directory:
            record_directory = os.path.join(pkg_share, 'rosbags')
    if not os.path.exists(record_directory):
        try:
            os.makedirs(record_directory, exist_ok=True)
        except Exception as e:
            print(f'[ERROR] Failed to create record directory {record_directory}: {e}')

    print(f'[INFO] Rosbags will be saved in: {record_directory}')

    overrides = {
        # The RosbagCollection node's own advertised service (private ~/command);
        # the gamepad client below targets it by owner-node relative name instead.
        'gamepad.command_service': '~/command',
        'use_sim_time': use_sim_time,
    }
    # Computed default only when the config doesn't own the value; a CLI arg
    # always wins. Keeps the YAML the single source of truth.
    record_dir_from_cli = bool(
        LaunchConfiguration('record_directory').perform(context)
    )
    if record_dir_from_cli or not config_declares(
            rosbag_config, 'rosbag_config.record_directory'):
        overrides['rosbag_config.record_directory'] = record_directory

    parameters = [rosbag_config, gamepad_config, overrides]

    container = ComposableNodeContainer(
        name='vla_rosbag_collection_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='sobits_vla_rosbag_collection',
                plugin='sobits_vla::RosbagCollection',
                name='vla_rosbag_collection',
                namespace=robot_name,
                parameters=parameters,
            ),
            ComposableNode(
                package='sobits_vla_common',
                plugin='sobits_vla::GamepadClient',
                name='gamepad_clt_node',
                namespace=robot_name,
                parameters=[
                    gamepad_config,
                    {'gamepad.command_service': 'vla_rosbag_collection/command',
                     'use_sim_time': use_sim_time},
                ],
            ),
        ],
        output='screen',
    )

    actions = [container]

    if enable_world_reset:
        world_reset_config = os.path.join(
            get_package_share_directory('sobits_vla_common'),
            'config',
            'world_reset_' + robot_name + '.yaml',
        )
        if os.path.isfile(world_reset_config):
            actions.append(Node(
                package='sobits_vla_common',
                executable='world_reset_node',
                name='world_reset_node',
                namespace=robot_name,
                output='screen',
                parameters=[
                    world_reset_config,
                    {'use_sim_time': use_sim_time},
                ],
            ))
        else:
            actions.append(LogInfo(msg=(
                '[world_reset] no scene YAML at {} -- reset node not started; '
                'RESET will not reset the scene.'.format(world_reset_config)
            )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_directory',
            default_value='',
            description='Absolute path to where rosbags are saved. Defaults to <pkg_src>/rosbags.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='sobit_home',
            description='Name of the robot to record rosbags for.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.',
        ),
        DeclareLaunchArgument(
            'enable_world_reset',
            default_value='false',
            description=(
                'Bring up the shared world_reset_node and honour VlaCommand '
                'RESET button. Leave false for real-robot '
                'collection, where there is no scene to teleport.'
            ),
        ),
        OpaqueFunction(function=generate_launch_description_impl),
    ])
