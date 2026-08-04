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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from sobits_vla_common.launch.utils import default_pixi_manifest, pixi_prefix

# Required for PI05 bfloat16 model loading on CUDA without OOM.
# Set before any child process is spawned so it is inherited.
os.environ.setdefault('PYTORCH_ALLOC_CONF', 'expandable_segments:True')

# Default pixi env for the deploy node. Override with pixi_env:=deploy-cpu on
# machines without a GPU, or pixi_env:="" to disable the prefix.
_DEFAULT_PIXI_ENV = 'deploy-gpu'
_DEFAULT_PIXI_MANIFEST = default_pixi_manifest()


def _str_to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _create_deploy_node(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory

    config_file = LaunchConfiguration('config_file').perform(context)
    deploy_config = LaunchConfiguration('deploy_config').perform(context).strip()

    # deploy_config is a filename stem inside the package's config/ dir.
    # If provided it takes precedence over config_file.
    if deploy_config:
        pkg_config_dir = os.path.join(
            get_package_share_directory('sobits_vla_deploy'), 'config'
        )
        # Accept with or without .yaml extension
        if not deploy_config.endswith('.yaml'):
            deploy_config += '.yaml'
        config_file = os.path.join(pkg_config_dir, deploy_config)

    # Shared gamepad config (sobits_vla_common) supplies gamepad.command_service,
    # the VlaCommand service name this node advertises for gamepad-driven play/stop.
    gamepad_config = os.path.join(
        get_package_share_directory('sobits_vla_common'),
        'config',
        'gamepad_config.yaml',
    )

    robot_name = LaunchConfiguration('robot_name').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    use_sim_time = _str_to_bool(LaunchConfiguration('use_sim_time').perform(context))

    prefix = pixi_prefix(
        LaunchConfiguration('pixi_env').perform(context),
        LaunchConfiguration('pixi_manifest').perform(context),
    )

    model_repo_id = LaunchConfiguration('model_repo_id').perform(context).strip()
    model_policy_class = LaunchConfiguration('model_policy_class').perform(context).strip()
    model_device = LaunchConfiguration('model_device').perform(context).strip()
    model_use_amp_raw = LaunchConfiguration('model_use_amp').perform(context).strip()

    overrides = {'use_sim_time': use_sim_time}
    if model_repo_id:
        overrides['model.repo_id'] = model_repo_id
    if model_policy_class:
        overrides['model.policy_class'] = model_policy_class
    if model_device:
        overrides['model.device'] = model_device
    if model_use_amp_raw:
        overrides['model.use_amp'] = _str_to_bool(model_use_amp_raw)

    actions = [
        Node(
            package='sobits_vla_deploy',
            executable='sobits_vla_deploy',
            name=node_name,
            namespace=robot_name,
            output='screen',
            prefix=prefix or None,
            parameters=[
                gamepad_config,
                config_file,
                overrides,
            ],
        )
    ]

    # Optional controller bring-up for REAL deployment: reuses the teleop
    # package's input-driver include (quest/ps4/keyboard -> /<ns>/joy) —
    # WITHOUT the teleop control node, which would fight the VLA for the
    # arm — plus the gamepad client in deploy mode (play toggle + reset).
    controller = LaunchConfiguration('controller').perform(context).strip()
    if controller:
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sobits_teleop'),
                'launch', 'include', 'controller_input.launch.py')),
            launch_arguments={
                'robot_name': robot_name,
                'device': controller,
                'ros_ip': LaunchConfiguration('ros_ip').perform(context),
                'use_sim_time': 'true' if use_sim_time else 'false',
            }.items(),
        ))
        actions.append(Node(
            package='sobits_vla_common',
            executable='gamepad_clt_node',
            name='gamepad_client',
            namespace=robot_name,
            output='screen',
            parameters=[
                gamepad_config,
                {'gamepad.mode': 'deploy',
                 'use_sim_time': use_sim_time},
            ],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare('sobits_vla_deploy'),
            'config',
            'deploy_config.yaml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controller',
                default_value='',
                description=(
                    'Bring up controller input for REAL deployment: quest, '
                    'ps4, ps5 or keyboard. Includes sobits_teleop '
                    'controller_input (joy drivers only) + the gamepad '
                    'client in deploy mode. Empty = no controller bring-up.'
                ),
            ),
            DeclareLaunchArgument(
                'ros_ip',
                default_value='127.0.0.1',
                description='ROS IP for the Quest tcp endpoint (controller:=quest).',
            ),
            DeclareLaunchArgument(
                'deploy_config',
                default_value='',
                description=(
                    'Config filename (with or without .yaml) inside the package config/ dir. '
                    'e.g. deploy_config_sobit_home  — takes precedence over config_file.'
                ),
            ),
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description=(
                    'Absolute path to deploy parameter YAML. '
                    'Ignored when deploy_config is set.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_name',
                default_value='sobit_home',
                description='Robot namespace used for the deploy node.',
            ),
            DeclareLaunchArgument(
                'node_name',
                default_value='sobits_vla_deploy',
                description='Node name for the deploy process.',
            ),
            DeclareLaunchArgument(
                'pixi_env',
                default_value=_DEFAULT_PIXI_ENV,
                description=(
                    'pixi environment (Python deps) to run the node in. '
                    'Use deploy-cpu on machines without a GPU, or "" to disable '
                    'the pixi prefix.'
                ),
            ),
            DeclareLaunchArgument(
                'pixi_manifest',
                default_value=_DEFAULT_PIXI_MANIFEST,
                description='Path to pixi.toml (override for installed layouts).',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock if true.',
            ),
            DeclareLaunchArgument(
                'model_repo_id',
                default_value='',
                description='Override model.repo_id when non-empty.',
            ),
            DeclareLaunchArgument(
                'model_policy_class',
                default_value='',
                description='Override model.policy_class when non-empty.',
            ),
            DeclareLaunchArgument(
                'model_device',
                default_value='',
                description='Override model.device when non-empty (e.g., cuda, cpu).',
            ),
            DeclareLaunchArgument(
                'model_use_amp',
                default_value='',
                description='Override model.use_amp when non-empty (true/false).',
            ),
            OpaqueFunction(function=_create_deploy_node),
        ]
    )
