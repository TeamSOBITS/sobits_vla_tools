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

from sobits_vla_common.launch.utils import (
    default_pixi_manifest, pixi_launch_arguments, pixi_prefix, resolve_pixi_env,
)
from sobits_vla_deploy.launch_helpers import (
    controller_and_teleop_actions, world_reset_actions, world_reset_config_path,
)

# Required for PI05 bfloat16 model loading on CUDA without OOM.
# Set before any child process is spawned so it is inherited.
os.environ.setdefault('PYTORCH_ALLOC_CONF', 'expandable_segments:True')

# The deploy node runs in the shared pixi env; only the accelerator varies.
# GPU is the default -- override with  enable_gpu:=false
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
        resolve_pixi_env(context),
        LaunchConfiguration('pixi_manifest').perform(context),
    )

    model_repo_id = LaunchConfiguration('model_repo_id').perform(context).strip()
    model_policy_class = LaunchConfiguration('model_policy_class').perform(context).strip()
    model_device = LaunchConfiguration('model_device').perform(context).strip()
    model_use_amp_raw = LaunchConfiguration('model_use_amp').perform(context).strip()

    world_reset_config = world_reset_config_path(robot_name)

    overrides = {'use_sim_time': use_sim_time}
    if os.path.isfile(world_reset_config):
        overrides['logging.scene_config'] = world_reset_config
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

    enable_world_reset = _str_to_bool(
        LaunchConfiguration('enable_world_reset').perform(context)
    )
    actions += world_reset_actions(
        world_reset_config, enable_world_reset, use_sim_time, prefix,
        'STOP/RESET will not reset the scene.',
    )
    actions += controller_and_teleop_actions(
        context, robot_name, gamepad_config, use_sim_time,
    )

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
            *pixi_launch_arguments(_DEFAULT_PIXI_MANIFEST),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock if true.',
            ),
            DeclareLaunchArgument(
                'enable_world_reset',
                default_value='true',
                description=(
                    'Bring up the shared world_reset_node so PLAY/STOP/RESET '
                    'reset the scene. Disable if a reset node is already '
                    'running elsewhere (e.g. brought up by collection).'
                ),
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
