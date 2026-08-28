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
Deploy-specific launch blocks shared across sobits_vla_deploy launch files.

Used by sobits_vla_deploy.launch.py and vla_experiment.launch.py -- both
bring up the deploy node against the same scene, and both optionally add a
real-hardware controller.
"""

import os

from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def world_reset_config_path(robot_name: str) -> str:
    """
    Path to the world_reset scene YAML for *robot_name*.

    Same scene the reset node teleports to: the deploy node's episode logger
    derives its lift/fall baselines from it so they cannot drift from the
    reset targets.
    """
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('sobits_vla_common'),
        'config',
        'world_reset_' + robot_name + '.yaml',
    )


def world_reset_actions(
    world_reset_config: str,
    enable_world_reset: bool,
    use_sim_time: bool,
    prefix: str,
    missing_scene_msg: str,
) -> list:
    """
    Bring up the shared world_reset_node, or log why it was skipped.

    ``missing_scene_msg`` lets each caller keep its own wording for what
    won't happen without the scene (e.g. "STOP/RESET" vs "episode resets").
    """
    if not enable_world_reset:
        return []
    if os.path.isfile(world_reset_config):
        return [Node(
            package='sobits_vla_common',
            executable='world_reset_node',
            name='world_reset_node',
            output='screen',
            prefix=prefix or None,
            parameters=[
                world_reset_config,
                {'use_sim_time': use_sim_time},
            ],
        )]
    return [LogInfo(msg=(
        '[world_reset] no scene YAML at {} -- reset node not started; '
        '{}'.format(world_reset_config, missing_scene_msg)
    ))]


def controller_and_teleop_actions(
    context,
    robot_name: str,
    gamepad_config: str,
    use_sim_time: bool,
) -> list:
    """
    Build the optional real-hardware bring-up actions.

    Controller input-driver include (joy only, no arm/base tracking that'd
    fight the VLA) + gamepad client (play/reset) + a poses-only teleop profile.

    No-op (empty list) when the ``controller`` launch argument is unset --
    the sim default needs neither drivers nor a joy-driven pose button.
    """
    controller = LaunchConfiguration('controller').perform(context).strip()
    if not controller:
        return []

    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sobits_teleop'),
                'launch', 'include', 'controller_input.launch.py')),
            launch_arguments={
                'robot_name': robot_name,
                'device': controller,
                'ros_ip': LaunchConfiguration('ros_ip').perform(context),
                'use_sim_time': 'true' if use_sim_time else 'false',
            }.items(),
        ),
        Node(
            package='sobits_vla_common',
            executable='gamepad_clt_node',
            name='gamepad_client',
            namespace=robot_name,
            output='screen',
            parameters=[
                gamepad_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ]

    # Poses-only profile (<device>_vla.yaml: no control_velocity/quest_control) so the
    # pose button works without fighting the VLA; reset itself uses world_reset_node instead.
    teleop_share = get_package_share_directory('sobits_teleop')
    teleop_pose_config = os.path.join(
        teleop_share, 'config', robot_name,
        controller + '_vla.yaml')
    if os.path.isfile(teleop_pose_config):
        actions.append(Node(
            package='sobits_teleop',
            executable='sobits_teleop',
            name='sobits_teleop',
            namespace=robot_name,
            output='screen',
            parameters=[
                os.path.join(teleop_share, 'config', robot_name, 'robot.yaml'),
                teleop_pose_config,
                {'use_sim_time': use_sim_time},
            ],
        ))
    else:
        actions.append(LogInfo(msg=(
            '[teleop] no poses-only profile at {} -- reset will not re-pose '
            'the robot.'.format(teleop_pose_config)
        )))

    return actions


def str_to_bool(value: str) -> bool:
    """Parse a launch-argument boolean string."""
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def resolve_deploy_config(deploy_config: str, fallback: str = 'deploy_config') -> str:
    """
    Resolve a deploy_config filename stem to the package config/ path.

    Accepts the stem with or without .yaml; empty falls back to `fallback`.
    """
    from ament_index_python.packages import get_package_share_directory
    cfg = deploy_config.strip() if deploy_config else ''
    if not cfg:
        cfg = fallback
    if not cfg.endswith('.yaml'):
        cfg += '.yaml'
    return os.path.join(
        get_package_share_directory('sobits_vla_deploy'), 'config', cfg
    )
