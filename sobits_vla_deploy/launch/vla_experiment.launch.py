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

r"""
Unattended VLA evaluation launch — one model per invocation.

Starts the deploy node with episode logging force-enabled (writing to
<log_dir>/<model_label>) and the experiment runner, which auto-loops
num_episodes episodes (PLAY -> wait for termination -> world reset -> repeat).

Assumes Gazebo + the robot are already running. Defaults to use_sim_time:=true
so the 60 s episode timeout is measured against the simulation clock.

Example:
-------
  ros2 launch sobits_vla_deploy vla_experiment.launch.py \\
    deploy_config:=deploy_config_sobit_home_left_smolvla \\
    model_label:=smolvla num_episodes:=20

"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sobits_vla_common.launch.utils import default_pixi_manifest, pixi_prefix

# Required for PI05 bfloat16 model loading on CUDA without OOM.
os.environ.setdefault('PYTORCH_ALLOC_CONF', 'expandable_segments:True')

# Both nodes here are sobits_vla_deploy executables -> same pixi env.
_DEFAULT_PIXI_ENV = 'deploy-gpu'
_DEFAULT_PIXI_MANIFEST = default_pixi_manifest()


def _str_to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _setup(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory

    deploy_config = LaunchConfiguration('deploy_config').perform(context).strip()
    model_label = LaunchConfiguration('model_label').perform(context).strip()
    log_dir = LaunchConfiguration('log_dir').perform(context).strip()
    robot_name = LaunchConfiguration('robot_name').perform(context).strip()
    num_episodes = int(LaunchConfiguration('num_episodes').perform(context))
    use_sim_time = _str_to_bool(LaunchConfiguration('use_sim_time').perform(context))
    episode_timeout_s = float(
        LaunchConfiguration('episode_timeout_s').perform(context)
    )
    lift_success_m = float(LaunchConfiguration('lift_success_m').perform(context))
    fall_z_drop_m = float(LaunchConfiguration('fall_z_drop_m').perform(context))
    done_wait_margin_s = float(LaunchConfiguration('done_wait_margin_s').perform(context))

    pkg_config_dir = os.path.join(
        get_package_share_directory('sobits_vla_deploy'), 'config'
    )
    cfg = deploy_config if deploy_config else 'deploy_config'
    if not cfg.endswith('.yaml'):
        cfg += '.yaml'
    config_file = os.path.join(pkg_config_dir, cfg)

    # Derive a model label from the config stem when not supplied.
    if not model_label:
        model_label = os.path.splitext(os.path.basename(cfg))[0]

    gamepad_config = os.path.join(
        get_package_share_directory('sobits_vla_common'),
        'config',
        'gamepad_config.yaml',
    )

    prefix = pixi_prefix(
        LaunchConfiguration('pixi_env').perform(context),
        LaunchConfiguration('pixi_manifest').perform(context),
    )

    episode_log_dir = os.path.join(log_dir, model_label)

    overrides = {
        'use_sim_time': use_sim_time,
        'logging.enabled': True,
        'logging.log_dir': episode_log_dir,
        'logging.episode_timeout_s': episode_timeout_s,
        'logging.lift_success_m': lift_success_m,
        'logging.fall_z_drop_m': fall_z_drop_m,
    }

    deploy_node = Node(
        package='sobits_vla_deploy',
        executable='sobits_vla_deploy',
        name='sobits_vla_deploy',
        namespace=robot_name,
        output='screen',
        prefix=prefix or None,
        parameters=[gamepad_config, config_file, overrides],
    )

    runner_node = Node(
        package='sobits_vla_deploy',
        executable='vla_experiment_runner',
        name='vla_experiment_runner',
        output='screen',
        prefix=prefix or None,
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_episodes': num_episodes,
            'command_service': '/vla/command',
            'episode_done_topic': '/vla/episode_done',
            'episode_timeout_s': episode_timeout_s,
            'done_wait_margin_s': done_wait_margin_s,
        }],
    )

    # When the runner finishes all episodes it exits; tear down the whole
    # launch (deploy node included) so a sequential wrapper can move on.
    shutdown_on_runner_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=runner_node,
            on_exit=[EmitEvent(event=Shutdown(reason='experiment complete'))],
        )
    )

    actions = [deploy_node, runner_node, shutdown_on_runner_exit]

    # Optional controller bring-up for REAL trial runs: teleop input drivers
    # (joy only — no teleop control node) + the gamepad client in deploy
    # mode (play toggle, reset button, deadman trigger source).
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='',
            description=(
                'Bring up controller input (quest, ps4, ps5, keyboard) + '
                'gamepad client in deploy mode for REAL trial runs. '
                'Empty = none (sim default).'
            ),
        ),
        DeclareLaunchArgument(
            'ros_ip',
            default_value='127.0.0.1',
            description='ROS IP for the Quest tcp endpoint (controller:=quest).',
        ),
        DeclareLaunchArgument(
            'deploy_config',
            default_value='deploy_config_sobit_home_left_smolvla',
            description='Config filename stem inside the package config/ dir.',
        ),
        DeclareLaunchArgument(
            'model_label',
            default_value='',
            description=(
                'Subdirectory under log_dir for this model run. '
                'Defaults to the deploy_config stem.'
            ),
        ),
        DeclareLaunchArgument(
            'log_dir',
            default_value='/tmp/vla_logs',
            description='Base log directory; episodes go to <log_dir>/<model_label>.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='sobit_home',
            description='Robot namespace for the deploy node.',
        ),
        DeclareLaunchArgument(
            'pixi_env',
            default_value=_DEFAULT_PIXI_ENV,
            description=(
                'pixi environment (Python deps) for both deploy nodes. '
                'Use deploy-cpu without a GPU, or "" to disable the prefix.'
            ),
        ),
        DeclareLaunchArgument(
            'pixi_manifest',
            default_value=_DEFAULT_PIXI_MANIFEST,
            description='Path to pixi.toml (override for installed layouts).',
        ),
        DeclareLaunchArgument(
            'done_wait_margin_s',
            default_value='30.0',
            description=(
                'Runner grace period beyond episode_timeout_s before forcing '
                'a STOP. Raise for real-robot runs where the scene is staged '
                'with the safety trigger released after auto-PLAY.'
            ),
        ),
        DeclareLaunchArgument(
            'num_episodes',
            default_value='20',
            description='Number of episodes to run automatically.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock (recommended for the timeout).',
        ),
        DeclareLaunchArgument(
            'episode_timeout_s',
            default_value='60.0',
            description='Auto-terminate an episode after this many sim seconds.',
        ),
        DeclareLaunchArgument(
            'lift_success_m',
            default_value='0.05',
            description='Block lift (m) above which the episode is a success.',
        ),
        DeclareLaunchArgument(
            'fall_z_drop_m',
            default_value='0.15',
            description='Robot world-z drop (m) above which it counts as fallen.',
        ),
        OpaqueFunction(function=_setup),
    ])
