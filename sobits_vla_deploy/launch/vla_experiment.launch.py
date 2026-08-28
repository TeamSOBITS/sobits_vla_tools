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
from sobits_vla_common.launch.utils import (
    default_package_root, default_pixi_manifest, pixi_launch_arguments,
    pixi_prefix, resolve_pixi_env,
)
from sobits_vla_deploy.launch_helpers import (
    controller_and_teleop_actions, world_reset_actions, world_reset_config_path,
)
import yaml

# Required for PI05 bfloat16 model loading on CUDA without OOM.
os.environ.setdefault('PYTORCH_ALLOC_CONF', 'expandable_segments:True')

# Both nodes here are sobits_vla_deploy executables -> same pixi env; only the
# accelerator varies, GPU by default. Override with  enable_gpu:=false
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
        resolve_pixi_env(context),
        LaunchConfiguration('pixi_manifest').perform(context),
    )

    if log_dir:
        episode_log_dir = os.path.join(log_dir, model_label)
    else:
        with open(config_file, 'r') as f:
            cfg_data = yaml.safe_load(f) or {}
        model_repo_id = (
            cfg_data.get('/**', {}).get('ros__parameters', {})
            .get('model', {}).get('repo_id', '')
        ).strip()
        logs_root = default_package_root('sobits_vla_deploy', 'logs', __file__)
        # "<...>" placeholder repo_id (unedited template) is not a real org/id.
        if model_repo_id and '<' not in model_repo_id:
            episode_log_dir = os.path.join(logs_root, model_repo_id)
        else:
            episode_log_dir = os.path.join(logs_root, model_label)

    controller = LaunchConfiguration('controller').perform(context).strip()

    world_reset_config = world_reset_config_path(robot_name)

    overrides = {
        'use_sim_time': use_sim_time,
        'logging.enabled': True,
        'logging.log_dir': episode_log_dir,
        'task.common.episode_timeout_s': episode_timeout_s,
        'task.common.fall_z_drop_m': fall_z_drop_m,
        # lift_success_m is per mode; only the active block is read, so
        # setting both keeps the override mode-agnostic.
        'task.pick.lift_success_m': lift_success_m,
        'task.place.lift_success_m': lift_success_m,
    }
    if os.path.isfile(world_reset_config):
        overrides['logging.scene_config'] = world_reset_config
    if not controller:
        # No /joy without a controller -- deadman would never be pressed, freezing
        # every unattended episode. Safety lives under the active controller's deploy block.
        with open(gamepad_config, 'r') as f:
            gp = (yaml.safe_load(f) or {}).get('/**', {}).get(
                'ros__parameters', {}).get('gamepad', {})
        active = str(gp.get('controller', 'quest'))
        overrides[
            'gamepad.{}.button_mapping.deploy.safety.enabled'.format(active)
        ] = False

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
            'command_service': '/vla/deploy_command',
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

    enable_world_reset = _str_to_bool(
        LaunchConfiguration('enable_world_reset').perform(context)
    )
    actions += world_reset_actions(
        world_reset_config, enable_world_reset, use_sim_time, prefix,
        'episode resets will not teleport the scene.',
    )
    actions += controller_and_teleop_actions(
        context, robot_name, gamepad_config, use_sim_time,
    )

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
            default_value='',
            description=(
                'Base log directory; episodes go to <log_dir>/<model_label>. '
                'Empty = <package_src>/logs/<model.repo_id or model_label>.'
            ),
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='sobit_home',
            description='Robot namespace for the deploy node.',
        ),
        *pixi_launch_arguments(_DEFAULT_PIXI_MANIFEST),
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
        DeclareLaunchArgument(
            'enable_world_reset',
            default_value='true',
            description=(
                "Bring up the shared world_reset_node so the runner's resets "
                'actually teleport the scene. Disable if one is already running.'
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
