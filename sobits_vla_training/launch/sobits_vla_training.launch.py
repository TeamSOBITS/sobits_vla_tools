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

"""Launch file for the sobits_vla_training ROS 2 node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _str_to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _create_train_node(context, *args, **kwargs):
    """Create the training Node with CLI overrides applied."""
    config_file = LaunchConfiguration('config_file').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)

    policy = LaunchConfiguration('policy').perform(context).strip()
    dataset_repo_id = LaunchConfiguration('dataset_repo_id').perform(context).strip()
    output_dir = LaunchConfiguration('output_dir').perform(context).strip()
    pretrained_path = LaunchConfiguration('pretrained_path').perform(context).strip()
    steps_raw = LaunchConfiguration('steps').perform(context).strip()
    batch_size_raw = LaunchConfiguration('batch_size').perform(context).strip()
    num_gpus_raw = LaunchConfiguration('num_gpus').perform(context).strip()
    wandb_enable_raw = LaunchConfiguration('wandb_enable').perform(context).strip()
    wandb_project = LaunchConfiguration('wandb_project').perform(context).strip()
    hub_repo_id = LaunchConfiguration('hub_repo_id').perform(context).strip()
    resume_raw = LaunchConfiguration('resume').perform(context).strip()

    overrides: dict = {}
    if policy:
        overrides['policy'] = policy
    if dataset_repo_id:
        overrides['dataset.repo_id'] = dataset_repo_id
    if output_dir:
        overrides['checkpoint.output_dir'] = output_dir
    if pretrained_path:
        overrides['checkpoint.pretrained_path'] = pretrained_path
    if steps_raw:
        overrides['training.steps'] = int(steps_raw)
    if batch_size_raw:
        overrides['training.batch_size'] = int(batch_size_raw)
    if num_gpus_raw:
        overrides['num_gpus'] = int(num_gpus_raw)
    if wandb_enable_raw:
        overrides['wandb.enable'] = _str_to_bool(wandb_enable_raw)
    if wandb_project:
        overrides['wandb.project'] = wandb_project
    if hub_repo_id:
        overrides['hub.repo_id'] = hub_repo_id
    if resume_raw:
        overrides['checkpoint.resume'] = _str_to_bool(resume_raw)

    return [
        Node(
            package='sobits_vla_training',
            executable='train_node.py',
            name=node_name,
            output='screen',
            parameters=[
                config_file,
                overrides,
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the LaunchDescription for sobits_vla_training."""
    default_config = PathJoinSubstitution(
        [
            FindPackageShare('sobits_vla_training'),
            'config',
            'training_config.yaml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to training parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'node_name',
                default_value='sobits_vla_training',
                description='ROS 2 node name.',
            ),
            DeclareLaunchArgument(
                'policy',
                default_value='',
                description='Override policy type: smolvla | pi0 | pi05 | pi0_fast',
            ),
            DeclareLaunchArgument(
                'dataset_repo_id',
                default_value='',
                description='Override dataset.repo_id (HF Hub path or local dir).',
            ),
            DeclareLaunchArgument(
                'output_dir',
                default_value='',
                description='Override checkpoint.output_dir.',
            ),
            DeclareLaunchArgument(
                'pretrained_path',
                default_value='',
                description='Override checkpoint.pretrained_path (local path or HF repo_id).',
            ),
            DeclareLaunchArgument(
                'resume',
                default_value='',
                description='Override checkpoint.resume (true/false).',
            ),
            DeclareLaunchArgument(
                'steps',
                default_value='',
                description='Override training.steps (integer).',
            ),
            DeclareLaunchArgument(
                'batch_size',
                default_value='',
                description='Override training.batch_size (integer).',
            ),
            DeclareLaunchArgument(
                'num_gpus',
                default_value='',
                description='Override num_gpus (0=CPU, 1=single GPU, >1=DDP).',
            ),
            DeclareLaunchArgument(
                'wandb_enable',
                default_value='',
                description='Override wandb.enable (true/false).',
            ),
            DeclareLaunchArgument(
                'wandb_project',
                default_value='',
                description='Override wandb.project name.',
            ),
            DeclareLaunchArgument(
                'hub_repo_id',
                default_value='',
                description='Override hub.repo_id for HF Hub push after training.',
            ),
            OpaqueFunction(function=_create_train_node),
        ]
    )
