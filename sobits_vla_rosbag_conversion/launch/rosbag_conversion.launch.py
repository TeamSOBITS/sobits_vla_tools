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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sobits_vla_common.launch.utils import default_pixi_manifest, pixi_prefix

# Conversion imports pandas/scipy/matplotlib/rosbags/torch -> conversion env.
_DEFAULT_PIXI_ENV = 'conversion-gpu'
_DEFAULT_PIXI_MANIFEST = default_pixi_manifest()


def generate_launch_description_impl(context, *args, **kwargs):
    conversion_share = get_package_share_directory('sobits_vla_rosbag_conversion')
    collection_share = get_package_share_directory('sobits_vla_rosbag_collection')

    # Configuration File — robot name selects conversion_config_<robot>.yaml
    config_file = LaunchConfiguration('config_file').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    if not config_file:
        config_file = (
            f'conversion_config_{robot}.yaml' if robot
            else 'conversion_config.yaml'
        )
    if not os.path.isabs(config_file):
        config_file = os.path.join(conversion_share, 'config', config_file)

    rosbag_directory = LaunchConfiguration('rosbag_directory').perform(context)
    recorded_bags_meta_file = LaunchConfiguration(
        'recorded_bags_meta_file'
    ).perform(context)
    dataset_name = LaunchConfiguration('dataset_name').perform(context)
    vcodec = LaunchConfiguration('vcodec').perform(context)
    overwrite = (
        LaunchConfiguration('overwrite').perform(context).lower() == 'true'
    )

    # Default rosbag_directory: src-tree sobits_vla_rosbag_collection/rosbags/
    # With --symlink-install, realpath resolves the installed launch file back
    # into the source tree and the sibling check finds it directly. With
    # regular copy installs realpath stays in the install space, so also look
    # for the package under the workspace's src/ while walking up.
    if not rosbag_directory:
        src_file = os.path.realpath(__file__)
        candidate = os.path.dirname(src_file)
        import glob as _glob
        for _ in range(8):
            sibling = os.path.join(
                candidate, 'sobits_vla_rosbag_collection', 'rosbags'
            )
            if os.path.isdir(sibling):
                rosbag_directory = sibling
                break
            src_root = os.path.join(candidate, 'src')
            if os.path.isdir(src_root):
                hits = (
                    _glob.glob(os.path.join(
                        src_root, 'sobits_vla_rosbag_collection', 'rosbags'))
                    + _glob.glob(os.path.join(
                        src_root, '*', 'sobits_vla_rosbag_collection', 'rosbags'))
                )
                if hits:
                    rosbag_directory = hits[0]
                    break
            candidate = os.path.dirname(candidate)
        if not rosbag_directory:
            rosbag_directory = os.path.join(collection_share, 'rosbags')

    if not recorded_bags_meta_file:
        recorded_bags_meta_file = os.path.join(
            rosbag_directory, 'recorded_bags_meta.yaml'
        )

    parameters = [config_file]

    # Override from launch arguments if explicitly provided
    override_params = {}
    if rosbag_directory:
        override_params['rosbag_directory'] = rosbag_directory
    if recorded_bags_meta_file:
        override_params['recorded_bags_meta_file'] = recorded_bags_meta_file
    if dataset_name:
        override_params['dataset_name'] = dataset_name
    if vcodec:
        override_params['vcodec'] = vcodec
    override_params['overwrite'] = overwrite

    if override_params:
        parameters.append(override_params)

    prefix = pixi_prefix(
        LaunchConfiguration('pixi_env').perform(context),
        LaunchConfiguration('pixi_manifest').perform(context),
    )

    rosbag_conversion_node = Node(
        package='sobits_vla_rosbag_conversion',
        executable='ros2bag_to_lerobotdataset',
        name='rosbag_conversion_node',
        output='screen',
        prefix=prefix or None,
        parameters=parameters,
    )

    return [rosbag_conversion_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot',
                default_value='',
                description=(
                    'Robot name — selects conversion_config_<robot>.yaml '
                    '(e.g. sobit_home).'
                ),
            ),
            DeclareLaunchArgument(
                'config_file',
                default_value='',
                description=(
                    'Explicit config file path or name (overrides robot). '
                    'Defaults to conversion_config_<robot>.yaml '
                    'or conversion_config.yaml.'
                ),
            ),
            DeclareLaunchArgument(
                'rosbag_directory',
                default_value='',
                description='Path to the rosbags directory.',
            ),
            DeclareLaunchArgument(
                'recorded_bags_meta_file',
                default_value='',
                description='Path to the recorded_bags_meta.yaml file.',
            ),
            DeclareLaunchArgument(
                'dataset_name',
                default_value='',
                description='Dataset name.',
            ),
            DeclareLaunchArgument(
                'vcodec',
                default_value='',
                description=(
                    'Video codec override (e.g., auto, h264, av1). '
                    'Uses config value when empty.'
                ),
            ),
            DeclareLaunchArgument(
                'overwrite',
                default_value='false',
                description='Delete existing output dataset before converting.',
            ),
            DeclareLaunchArgument(
                'pixi_env',
                default_value=_DEFAULT_PIXI_ENV,
                description=(
                    'pixi environment (Python deps) to run conversion in. '
                    'Use conversion-cpu without a GPU, or "" to disable the prefix.'
                ),
            ),
            DeclareLaunchArgument(
                'pixi_manifest',
                default_value=_DEFAULT_PIXI_MANIFEST,
                description='Path to pixi.toml (override for installed layouts).',
            ),
            OpaqueFunction(function=generate_launch_description_impl),
        ]
    )
