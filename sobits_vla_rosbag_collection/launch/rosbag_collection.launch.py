import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description_impl(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    pkg_share = get_package_share_directory("sobits_vla_rosbag_collection")
    
    rosbag_config = os.path.join(
        pkg_share,
        "config",
        "record_settings_" + robot_name + ".yaml",
    )

    record_directory = LaunchConfiguration('record_directory').perform(context)
    if not record_directory:
        record_directory = os.path.join(pkg_share, 'rosbags')

    parameters = [
        rosbag_config,
        {'rosbag_config.record_directory': record_directory}
    ]

    container = ComposableNodeContainer(
        name='vla_rosbag_collection_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sobits_vla_rosbag_collection',
                plugin='sobits_vla::RosbagCollection',
                name='rosbag_collection_node',
                namespace=robot_name,
                parameters=parameters,
            ),
            ComposableNode(
                package='sobits_vla_rosbag_collection',
                plugin='sobits_vla::GamepadClient',
                name='gamepad_clt_node',
                namespace=robot_name,
                parameters=[rosbag_config],
            )
        ],
        output='screen',
    )

    return [
        container,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_directory',
            default_value='',
            description='Absolute path to where the rosbags should be dumped. Defaults to <package_share>/rosbags.'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Name of the robot to record rosbags for.'
        ),
        OpaqueFunction(function=generate_launch_description_impl)
    ])
