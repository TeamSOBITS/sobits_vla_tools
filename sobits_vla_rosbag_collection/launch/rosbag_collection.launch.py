import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
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

    rosbag_collection_node = Node(
        package="sobits_vla_rosbag_collection",
        executable="rosbag_collection_node",
        name="rosbag_collection_node",
        namespace=robot_name,
        parameters=parameters,
        output="screen",
    )

    gamepad_clt_node = Node(
        package="sobits_vla_rosbag_collection",
        executable="gamepad_clt_node",
        name="gamepad_clt_node",
        namespace=robot_name,
        parameters=[rosbag_config],
        output="screen",
    )


    return [
        rosbag_collection_node,
        gamepad_clt_node,
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
