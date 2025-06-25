import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_light" # Change this to the desired robot name

    rosbag_config = os.path.join(
        get_package_share_directory("sobits_vla_rosbag_collection"),
        "config",
        "record_settings_" + robot_name + ".yaml",
    )

    rosbag_collection_node = Node(
        package="sobits_vla_rosbag_collection",
        executable="rosbag_collection_node",
        name="rosbag_collection_node",
        namespace=robot_name,
        parameters=[rosbag_config],
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


    return LaunchDescription([
        rosbag_collection_node,
        gamepad_clt_node,
    ])
