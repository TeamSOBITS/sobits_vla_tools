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

    gamepad_config = os.path.join(
        pkg_share,
        "config",
        "gamepad_settings.yaml",
    )

    record_directory = LaunchConfiguration('record_directory').perform(context)
    # If no record directory is specified, use the `rosbags` directory in the source directory of the package
    if not record_directory:
        launch_file_path = os.path.abspath(__file__)
        pkg_name = "sobits_vla_rosbag_collection"
        ws_root = launch_file_path.split('/install/')[0]
        src_base = os.path.join(ws_root, 'src')
        sobits_vla_path = os.path.join(src_base, "sobits_vla_tools", pkg_name)
        if os.path.exists(sobits_vla_path):
            record_directory = os.path.join(sobits_vla_path, 'rosbags')
    if not os.path.exists(record_directory):
        try:
            os.makedirs(record_directory, exist_ok=True)
        except Exception as e:
            print(f"[ERROR] Failed to create record directory {record_directory}: {e}")

    print(f"[INFO] Rosbags will be saved in: {record_directory}")

    parameters = [
        rosbag_config,
        gamepad_config,
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
                name='vla_rosbag_collection',
                namespace=robot_name,
                parameters=parameters,
            ),
            ComposableNode(
                package='sobits_vla_rosbag_collection',
                plugin='sobits_vla::GamepadClient',
                name='gamepad_clt_node',
                namespace=robot_name,
                parameters=[gamepad_config],
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
