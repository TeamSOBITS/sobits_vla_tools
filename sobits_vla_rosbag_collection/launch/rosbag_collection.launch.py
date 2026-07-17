import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description_impl(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    pkg_share = get_package_share_directory('sobits_vla_rosbag_collection')

    rosbag_config = os.path.join(
        pkg_share,
        'config',
        'collection_config_' + robot_name + '.yaml',
    )

    gamepad_config = os.path.join(
        pkg_share,
        'config',
        'gamepad_config.yaml',
    )

    record_directory = LaunchConfiguration('record_directory').perform(context)
    # If no record directory is specified, try the source tree first, then fall back to pkg_share
    if not record_directory:
        pkg_name = 'sobits_vla_rosbag_collection'
        launch_file_path = os.path.abspath(__file__)
        if '/install/' in launch_file_path:
            ws_root = launch_file_path.split('/install/')[0]
            src_candidate = os.path.join(
                ws_root, 'src', 'robocup_opl_doinglaundry', 'sobits_vla_tools', pkg_name)
            src_candidate_flat = os.path.join(ws_root, 'src', 'sobits_vla_tools', pkg_name)
            if os.path.exists(src_candidate):
                record_directory = os.path.join(src_candidate, 'rosbags')
            elif os.path.exists(src_candidate_flat):
                record_directory = os.path.join(src_candidate_flat, 'rosbags')
        if not record_directory:
            record_directory = os.path.join(pkg_share, 'rosbags')
    if not os.path.exists(record_directory):
        try:
            os.makedirs(record_directory, exist_ok=True)
        except Exception as e:
            print(f'[ERROR] Failed to create record directory {record_directory}: {e}')

    print(f'[INFO] Rosbags will be saved in: {record_directory}')

    parameters = [
        rosbag_config,
        gamepad_config,
        {'rosbag_config.record_directory': record_directory,
         'use_sim_time': use_sim_time},
    ]

    container = ComposableNodeContainer(
        name='vla_rosbag_collection_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='sobits_vla_rosbag_collection',
                plugin='sobits_vla::RosbagCollection',
                name='vla_rosbag_collection',
                namespace=robot_name,
                parameters=parameters,
            ),
            ComposableNode(
                package='sobits_vla_common',
                plugin='sobits_vla::GamepadClient',
                name='gamepad_clt_node',
                namespace=robot_name,
                parameters=[gamepad_config, {'use_sim_time': use_sim_time}],
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_directory',
            default_value='',
            description='Absolute path to where rosbags are saved. Defaults to <pkg_src>/rosbags.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='sobit_home',
            description='Name of the robot to record rosbags for.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.',
        ),
        OpaqueFunction(function=generate_launch_description_impl),
    ])
