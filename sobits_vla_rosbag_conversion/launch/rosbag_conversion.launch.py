import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def generate_launch_description_impl(context, *args, **kwargs):
    conversion_share = get_package_share_directory("sobits_vla_rosbag_conversion")
    collection_share = get_package_share_directory("sobits_vla_rosbag_collection")
    
    # Configuration File
    config_file = LaunchConfiguration('config_file').perform(context)
    if not os.path.isabs(config_file):
        config_file = os.path.join(conversion_share, 'config', config_file)
        
    rosbag_directory = LaunchConfiguration('rosbag_directory').perform(context)
    recorded_bags_meta_file = LaunchConfiguration('recorded_bags_meta_file').perform(context)
    dataset_name = LaunchConfiguration('dataset_name').perform(context)
    vcodec = LaunchConfiguration('vcodec').perform(context)
    overwrite = LaunchConfiguration('overwrite').perform(context).lower() == 'true'

    # Defaults: prefer src-tree rosbags dir (written at runtime), fall back to install share
    if not rosbag_directory:
        launch_file_path = os.path.abspath(__file__)
        if '/install/' in launch_file_path:
            ws_root = launch_file_path.split('/install/')[0]
            pkg_name = 'sobits_vla_rosbag_collection'
            src_candidate = os.path.join(
                ws_root, 'src', 'robocup_opl_doinglaundry', 'sobits_vla_tools', pkg_name)
            src_candidate_flat = os.path.join(ws_root, 'src', 'sobits_vla_tools', pkg_name)
            if os.path.isdir(os.path.join(src_candidate, 'rosbags')):
                rosbag_directory = os.path.join(src_candidate, 'rosbags')
            elif os.path.isdir(os.path.join(src_candidate_flat, 'rosbags')):
                rosbag_directory = os.path.join(src_candidate_flat, 'rosbags')
        if not rosbag_directory:
            rosbag_directory = os.path.join(collection_share, 'rosbags')
        
    if not recorded_bags_meta_file:
        recorded_bags_meta_file = os.path.join(rosbag_directory, "recorded_bags_meta.yaml")

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

    rosbag_conversion_node = Node(
        package="sobits_vla_rosbag_conversion",
        executable="ros2bag_to_lerobotdataset",
        name="rosbag_conversion_node",
        output="screen",
        parameters=parameters,
    )

    return [rosbag_conversion_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='conversion_settings.yaml',
            description='Path to the conversion configuration file'
        ),
        DeclareLaunchArgument(
            'rosbag_directory',
            default_value='',
            description='Path to the rosbags directory.'
        ),
        DeclareLaunchArgument(
            'recorded_bags_meta_file',
            default_value='',
            description='Path to the recorded_bags_meta.yaml file.'
        ),
        DeclareLaunchArgument(
            'dataset_name',
            default_value='',
            description='Dataset name.'
        ),
        DeclareLaunchArgument(
            'vcodec',
            default_value='',
            description='Video codec override (e.g., auto, h264, av1). Uses config value when empty.'
        ),
        DeclareLaunchArgument(
            'overwrite',
            default_value='false',
            description='Delete existing output dataset before converting.'
        ),
        OpaqueFunction(function=generate_launch_description_impl)
    ])
