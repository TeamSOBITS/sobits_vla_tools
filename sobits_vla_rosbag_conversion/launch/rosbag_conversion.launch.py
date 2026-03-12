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

    # Defaults
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
        OpaqueFunction(function=generate_launch_description_impl)
    ])
