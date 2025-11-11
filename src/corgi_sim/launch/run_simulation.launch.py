#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def generate_launch_description():
    # Set WEBOTS_HOME if not already set
    if 'WEBOTS_HOME' not in os.environ:
        os.environ['WEBOTS_HOME'] = '/usr/local/webots'
    
    # Set ROS2 middleware configuration for better local discovery
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    
    # Set Fast-DDS profile for localhost discovery
    fastdds_profile = os.path.join(os.path.dirname(os.path.dirname(get_package_share_directory('corgi_sim'))), 
                                   'fastdds_localhost.xml')
    if os.path.exists(fastdds_profile):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_profile
        print(f"Using Fast-DDS profile: {fastdds_profile}")
    
    # Get package directories
    pkg_share = get_package_share_directory('corgi_sim')
    pkg_prefix = get_package_prefix('corgi_sim')
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'corgi_origin.wbt'),
        description='Path to the Webots world file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots simulation mode: realtime, pause, or fast'
    )

    # Get launch configurations
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    # Webots launcher script path
    webots_launcher_script = os.path.join(pkg_prefix, 'lib', 'corgi_sim', 'webots_launcher.py')

    # Webots launcher node (Python script)
    webots_launcher = ExecuteProcess(
        cmd=[
            webots_launcher_script,
            '--world', world,
            '--mode', mode
        ],
        output='screen'
    )

    # Corgi simulation node (ROS2)
    corgi_sim_node = Node(
        package='corgi_sim',
        executable='corgi_sim_trq',  # or corgi_sim_pos
        name='corgi_sim',
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': True}]
    )

    # Data recorder node (if it's been converted to ROS2)
    # Uncomment when corgi_data_recorder is ported to ROS2
    # data_recorder_node = Node(
    #     package='corgi_data_recorder',
    #     executable='corgi_data_recorder',
    #     name='corgi_data_recorder',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    return LaunchDescription([
        world_arg,
        mode_arg,
        webots_launcher,
        corgi_sim_node,
        # data_recorder_node,  # Uncomment when available
    ])
