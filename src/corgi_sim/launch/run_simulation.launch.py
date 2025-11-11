#!/usr/bin/env python3
"""
ROS2 launch file for Corgi Webots simulation.
Starts Webots with embedded controller and the corgi_sim node.

Usage:
  ros2 launch corgi_sim run_simulation.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
<<<<<<< HEAD
    package_dir = get_package_share_directory('corgi_sim')
    world = os.path.join(package_dir, 'worlds', 'corgi_origin.wbt')
=======
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
>>>>>>> 4c41e95 (debugging corgi_sim webots runtime issues)

    # Start Webots simulator
    webots_process = ExecuteProcess(
        cmd=['/usr/local/bin/webots', '--mode=realtime', world],
        output='screen'
    )

    # corgi_sim node (converts motor/command <-> Webots topics)
    corgi_sim_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='corgi_sim',
                executable='corgi_sim_trq',
                name='corgi_sim',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        webots_process,
        corgi_sim_node
    ])
