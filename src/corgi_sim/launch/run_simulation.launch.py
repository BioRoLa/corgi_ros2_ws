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
    package_dir = get_package_share_directory('corgi_sim')
    world = os.path.join(package_dir, 'worlds', 'corgi_origin.wbt')

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