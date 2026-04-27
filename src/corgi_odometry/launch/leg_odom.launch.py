#!/usr/bin/env python3
"""Real-robot launch for leg odometry.

This launch starts only corgi_leg_odom and remaps IMU input:
  /imu -> /imu_raw
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/imu', '/imu_raw'),
        ]
    )

    return LaunchDescription([
        corgi_leg_odom_node,
    ])
