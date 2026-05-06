#!/usr/bin/env python3
"""Real-robot launch for leg odometry.

This launch starts imu_raw_node and corgi_leg_odom.
corgi_leg_odom remaps IMU input:
    /imu -> /imu_raw
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_raw_node = Node(
        package='corgi_imu',
        executable='imu_raw_node',
        name='imu_raw_node',
        output='screen',
    )

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
        imu_raw_node,
        corgi_leg_odom_node,
    ])
