#!/usr/bin/env python3
"""Real-robot launch for leg odometry.

This launch starts bag recording first, then imu_raw_node and corgi_leg_odom.
corgi_leg_odom remaps IMU input:
    /imu -> /imu_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('corgi_odometry')
    bag_script = os.path.join(package_share_dir, 'script', 'leg_odom_bag.sh')

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

    # Delay bag recording so nodes are fully initialized and topics are
    # stable in the DDS graph before ros2 bag record does discovery.
    bag_record_process = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', bag_script],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        imu_raw_node,
        corgi_leg_odom_node,
        bag_record_process,
    ])
