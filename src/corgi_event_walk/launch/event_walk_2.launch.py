#!/usr/bin/env python3
"""Event walk experiment 2 with ESEKF/LiDAR odometry and rosbag recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    event_walk_share = get_package_share_directory('corgi_event_walk')
    odom_share = get_package_share_directory('corgi_odometry')

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share, 'launch', 'corgi_odom_real.launch.py')
        ),
        launch_arguments={'record_bag': 'false'}.items(),
    )

    event_walk_node = Node(
        package='corgi_event_walk',
        executable='event_walk_node',
        name='event_walk_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sim': False,
            'contact_source': 'gmo',
            'velocity': 0.12,
            'stand_height': 0.20,
            'step_length': 0.25,
            'step_height': 0.06,
        }],
        remappings=[('imu', '/imu_raw')],
    )

    attitude_node = Node(
        package='corgi_attitude_control',
        executable='attitude_node',
        name='attitude_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sim': False,
            'attitude_source': 'ekf',
        }],
        remappings=[('imu', '/imu_raw')],
    )

    bag_script = os.path.join(event_walk_share, 'script', 'event_walk_bag.sh')
    bag_recorder = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['bash', bag_script, 'event_walk_2'],
            output='screen',
        )],
    )

    return LaunchDescription([
        odom_launch, event_walk_node, attitude_node, bag_recorder,
    ])
