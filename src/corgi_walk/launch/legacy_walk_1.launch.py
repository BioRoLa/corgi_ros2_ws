#!/usr/bin/env python3
"""Legacy walk experiment 1 with real odometry and rosbag recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    walk_share = get_package_share_directory('corgi_walk')
    odom_share = get_package_share_directory('corgi_odometry')

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share, 'launch', 'corgi_odom_real.launch.py')
        ),
        launch_arguments={'record_bag': 'false'}.items(),
    )

    walk_node = Node(
        package='corgi_walk',
        executable='walk_exp',
        name='walk_test',
        output='screen',
        parameters=[{
            'velocity': 0.10,
            'stand_height': 0.20,
            'step_length': 0.20,
            'step_height': 0.06,
        }],
    )

    bag_script = os.path.join(walk_share, 'script', 'legacy_walk_bag.sh')
    bag_recorder = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['bash', bag_script, 'legacy_walk_1'],
            output='screen',
        )],
    )

    return LaunchDescription([odom_launch, walk_node, bag_recorder])
