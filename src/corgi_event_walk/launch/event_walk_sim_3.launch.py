#!/usr/bin/env python3
"""Event walk experiment 3 for simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    odom_share = get_package_share_directory('corgi_odometry')

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share, 'launch', 'corgi_odom_sim.launch.py')
        )
    )

    attitude_node = Node(
        package='corgi_attitude_control',
        executable='attitude_node',
        name='attitude_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    event_walk_node = Node(
        package='corgi_event_walk',
        executable='event_walk_node',
        name='event_walk_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'velocity': 0.15,
            'stand_height': 0.20,
            'step_length': 0.30,
            'step_height': 0.06,
        }],
    )

    return LaunchDescription([odom_launch, attitude_node, event_walk_node])
