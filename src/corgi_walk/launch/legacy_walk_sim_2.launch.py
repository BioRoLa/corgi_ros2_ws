#!/usr/bin/env python3
"""Legacy walk experiment 2 for simulation."""

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

    walk_node = Node(
        package='corgi_walk',
        executable='walk_exp',
        name='walk_test',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'velocity': 0.12,
            'stand_height': 0.20,
            'step_length': 0.25,
            'step_height': 0.06,
        }],
    )

    return LaunchDescription([odom_launch, walk_node])
