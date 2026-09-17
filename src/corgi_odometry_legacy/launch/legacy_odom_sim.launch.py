#!/usr/bin/env python3
"""Run both legacy odometry nodes with simulation time."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('corgi_odometry_legacy')
    config_file = os.path.join(package_share, 'config', 'config_online.yaml')
    deterministic_replay = LaunchConfiguration('deterministic_replay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'deterministic_replay',
            default_value='false',
            description='Synchronize replay inputs by message sequence and timestamp.',
        ),
        Node(
            package='corgi_odometry_legacy',
            executable='corgi_odometry_legacy',
            name='corgi_odometry_legacy',
            parameters=[
                config_file,
                {
                    'use_sim_time': True,
                    'deterministic_replay': ParameterValue(
                        deterministic_replay, value_type=bool
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='corgi_odometry_legacy',
            executable='corgi_z_position_legacy',
            name='corgi_z_position_legacy',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
