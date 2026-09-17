#!/usr/bin/env python3
"""Run both legacy odometry nodes on the real robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('corgi_odometry_legacy')
    config_file = os.path.join(package_share, 'config', 'config_online.yaml')
    bag_script = os.path.join(package_share, 'script', 'legacy_odom_bag.sh')

    record_bag = LaunchConfiguration('record_bag')
    record_delay = LaunchConfiguration('record_delay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Record legacy odometry inputs and outputs.',
        ),
        DeclareLaunchArgument(
            'record_delay',
            default_value='3.0',
            description='Seconds to wait before starting bag recording.',
        ),
        Node(
            package='corgi_odometry_legacy',
            executable='corgi_odometry_legacy',
            name='corgi_odometry_legacy',
            parameters=[config_file, {'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='corgi_odometry_legacy',
            executable='corgi_z_position_legacy',
            name='corgi_z_position_legacy',
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
        TimerAction(
            period=record_delay,
            condition=IfCondition(record_bag),
            actions=[
                ExecuteProcess(
                    cmd=['bash', bag_script],
                    output='screen',
                ),
            ],
        ),
    ])
