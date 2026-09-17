#!/usr/bin/env python3
"""Real-robot leg odometry with optional bag recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_only = LaunchConfiguration('imu_only')
    record_bag = LaunchConfiguration('record_bag')
    record_delay = LaunchConfiguration('record_delay')

    imu_only_arg = DeclareLaunchArgument(
        'imu_only', default_value='false',
        description='Run ESEKF prediction only; disable leg, GMO, ZUPT and fusion updates.',
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='Record leg-odometry inputs and outputs.',
    )
    record_delay_arg = DeclareLaunchArgument(
        'record_delay', default_value='3.0',
        description='Seconds to wait before starting bag recording.',
    )

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
        parameters=[{
            'use_sim_time': False,
            'imu_only': ParameterValue(imu_only, value_type=bool),
        }],
        remappings=[
            ('/imu', '/imu_raw'),
        ]
    )

    bag_script = os.path.join(
        get_package_share_directory('corgi_odometry'), 'script', 'leg_odom_bag.sh')
    bag_recorder = TimerAction(
        period=record_delay,
        condition=IfCondition(record_bag),
        actions=[ExecuteProcess(cmd=['bash', bag_script], output='screen')],
    )

    return LaunchDescription([
        imu_only_arg,
        record_bag_arg,
        record_delay_arg,
        imu_raw_node,
        corgi_leg_odom_node,
        bag_recorder,
    ])
