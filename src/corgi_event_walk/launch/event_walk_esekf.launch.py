#!/usr/bin/env python3
"""Real-robot event-walk launch with ESEKF/LiDAR odometry.

Starts:
  * corgi_odometry/launch/corgi_odom_real.launch.py
  * corgi_event_walk/event_walk_node
  * corgi_attitude_control/attitude_node

The motor driver and power node are expected to be started separately.
Bag recording starts after a short delay so FAST-LIO/ESEKF can initialize.
"""

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
        )
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
        }],
        remappings=[
            ('imu', '/imu_raw'),
        ],
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
        remappings=[
            ('imu', '/imu_raw'),
        ],
    )

    bag_script = os.path.join(event_walk_share, 'script', 'event_walk_esekf_bag.sh')
    bag_recorder = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', bag_script],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        odom_launch,
        event_walk_node,
        attitude_node,
        bag_recorder,
    ])
