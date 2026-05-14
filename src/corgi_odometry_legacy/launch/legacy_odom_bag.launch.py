#!/usr/bin/env python3
"""Launch corgi_odometry_legacy with bag recording.

Records all input and output topics of the corgi_odometry_legacy node:
  Inputs : /trigger, /motor/state, /imu
  Outputs: /odometry/legacy/velocity, /odometry/legacy/position,
           /odometry/legacy/contact
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('corgi_odometry_legacy')
    bag_script = os.path.join(package_share_dir, 'script', 'legacy_odom_bag.sh')

    corgi_odometry_legacy_node = Node(
        package='corgi_odometry_legacy',
        executable='corgi_odometry_legacy',
        name='corgi_odometry_legacy',
        parameters=[{'use_sim_time': False}],
        output='screen',
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
        corgi_odometry_legacy_node,
        bag_record_process,
    ])
