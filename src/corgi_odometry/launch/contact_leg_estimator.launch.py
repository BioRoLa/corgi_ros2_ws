#!/usr/bin/env python3
"""
Launch file for Contact Leg Estimator (Online Version)

Tunable parameters are loaded by the node at startup from:
  share/corgi_odometry/config/config_online.yaml
Edit that file to change observer/contact thresholds without recompiling.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    contact_leg_estimator_node = Node(
        package='corgi_odometry',
        executable='corgi_contact_leg_est',
        name='corgi_contact_leg_est',
        output='screen',
        # config_online.yaml is loaded internally by the node via yaml-cpp.
        # Only system parameters (use_sim_time, remappings) are passed here.
        parameters=[{'use_sim_time': False}],
    )
    return LaunchDescription([
        contact_leg_estimator_node,
    ])
