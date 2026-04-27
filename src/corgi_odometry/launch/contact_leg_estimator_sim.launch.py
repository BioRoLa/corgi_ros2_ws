#!/usr/bin/env python3
"""
Launch file for Contact Leg Estimator + Velocity Estimator (Simulation Version)

Tunable parameters (observer cutoff freq, contact thresholds) are loaded
by corgi_contact_leg_est at startup from:
  share/corgi_odometry/config/config_online.yaml
Edit that file to change values without recompiling.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('corgi_odometry'), 'config', 'config_online.yaml'
    )

    velocity_estimator_node = Node(
        package='corgi_odometry',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sample_rate': 1000.0,
            'position_topic': 'sim/data',
            'velocity_topic': 'sim/odometry',
            'position_topic': 'sim/position',
        }]
    )

    contact_leg_estimator_node = Node(
        package='corgi_odometry',
        executable='corgi_contact_leg_est',
        name='corgi_contact_leg_est',
        output='screen',
        # config_online.yaml is loaded internally by the node via yaml-cpp.
        # Only system parameters (use_sim_time, remappings) are passed here.
        parameters=[{'use_sim_time': True}],
        remappings=[
            # ('motor/state', '/custom/motor/state'),
        ]
    )
    return LaunchDescription([
        velocity_estimator_node,
        contact_leg_estimator_node,
    ])
