#!/usr/bin/env python3
"""Simulation-only standalone contact-state estimation pipeline.

The contact estimator requires externally supplied position and velocity.
This launch derives both from the simulator's ``odom -> base_link`` TF.
Tuning is loaded internally from ``config/leg_odom/config_online.yaml``.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    velocity_estimator_node = Node(
        package='corgi_odometry',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sample_rate': 1000.0,
            'velocity_topic': 'sim/velocity',
            'position_topic': 'sim/position',
        }],
    )

    contact_leg_estimator_node = Node(
        package='corgi_odometry',
        executable='corgi_contact_leg_est',
        name='corgi_contact_leg_est',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        velocity_estimator_node,
        contact_leg_estimator_node,
    ])
