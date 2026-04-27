#!/usr/bin/env python3
"""Simulation launch for leg odometry with IMU noise injection.

Topic wiring:
    simulator /imu -> imu_noise_sim -> imu_noisy
    corgi_leg_odom subscribes to imu_noisy (remapped from imu)
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
            'position_topic': 'sim/data',
            'velocity_topic': 'sim/odometry',
            'position_topic': 'sim/position',
        }]
    )

    imu_noise_sim_node = Node(
        package='corgi_odometry',
        executable='imu_noise_sim',
        name='imu_noise_sim',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'seed': 0,
            'sample_rate': 1000.0,
            'input_topic': '/imu',
            'output_topic': 'imu_noisy',
        }]
    )

    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        # config_online.yaml is loaded internally by the node via yaml-cpp.
        # Only system parameters (use_sim_time, remappings) are passed here.
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('imu', 'imu_noisy'),
        ]
    )
    return LaunchDescription([
        velocity_estimator_node,
        imu_noise_sim_node,
        corgi_leg_odom_node,
    ])
