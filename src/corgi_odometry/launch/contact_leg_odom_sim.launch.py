#!/usr/bin/env python3
"""
Launch file for Leg Odometry with IMU noise simulation (Simulation Version)

Nodes launched:
  1. velocity_estimator  — derives velocity from TF (odom→base_link)
  2. imu_noise_sim       — adds realistic 3DM-CX5-AHRS noise to clean sim IMU
  3. corgi_leg_odom      — leg odometry + ES-EKF (subscribes to noisy IMU)

Tunable parameters (observer cutoff freq, contact thresholds, ESEKF noise)
are loaded by corgi_leg_odom at startup from:
  share/corgi_odometry/config/config_online.yaml
Edit that file to change values without recompiling.

Topic wiring:
  simulator → "imu"  →(remap to imu_raw)→ imu_noise_sim → "imu_noisy"
  corgi_leg_odom subscribes to "imu_noisy" (remapped from "imu")
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
            'velocity_topic': 'odometry/velocity',
            'position_output_topic': 'odometry/position',
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
            'input_topic': 'imu_raw',
            'output_topic': 'imu_noisy',
        }],
        remappings=[
            ('imu_raw', 'imu'),
        ]
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
