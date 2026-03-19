#!/usr/bin/env python3
"""
Launch file for Leg Odometry with IMU noise simulation (Simulation Version)

Nodes launched:
  1. velocity_estimator  — derives velocity from TF (odom→base_link)
  2. imu_noise_sim       — adds realistic 3DM-CX5-AHRS noise to clean sim IMU
  3. corgi_leg_odom      — leg odometry + ES-EKF (subscribes to noisy IMU)

Topic wiring:
  simulator → "imu"  →(remap to imu_raw)→ imu_noise_sim → "imu_noisy"
  corgi_leg_odom subscribes to "imu_noisy" (remapped from "imu")
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for contact leg estimator and velocity estimator."""
    
    cutoff_freq_arg = DeclareLaunchArgument(
        'cutoff_freq',
        default_value='30.0',
        description='Low-pass filter cutoff frequency (Hz)'
    )
    
    # Create velocity estimator node
    velocity_estimator_node = Node(
        package='corgi_odometry',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cutoff_freq': LaunchConfiguration('cutoff_freq'),
            'sample_rate': 1000.0,
            'position_topic': 'sim/data',
            'velocity_topic': 'odometry/velocity',
            'position_output_topic': 'odometry/position',
        }]
    )

    # IMU noise simulator node
    # Subscribes to clean sim IMU ("imu" remapped to "imu_raw"), publishes noisy IMU to "imu_noisy"
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
            # Simulator publishes ground-truth IMU on "imu";
            # remap so the noise node reads it as "imu_raw"
            ('imu_raw', 'imu'),
        ]
    )
    
    # Create leg odom node — subscribes to "imu_noisy" instead of "imu"
    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ('imu', 'imu_noisy'),
        ]
    )
    return LaunchDescription([
        cutoff_freq_arg,
        velocity_estimator_node,
        imu_noise_sim_node,
        corgi_leg_odom_node,
    ])
