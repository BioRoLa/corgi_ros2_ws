#!/usr/bin/env python3
"""Simulation launch for leg odometry with IMU noise injection.

Topic wiring:
    simulator /imu -> imu_noise_sim -> imu_noisy
    corgi_leg_odom subscribes to imu_noisy (remapped from imu)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_only = LaunchConfiguration('imu_only')
    imu_seed = LaunchConfiguration('imu_seed')

    imu_only_arg = DeclareLaunchArgument(
        'imu_only', default_value='false',
        description='Run ESEKF prediction only; disable leg, GMO, ZUPT and fusion updates.',
    )
    imu_seed_arg = DeclareLaunchArgument(
        'imu_seed', default_value='42',
        description='Deterministic IMU noise seed.',
    )

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
        }]
    )

    imu_noise_sim_node = Node(
        package='corgi_odometry',
        executable='imu_noise_sim',
        name='imu_noise_sim',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Fixed seed makes the CX5 noise/bias realization reproducible.
            'seed': ParameterValue(imu_seed, value_type=int),
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
        parameters=[{
            'use_sim_time': True,
            'imu_only': ParameterValue(imu_only, value_type=bool),
        }],
        remappings=[
            ('imu', 'imu_noisy'),
        ]
    )
    return LaunchDescription([
        imu_only_arg,
        imu_seed_arg,
        velocity_estimator_node,
        imu_noise_sim_node,
        corgi_leg_odom_node,
    ])
