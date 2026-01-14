#!/usr/bin/env python3
"""
Launch file for Contact Leg Estimator and Velocity Estimator (Online Version)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for contact leg estimator and velocity estimator."""
    
    # Declare launch arguments for velocity estimator
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
            'cutoff_freq': LaunchConfiguration('cutoff_freq'),
            'sample_rate': 1000.0,
            'position_topic': 'sim/data',
            'velocity_topic': 'odometry/velocity',
            'position_output_topic': 'odometry/position',
        }]
    )
    
    # Create contact leg estimator node
    contact_leg_estimator_node = Node(
        package='corgi_odometry',
        executable='corgi_contact_leg_est',
        name='corgi_contact_leg_est',
        output='screen',
        parameters=[],
        remappings=[
            # Uncomment and modify if you need to remap topics
            # ('motor/state', '/custom/motor/state'),
            # ('imu', '/custom/imu/filtered'),
            # ('odometry/position', '/custom/odometry/position'),
        ]
    )
    
    return LaunchDescription([
        cutoff_freq_arg,
        velocity_estimator_node,
        contact_leg_estimator_node,
    ])
