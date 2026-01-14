#!/usr/bin/env python3
"""
Launch file for Velocity Estimator Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for velocity estimator."""
    
    # Declare launch arguments
    cutoff_freq_arg = DeclareLaunchArgument(
        'cutoff_freq',
        default_value='30.0',
        description='Low-pass filter cutoff frequency (Hz)'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='1000.0',
        description='Sample rate (Hz)'
    )
    
    position_topic_arg = DeclareLaunchArgument(
        'position_topic',
        default_value='sim/data',
        description='Input topic for position data (SimDataStamped)'
    )
    
    velocity_topic_arg = DeclareLaunchArgument(
        'velocity_topic',
        default_value='odometry/velocity',
        description='Output topic for velocity data'
    )
    
    position_output_topic_arg = DeclareLaunchArgument(
        'position_output_topic',
        default_value='odometry/position',
        description='Output topic for position data (Vector3)'
    )
    
    # Create velocity estimator node
    velocity_estimator_node = Node(
        package='corgi_odometry',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[{
            'cutoff_freq': LaunchConfiguration('cutoff_freq'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'position_topic': LaunchConfiguration('position_topic'),
            'velocity_topic': LaunchConfiguration('velocity_topic'),
            'position_output_topic': LaunchConfiguration('position_output_topic'),
        }]
    )
    
    return LaunchDescription([
        cutoff_freq_arg,
        sample_rate_arg,
        position_topic_arg,
        velocity_topic_arg,
        position_output_topic_arg,
        velocity_estimator_node,
    ])
