#!/usr/bin/env python3
"""
Launch file for Contact Leg Estimator (Online Version)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for contact leg estimator."""
    
    contact_leg_estimator_node = Node(
        package='corgi_contact_leg_est',
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
        contact_leg_estimator_node,
    ])
