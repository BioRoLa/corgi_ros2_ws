from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corgi_transform',
            executable='transform_main',
            name='transform_main',
            output='screen'
        )
    ])
