from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='corgi_panel',
            executable='corgi_control_panel_dev.py',
            name='corgi_control_panel',
            output='screen',
        ),
    ])
