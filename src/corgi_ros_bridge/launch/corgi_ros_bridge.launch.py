from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corgi_ros_bridge',
            executable='corgi_ros_bridge',
            name='corgi_ros_bridge',
            output='screen',
        ),
    ])