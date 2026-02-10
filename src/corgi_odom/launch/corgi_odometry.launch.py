from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corgi_odom',
            executable='corgi_odometry',
            name='corgi_odometry',
            output='screen'
        ),
        Node(
            package='corgi_odom',
            executable='corgi_z_position',
            name='corgi_z_position',
            output='screen'
        )
    ])
