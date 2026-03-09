from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    common_params = [{'use_sim_time': False}]
    return LaunchDescription([

        Node(
            package='corgi_odom',
            executable='corgi_odometry',
            name='corgi_odometry',
            parameters=common_params,
            output='screen'
        ),

        Node(
            package='corgi_odom',
            executable='corgi_z_position',
            name='corgi_z_position',
            parameters=common_params,
            output='screen'
        )
    ])
