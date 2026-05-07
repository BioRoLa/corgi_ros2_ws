from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    common_params = [{'use_sim_time': False}]
    return LaunchDescription([

        Node(
            package='corgi_odometry_legacy',
            executable='corgi_odometry_legacy',
            name='corgi_odometry_legacy',
            parameters=common_params,
            output='screen'
        ),

        Node(
            package='corgi_odometry_legacy',
            executable='corgi_z_position_legacy',
            name='corgi_z_position_legacy',
            parameters=common_params,
            output='screen'
        )
    ])
