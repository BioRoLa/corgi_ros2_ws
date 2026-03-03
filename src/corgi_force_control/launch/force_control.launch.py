from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = [{'use_sim_time': False}]

    return LaunchDescription([

        Node(
            package='corgi_force_control',
            executable='force_control_node',
            name='force_control_node',
            parameters=common_params
        ),

        Node(
            package='corgi_force_estimation',
            executable='force_estimation_node',
            name='force_estimation_node',
            parameters=common_params
        )
    ])