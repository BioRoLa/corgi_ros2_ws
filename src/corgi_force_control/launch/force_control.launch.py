from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common_params = [{'use_sim_time': True}]
    force_feedback_ki = ParameterValue(LaunchConfiguration('force_feedback_ki'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('force_feedback_ki', default_value='0.0',
                              description='outer force loop gain (1/s), 0 = off'),
        Node(
            package='corgi_force_estimation',
            executable='force_estimation_node',
            name='force_estimation_node',
            parameters=common_params
        ),
        Node(
            package='corgi_force_control',
            executable='force_control_node',
            name='force_control_node',
            parameters=common_params + [{'force_feedback_ki': force_feedback_ki}]
        )
    ])
