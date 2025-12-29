from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            Node(
                package="corgi_data_recorder",
                executable="corgi_data_recorder",
                name="corgi_data_recorder",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="corgi_sim",
                executable="corgi_sim_trq",
                name="corgi_sim_trq",
                output="screen",
                respawn=True,
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="corgi_force_estimation",
                executable="force_estimation",
                name="force_estimation",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="corgi_force_control",
                executable="force_control",
                name="force_control",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )