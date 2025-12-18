from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")

    default_world = PathJoinSubstitution(
        [FindPackageShare("corgi_sim"), "worlds", "corgi_old_proto.wbt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo/Webots) clock",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Webots world file",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="realtime",
                description="Webots run mode: realtime | pause | fast",
            ),
            ExecuteProcess(
                cmd=["/usr/local/bin/webots", [f"--mode=", mode], world],
                output="screen",
                on_exit=Shutdown(),
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
        ]
    )
