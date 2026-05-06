from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="corgi_legwheel_ros",
            executable="legwheel_smoke",
            name="legwheel_smoke",
            output="screen",
            parameters=[
                {
                    "leg_index": 0,
                    "theta_deg": 75.0,
                    "beta_deg": 90.0,
                    "gamma_deg": 0.0,
                }
            ],
        )
    ])
