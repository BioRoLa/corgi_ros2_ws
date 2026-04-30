from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Find the package share directory and construct path to the script
    imu_script = PathJoinSubstitution([
        FindPackageShare('corgi_imu'),
        'lib',
        'corgi_imu',
        'imu_node.sh'
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[imu_script],
            name='imu_node',
            output='screen'
        )
    ])
