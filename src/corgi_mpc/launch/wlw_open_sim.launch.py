from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    config_profile_arg = DeclareLaunchArgument(
        'config_profile',
        default_value='sim',
        description='Gait config profile: sim or real',
    )

    wlw_open_node = Node(
        package='corgi_mpc',
        executable='wlw_open',
        name='corgi_wlw_open',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_profile': LaunchConfiguration('config_profile'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_profile_arg,
        wlw_open_node,
    ])