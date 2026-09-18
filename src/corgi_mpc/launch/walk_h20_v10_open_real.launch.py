"""Compatibility entry point; configuration lives in walk_open.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = os.path.join(get_package_share_directory('corgi_mpc'),
                          'launch', 'walk_open.launch.py')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    config_profile_arg = DeclareLaunchArgument('config_profile', default_value='real')
    args = {
        'environment': 'real',
        'gait': 'h20_v10',
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'config_profile': LaunchConfiguration('config_profile'),
    }
    return LaunchDescription([
        use_sim_time_arg,
        config_profile_arg,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
                                 launch_arguments=args.items()),
    ])
