"""Compatibility entry point; configuration lives in walk_closed.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = os.path.join(get_package_share_directory('corgi_mpc'),
                          'launch', 'walk_closed.launch.py')
    args = {
        'environment': 'real',
        'stop_mode': 'distance',
        'state_source': 'esekf',
    }
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
                                 launch_arguments=args.items()),
    ])
