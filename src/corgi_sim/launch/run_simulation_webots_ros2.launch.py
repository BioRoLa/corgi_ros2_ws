#!/usr/bin/env python3

import os
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('corgi_sim')
    world = os.path.join(package_dir, 'worlds', 'corgi_origin.wbt')

    webots = WebotsLauncher(
        world=world,
        mode='realtime',
        ros2_supervisor=True
    )

    # Robot driver - this will create the ROS2 interface for the robot
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('webots_ros2_driver'), 
                        'launch', 'robot_launch.py')
        ]),
        launch_arguments={
            'robot_name': 'CORGI',
        }.items()
    )

    # Corgi simulation node (ROS2)
    corgi_sim_node = Node(
        package='corgi_sim',
        executable='corgi_sim_trq',
        name='corgi_sim',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        corgi_sim_node,
    ])
