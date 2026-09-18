#!/usr/bin/env python3
"""Open-loop H20/V10 or WLW gait launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value, name):
    if value.lower() in ('true', '1'):
        return True
    if value.lower() in ('false', '0'):
        return False
    raise ValueError(f'{name} must be true or false')


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    environment = get('environment')
    gait = get('gait')
    if environment not in ('sim', 'real'):
        raise ValueError('environment must be sim or real')
    if gait not in ('h20_v10', 'wlw'):
        raise ValueError('gait must be h20_v10 or wlw')
    time_arg = get('use_sim_time')
    use_sim_time = environment == 'sim' if time_arg == 'auto' else _boolean(time_arg, 'use_sim_time')
    profile = get('config_profile')
    profile = environment if profile == 'auto' else profile
    if profile not in ('sim', 'real'):
        raise ValueError('config_profile must be sim or real')
    record_arg = get('record_bag')
    record_bag = environment == 'real' and gait == 'h20_v10' if record_arg == 'auto' else _boolean(record_arg, 'record_bag')

    params = [{'use_sim_time': use_sim_time}]
    actions = []
    if gait == 'h20_v10' and environment == 'real':
        actions.extend([
            Node(package='corgi_imu', executable='imu_node', name='imu_node',
                 output='screen', parameters=params),
            Node(package='corgi_force_estimation', executable='force_estimation_node',
                 name='force_estimation_node', output='screen', parameters=params),
            Node(package='corgi_odometry_legacy', executable='corgi_odometry_legacy',
                 name='corgi_odometry_legacy', output='screen', parameters=params),
            Node(package='corgi_odometry_legacy', executable='corgi_z_position_legacy',
                 name='corgi_z_position_legacy', output='screen', parameters=params),
        ])
    actions.append(Node(package='corgi_mpc',
                        executable='walk_h20_v10_open' if gait == 'h20_v10' else 'wlw_open',
                        name='corgi_mpc' if gait == 'h20_v10' else 'corgi_wlw_open',
                        output='screen', parameters=[{
                            'use_sim_time': use_sim_time,
                            'config_profile': profile,
                        }]))
    if record_bag:
        script = os.path.join(get_package_share_directory('corgi_mpc'),
                              'script', 'record_mpc_bag.py')
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=['python3', script, '--controller', 'open', '--gait', gait],
            output='screen')]))
    return actions


def generate_launch_description():
    defaults = (
        ('environment', 'sim', 'sim or real'),
        ('gait', 'h20_v10', 'h20_v10 or wlw'),
        ('use_sim_time', 'auto', 'auto, true or false'),
        ('config_profile', 'auto', 'auto, sim or real'),
        ('record_bag', 'auto', 'auto, true or false'),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in defaults),
        OpaqueFunction(function=_setup),
    ])
