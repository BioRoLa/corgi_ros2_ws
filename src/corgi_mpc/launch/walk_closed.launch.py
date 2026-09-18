#!/usr/bin/env python3
"""Closed-loop MPC launch with explicit stop and state-source selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    stop_mode = get('stop_mode')
    state_source = get('state_source')
    contact_source = get('contact_source')
    if environment not in ('sim', 'real'):
        raise ValueError('environment must be sim or real')
    if stop_mode not in ('time', 'distance'):
        raise ValueError('stop_mode must be time or distance')
    if state_source not in ('odom_legacy', 'sim_driver', 'esekf'):
        raise ValueError('invalid state_source')
    if state_source == 'esekf' and environment != 'real':
        raise ValueError('esekf stack is configured for the real robot only')
    if state_source == 'sim_driver' and environment != 'sim':
        raise ValueError('sim_driver requires environment:=sim')
    if contact_source not in ('gait', 'gmo'):
        raise ValueError('contact_source must be gait or gmo')
    if contact_source == 'gmo' and state_source != 'esekf':
        raise ValueError('contact_source:=gmo requires state_source:=esekf')

    use_sim_time_arg = get('use_sim_time')
    use_sim_time = environment == 'sim' if use_sim_time_arg == 'auto' else _boolean(use_sim_time_arg, 'use_sim_time')
    if state_source == 'esekf' and use_sim_time:
        raise ValueError('the real-robot ESEKF stack requires use_sim_time:=false')
    profile = get('config_profile')
    profile = environment if profile == 'auto' else profile
    if profile not in ('sim', 'real'):
        raise ValueError('config_profile must be sim or real')
    record_arg = get('record_bag')
    record_bag = environment == 'real' if record_arg == 'auto' else _boolean(record_arg, 'record_bag')
    raw_lidar = _boolean(get('record_raw_lidar'), 'record_raw_lidar')
    if raw_lidar and state_source != 'esekf':
        raise ValueError('record_raw_lidar requires state_source:=esekf')

    params = [{'use_sim_time': use_sim_time}]
    actions = []
    if state_source in ('odom_legacy', 'sim_driver'):
        actions.extend([
            Node(package='corgi_odometry_legacy', executable='corgi_odometry_legacy',
                 name='corgi_odometry_legacy', output='screen', parameters=params),
            Node(package='corgi_odometry_legacy', executable='corgi_z_position_legacy',
                 name='corgi_z_position_legacy', output='screen', parameters=params),
        ])
        if environment == 'real':
            actions.append(Node(package='corgi_imu', executable='imu_node',
                                name='imu_node', output='screen', parameters=params))
    else:
        stack = os.path.join(get_package_share_directory('corgi_mpc'),
                             'launch', 'esekf_stack.launch.py')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(stack)))

    actions.extend([
        Node(package='corgi_force_estimation', executable='force_estimation_node',
             name='force_estimation_node', output='screen', parameters=params),
        Node(package='corgi_force_control', executable='force_control_node',
             name='force_control_node', output='screen', parameters=params),
        Node(package='corgi_mpc',
             executable='walk_closed_time' if stop_mode == 'time' else 'walk_closed_dist',
             name='corgi_mpc', output='screen', parameters=[{
                 'use_sim_time': use_sim_time,
                 'config_profile': profile,
                 'state_source': state_source,
                 'contact_source': contact_source,
             }]),
    ])

    if record_bag:
        script = os.path.join(get_package_share_directory('corgi_mpc'),
                              'script', 'record_mpc_bag.py')
        cmd = ['python3', script, '--controller', 'closed',
               '--state-source', state_source, '--contact-source', contact_source]
        if raw_lidar:
            cmd.append('--raw-lidar')
        actions.append(TimerAction(period=15.0 if state_source == 'esekf' else 3.0,
                                   actions=[ExecuteProcess(cmd=cmd, output='screen')]))
    return actions


def generate_launch_description():
    defaults = (
        ('environment', 'sim', 'sim or real'),
        ('stop_mode', 'time', 'time or distance'),
        ('state_source', 'odom_legacy', 'odom_legacy, sim_driver or esekf'),
        ('contact_source', 'gait', 'gait or gmo'),
        ('use_sim_time', 'auto', 'auto, true or false'),
        ('config_profile', 'auto', 'auto, sim or real'),
        ('record_bag', 'auto', 'auto, true or false'),
        ('record_raw_lidar', 'false', 'Record raw Livox topics in the ESEKF bag'),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in defaults),
        OpaqueFunction(function=_setup),
    ])
