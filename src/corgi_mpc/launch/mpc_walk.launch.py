#!/usr/bin/env python3
"""Closed-loop MPC walk: mpc_walk with force estimation, force control and odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    state_source = get('state_source')
    if environment not in ('sim', 'real'):
        raise ValueError('environment must be sim or real')
    if state_source not in ('odom_legacy', 'sim_driver'):
        raise ValueError('state_source must be odom_legacy or sim_driver')
    if state_source == 'sim_driver' and environment != 'sim':
        raise ValueError('state_source:=sim_driver requires environment:=sim')
    time_arg = get('use_sim_time')
    use_sim_time = environment == 'sim' if time_arg == 'auto' else _boolean(time_arg, 'use_sim_time')
    if use_sim_time != (environment == 'sim'):
        raise ValueError('use_sim_time must match environment for force control and estimation')

    params = [{'use_sim_time': use_sim_time}]
    mpc_params = [{'use_sim_time': use_sim_time, 'state_source': state_source}]
    params_file = get('params_file')
    if params_file:
        mpc_params.append(params_file)

    actions = [
        # Legacy odometry: body position, velocity and hip height. With
        # state_source:=sim_driver the controller uses the simulator instead.
        Node(package='corgi_odometry_legacy', executable='corgi_odometry_legacy',
             name='corgi_odometry_legacy', output='screen', parameters=params),
        Node(package='corgi_odometry_legacy', executable='corgi_z_position_legacy',
             name='corgi_z_position_legacy', output='screen', parameters=params),
        Node(package='corgi_force_estimation', executable='force_estimation_node',
             name='force_estimation_node', output='screen', parameters=params),
        Node(package='corgi_force_control', executable='force_control_node',
             name='force_control_node', output='screen', parameters=params),
        Node(package='corgi_mpc', executable='mpc_walk', name='corgi_mpc',
             output='screen', parameters=mpc_params),
    ]
    if environment == 'real':
        actions.append(Node(package='corgi_imu', executable='imu_node', name='imu_node',
                            output='screen', parameters=params))
    return actions


def generate_launch_description():
    defaults = (
        ('environment', 'sim', 'sim or real'),
        ('state_source', 'odom_legacy', 'odom_legacy, or sim_driver (Webots ground truth)'),
        ('use_sim_time', 'auto', 'auto (follows environment), true or false'),
        ('params_file', '', 'optional YAML with mpc_walk parameters (gait, gains, stop)'),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in defaults),
        OpaqueFunction(function=_setup),
    ])
