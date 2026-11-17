#!/usr/bin/env python3
"""Open-loop walk: corgi_walk's WalkGait as position commands (no MPC).

gait:=hXX_vYY sets the stand height to XX cm and the speed to YY cm/s,
for example h20_v10 (0.20 m, 0.10 m/s).
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# WalkGait needs stand_height >= foot radius + step_height.
FOOT_RADIUS = 0.145


def _boolean(value, name):
    if value.lower() in ('true', '1'):
        return True
    if value.lower() in ('false', '0'):
        return False
    raise ValueError(f'{name} must be true or false')


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    environment = get('environment')
    if environment not in ('sim', 'real'):
        raise ValueError('environment must be sim or real')
    match = re.fullmatch(r'h(\d+)_v(\d+)', get('gait'))
    if not match:
        raise ValueError('gait must look like h20_v10 (stand height cm, speed cm/s)')
    time_arg = get('use_sim_time')
    use_sim_time = environment == 'sim' if time_arg == 'auto' else _boolean(time_arg, 'use_sim_time')

    stand_height = int(match.group(1)) / 100.0
    step_height = float(get('step_height'))
    if stand_height < FOOT_RADIUS + step_height:
        raise ValueError(f'stand height {stand_height:.2f} m needs step_height <= '
                         f'{stand_height - FOOT_RADIUS:.3f} m (foot radius {FOOT_RADIUS} m)')

    params = [{'use_sim_time': use_sim_time}]
    walk_params = [{
        'use_sim_time': use_sim_time,
        'stand_height': stand_height,
        'velocity': int(match.group(2)) / 100.0,
        'step_length': float(get('step_length')),
        'step_height': step_height,
        'duration': float(get('duration')),
    }]
    params_file = get('params_file')
    if params_file:
        walk_params.append(params_file)

    actions = [Node(package='corgi_mpc', executable='walk_open', name='corgi_walk_open',
                    output='screen', parameters=walk_params)]
    if environment == 'real':
        # Not used by the open-loop controller; started so the run can be logged.
        actions += [
            Node(package='corgi_imu', executable='imu_node', name='imu_node',
                 output='screen', parameters=params),
            Node(package='corgi_force_estimation', executable='force_estimation_node',
                 name='force_estimation_node', output='screen', parameters=params),
            Node(package='corgi_odometry_legacy', executable='corgi_odometry_legacy',
                 name='corgi_odometry_legacy', output='screen', parameters=params),
            Node(package='corgi_odometry_legacy', executable='corgi_z_position_legacy',
                 name='corgi_z_position_legacy', output='screen', parameters=params),
        ]
    return actions


def generate_launch_description():
    defaults = (
        ('environment', 'sim', 'sim or real'),
        ('gait', 'h20_v10', 'hXX_vYY: stand height XX cm, speed YY cm/s'),
        ('step_length', '0.2', 'hip travel per gait cycle (m)'),
        ('step_height', '0.04', 'swing height (m), at most stand height - 0.145'),
        ('duration', '20.0', 'walking time, 1 s ramp-up included (s)'),
        ('use_sim_time', 'auto', 'auto (follows environment), true or false'),
        ('params_file', '', 'optional YAML with walk_open parameters (kp, kd, ...)'),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in defaults),
        OpaqueFunction(function=_setup),
    ])
