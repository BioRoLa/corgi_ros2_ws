#!/usr/bin/env python3
"""Real-robot MPC launch — odom_legacy mode.

Node wiring
-----------
  corgi_imu/imu_node          →  /imu  (corgi_msgs/ImuStamped, gravity-compensated)
  corgi_odometry_legacy       sub /imu, /motor/state, /trigger
                              pub /odometry/legacy/position, /odometry/legacy/velocity,
                                  /odometry/legacy/contact
  corgi_z_position_legacy     pub /odometry/legacy/z_position_hip
  corgi_force_estimation      sub /motor/state
                              pub /force/state
  corgi_force_control         sub /force/state, /impedance/command
                              pub /motor/command
  corgi_mpc (walk_h20_v10_closed)
                              state_source = odom_legacy
                              sub /motor/state, /imu, /trigger,
                                  /odometry/legacy/position, /odometry/legacy/velocity,
                                  /odometry/legacy/z_position_hip
                              pub /impedance/command, /walk/swing_phase

Prerequisites
-------------
  Motor driver node must be started separately before launching this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('corgi_mpc')
    bag_script = os.path.join(package_share_dir, 'script', 'mpc_closed_legacy_bag.sh')

    real_params = [{'use_sim_time': False}]

    # ── IMU node (AHRS filter, gravity-compensated) ──────────────────────────
    imu_node = Node(
        package='corgi_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=real_params,
    )

    # ── Legacy odometry (kinematic + IMU integration) ────────────────────────
    odometry_node = Node(
        package='corgi_odometry_legacy',
        executable='corgi_odometry_legacy',
        name='corgi_odometry_legacy',
        output='screen',
        parameters=real_params,
    )

    z_position_node = Node(
        package='corgi_odometry_legacy',
        executable='corgi_z_position_legacy',
        name='corgi_z_position_legacy',
        output='screen',
        parameters=real_params,
    )

    # ── Force estimation and control ─────────────────────────────────────────
    force_estimation_node = Node(
        package='corgi_force_estimation',
        executable='force_estimation_node',
        name='force_estimation_node',
        output='screen',
        parameters=real_params,
    )

    force_control_node = Node(
        package='corgi_force_control',
        executable='force_control_node',
        name='force_control_node',
        output='screen',
        parameters=real_params,
    )

    # ── MPC controller ────────────────────────────────────────────────────────
    mpc_node = Node(
        package='corgi_mpc',
        executable='walk_h20_v10_closed',
        name='corgi_mpc',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'config_profile': 'real',
            'state_source': 'odom_legacy',
        }],
    )

    # ── Bag recorder (delayed 3 s to allow node startup) ─────────────────────
    bag_record_process = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', bag_script],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        imu_node,
        odometry_node,
        z_position_node,
        force_estimation_node,
        force_control_node,
        mpc_node,
        bag_record_process,
    ])
