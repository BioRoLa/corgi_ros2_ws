#!/usr/bin/env python3
"""Real-robot MPC launch — esekf mode.

Node wiring
-----------
  corgi_imu/imu_raw_node      →  /imu_raw  (corgi_msgs/ImuStamped, raw with gravity)
  corgi_leg_odom (ESEKF)      sub /imu_raw [as /imu], /motor/state, /trigger
                              pub /ekf (nav_msgs/Odometry, odom→base_link)
                              broadcast TF odom→base_link
  livox_ros_driver2_node      pub /livox/lidar, /livox/imu
  fastlio_mapping             sub /livox/lidar, /livox/imu
                              pub /Odometry (camera_init→body)
                              broadcast TF camera_init→body
  odom_tf_relay               sub /Odometry, apply hardcoded T_body←base_link
                              pub /lidar_odom (camera_init→base_link)
  corgi_fusion_node           sub /ekf, /lidar_odom
                              pub /odom_mapping, /fusion/bv
                              broadcast TF map→odom
  corgi_force_estimation      sub /motor/state  →  pub /force/state
  corgi_force_control         sub /force/state, /impedance/command  →  pub /motor/command
  corgi_mpc (walk_closed_time or walk_closed_dist)
                              state_source = esekf
                              sub /motor/state, /imu_raw, /trigger, /ekf
                              pub /impedance/command, /walk/swing_phase

TF tree
-------
  map ──(fusion)──► odom ──(leg_odom)──► base_link ──(static)──► mid360_optical

FAST-LIO also broadcasts camera_init→body as a separate TF chain.  The relay
converts that pose at the message level only; it does not broadcast
camera_init→base_link.

Prerequisites
-------------
  Motor driver node must be started separately before launching this file.
  FAST-LIO needs ~10–15 s to initialize; bag recording starts after 15 s.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir  = get_package_share_directory('corgi_mpc')
    fast_lio_config_dir = os.path.join(
        get_package_share_directory('fast_lio'), 'config')
    livox_config_path  = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json')

    bag_script = os.path.join(package_share_dir, 'script', 'mpc_closed_esekf_bag.sh')

    # ── 1. IMU raw node (no gravity compensation) ────────────────────────────
    imu_raw_node = Node(
        package='corgi_imu',
        executable='imu_raw_node',
        name='imu_raw_node',
        output='screen',
    )

    # ── 2. Leg odometry (ESEKF inner filter) ────────────────────────────────
    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/imu', '/imu_raw'),
        ],
    )

    # ── 3. Livox MID-360 driver ──────────────────────────────────────────────
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,           # livox_ros_driver2/msg/CustomMsg (FAST-LIO)
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'mid360_optical',
            'user_config_path': livox_config_path,
        }],
    )

    # ── 4. FAST-LIO (LiDAR-inertial odometry) ───────────────────────────────
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(fast_lio_config_dir, 'mid360.yaml'),
            {'use_sim_time': False},
        ],
    )

    # ── 5. Odometry frame relay (body → base_link) ───────────────────────────
    odom_tf_relay_node = Node(
        package='corgi_odometry',
        executable='odom_tf_relay.py',
        name='odom_tf_relay',
        output='screen',
        parameters=[{
            'input_topic':   '/Odometry',
            'output_topic':  '/lidar_odom',
            'source_frame':  'body',
            'target_frame':  'base_link',
            # T_{body←base_link} (pre-computed from CAD chain)
            'static_t_x': -0.010999,
            'static_t_y':  0.040636,
            'static_t_z': -0.179324,
            'static_q_w':  0.696361,
            'static_q_x': -0.122799,
            'static_q_y': -0.122800,
            'static_q_z': -0.696363,
        }],
    )

    # ── 6. Fusion node (outer EKF: leg ESEKF + FAST-LIO) ────────────────────
    corgi_fusion_node = Node(
        package='corgi_odometry',
        executable='corgi_fusion_node',
        name='corgi_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── 7. Static TF: base_link → mid360_optical ────────────────────────────
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_mid360',
        output='screen',
        # x y z yaw(Z) pitch(Y) roll(X)  parent  child
        arguments=['0.1365', '0.0', '0.1881', '1.5708', '0.0', '0.3491',
                   'base_link', 'mid360_optical'],
    )

    # ── 8. Force estimation and control ─────────────────────────────────────
    force_estimation_node = Node(
        package='corgi_force_estimation',
        executable='force_estimation_node',
        name='force_estimation_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    force_control_node = Node(
        package='corgi_force_control',
        executable='force_control_node',
        name='force_control_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── 9. MPC controller ────────────────────────────────────────────────────
    mpc_node = Node(
        package='corgi_mpc',
        executable='walk_closed_dist',
        name='corgi_mpc',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'config_profile': 'real',
            'state_source': 'esekf',
        }],
    )

    # ── Bag recorder (delayed 15 s to allow FAST-LIO to initialize) ──────────
    bag_record_process = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', bag_script],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        imu_raw_node,
        corgi_leg_odom_node,
        livox_driver_node,
        fast_lio_node,
        odom_tf_relay_node,
        corgi_fusion_node,
        static_tf_base_to_lidar,
        force_estimation_node,
        force_control_node,
        mpc_node,
        bag_record_process,
    ])
