#!/usr/bin/env python3
"""Real-robot ESEKF, Livox, FAST-LIO and fusion support nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_config_dir = os.path.join(
        get_package_share_directory('fast_lio'), 'config')
    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json')

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

    return LaunchDescription([
        imu_raw_node,
        corgi_leg_odom_node,
        livox_driver_node,
        fast_lio_node,
        odom_tf_relay_node,
        corgi_fusion_node,
        static_tf_base_to_lidar,
    ])
