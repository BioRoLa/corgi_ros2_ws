#!/usr/bin/env python3
"""Bag-replay launch for leg odometry + fusion (no hardware required).

Usage (on analysis PC):
    1. Terminal A — start nodes:
         ros2 launch corgi_odometry odom_fusion_replay.launch.py

    2. Terminal B — replay bag (raw topics only, NO /ekf or /odom_mapping):
         ros2 bag play <bag_path> \
             --clock \
             --topics /imu_raw /motor/state /trigger /gmo/contact_state /lidar_odom \
             --rate 1.0

    3. Terminal C (optional) — record new bag:
         bash <pkg>/script/odom_fusion_bag.sh <output_path>

Nodes started:
    corgi_leg_odom  — processes /imu_raw, /motor/state, /trigger,
                      /gmo/contact_state → publishes /ekf
    corgi_fusion_node — subscribes /ekf + /lidar_odom → publishes /odom_mapping

Both nodes use use_sim_time=true to sync with bag playback clock.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── 1. Leg odometry (ESEKF) ──────────────────────────────────────────────
    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/imu', '/imu_raw'),
        ]
    )

    # ── 2. Fusion node ───────────────────────────────────────────────────────
    # Subscribes: /ekf (leg ESEKF), /lidar_odom (from bag)
    # Publishes:  /odom_mapping, /fusion/bv
    # After the coordinate-frame fix, /odom_mapping is expressed in the same
    # initial frame as /ekf (odom frame, starting at the robot initial pose).
    corgi_fusion_node = Node(
        package='corgi_odometry',
        executable='corgi_fusion_node',
        name='corgi_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        corgi_leg_odom_node,
        corgi_fusion_node,
    ])
