#!/usr/bin/env python3
"""Real-robot launch: leg odometry + Livox MID-360 + FAST-LIO + fusion.

Topic wiring
------------
  corgi_imu/imu_raw_node  →  /imu_raw  (corgi_msgs/ImuStamped)
  corgi_leg_odom          sub /imu_raw [as imu]
                          pub /ekf     (nav_msgs/Odometry, odom→base_link)
                          broadcast TF  odom → base_link

  livox_ros_driver2       pub /livox/lidar  (CustomMsg, frame=mid360_optical)
                          pub /livox/imu    (sensor_msgs/Imu)

  fast_lio (fastlio_mapping)
                          sub /livox/lidar, /livox/imu
                          pub /Odometry  (nav_msgs/Odometry, camera_init→body)
                          broadcast TF  camera_init → body

  odom_tf_relay           sub /Odometry  (camera_init→body)
                          TF lookup body→base_link (from static TFs)
                          pub /lidar_odom (nav_msgs/Odometry, camera_init→base_link)

  corgi_fusion_node       sub /ekf, /lidar_odom  (both in base_link child frame)
                          pub /odom_mapping (nav_msgs/Odometry, map→base_link)
                          broadcast TF  map → odom

TF tree (at steady-state)
--------------------------
  map ──(fusion)──► odom ──(leg_odom)──► base_link
                                         └──(static)──► mid360_optical
                                                        └──(static)──► body

NOTE  fast_lio also broadcasts camera_init→body independently.
      camera_init ≈ map (both originate at the robot start pose).

Known limitations
-----------------
* corgi_leg_odom additionally requires motor/state and trigger topics
  from the motor driver — start the motor driver node separately before
  launching this file.
* TODO: Retune fusion noise params for real-robot performance.
  config/fusion/config_fusion.yaml was originally tuned for simulation
  (fake_lidar σ_p = 0.02 m).  After collecting real data, adjust r_p and
  r_th to match actual FAST-LIO position/orientation noise.
* child-frame correction: fast_lio outputs pose in 'body' (IMU centre)
  while leg_odom outputs in 'base_link' (robot centre).
  odom_tf_relay converts /Odometry (body) → /lidar_odom (base_link) at the
  launch level so FusionNode algorithm remains unchanged from simulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Start the built-in odometry rosbag recorder.',
    )

    fast_lio_config_dir = os.path.join(
        get_package_share_directory('fast_lio'), 'config')

    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json')

    # ── 1. IMU raw node ──────────────────────────────────────────────────────
    # Reads hardware IMU and publishes corgi_msgs/ImuStamped on /imu_raw.
    imu_raw_node = Node(
        package='corgi_imu',
        executable='imu_raw_node',
        name='imu_raw_node',
        output='screen',
    )

    # ── 2. Leg odometry (ESEKF) ──────────────────────────────────────────────
    # Requires: /imu_raw, motor/state, trigger.
    # The motor driver is started separately — ensure it is running first.
    # Publishes: /ekf (nav_msgs/Odometry, frame odom→base_link).
    corgi_leg_odom_node = Node(
        package='corgi_odometry',
        executable='corgi_leg_odom',
        name='corgi_leg_odom',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/imu', '/imu_raw'),
        ]
    )

    # ── 3. Livox MID-360 driver ──────────────────────────────────────────────
    # Publishes:
    #   /livox/lidar  (livox_ros_driver2/msg/CustomMsg,  frame=mid360_optical)
    #   /livox/imu    (sensor_msgs/Imu)
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,            # livox_ros_driver2::msg::CustomMsg (required by FAST-LIO)
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'mid360_optical',  # matches TF tree
            'user_config_path': livox_config_path,
        }]
    )

    # ── 4. FAST-LIO (LiDAR-inertial odometry) ───────────────────────────────
    # Config: config/mid360.yaml  (lid_topic=/livox/lidar, imu_topic=/livox/imu)
    # Publishes: /Odometry (nav_msgs/Odometry, camera_init→body)
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

    # ── 4b. Odometry frame relay ─────────────────────────────────────────────
    # fast_lio outputs /Odometry with child_frame_id='body' (IMU centre).
    # In simulation fake_lidar_odom already published in base_link frame,
    # so no conversion was needed there.
    # Here we re-express the pose into 'base_link' using a hardcoded static
    # transform T_{body←base_link}, pre-computed from the known chain:
    #   base_link→mid360_optical: t=[0.1365,0,0.1881] RPY=[0.3491,0,1.5708]
    #   mid360_optical→body:      t=[0.011,0.02329,-0.04412] RPY=[0,0,0]
    #
    # NOTE: We do NOT publish a mid360_optical→body static TF here.
    #   fast_lio already broadcasts camera_init→body dynamically.
    #   A second static mid360_optical→body would give 'body' two parents,
    #   which TF2 rejects (splits into disconnected trees).
    #
    # Math (T_{world←base_link} = T_{world←body} * T_{body←base_link}):
    #   p_bl = p_body + R(q_body) @ t_bl_in_body
    #   q_bl = q_body ⊗ q_body_from_bl
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
            # T_{body←base_link} (pre-computed, see comment above)
            'static_t_x': -0.010999,
            'static_t_y':  0.040636,
            'static_t_z': -0.179324,
            'static_q_w':  0.696361,
            'static_q_x': -0.122799,
            'static_q_y': -0.122800,
            'static_q_z': -0.696363,
        }]
    )

    # ── 5. Fusion node ───────────────────────────────────────────────────────
    # Subscribes: /ekf (leg ESEKF), /lidar_odom (FAST-LIO)
    # Publishes:  /odom_mapping, /fusion/bv
    # Broadcasts: TF map → odom
    # Noise params loaded from config/fusion/config_fusion.yaml.
    # TODO: Retune r_p and r_th in config_fusion.yaml based on real FAST-LIO
    #       measurement noise (current values were tuned for simulation only).
    corgi_fusion_node = Node(
        package='corgi_odometry',
        executable='corgi_fusion_node',
        name='corgi_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ── 6. Static TF: base_link → mid360_optical ────────────────────────────
    # Translation (from CAD): x=0.1365 m, y=0.0 m, z=0.1881 m
    # Rotation: yaw=1.5708 rad (90° CCW around Z), roll=0.3491 rad (20° around new X)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_mid360',
        output='screen',
        # x y z yaw(Z) pitch(Y) roll(X)  parent  child
        arguments=['0.1365', '0.0', '0.1881', '1.5708', '0.0', '0.3491',
                   'base_link', 'mid360_optical'],
    )

    # NOTE: mid360_optical→body static TF is intentionally omitted.
    # fast_lio broadcasts camera_init→body dynamically; a second static
    # publisher for mid360_optical→body would give 'body' two parents,
    # causing TF2 to report disconnected trees.  The relay node uses a
    # hardcoded transform instead (see Node 4b above).

    # ── Auto-trigger: send enable=true to /trigger after a short delay ──────
    # corgi_leg_odom waits for a TriggerStamped on /trigger before processing.
    # This node sends it once automatically so no manual command is needed.
    auto_trigger_node = Node(
        package='corgi_odometry',
        executable='auto_trigger.py',
        name='auto_trigger',
        output='screen',
        parameters=[{'delay_sec': 8.0}],
    )

    # ── Bag recorder: record fusion+EKF inputs/outputs (no point clouds) ─────
    # Starts 15 s after launch (after auto_trigger has fired and EKF is up).
    bag_script = os.path.join(
        get_package_share_directory('corgi_odometry'), 'script', 'odom_fusion_bag.sh')
    bag_recorder = TimerAction(
        period=15.0,
        condition=IfCondition(LaunchConfiguration('record_bag')),
        actions=[
            ExecuteProcess(
                cmd=['bash', bag_script],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        record_bag_arg,
        imu_raw_node,
        corgi_leg_odom_node,
        livox_driver_node,
        fast_lio_node,
        odom_tf_relay_node,
        corgi_fusion_node,
        static_tf_base_to_lidar,
        # auto_trigger_node,
        bag_recorder,
    ])
