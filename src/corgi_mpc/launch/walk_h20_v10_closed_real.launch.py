import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('corgi_mpc')
    bag_script = os.path.join(package_share_dir, 'script', 'mpc_closed_bag.sh')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    config_profile_arg = DeclareLaunchArgument(
        'config_profile',
        default_value='real',
        description='MPC config profile: sim or real',
    )

    state_source_arg = DeclareLaunchArgument(
        'state_source',
        default_value='odom_legacy',
        description='MPC state source: odom_legacy in real robot',
    )

    mpc_node = Node(
        package='corgi_mpc',
        executable='walk_h20_v10_closed',
        name='corgi_mpc',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_profile': LaunchConfiguration('config_profile'),
            'state_source': LaunchConfiguration('state_source'),
        }],
    )

    real_params = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]

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

    imu_node = Node(
        package='corgi_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=real_params,
    )
    
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
        use_sim_time_arg,
        config_profile_arg,
        state_source_arg,
        force_estimation_node,
        force_control_node,
        odometry_node,
        z_position_node,
        imu_node,
        mpc_node,
        bag_record_process,
    ])
