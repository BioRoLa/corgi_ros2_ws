from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    config_profile_arg = DeclareLaunchArgument(
        'config_profile',
        default_value='sim',
        description='MPC config profile: sim or real',
    )

    state_source_arg = DeclareLaunchArgument(
        'state_source',
        default_value='odom_legacy',
        description='MPC state source: odom_legacy or sim_driver',
    )

    sim_params = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]

    mpc_node = Node(
        package='corgi_mpc',
        executable='walk_closed_time',
        name='corgi_mpc',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_profile': LaunchConfiguration('config_profile'),
            'state_source': LaunchConfiguration('state_source'),
        }],
    )

    force_estimation_node = Node(
        package='corgi_force_estimation',
        executable='force_estimation_node',
        name='force_estimation_node',
        output='screen',
        parameters=sim_params,
    )

    force_control_node = Node(
        package='corgi_force_control',
        executable='force_control_node',
        name='force_control_node',
        output='screen',
        parameters=sim_params,
    )

    odometry_node = Node(
        package='corgi_odometry_legacy',
        executable='corgi_odometry_legacy',
        name='corgi_odometry_legacy',
        output='screen',
        parameters=sim_params,
    )

    z_position_node = Node(
        package='corgi_odometry_legacy',
        executable='corgi_z_position_legacy',
        name='corgi_z_position_legacy',
        output='screen',
        parameters=sim_params,
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_profile_arg,
        state_source_arg,
        force_estimation_node,
        force_control_node,
        odometry_node,
        z_position_node,
        mpc_node,
    ])
