from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('corgi_sim'),
            'worlds',
            'corgi_old_proto.wbt'
        ]),
        description='Path to Webots world file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots simulation mode: realtime, pause, or fast'
    )
    
    # Use simulation time
    use_sim_time = {'use_sim_time': True}
    
    # Webots launcher node
    webots_launcher = Node(
        package='corgi_sim',
        executable='webots_launcher.py',
        name='webots_launcher',
        arguments=[
            '--world', LaunchConfiguration('world'),
            '--mode', LaunchConfiguration('mode')
        ],
        parameters=[use_sim_time],
        output='screen'
    )
    
    # Data recorder node
    data_recorder = Node(
        package='corgi_data_recorder',
        executable='corgi_data_recorder',
        name='corgi_data_recorder',
        parameters=[use_sim_time]
    )
    
    # Simulation torque node
    sim_trq = Node(
        package='corgi_sim',
        executable='corgi_sim_trq',
        name='corgi_sim_trq',
        parameters=[use_sim_time],
        respawn=True,
        output='screen'
    )
    
    # Force estimation node
    force_estimation = Node(
        package='corgi_force_estimation',
        executable='force_estimation',
        name='force_estimation',
        parameters=[use_sim_time],
        output='log'
    )
    
    # MPC node - wlw_open_loop (commented sections from original can be uncommented as needed)
    mpc_node = Node(
        package='corgi_mpc',
        executable='wlw_open_loop',
        name='corgi_mpc',
        parameters=[use_sim_time],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        mode_arg,
        webots_launcher,
        data_recorder,
        sim_trq,
        force_estimation,
        mpc_node,
    ])
