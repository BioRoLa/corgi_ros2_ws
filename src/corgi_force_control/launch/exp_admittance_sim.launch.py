import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve bag output directory: prefer source tree, fall back to install share.
    # share_dir = get_package_share_directory('corgi_force_control')
    # ws_dir = os.path.normpath(os.path.join(share_dir, '..', '..', '..', '..'))
    # src_dir = os.path.join(ws_dir, 'src', 'corgi_force_control')
    # base_dir = src_dir if os.path.isdir(src_dir) else os.path.dirname(share_dir)
    # bag_output = os.path.join(
    #     base_dir, 'bag',
    #     'exp_admittance_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    # )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    sim_params = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]

    # Webots simulator + robot driver + control panel
    corgi_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('corgi_sim'),
                'launch',
                'Corgi_force_plate_launch.py'
            )
        )
    )

    force_estimation_node = Node(
        package='corgi_force_estimation',
        executable='force_estimation_node',
        name='force_estimation_node',
        output='screen',
        parameters=sim_params,
    )

    admittance_control_node = Node(
        package='corgi_force_control',
        executable='admittance_control_node',
        name='admittance_control_node',
        output='screen',
        parameters=sim_params,
    )

    exp_admittance_node = Node(
        package='corgi_force_control',
        executable='exp_admittance_node',
        name='exp_admittance_node',
        output='screen',
        parameters=sim_params,
    )

    # # Record all topics relevant to admittance force-tracking debug.
    # bag_record = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'bag', 'record',
    #         '/trigger',
    #         '/impedance/command',
    #         '/motor/command',
    #         '/motor/state',
    #         '/force/state',
    #         '/sensor/force_plate_1',
    #         '/sensor/force_plate_2',
    #         '/sensor/force_plate_3',
    #         '/sensor/force_plate_4',
    #         '-o', bag_output,
    #     ],
    #     output='screen',
    # )

    return LaunchDescription([
        use_sim_time_arg,
        corgi_sim_launch,
        force_estimation_node,
        admittance_control_node,
        exp_admittance_node,
        # bag_record,
    ])