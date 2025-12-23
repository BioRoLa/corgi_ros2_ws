from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='joystick',
        description='Control mode: joystick, teleop, or pure'
    )

    # Get launch configuration
    control_mode = LaunchConfiguration('control_mode')

    # Publisher node
    publisher_node = Node(
        package='corgi_wheeled',
        executable='publisher',
        name='publisher',
        output='screen'
    )

    # Wheeled test node with control_mode parameter
    wheeled_test_node = Node(
        package='corgi_wheeled',
        executable='wheeled_test',
        name='wheeled_test',
        output='screen',
        parameters=[{'control_mode': control_mode}]
    )

    # Teleop node (only if control_mode == 'teleop')
    teleop_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", control_mode, "' == 'teleop'"])
        ),
        actions=[
            Node(
                package='corgi_wheeled',
                executable='teleop',
                name='teleop',
                output='screen'
            )
        ]
    )

    # Joy node (if control_mode != 'teleop')
    joy_group = GroupAction(
        condition=UnlessCondition(
            PythonExpression(["'", control_mode, "' == 'teleop'"])
        ),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        control_mode_arg,
        publisher_node,
        wheeled_test_node,
        teleop_group,
        joy_group
    ])
