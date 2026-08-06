"""Webots simulator for G-SLIP validation, with the run mode exposed.

Same robot and world as corgi_sim/launch/Corgi_launch.py, but with Webots'
run mode as a launch argument so the simulation starts stepping on its own
instead of needing the Play button.

WebotsLauncher already passes `--batch --mode=<mode>` and defaults to
'realtime', so this does not add a capability so much as make it settable --
useful because 'fast' removes the real-time throttle, which both starts the
simulation unambiguously and makes long validation runs quicker.

Arguments:
    mode        'fast' (default), 'realtime' or 'pause'.
                'fast' runs as quickly as the machine allows -- best for
                collecting stride data. Use 'realtime' when watching the
                robot, since 'fast' can be too quick to follow.
    gui         'true' (default) or 'false'. False sets WEBOTS_OFFSCREEN,
                which runs Webots under xvfb with no window.
    panel       'true' (default) to start the corgi control panel.

Typical use:
    ros2 launch corgi_force_control gslip_sim.launch.py mode:=realtime
    ros2 launch corgi_force_control gslip_pronk.launch.py

Notes for WSL, from getting this working:

* Leave WEBOTS_HOME unset. webots_ros2 detects WSL and launches the Windows
  Webots through /mnt/c; pointing WEBOTS_HOME at a Linux install makes it
  look for msys64/mingw64/bin/webots.exe and fail outright.

* Only ever run one launch at a time. WebotsController sets respawn=True, so
  a killed launch can leave a driver behind; a second launch then produces
  duplicate /CorgiRobotABAD and /corgi_driver_internal nodes, which corrupts
  the DDS graph. Check with `ros2 node list` -- duplicates mean stale
  processes. Clear them with
      pkill -f webots_ros2_driver && ros2 daemon stop

* The `mode` argument alone does not start the simulation. corgi_driver.py
  calls
      self.__robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
  on init whenever CORGI_EXPERIMENT_MODE is unset, so whatever --mode
  WebotsLauncher passed is overridden the moment the controller connects.
  This is intended behaviour -- the normal workflow is to press Play -- but
  it does mean `mode` on its own looks broken.

* experiment_mode:=true here does NOT work, and the argument is kept only so
  the finding is not lost: SetEnvironmentVariable from a launch file does not
  reach the driver, because webots-controller re-execs with a sanitised
  environment (verified by reading /proc/<driver-pid>/environ). To skip the
  pause, export it in the shell before launching:
      export CORGI_EXPERIMENT_MODE=1
  Note that also enables trigger-driven removal of the SUPPORT_BOX, which is
  NOT the normal flow -- normally the robot stands up off the box under CSV
  position control and the box simply stays there.
"""

import os
import socket

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def _find_free_port(start_port=1234, max_tries=100):
    for port in range(start_port, start_port + max_tries):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if sock.connect_ex(('127.0.0.1', port)) != 0:
                return port
    return start_port


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    experiment = LaunchConfiguration('experiment_mode').perform(context).lower() in ('true', '1')

    package_dir = get_package_share_directory('corgi_sim')
    world_path = os.path.join(package_dir, 'worlds', 'Corgi_ABAD.wbt')
    webots_port = str(_find_free_port(start_port=int(os.environ.get('WEBOTS_PORT', '1234'))))

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False,
        port=webots_port,
        mode=mode,
    )

    robot_driver = WebotsController(
        robot_name='CorgiRobotABAD',
        port=webots_port,
        parameters=[{'robot_description': os.path.join(package_dir, 'resource', 'corgi.urdf')}],
        respawn=True,
    )

    actions = []
    if not gui:
        # WebotsLauncher checks for this and wraps the command in xvfb-run.
        actions.append(SetEnvironmentVariable(name='WEBOTS_OFFSCREEN', value='1'))
    if experiment:
        # Without this, corgi_driver calls simulationSetMode(PAUSE) on init and
        # the simulation sits still no matter what --mode WebotsLauncher passed.
        # It also enables trigger-driven removal of the SUPPORT_BOX, so the
        # robot is held up until /trigger goes true.
        actions.append(SetEnvironmentVariable(name='CORGI_EXPERIMENT_MODE', value='1'))
    actions += [
        webots,
        robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]
    return actions


def generate_launch_description():
    launch_user = os.environ.get('USER') or os.environ.get('USERNAME') or 'root'

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fast'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('panel', default_value='true'),
        DeclareLaunchArgument('experiment_mode', default_value='true'),

        launch.actions.SetEnvironmentVariable(name='USER', value=launch_user),
        launch.actions.SetEnvironmentVariable(name='USERNAME', value=launch_user),

        OpaqueFunction(function=_setup),

        Node(
            package='corgi_panel',
            executable='corgi_control_panel',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('panel')),
        ),
    ])
