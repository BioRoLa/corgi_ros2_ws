"""Phase 5 validation stack for the G-SLIP pronk controller.

Brings up the impedance chain and the G-SLIP template player:

    gslip_pronk_node --(impedance/command)--> force_control_node
                                                    |
                                            (motor/command)
                                                    v
                                       corgi_sim / corgi_ros_bridge

Launch the simulator separately (ros2 launch corgi_sim Corgi_launch.py) and
then this. Keeping them apart matters in WSL: webots_ros2 launches the
*Windows* Webots across /mnt/c, which loads the world into a temp file and
comes up paused, so the simulator generally needs a hand on the play button
before any of this will step.

Arguments:
    use_sim_time   true against Webots, false on hardware (default true)
    k_radial       stance radial stiffness per leg, N/m. Default 8941 sizes
                   the k_rel=18 spring at the nominal stance theta = 100 deg
                   for 30.0 kg (measured); CorgiRobotABAD.proto sums to
                   30.84 kg, so the same value serves both sim and hardware.
    b_radial       stance radial damping per leg, N.s/m
    k_lateral      stance ABAD stiffness per leg, N/m. Must not be zero --
                   with leg_frame this axis is the hip roll, and a zero here
                   leaves kp_h near zero and the ABAD floppy.
    k_tangential   stance tangential stiffness per leg, N/m
    template_path  override the stride template CSV
    hop_in_place   see below

Suggested order, each step gating the next:

  0. Baseline, no G-SLIP: run corgi_force_control's exp_sim_stay_node against
     this sim build first. force_control was developed against a different
     corgi_sim commit, so this proves the pairing before anything new is
     added. A failure here is a sim-model problem, not a controller one.
  1. Standing spring: launch with hop_in_place:=true and k_radial low
     (~3000), push the body in Webots, confirm it behaves like the commanded
     stiffness.
  2. Hop in place: hop_in_place:=true at the full k_radial. This is the first
     real test of the stance/flight switching.
  3. Open-loop replay: skip this stack, play output/csv/gslip_pronk.csv
     through corgi_csv_control instead. Validates the kinematic mapping with
     no controller in the loop.
  4. Full pronk: hop_in_place:=false, then publish the trigger:
       ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped '{enable: true}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    k_radial = LaunchConfiguration('k_radial')
    b_radial = LaunchConfiguration('b_radial')
    k_lateral = LaunchConfiguration('k_lateral')
    k_tangential = LaunchConfiguration('k_tangential')
    template_path = LaunchConfiguration('template_path')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('k_radial', default_value='8941.0'),
        DeclareLaunchArgument('b_radial', default_value='72.0'),
        DeclareLaunchArgument('k_lateral', default_value='30000.0'),
        DeclareLaunchArgument('hold_stance', default_value='false'),
        DeclareLaunchArgument('k_tangential', default_value='1200.0'),
        DeclareLaunchArgument('template_path', default_value=''),

        Node(
            package='corgi_force_estimation',
            executable='force_estimation_node',
            name='force_estimation_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='corgi_force_control',
            executable='force_control_node',
            name='force_control_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='corgi_force_control',
            executable='gslip_pronk_node',
            name='gslip_pronk_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'k_radial': k_radial,
                'b_radial': b_radial,
                'k_lateral': k_lateral,
                'hold_stance': LaunchConfiguration('hold_stance'),
                'k_tangential': k_tangential,
                'template_path': template_path,
            }],
            output='screen',
        ),
    ])
