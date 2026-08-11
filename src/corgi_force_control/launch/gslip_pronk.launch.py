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
    k_tangential   stance tangential stiffness per leg, N/m. Default 600,
                   lowered from 1200 on 2026-08-08. The tangential force acts
                   perpendicular to the leg, so unlike the axial spring it does
                   NOT pass through the hip: it carries a pitch moment on the
                   full 0.29 m hip-to-contact lever, ~30x the axial term at a
                   realistic beta tracking error. That moment desynchronises
                   front and rear legs. Measured over a sweep (n=3 at 600):
                     k_t   leg split      flight        top-rung v
                     1200  40.1%          41.1%         0.658   (n=1)
                      900  38.5 +-0.9%    42.4 +-0.2%   0.585   (n=2)
                      600  32.7 +-2.1%    46.6 +-1.5%   0.826   (n=3)
                      300  37.1%          42.6%         0.686   (n=1)
                   There is a MINIMUM, not a monotonic trend: k_t is also the
                   compliance that lets the leg track the template's sweep, so
                   cutting it too far degrades tracking, which feeds back into
                   the very error driving the moment. See
                   examples/gslip/pitch_moment_kt.py in LegWheel.
    template_path  override the stride template CSV
    hop_in_place   see below

    steer_offset   per-side beta offset in RADIANS, applied only while the leg
                   is in stance. This is the steering channel: the foot is a
                   0.145 m arc in rolling contact, so a differential sweep
                   during stance drives the robot like a differential drive,
                   d_psi ~ r*(sweep_L - sweep_R)/track with track = 0.24 m. A
                   5 deg (0.0873 rad) differential predicts ~3.0 deg/stride,
                   comparable to the parasitic yaw it has to cancel.
                   Stance-gating is not optional: in the hop segment beta is
                   identically zero, so a constant offset is a static leg angle
                   that rolls nothing.
    k_steer        per-side beta AMPLITUDE scale, beta*(1 +- k_steer). Only
                   bites where the template already sweeps, i.e. the forward
                   rungs, and moves speed as well as heading.
    steer_prepose  also drive the leg -s*u during flight, so it lands
                   pre-positioned and sweeps through the full 2*u. Doubles the
                   authority, bigger step at the phase boundaries.
    steer_limit    clamp on the total beta perturbation, radians.
    k_steer_yaw    heading feedback: yaw error -> steer offset. The SIGN is not
                   derivable from the geometry -- take it from an open-loop
                   +-steer_offset pair and negate if the correction makes
                   things worse.
    d_steer_yaw    damping on yaw rate for the same loop.

    All six default to values that make the node behave exactly as it did
    before the channel existed, so old runs stay reproducible.

    turn_rate      commanded turn rate in RADIANS PER SECOND, positive left.
                   The heading reference is advanced at this rate instead of
                   being held, so the steering loop above tracks a rotating
                   setpoint -- which is a turn, with no change to the control
                   law. Radius follows from the speed, R = v/turn_rate: at the
                   measured 0.787 m/s, 0.394 rad/s is R = 2.0 m and 0.315 rad/s
                   is R = 2.5 m. Zero (the default) is the heading hold.

                   Use a CONSTANT-SPEED template with this, not the speed ramp.
                   The ramp changes rung every ~6 strides and cannot settle into
                   a circle; gslip_pronk_template.csv is a single stride at the
                   v~1.20 fixed point which the node loops indefinitely. Set
                   RAMP_UNTIL explicitly when recording, because the recorder's
                   default stop is one template duration (0.22 s) and it will
                   otherwise exit immediately.
    turn_err_limit cap on how far the advancing reference may get ahead of the
                   measured heading, radians (default pi/2). Without it an
                   over-commanded turn lets the reference lap the robot: the
                   wrapped error passes pi and the steering correction flips
                   sign at full clamp. That regime is exactly what an envelope
                   sweep goes looking for.

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
        DeclareLaunchArgument("b_radial", default_value="0.0"),
        DeclareLaunchArgument("k_lateral", default_value="7500.0"),
        DeclareLaunchArgument('hold_stance', default_value='false'),
        DeclareLaunchArgument("k_tangential", default_value="600.0"),
        DeclareLaunchArgument("k_flight", default_value="12000.0"),
        DeclareLaunchArgument("k_roll", default_value="0.25"),
        DeclareLaunchArgument("k_yaw", default_value="0.15"),
        DeclareLaunchArgument("spring_rest_reference", default_value="false"),
        DeclareLaunchArgument("k_steer", default_value="0.0"),
        DeclareLaunchArgument("steer_offset", default_value="0.0"),
        DeclareLaunchArgument("steer_limit", default_value="0.13963"),  # 8 deg
        DeclareLaunchArgument("steer_prepose", default_value="false"),
        DeclareLaunchArgument("k_steer_yaw", default_value="0.0"),
        DeclareLaunchArgument("d_steer_yaw", default_value="0.0"),
        DeclareLaunchArgument("turn_rate", default_value="0.0"),
        DeclareLaunchArgument("turn_err_limit", default_value="1.5708"),  # 90 deg
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
                'k_flight': LaunchConfiguration('k_flight'),
                'k_roll': LaunchConfiguration('k_roll'),
                'k_yaw': LaunchConfiguration('k_yaw'),
                'spring_rest_reference': LaunchConfiguration('spring_rest_reference'),
                'k_steer': LaunchConfiguration('k_steer'),
                'steer_offset': LaunchConfiguration('steer_offset'),
                'steer_limit': LaunchConfiguration('steer_limit'),
                'steer_prepose': LaunchConfiguration('steer_prepose'),
                'k_steer_yaw': LaunchConfiguration('k_steer_yaw'),
                'd_steer_yaw': LaunchConfiguration('d_steer_yaw'),
                'turn_rate': LaunchConfiguration('turn_rate'),
                'turn_err_limit': LaunchConfiguration('turn_err_limit'),
                'template_path': template_path,
            }],
            output='screen',
        ),
    ])
