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
    k_tangential   stance tangential stiffness per leg, N/m. Default 1200
                   (config of record since 2026-08-30, log S300; was 600),
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
        # Gain regime follows measured per-leg contact instead of the template's
        # single global phase.
        #
        # !!! TRIED 2026-08-17 AND IT WRECKS THE ROBOT -- DO NOT ENABLE. !!!
        # It gates the gains but NOT the reference trajectory, so the command
        # becomes internally inconsistent and the legs break. Full reasoning in
        # the gslip_pronk.cpp comment. Kept as the record; the principled fix is
        # per-leg PHASE, not per-leg gains.
        DeclareLaunchArgument('contact_gated_gains', default_value='false'),
        DeclareLaunchArgument('contact_debounce', default_value='3'),
        DeclareLaunchArgument('contact_timeout_s', default_value='0.25'),
        # Constant rotation of the template's in_stance labels, seconds;
        # positive delays the gain switch relative to the trajectory. Measured
        # need: ~+0.080 (the schedule runs ~80 ms ahead of real touchdown).
        # 0.0 = shipped behaviour. Runs with it set are NOT comparable to
        # unshifted campaigns.
        DeclareLaunchArgument('stance_label_shift_s', default_value='0.0'),
        # Fraction of the cycle the stance labels span; resized at the LIFTOFF
        # edge only (onset untouched, composes with the shift). Measured real
        # duty on the on-design legs ~0.40 vs the template's ~0.43 window.
        # 0.0 = off, template's own labels. Runs with it set are NOT
        # comparable to unmodified campaigns.
        DeclareLaunchArgument('stance_label_duty', default_value='0.0'),
        # beta-dot slaving (S29): scale the stance beta sweep about touchdown
        # by f = v_slave / v_design, so rolling holds at f times the
        # template's design speed. Flight is blended back so touchdown beta
        # is unchanged. 1.0 = off, shipped behaviour. Runs with it set are
        # NOT comparable to unscaled campaigns.
        DeclareLaunchArgument('stance_sweep_scale', default_value='1.0'),
        # contact_align (S309, #20): dwell -> roll-matched sweep -> return,
        # touchdown beta pinned. rows=0 = off, bit-identical. Runs with it
        # set are NOT comparable to unaligned campaigns.
        DeclareLaunchArgument('contact_align_rows', default_value='0'),
        DeclareLaunchArgument('contact_sweep_rows', default_value='0'),
        DeclareLaunchArgument('contact_sweep_scale', default_value='1.0'),
        DeclareLaunchArgument('contact_ramp_rows', default_value='0'),
        # ADOPTED 2026-08-30 (log S300, Alex's decision on S297's data):
        # config of record is the 'both' cell -- k_tangential 1200 +
        # k_pitch -0.30. Was 600/0.0. Paired gain +0.032 at no torque
        # cost, laggard raised 0.168->0.198 (S294/S297).
        DeclareLaunchArgument("k_tangential", default_value="1200.0"),
        # #19 step 2 (2026-08-24): default = config of record 7150/115.8,
        # was the rejected 12000/150 point. Campaigns keep passing k_flight
        # explicitly regardless (#19 step 3).
        DeclareLaunchArgument("k_flight", default_value="7150.0"),
        DeclareLaunchArgument("k_roll", default_value="0.25"),
        # Heading hold OFF by default since 2026-08-22 -- see the ctor comment
        # in gslip_pronk.cpp and log S168. Was 0.15. The ctor default now
        # AGREES with this one; d_yaw had silently diverged (ctor 0.02 against
        # this file's 0.0), the same trap as k_tangential's 900-vs-600.
        DeclareLaunchArgument("k_yaw", default_value="0.0"),
        DeclareLaunchArgument("spring_rest_reference", default_value="false"),
        DeclareLaunchArgument("k_steer", default_value="0.0"),
        DeclareLaunchArgument("steer_offset", default_value="0.0"),
        DeclareLaunchArgument("steer_limit", default_value="0.13963"),  # 8 deg
        DeclareLaunchArgument("steer_prepose", default_value="false"),
        DeclareLaunchArgument("k_steer_yaw", default_value="0.0"),
        DeclareLaunchArgument("d_steer_yaw", default_value="0.0"),
        DeclareLaunchArgument("turn_rate", default_value="0.0"),
        DeclareLaunchArgument("turn_err_limit", default_value="1.5708"),  # 90 deg
        # --- Added 2026-08-17 -------------------------------------------------
        # gslip_pronk declares all of these; the launch file did NOT, so
        # passing them as name:=value was SILENTLY IGNORED -- ros2 launch
        # accepts undeclared launch configurations without complaint and
        # nothing consumed them. An f_lateral sweep therefore ran its entire
        # matrix at the constructor default 0.0 and read as "the channel does
        # nothing at all", which is indistinguishable from a real null.
        #
        # Defaults below are copied from the constructor, so adding them
        # changes no result recorded before this date.
        DeclareLaunchArgument("f_lateral", default_value="0.0"),
        DeclareLaunchArgument("f_lateral_front_only", default_value="false"),
        DeclareLaunchArgument("settle_ticks", default_value="2500"),
        DeclareLaunchArgument("standup_ticks", default_value="2000"),
        DeclareLaunchArgument("b_tangential", default_value="30.0"),
        DeclareLaunchArgument("b_lateral", default_value="60.0"),
        DeclareLaunchArgument("b_flight", default_value="115.8"),  # 19 step 2
        # --- Added 2026-08-22: the clocked-torque feedforward (log S164) ------
        #
        # Lu & Lin 2024 eq 11 damps the leg angle to the CLOCK'S rate; this
        # controller damped to zero, i.e. it carried an unintended brake
        # proportional to the commanded sweep rate and opposing it
        # (3.2 N.m/motor at the v070 config of record, 8.3 N.m/motor in
        # flight). 0.0 = OFF and bit-identical to everything before that date.
        #
        # clock_ff_phase: "stance" | "both". The paper regulates the leg in
        # BOTH phases (p5, S2.3); the registered P-N-1..3 are stance-only.
        DeclareLaunchArgument("clock_ff_scale", default_value="0.0"),
        DeclareLaunchArgument("clock_ff_phase", default_value="stance"),
        DeclareLaunchArgument("d_roll", default_value="0.0"),
        DeclareLaunchArgument("d_yaw", default_value="0.0"),
        DeclareLaunchArgument("gamma_limit", default_value="0.0873"),  # 5 deg
        # Separate clamp on the yaw term of gamma_correction alone (rad).
        # 0 = off (bit-identical). ~0.0175 (1 deg) is the S89 C2 gentle hold.
        DeclareLaunchArgument("gamma_yaw_limit", default_value="0.0"),
        # --- Added 2026-08-20: body PITCH channel (log S115) ------------------
        # Differential leg length (front A/B extend, rear C/D retract) driven
        # by measured pitch. The attitude loop previously had roll and yaw
        # channels only; pitch was computed and discarded because it was once
        # < 1.5 deg. It now runs 18-34 deg peak-to-peak, and landing pitched
        # degrades that stance (rho -0.32/-0.36). Stance-gated, clamped.
        # 0.0/0.0 = EXACTLY off, bit-identical. SIGN is empirical: negate if
        # the correction makes things worse (same rule as k_roll/k_yaw).
        # ADOPTED 2026-08-30 with k_tangential above (log S300). Was 0.0.
        DeclareLaunchArgument("k_pitch", default_value="-0.30"),
        DeclareLaunchArgument("d_pitch", default_value="0.0"),
        DeclareLaunchArgument("pitch_limit", default_value="0.05236"),  # 3 deg

        # Apex beta feedback (log SS133-135). ONE row of the Stage 2b
        # return-map deadbeat, re-solved at the PLANT's operating point and
        # reduced to the only input direction that survives regularisation
        # there: beta responding to forward speed and apex height. The camber
        # rows are deliberately absent -- at this speed sigma(camber)/sigma(beta)
        # is 0.115, and the full gain demands 104 N.m against a 44.25 ceiling
        # and performs WORSE than passive.
        # ^ CORRECTED 2026-08-29: pre-crown-fix numbers (log S183). Post-fix
        # the camber direction is a real second actuator (S263) and exists as
        # the apex_dc_* block below (log S267) -- still default off.
        # apex_fb_gain 0.0 is OFF and bit-identical to every prior run.
        DeclareLaunchArgument("apex_fb_gain", default_value="0.0"),
        DeclareLaunchArgument("apex_k_vx", default_value="-0.5485"),
        DeclareLaunchArgument("apex_k_h", default_value="0.6119"),
        DeclareLaunchArgument("apex_vx_star", default_value="0.3106"),
        DeclareLaunchArgument("apex_h_star", default_value="0.2886"),
        DeclareLaunchArgument("apex_beta_limit", default_value="0.06981"),  # 4 deg
        # /sim/base_odom is GROUND TRUTH -- sim only. /ekf needs corgi_fusion_node,
        # which the campaign harness does not launch. See log S138.
        DeclareLaunchArgument("apex_odom_topic", default_value="/sim/base_odom"),
        # Apex differential camber (log S267): the deadbeat's SECOND input,
        # K's lam_l row at the S263 plant orbit, held per stride, applied
        # gamma += lr_sign * d_lam. apex_dc_gain 0.0 is OFF and bit-identical
        # to every prior run. Gains are per-orbit facts (S57): re-extract
        # before running any other orbit.
        DeclareLaunchArgument("apex_dc_gain", default_value="0.0"),
        DeclareLaunchArgument("apex_dc_k_vy", default_value="-1.054120"),
        DeclareLaunchArgument("apex_dc_k_rho", default_value="-5.769254"),
        DeclareLaunchArgument("apex_dc_k_drho", default_value="-0.168636"),
        DeclareLaunchArgument("apex_dc_limit", default_value="0.06981"),  # 4 deg
        # Latch-up guard (log S268/S269): no apex for grace_s -> both held
        # offsets bleed linearly to zero over decay_s. decay 0.0 = never
        # (the pre-S269 behaviour, which latched at full scale in S268).
        # S304: 1.3 s (was 0.8) -- the old grace assumed per-stride firing;
        # the fixed detector (S296) keeps 0-4 genuine >0.8 s gaps per
        # healthy run, which are texture, not stumbles.
        DeclareLaunchArgument("apex_hold_grace_s", default_value="1.3"),
        DeclareLaunchArgument("apex_hold_decay_s", default_value="0.5"),
        # --- Added 2026-08-19: open-loop Ackermann camber pair (Stage 3) ------
        # Held left/right camber differential, RADIANS: the pair on the turn
        # side leans at gamma_acker_in, the outer pair at gamma_acker_out,
        # both toward the turn centre. gamma_acker_dir +1 = lean RIGHT =
        # RIGHT/CW turn (inner pair B,C); -1 mirrors. Measured, log s88 --
        # this line originally said "+1 left" and was wrong.
        # Compute the pair offline with ackermann_pair() (LegWheel
        # cambered_return_map.py) -- the node applies, it does not derive.
        # dir 0.0 (default) is EXACTLY off: bit-identical to before the
        # channel existed. The node announces engagement with an
        # "ACKER CAMBER set" WARN; a run that requested camber and did not
        # log that line is INVALID.
        DeclareLaunchArgument("gamma_acker_in", default_value="0.0"),
        DeclareLaunchArgument("gamma_acker_out", default_value="0.0"),
        DeclareLaunchArgument("gamma_acker_dir", default_value="0.0"),
        DeclareLaunchArgument("gamma_acker_limit", default_value="0.3491"),  # 20 deg
        DeclareLaunchArgument("gamma_acker_ramp_ticks", default_value="500"),
        # Stance-peak DIP on the camber term (log S210). Declared here because the
        # launch file forwards ONLY declared arguments: the first dip campaign
        # passed these, the launch file dropped them silently, every dip run
        # was quarantined by its own certify check -- S128's trap, again.
        DeclareLaunchArgument("gamma_acker_dip", default_value="0.0"),
        DeclareLaunchArgument("gamma_acker_dip_t0_ms", default_value="20"),
        DeclareLaunchArgument("gamma_acker_dip_t1_ms", default_value="60"),
        DeclareLaunchArgument("gamma_acker_dip_rearm_ms", default_value="30"),
        DeclareLaunchArgument("gamma_acker_yield", default_value="0.0"),
        # --- Added 2026-08-24: closed-loop camber (Stage 3 task 6, S232/S235).
        # gamma_acker_ff > 0 enables the loop and replaces dir/in/out (both
        # set is fatal in the node). min/hi bound the magnitude to the
        # measured proportional region, 5-15 deg.
        DeclareLaunchArgument("gamma_acker_ff", default_value="0.0"),
        DeclareLaunchArgument("k_acker_yaw", default_value="0.0"),
        DeclareLaunchArgument("d_acker_yaw", default_value="0.0"),
        DeclareLaunchArgument("gamma_acker_min", default_value="0.0872665"),
        DeclareLaunchArgument("gamma_acker_hi", default_value="0.2617994"),
        # --- Added 2026-08-20: event-driven per-leg gait scheduler (Tier 1) ---
        # Each leg replays the template on its own clock, snapped to the
        # stance-onset row by its own debounced touchdown (blended, clamped,
        # latched once per stride). false = EXACTLY off, bit-identical.
        # Engagement requires ~1 kHz contact (CORGI_CONTACT_INTERVAL=1) and
        # is announced with an "EVENT SCHED ENGAGED" WARN; a run that
        # requested the scheduler and did not log that line is INVALID.
        # Refused combinations: contact_gated_gains (the failed half-fix it
        # replaces), stance_label_shift_s != 0 (double correction).
        # Defaults are the constructor's; no pre-2026-08-20 result changes.
        DeclareLaunchArgument("event_sched", default_value="false"),
        DeclareLaunchArgument("event_snap_limit_s", default_value="0.030"),
        DeclareLaunchArgument("event_blend_ticks", default_value="20"),
        DeclareLaunchArgument("event_snap_per_stride", default_value="1"),
        # Tier 0's decision output (log S92): arm at each leg's first
        # debounced touchdown.
        DeclareLaunchArgument("event_arm_strides", default_value="0"),
        DeclareLaunchArgument("stance_max_overrun_ratio", default_value="1.5"),
        # Advance-only snapping (log S95): large backward corrections are
        # replaced by the complementary forward one -- a backward freeze
        # lengthens the physical stride 1:1 and regenerates the error.
        DeclareLaunchArgument("event_forward_only", default_value="false"),
        # --- Added 2026-08-20: Tier 3 velocity-slaved stance (log S96) --------
        # On touchdown the leg integrates beta-dot = v / L(theta) from its
        # current commanded beta (state-continuous, no reference jump);
        # flight stays clocked. Requires event_sched. Engagement announces
        # "SLAVE STANCE ENGAGED"; absence on a run that asked = INVALID.
        # Tier 2 gentle cross-leg phase coupling (log S97): flight-row-only
        # nudge toward the four-leg mean phase. 0.0 = off, bit-identical.
        DeclareLaunchArgument("couple_gain", default_value="0.0"),
        DeclareLaunchArgument("couple_clamp", default_value="0.05"),
        DeclareLaunchArgument("slave_stance", default_value="false"),
        DeclareLaunchArgument("slave_v_source", default_value="fixed"),
        DeclareLaunchArgument("slave_v_fixed", default_value="1.187"),
        DeclareLaunchArgument("ekf_timeout_s", default_value="0.05"),
        # Push-timing pair synchronisation (log S100): an early leg holds
        # beta at its entry value under stance gains until `quorum` legs
        # are down, then all waiting legs release together. Requires
        # slave_stance. false = off, bit-identical.
        DeclareLaunchArgument("push_sync", default_value="false"),
        DeclareLaunchArgument("push_sync_quorum", default_value="4"),
        DeclareLaunchArgument("push_sync_max_hold_s", default_value="0.080"),
        DeclareLaunchArgument("push_sync_arm_strides", default_value="5"),
        DeclareLaunchArgument("ltheta_lut_path", default_value=""),
        DeclareLaunchArgument('template_path', default_value=''),

        Node(
            package='corgi_force_estimation',
            executable='force_estimation_node',
            name='force_estimation_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
        ),
        Node(
            package='corgi_force_control',
            executable='force_control_node',
            name='force_control_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
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
                'contact_gated_gains': LaunchConfiguration('contact_gated_gains'),
                'contact_debounce': LaunchConfiguration('contact_debounce'),
                'contact_timeout_s': LaunchConfiguration('contact_timeout_s'),
                'stance_label_shift_s': LaunchConfiguration('stance_label_shift_s'),
                'stance_label_duty': LaunchConfiguration('stance_label_duty'),
                'stance_sweep_scale': LaunchConfiguration('stance_sweep_scale'),
                'contact_align_rows': LaunchConfiguration('contact_align_rows'),
                'contact_sweep_rows': LaunchConfiguration('contact_sweep_rows'),
                'contact_sweep_scale': LaunchConfiguration('contact_sweep_scale'),
                'contact_ramp_rows': LaunchConfiguration('contact_ramp_rows'),
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
                'f_lateral': LaunchConfiguration('f_lateral'),
                'f_lateral_front_only': LaunchConfiguration('f_lateral_front_only'),
                'settle_ticks': LaunchConfiguration('settle_ticks'),
                'standup_ticks': LaunchConfiguration('standup_ticks'),
                'b_tangential': LaunchConfiguration('b_tangential'),
                'b_lateral': LaunchConfiguration('b_lateral'),
                'b_flight': LaunchConfiguration('b_flight'),
                'clock_ff_scale': LaunchConfiguration('clock_ff_scale'),
                'clock_ff_phase': LaunchConfiguration('clock_ff_phase'),
                'd_roll': LaunchConfiguration('d_roll'),
                'd_yaw': LaunchConfiguration('d_yaw'),
                'gamma_limit': LaunchConfiguration('gamma_limit'),
                'gamma_yaw_limit': LaunchConfiguration('gamma_yaw_limit'),
                'k_pitch': LaunchConfiguration('k_pitch'),
                'd_pitch': LaunchConfiguration('d_pitch'),
                'pitch_limit': LaunchConfiguration('pitch_limit'),
                'apex_fb_gain': LaunchConfiguration('apex_fb_gain'),
                'apex_k_vx': LaunchConfiguration('apex_k_vx'),
                'apex_k_h': LaunchConfiguration('apex_k_h'),
                'apex_vx_star': LaunchConfiguration('apex_vx_star'),
                'apex_h_star': LaunchConfiguration('apex_h_star'),
                'apex_beta_limit': LaunchConfiguration('apex_beta_limit'),
                'apex_odom_topic': LaunchConfiguration('apex_odom_topic'),
                'apex_dc_gain': LaunchConfiguration('apex_dc_gain'),
                'apex_dc_k_vy': LaunchConfiguration('apex_dc_k_vy'),
                'apex_dc_k_rho': LaunchConfiguration('apex_dc_k_rho'),
                'apex_dc_k_drho': LaunchConfiguration('apex_dc_k_drho'),
                'apex_dc_limit': LaunchConfiguration('apex_dc_limit'),
                'apex_hold_grace_s': LaunchConfiguration('apex_hold_grace_s'),
                'apex_hold_decay_s': LaunchConfiguration('apex_hold_decay_s'),
                'gamma_acker_in': LaunchConfiguration('gamma_acker_in'),
                'gamma_acker_out': LaunchConfiguration('gamma_acker_out'),
                'gamma_acker_dir': LaunchConfiguration('gamma_acker_dir'),
                'gamma_acker_limit': LaunchConfiguration('gamma_acker_limit'),
                'gamma_acker_ramp_ticks':
                    LaunchConfiguration('gamma_acker_ramp_ticks'),
                'gamma_acker_dip': LaunchConfiguration('gamma_acker_dip'),
                'gamma_acker_dip_t0_ms': LaunchConfiguration('gamma_acker_dip_t0_ms'),
                'gamma_acker_dip_t1_ms': LaunchConfiguration('gamma_acker_dip_t1_ms'),
                'gamma_acker_dip_rearm_ms': LaunchConfiguration('gamma_acker_dip_rearm_ms'),
                'gamma_acker_yield': LaunchConfiguration('gamma_acker_yield'),
                'gamma_acker_ff': LaunchConfiguration('gamma_acker_ff'),
                'k_acker_yaw': LaunchConfiguration('k_acker_yaw'),
                'd_acker_yaw': LaunchConfiguration('d_acker_yaw'),
                'gamma_acker_min': LaunchConfiguration('gamma_acker_min'),
                'gamma_acker_hi': LaunchConfiguration('gamma_acker_hi'),
                'event_sched': LaunchConfiguration('event_sched'),
                'event_snap_limit_s': LaunchConfiguration('event_snap_limit_s'),
                'event_blend_ticks': LaunchConfiguration('event_blend_ticks'),
                'event_snap_per_stride':
                    LaunchConfiguration('event_snap_per_stride'),
                'event_arm_strides': LaunchConfiguration('event_arm_strides'),
                'stance_max_overrun_ratio':
                    LaunchConfiguration('stance_max_overrun_ratio'),
                'event_forward_only':
                    LaunchConfiguration('event_forward_only'),
                'couple_gain': LaunchConfiguration('couple_gain'),
                'couple_clamp': LaunchConfiguration('couple_clamp'),
                'slave_stance': LaunchConfiguration('slave_stance'),
                'slave_v_source': LaunchConfiguration('slave_v_source'),
                'slave_v_fixed': LaunchConfiguration('slave_v_fixed'),
                'ekf_timeout_s': LaunchConfiguration('ekf_timeout_s'),
                'push_sync': LaunchConfiguration('push_sync'),
                'push_sync_quorum': LaunchConfiguration('push_sync_quorum'),
                'push_sync_max_hold_s':
                    LaunchConfiguration('push_sync_max_hold_s'),
                'push_sync_arm_strides':
                    LaunchConfiguration('push_sync_arm_strides'),
                'ltheta_lut_path': LaunchConfiguration('ltheta_lut_path'),
                'template_path': template_path,
            }],
            output='log',
        ),
    ])
