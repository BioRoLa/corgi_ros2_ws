float64 theta
float64 beta
float64 gamma
float64 fx
float64 fy
float64 fz
float64 mx
float64 my
float64 mz
float64 bx
float64 by
float64 bz
float64 kx
float64 ky
float64 kz

# Interpret the stiffness and damping triples in the LEG frame rather than the
# body frame. Needed by the G-SLIP template, whose spring acts along the leg,
# not along a body axis.
#
#   false (default): (kx, ky, kz) are body-frame x / y / z, as before.
#   true:            kx -> radial      (along hip -> contact, in the sagittal plane)
#                    ky -> lateral     (body y, unchanged)
#                    kz -> tangential  (perpendicular to the leg, in the sagittal plane)
#
# Same reinterpretation applies to (bx, by, bz). force_control rotates them
# into the body frame using the measured contact point, so the resulting K is
# non-diagonal; the K_joint = J^T K J mapping already handles that.
bool leg_frame

# Reference rate for the TANGENTIAL damper, rad/s on the leg angle (beta).
#
# Lu & Lin 2024 eq 11 (Bioinspir. Biomim. 19 026017, p5) is
#     tau = k_P*(theta_fp - theta) + k_D*(dtheta_fp - dtheta)
# where their theta is the LEG ANGLE (our beta) and their phi is the spring
# (our theta) -- see eq 6, whose potential is -0.5*k_t*(phi_0 - phi)^2.
# The D term damps to the CLOCK'S RATE. This controller damped to ZERO, which
# is an additive braking torque proportional to the commanded sweep rate and
# opposing it (log S164).
#
# force_control turns this into a feedforward contact force
#     f_clock = bz * (dbeta_ref * |hip->contact|) * e_t
# so the tangential law becomes bz*(v_ref - v_measured) instead of bz*(0 - v).
# Units are rad/s, NOT m/s: the lever is applied where e_t is built, from the
# measured contact point, so it is never re-derived in a second place.
#
# 0.0 (default) reproduces the pre-2026-08-22 behaviour exactly. Only meaningful
# when the contact point is non-degenerate; ignored otherwise.
float64 dbeta_ref
