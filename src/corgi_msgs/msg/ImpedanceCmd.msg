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