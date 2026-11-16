# Frames and signs

The one convention every node above the hardware boundary uses. If code and
this file disagree, one of them is a bug: fix it, then say which here.

Adopted 2026-11-13 (implementation log §344–345). Background: the force-plate
sessions of 2026-10 traced every sign surprise to one place, the hardware
boundary converting positions into a common frame but not torques, velocities
or gains (implementation log §343–344; vault note *Torque Frame Mismatch —
dir_beta vs Feedforward*).

## 1. The rule

All topics above the hardware boundary (`motor/command`, `motor/state`,
`impedance/command`, `force/state`) use **one convention, identical on all
four legs**. Exactly two places convert between it and each leg module's own
wiring:

| boundary | where |
|---|---|
| robot | `corgi_ros_bridge` |
| sim | `corgi_sim/corgi_driver_pkg/corgi_driver.py` |

Both read **one direction table**, `corgi_utils/config/module_directions.yaml`
(installed to `share/corgi_utils/config/`; `CORGI_MODULE_DIRECTIONS` overrides
the path for both and for the control panel, which displays it). Both convert
**positions, velocities, torques and gains together**, with the algebra in §5:
C++ in `corgi_utils/module_frame.hpp` (table loader `module_directions.hpp`),
Python in the sim driver's `ros_to_module` / `convert_torque` /
`convert_gains` (loader `corgi_driver_pkg/module_directions.py`, same rules).

No node above the boundary flips a sign because of which leg it is on **for
frame reasons**. Signs that come from *where* a leg is (left or right, front
or rear) are geometry, are allowed, and are named as geometry (§6).

## 2. Body frame and legs

Body frame: **x forward, y left, z up**.

| leg | position | hip (x, y) m |
|---|---|---|
| A | front-left | (+0.255, +0.12) |
| B | front-right | (+0.255, −0.12) |
| C | rear-right | (−0.255, −0.12) |
| D | rear-left | (−0.255, +0.12) |

Left {A, D}, right {B, C}, front {A, B}, rear {C, D}; diagonals {A, C} and
{B, D}. Index order everywhere is A, B, C, D = 0, 1, 2, 3.

## 3. Joint coordinates — the same on every leg

| symbol | meaning | sign |
|---|---|---|
| θ (`theta`) | leg opening, 17°–160° | larger = longer leg |
| β (`beta`) | leg rotation | **+β puts the foot behind the hip**, on all four legs (static foot test, vault *Symbol Reference* §2; the 2026-10-29 stand-up) |
| γ (`gamma`) | ABAD (hip roll) | the same physical sense on every leg; which way + points is **not yet measured** — do not assume |
| φ_l, φ_r | the two leg-motor angles | `phi_l = β + θ − 17°`, `phi_r = β − θ + 17°` (as `force_control` and the sim's `Controller_TB`; the vault *Symbol Reference* has R and L the other way round) |

Velocities are the time derivatives of the same quantities. Torques
`torque_l`, `torque_r`, `torque_h` are the generalized forces on φ_l, φ_r and
γ, in this same frame on every leg: **+torque drives its angle up**, so a
motor doing work against friction has τ·φ̇ > 0.

That sign was measured on 2026-10-29 (`measure_friction.py`, run
`20261029_210644`): on all eight leg motors, at 10, 20 and 40 °/s, the torque
while φ rises exceeds the torque while it falls, each torque compared with its
own reported velocity. The friction feedforward in `force_control`, and its
mirror in `force_estimation`, had the opposite sign until 2026-11-13. The
ABAD (`torque_h`) has not been measured the same way.

## 4. Forces

`ImpedanceCmd` `fx fy fz` and `ForceState` `fx fy fz` are **the force the leg
puts on the ground, in the body frame**, the same on every leg. A standing leg
has fz < 0. One qualification: `force_control` and `force_estimation` both
evaluate the leg at β + IMU pitch, so fx and fz are levelled in pitch (x along
the ground, z vertical) rather than tilted with the body; roll is not
levelled.

The leg model (`LegModel`, `calculate_jacobian_3d`) works in its own frame per
leg: **x backward, y outward, z up**. One helper converts,
`corgi_utils/leg_frame.hpp`: `v_body = diag(−1, s, +1) v_leg`, with s = +1 on
the left legs (A, D) and −1 on the right (B, C). That s is geometry. Both
`force_control` (command in) and `force_estimation` (estimate out) call it;
nothing else should convert.

Until 2026-11-13 `force_control` fed the body-frame command into the leg frame
unconverted: +fx came out as −fx at the ground on every leg (log §344), and +fy
as −fy on B and C, which `gslip_pronk`'s `lat_sign` undid. Checked after the
fix on the sim plates: fore-aft and lateral rocks, plate vs command in phase
on all four legs at gain 0.98–1.01.

## 5. The module frame and the conversion

Each leg module has its own wiring. Per leg, a direction for each joint,
`d ∈ {+1, −1}`:

| | A | B | C | D | why |
|---|---|---|---|---|---|
| d_θ | +1 | +1 | +1 | +1 | must be +1 (below) |
| d_β | +1 | −1 | −1 | +1 | left/right mirror |
| d_γ | +1 | −1 | +1 | −1 | left/right mirror × front/rear modules rotated 180° (physical, not a bug) |

Robot and sim: `corgi_utils/config/module_directions.yaml` (§1). The sim's
`corgi_driver_pkg/webots_model.yaml` holds only Webots-model details (G-joint
direction, raw motor polarity) and may not set θ, β or γ.

**Conversion** (self-inverse: the same map goes both ways):

```
θ_m = θ          β_m = d_β·β          γ_m = d_γ·γ

if d_β = −1:   (τ_r, τ_l)_m = (−τ_l, −τ_r)     torques: swap AND negate
               (v_r, v_l)_m = (−v_l, −v_r)     velocities: the same
               (g_r, g_l)_m = ( g_l,  g_r)     kp, ki, kd: swap, never negate

τ_h,m = d_γ·τ_h        v_h,m = d_γ·v_h        kp_h, ki_h, kd_h unchanged
```

Why the swap: with β → −β, `phi_l,m = −β + θ − 17° = −phi_r` and
`phi_r,m = −phi_l`. Torques are conjugate to the angles, so they map the same
way and τ·φ̇ (power) is unchanged. A gain multiplies an error whose sign flips
with the angle, so it moves with its motor but stays positive; negating it
would invert the servo.

`d_θ = −1` is not supported: the 17° offset in φ breaks the clean swap. All
legs use +1; the code refuses anything else.

## 6. What still flips above the boundary

| where | what | status |
|---|---|---|
| `gslip_pronk` | `roll_sign`, `yaw_sign`, `pitch_sign`, `steer_sign` | geometry (left/right, front/rear) — keep; ideally derived from hip positions |

Removed on 2026-11-16: the hardware IMU mirror. `corgi_imu` published its
quaternion with pitch and yaw mirrored against its own gyro (vault Open Issues
#47); it now publishes body x forward, y left, z up with orientation and rate
agreeing in sign (nose down is +pitch and +wy), `force_estimation`'s
`pitch *= −1, yaw *= −1` is gone, and `force_control` (which never
compensated) is right on the robot for the first time.
`legacy_orientation:=true` on the IMU node restores the old quaternion.
**Gait gains tuned on the mirror must flip**: `k_pitch` on the robot takes the
sim's sign again (−0.30, not +0.30), `d_pitch` the same sign as `k_pitch`, and
any hardware `k_yaw` / heading gain reverses.

Removed on 2026-11-13: the sim's own copy of the direction table (both
boundaries read `module_directions.yaml`); `force_control`'s unconverted
command (+fx at the ground was −fx on every leg, +fy −fy on B/C;
`leg_frame.hpp`) and with it `gslip_pronk`'s `lat_sign`; the estimator's
per-leg output signs (`leg_frame.hpp`) and its `beta − pitch` on B/C
(uniform `beta + pitch`, checked on the sim plates); the friction
feedforward's sign in `force_control` and `force_estimation` (§3).

## 7. Checking a change

- `colcon test --packages-select corgi_utils` — the conversion, the table
  loader and the leg-frame map; `python3 -m pytest test/` in `corgi_sim` —
  the sim driver's side.
- `measure_kt.py --joint beta` on leg A and on leg B, the same physical load:
  the two slopes have the **same sign**.
- A force-plate rock scored with `force_plate_track.py`: every leg's
  *plate vs command* and *estimate vs plate* in phase.
- A held pitched stance scored with `pitch_sign_check.py`: every leg's best
  k at 0.
- `measure_friction.py`: SIGN "agrees" on every motor.
- `check_imu_frames.py` on a bag with the body rocked by hand: PASS (the
  quaternion and the rate agree on every axis).

The robot bridge keeps the old half-conversion behind
`legacy_torque_frame:=true` for A/B comparison and rollback only.
