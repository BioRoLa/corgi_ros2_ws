# Frames and signs

The one convention every node above the hardware boundary uses. If code and
this file disagree, one of them is a bug: fix it, then say which here.

Adopted 2026-11-13. Background: the force-plate sessions of 2026-10 traced
every sign surprise to one place, the hardware boundary converting positions
into a common frame but not torques, velocities or gains (implementation log
§343–344; vault note *Torque Frame Mismatch — dir_beta vs Feedforward*).

## 1. The rule

All topics above the hardware boundary (`motor/command`, `motor/state`,
`impedance/command`, `force/state`) use **one convention, identical on all
four legs**. Exactly two places convert between it and each leg module's own
wiring:

| boundary | where | reads |
|---|---|---|
| robot | `corgi_ros_bridge` | `corgi_ros_bridge/config/real_motor_config.yaml` |
| sim | `corgi_sim/corgi_driver_pkg/corgi_driver.py` | `corgi_sim/corgi_driver_pkg/motor_config.yaml` |

Both convert **positions, velocities, torques and gains together**, with the
algebra in §5: C++ in `corgi_utils/module_frame.hpp`, Python in the sim
driver's `ros_to_module` / `convert_torque` / `convert_gains`.

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
γ, in this same frame on every leg.

> Open: the sign of a torque relative to its angle (does +`torque_r` push φ_r
> up?) is not settled. `measure_friction.py` (2026-10-29) found
> `force_control`'s friction feedforward opposing the motion on all eight motors,
> which means either that feedforward or the reading of the sign is wrong. Settle
> it with a lever and a known mass before relying on it.

## 4. Forces

`ImpedanceCmd` `fx fy fz` and `ForceState` `fx fy fz` are **the force the leg
puts on the ground, in the body frame**. A standing leg has fz < 0.

> Known deviation: `force_control` currently produces a +fx command as −fx at
> the ground, in sim and on the robot (log §344, fore-aft rock). The fix belongs
> in `force_control`, not in another sign somewhere else.

## 5. The module frame and the conversion

Each leg module has its own wiring. Per leg, a direction for each joint,
`d ∈ {+1, −1}`:

| | A | B | C | D | why |
|---|---|---|---|---|---|
| d_θ | +1 | +1 | +1 | +1 | must be +1 (below) |
| d_β | +1 | −1 | −1 | +1 | left/right mirror |
| d_γ | +1 | −1 | +1 | −1 | left/right mirror × front/rear modules rotated 180° (physical, not a bug) |

Robot: `joint_dir` in `real_motor_config.yaml`. Sim: `joint_dir.beta` and
`motor_dir.ABAD` in `motor_config.yaml`.

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

To be removed or renamed as geometry, in this order:

| where | what | status |
|---|---|---|
| `force_estimation` | B/C evaluate at `beta − pitch`; B/C use fy = +est (A/D −est) | matches the sim plates on the converted plant (log §343); looks like hip-side geometry inside the estimator's leg model, to be made an explicit side term |
| `force_estimation` | `pitch *= −1`, `yaw *= −1` on hardware | an IMU convention; belongs in the IMU node |
| `force_control` | command fx sign (§4); friction feedforward direction (§3) | open |
| `gslip_pronk` | `roll_sign`, `yaw_sign`, `pitch_sign`, `steer_sign`, `lat_sign` | geometry — keep; ideally derived from hip positions |
| `corgi_sim` | the driver reads its own copy of the direction table (`motor_config.yaml`) | done: converts by default since 2026-11-13 (`CORGI_DIRBETA_TRANSFORM=0` = legacy plant); still to do: read the same table as the bridge |

## 7. Checking a change

- `colcon test --packages-select corgi_utils` — the conversion's unit tests.
- `measure_kt.py --joint beta` on leg A and on leg B, the same physical load:
  the two slopes have the **same sign**.
- A force-plate rock scored with `force_plate_track.py`: every leg's
  *plate vs command* and *estimate vs plate* in phase.

The robot bridge keeps the old half-conversion behind
`legacy_torque_frame:=true` for A/B comparison and rollback only.
