# corgi_mpc

Model Predictive Controller (MPC) for Corgi quadruped locomotion.  
Supports closed-loop walking (time-driven or distance-driven stop) and open-loop walking.

---

## Executables

| Executable | Node Name | Description |
|---|---|---|
| `walk_closed_time` | `corgi_mpc` | Closed-loop MPC, **time-driven stop**: walks for a fixed number of control cycles (`target_loop × dt` seconds) |
| `walk_closed_dist` | `corgi_mpc` | Closed-loop MPC, **distance-driven stop**: decelerates and stops near a target X-axis position (`stop_x`) |
| `walk_h20_v10_open` | `corgi_walk` | Open-loop MPC: outputs motor commands directly without impedance control |

> **Migration note**: `walk_h20_v10_closed` has been replaced by `walk_closed_time` (same behaviour, all hardcoded values moved to `config/config.yaml`).

---

## Launch Files

### Simulation — Closed-loop (time-driven)

```bash
ros2 launch corgi_mpc walk_closed_sim.launch.py
```

Arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `config_profile` | `sim` | MPC parameter profile |
| `state_source` | `odom_legacy` | Robot state source: `odom_legacy` \| `sim_driver` \| `esekf` |

Launched nodes:
- `force_estimation_node` (corgi_force_estimation)
- `force_control_node` (corgi_force_control)
- `corgi_odometry_legacy` (corgi_odometry_legacy)
- `corgi_z_position_legacy` (corgi_odometry_legacy)
- `walk_closed_time` / `corgi_mpc` (corgi_mpc)

---

### Real Robot — Closed-loop (time-driven, legacy odometry)

```bash
ros2 launch corgi_mpc walk_closed_real.launch.py
```

Arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | — |
| `config_profile` | `real` | MPC parameter profile |
| `state_source` | `odom_legacy` | Robot state source |

Launched nodes (same as sim, plus):
- `imu_node` (corgi_imu)
- Bag recording starts automatically after 3 seconds (see [Bag Recording](#bag-recording))

---

### Real Robot — Closed-loop (ESEKF + GMO, time-driven)

```bash
ros2 launch corgi_mpc walk_closed_esekf.launch.py
```

Launched nodes (in addition to base nodes):
- `imu_raw_node`, `corgi_leg_odom`, `livox_ros_driver2`, `fastlio_mapping`
- `odom_tf_relay`, `corgi_fusion_node`
- `walk_closed_time` with `state_source:=esekf`, `contact_source:=gmo`

---

### Distance-driven stop (direct run)

To stop the robot near a target position instead of after a fixed time, run `walk_closed_dist` directly:

```bash
# Simulation
ros2 run corgi_mpc walk_closed_dist --ros-args \
  -p config_profile:=sim \
  -p state_source:=sim_driver \
  --ros-args -r __ns:=/

# Real robot with ESEKF
ros2 run corgi_mpc walk_closed_dist --ros-args \
  -p config_profile:=real \
  -p state_source:=esekf \
  -p contact_source:=gmo
```

Set `stop_x` and `decel_margin` in `config/config.yaml` → `common:` before running.

---

### Real Robot — Open-loop

```bash
ros2 launch corgi_mpc walk_h20_v10_open_real.launch.py
```

Launched nodes:
- `imu_node` (corgi_imu)
- `force_estimation_node` (corgi_force_estimation)
- `corgi_odometry_legacy` (corgi_odometry_legacy)
- `corgi_z_position_legacy` (corgi_odometry_legacy)
- `walk_h20_v10_open` / `corgi_walk` (corgi_mpc)
- Bag recording starts automatically after 3 seconds

---

## Topics

### `walk_closed_time` / `walk_closed_dist` (Closed-loop)

**Published:**

| Topic | Type | Description |
|---|---|---|
| `/impedance/command` | `corgi_msgs/ImpedanceCmdStamped` | Impedance commands for all 4 modules |
| `/walk/swing_phase` | `std_msgs/Int32MultiArray` | Swing phase flag per leg (size 4) |

**Subscribed:**

| Topic | Type | Description |
|---|---|---|
| `/trigger` | `corgi_msgs/TriggerStamped` | Enable/disable walking |
| `/motor/state` | `corgi_msgs/MotorStateStamped` | Motor encoder states |
| `/imu` | `corgi_msgs/ImuStamped` | IMU orientation and angular velocity |
| `/odometry/legacy/position` | `geometry_msgs/Vector3` | Body position from legacy odometry |
| `/odometry/legacy/velocity` | `geometry_msgs/Vector3` | Body velocity from legacy odometry |
| `/odometry/legacy/z_position_hip` | `std_msgs/Float64` | Hip height from legacy odometry |
| `/sim/body/velocity` | `geometry_msgs/Vector3` | Body velocity from simulator (`state_source:=sim_driver` only) |
| `/tf` | `tf2_msgs/TFMessage` | `odom → base_link` transform (`state_source:=sim_driver` only) |
| `/ekf` | `nav_msgs/Odometry` | Inner ESEKF pose + twist from `corgi_leg_odom` (`state_source:=esekf` only) |
| `/imu_raw` | `corgi_msgs/ImuStamped` | Raw IMU gyro fallback (`state_source:=esekf` before `/ekf` is ready) |
| `/gmo/contact_state` | `corgi_msgs/GMOContactStateStamped` | Sensor-based contact detection (`contact_source:=gmo`) |

---

### `walk_h20_v10_open` (Open-loop)

**Published:**

| Topic | Type | Description |
|---|---|---|
| `/motor/command` | `corgi_msgs/MotorCmdStamped` | Direct motor angle commands |
| `/walk/swing_phase` | `std_msgs/Int32MultiArray` | Swing phase flag per leg (size 4) |

**Subscribed:**

| Topic | Type | Description |
|---|---|---|
| `/trigger` | `corgi_msgs/TriggerStamped` | Enable/disable walking |

---

## Parameters

| Parameter | Values | Default | Applicable Executable |
|---|---|---|---|
| `config_profile` | `sim` / `real` | `sim` | all |
| `state_source` | `odom_legacy` / `sim_driver` / `esekf` | `odom_legacy` | closed-loop only |
| `contact_source` | `gait` / `gmo` | `gait` | closed-loop only |
| `use_sim_time` | `true` / `false` | `true` (sim) / `false` (real) | all |

### `state_source` behaviour (closed-loop only)

| Value | pos / vel source | ang / ang_vel source | when not ready |
|---|---|---|---|
| `odom_legacy` | `/odometry/legacy/position`, `/odometry/legacy/velocity` | `/imu` (CX5 AHRS) | — |
| `sim_driver` | `/tf` (`odom→base_link`) + `/sim/body/velocity` | `/imu` | fallback to `odom_legacy` |
| `esekf` | `/ekf` (nav_msgs/Odometry) | `/ekf` (bias-corrected) | fallback to `odom_legacy` + `/imu_raw` gyro |

**Switching usage:**

```bash
# Legacy odometry (default baseline)
ros2 run corgi_mpc walk_closed_time --ros-args -p state_source:=odom_legacy

# ESEKF odometry (requires corgi_leg_odom to be running)
ros2 run corgi_mpc walk_closed_time --ros-args \
  -p state_source:=esekf -p contact_source:=gmo

# Distance-driven stop with ESEKF
ros2 run corgi_mpc walk_closed_dist --ros-args \
  -p config_profile:=real -p state_source:=esekf -p contact_source:=gmo
```

> **Note**: `esekf` requires `corgi_leg_odom` (inner ESEKF node) to be running.  
> `contact_source:=gmo` is only meaningful when `state_source:=esekf` is active.

---

## Configuration

All gait and walk parameters are defined in `config/config.yaml`.  
Lookup order: profile-specific section (`sim:` / `real:`) first, then `common:`.

```
common:       # physical constants + shared gait parameters
sim:          # gains, bounds, and init_eta for Webots
real:         # gains, bounds, and init_eta for the physical robot
```

### Gait parameters (`common:`)

| Key | Default | Description |
|---|---|---|
| `stand_height` | `0.2` | Target body CoM height above ground (m) |
| `cruise_velocity` | `0.1` | Maximum forward walking speed (m/s) |
| `step_length` | `0.2` | Foot step length (m) |
| `step_height` | `0.08` | Foot swing clearance height (m) |
| `ramp_loops` | `100` | Velocity ramp-up/down duration in control cycles (= 1 s at 100 Hz) |

### Time-driven stop (`walk_closed_time`, `common:`)

| Key | Default | Description |
|---|---|---|
| `target_loop` | `2200` | Total MPC cycles before stopping (= 22 s at 100 Hz) |

### Distance-driven stop (`walk_closed_dist`, `common:`)

| Key | Default | Description |
|---|---|---|
| `stop_x` | `2.0` | Target stop position on X axis in odom/world frame (m) |
| `decel_margin` | `1.2` | Safety multiplier on decel distance; `> 1.0` starts braking earlier |

Deceleration starts at `stop_x − decel_dist`, where:

$$\text{decel\_dist} = \tfrac{1}{2} \times \text{cruise\_velocity} \times (\text{ramp\_loops} \times dt) \times \text{decel\_margin}$$

With defaults: $0.5 \times 0.1 \times 1.0 \times 1.2 = 0.06\ \text{m}$

### Profile-specific parameters (`sim:` / `real:`)

| Key | Description |
|---|---|
| `Q_diagonal` | MPC state cost weights (13 elements). **X/Y position weights are 0** — forward motion is controlled purely through `target_vel_x`; `target_pos_x` has no effect on force output |
| `Bx/By_swing`, `Bx/By_stance` | Impedance damping coefficients |
| `Kx/Ky_swing`, `Kx/Ky_stance` | Impedance stiffness coefficients |
| `fz_lower_bound` | Lower bound on normal foot force (N) |
| `init_eta` | Initial joint angles `[θ_A, β_A, θ_B, β_B, θ_C, β_C, θ_D, β_D]` (rad) |

---

## Bag Recording

Scripts are located in `script/` and are called automatically by the real-robot launch files.

| Script | Output path | Recorded topics |
|---|---|---|
| `mpc_closed_bag.sh` | `bag/mpc_closed_<timestamp>` | `/trigger`, `/imu`, `/impedance/command`, `/motor/command`, `/motor/state`, `/force/state`, `/odometry/legacy/*`, `/walk/swing_phase` |
| `mpc_open_bag.sh` | `bag/mpc_open_<timestamp>` | `/trigger`, `/imu`, `/motor/command`, `/motor/state`, `/force/state`, `/odometry/legacy/*`, `/walk/swing_phase` |