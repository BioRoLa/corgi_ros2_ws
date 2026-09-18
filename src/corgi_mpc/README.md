# corgi_mpc

Model Predictive Controller (MPC) for Corgi quadruped locomotion.  
Supports closed-loop walking (time-driven or distance-driven stop) and open-loop walking.

---

## Executables

| Executable | Node Name | Description |
|---|---|---|
| `walk_closed_time` | `corgi_mpc` | Closed-loop MPC, **time-driven stop**: walks for a fixed number of control cycles (`target_loop × dt` seconds) |
| `walk_closed_dist` | `corgi_mpc` | Closed-loop MPC, **distance-driven stop**: decelerates and stops near a target X-axis position (`stop_x`) |
| `walk_h20_v10_open` | `corgi_mpc` | Open-loop H20/V10 gait; sends motor commands directly |
| `wlw_open` | `corgi_wlw_open` | Open-loop WLW Hybrid gait; sends motor commands directly |

> **Migration note**: `walk_h20_v10_closed` has been replaced by `walk_closed_time` (same behaviour, all hardcoded values moved to `config/config.yaml`).

---

## Launch Files

The two main entry points select the controller and its support nodes. The seven older launch file names remain as compatibility entry points.

### Closed-loop walking

```bash
# Simulation, stop after target_loop control cycles
ros2 launch corgi_mpc walk_closed.launch.py environment:=sim stop_mode:=time state_source:=odom_legacy

# Real robot, stop near common.walk.stop_x using legacy odometry
ros2 launch corgi_mpc walk_closed.launch.py environment:=real stop_mode:=distance state_source:=odom_legacy

# Real robot, ESEKF + FAST-LIO, distance stop and GMO contact
ros2 launch corgi_mpc walk_closed.launch.py environment:=real stop_mode:=distance state_source:=esekf contact_source:=gmo
```

| Argument | Default | Meaning |
|---|---|---|
| `environment` | `sim` | `sim` or `real`; selects support nodes and automatic clock/profile/bag defaults |
| `stop_mode` | `time` | `time` runs `walk_closed_time`; `distance` runs `walk_closed_dist` |
| `state_source` | `odom_legacy` | `odom_legacy`, `sim_driver`, or `esekf` |
| `contact_source` | `gait` | `gait` or `gmo`; GMO requires ESEKF |
| `use_sim_time` | `auto` | `auto` follows environment; can be `true` or `false` |
| `config_profile` | `auto` | `auto` follows environment; can be `sim` or `real` |
| `record_bag` | `auto` | `auto` records on real robot and does not record in simulation |
| `record_raw_lidar` | `false` | With ESEKF, also record raw Livox data and `/Odometry` for replay |

`odom_legacy` starts the legacy odometry and height nodes, plus `imu_node` on the real robot. `sim_driver` is simulation-only and also starts legacy odometry for the controller's fallback state; the simulator must supply `/tf`, `/sim/body/velocity`, `/imu`, motor state and trigger. `esekf` is real-robot-only and includes `esekf_stack.launch.py` (raw IMU, leg ESEKF, Livox, FAST-LIO, relay, fusion and static TF). All closed-loop modes start force estimation and force control. The motor driver must be started separately on the real robot.

The ESEKF compatibility entry point preserves the current effective `contact_source:=gait` default. Pass `contact_source:=gmo` to the new entry point when GMO should drive contact selection; the `real:` YAML `contact_source` field does not set this ROS parameter.

If you previously ran `walk_closed_real.launch.py state_source:=esekf` alongside a separately launched ESEKF stack, stop the separate stack when using the consolidated entry point. It now starts the ESEKF nodes automatically, so running both would create duplicate publishers.

### Open-loop walking

```bash
# Real H20/V10 (legacy sensor nodes retained for experiment data)
ros2 launch corgi_mpc walk_open.launch.py environment:=real gait:=h20_v10

# Simulation or real WLW; only the WLW controller is launched
ros2 launch corgi_mpc walk_open.launch.py environment:=sim gait:=wlw
ros2 launch corgi_mpc walk_open.launch.py environment:=real gait:=wlw record_bag:=true
```

`walk_open.launch.py` accepts `environment`, `gait`, `use_sim_time`, `config_profile`, and `record_bag`. Its automatic bag default preserves existing behavior: real H20/V10 records; WLW and simulation do not. Open-loop controllers publish `/motor/command` directly and do not launch force control. WLW has its own small bag topic set when recording is enabled.

### Compatibility entry points

| Existing launch file | Equivalent new selection |
|---|---|
| `walk_closed_sim.launch.py` | `environment:=sim stop_mode:=time state_source:=odom_legacy` |
| `walk_closed_real.launch.py` | `environment:=real stop_mode:=time state_source:=odom_legacy` |
| `walk_closed_legacy.launch.py` | `environment:=real stop_mode:=distance state_source:=odom_legacy` |
| `walk_closed_esekf.launch.py` | `environment:=real stop_mode:=distance state_source:=esekf` |
| `walk_h20_v10_open_real.launch.py` | `environment:=real gait:=h20_v10` |
| `wlw_open_sim.launch.py` | `environment:=sim gait:=wlw` |
| `wlw_open_real.launch.py` | `environment:=real gait:=wlw` |

The original clock, profile and state-source arguments on the applicable compatibility entry points are forwarded. The old H20/V10 `state_source` argument was removed because its controller never used it.

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
| `stop_x` | `3.0` | Target stop position on X axis in odom/world frame (m) |
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

`script/record_mpc_bag.py` builds the topic list from the selected controller mode. It records the common control topics `/trigger`, `/motor/state`, `/motor/command`, and `/walk/swing_phase` plus:

| Mode | Additional topics |
|---|---|
| Closed, `odom_legacy` | `/force/state`, `/impedance/command`, `/imu`, `/odometry/legacy/position`, `/odometry/legacy/velocity`, `/odometry/legacy/contact`, `/odometry/legacy/z_position_hip` |
| Closed, `sim_driver` | Closed legacy topics plus `/tf`, `/sim/body/velocity` for the primary simulated state |
| Closed, `esekf` | `/force/state`, `/impedance/command`, `/imu_raw`, `/ekf`, `/gmo/contact_state`, `/lidar_odom`, `/odom_mapping`, `/fusion/bv` |
| Open, `h20_v10` | `/force/state`, `/imu`, and legacy odometry topics (matching the existing real-robot experiment) |
| Open, `wlw` | No additional topics |

`record_raw_lidar:=true` adds `/livox/lidar`, `/livox/imu`, and `/Odometry` to ESEKF bags for LiDAR reprocessing. Bags start 3 seconds after launch, or 15 seconds for ESEKF, so the initial 15 seconds are not captured. Output is `bag/mpc_<state_source_or_gait>_<timestamp>` in the source package when available. The older shell recorders remain in `script/` for manual use; new launch files use the Python recorder.

To inspect a topic set without recording:

```bash
python3 src/corgi_mpc/script/record_mpc_bag.py --controller open --gait wlw --print-topics
```
