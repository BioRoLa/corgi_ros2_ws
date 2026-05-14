# corgi_mpc

Model Predictive Controller (MPC) for Corgi quadruped locomotion.  
Supports closed-loop and open-loop walking at stand height 20 cm, velocity 10 cm/s.

---

## Executables

| Executable | Node Name | Description |
|---|---|---|
| `walk_h20_v10_closed` | `corgi_mpc` | Closed-loop MPC: computes impedance commands |
| `walk_h20_v10_open` | `corgi_walk` | Open-loop MPC: outputs motor commands directly without impedance control |

---

## Launch Files

### Simulation — Closed-loop

```bash
ros2 launch corgi_mpc walk_h20_v10_closed_sim.launch.py
```

Arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `config_profile` | `sim` | MPC parameter profile |
| `state_source` | `odom_legacy` | Robot state source: `odom_legacy` or `sim_driver` |

Launched nodes:
- `force_estimation_node` (corgi_force_estimation)
- `force_control_node` (corgi_force_control)
- `corgi_odometry_legacy` (corgi_odometry_legacy)
- `corgi_z_position_legacy` (corgi_odometry_legacy)
- `corgi_mpc` (corgi_mpc)

---

### Real Robot — Closed-loop

```bash
ros2 launch corgi_mpc walk_h20_v10_closed_real.launch.py
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

### Real Robot — Open-loop

```bash
ros2 launch corgi_mpc walk_h20_v10_open_real.launch.py
```

Launched nodes:
- `imu_node` (corgi_imu)
- `force_estimation_node` (corgi_force_estimation)
- `corgi_odometry_legacy` (corgi_odometry_legacy)
- `corgi_z_position_legacy` (corgi_odometry_legacy)
- `corgi_walk` (corgi_mpc)
- Bag recording starts automatically after 3 seconds

---

## Topics

### `walk_h20_v10_closed` (Closed-loop)

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
| `/tf` | `tf2_msgs/TFMessage` | `odom → base_link` transform for body position (`state_source:=sim_driver` only) |
| `/force/state` | `corgi_msgs/ForceStateStamped` | Measured foot forces (unused in code) |
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
| `/force/state` | `corgi_msgs/ForceStateStamped` | Measured foot forces (unused in code) |

---

## Parameters

| Parameter | Values | Default | Applicable Executable |
|---|---|---|---|
| `config_profile` | `sim` / `real` | `sim` | both |
| `state_source` | `odom_legacy` / `sim_driver` | `odom_legacy` | closed-loop only |
| `use_sim_time` | `true` / `false` | `true` (sim) / `false` (real) | both |

### `state_source` behaviour (closed-loop only)

- `odom_legacy`: uses `/odometry/legacy/position`, `/odometry/legacy/velocity`, `/odometry/legacy/z_position_hip`
- `sim_driver`: uses `/tf` (`odom → base_link`) and `/sim/body/velocity`; falls back to `odom_legacy` with a warning if those topics are not available

---

## Configuration

Parameter profiles are defined in `config/config.yaml`.

```
common:       # physical constants shared between profiles
sim:          # gains and bounds tuned for Webots
real:         # gains and bounds tuned for real robot
```

Key parameters per profile: `Q_diagonal`, `Bx/By_swing`, `Bx/By_stance`, `Kx/Ky_swing`, `Kx/Ky_stance`, `fz_lower_bound`

---

## Bag Recording

Scripts are located in `script/` and are called automatically by the real-robot launch files.

| Script | Output path | Recorded topics |
|---|---|---|
| `mpc_closed_bag.sh` | `bag/mpc_closed_<timestamp>` | `/trigger`, `/imu`, `/impedance/command`, `/motor/command`, `/motor/state`, `/force/state`, `/odometry/legacy/*`, `/walk/swing_phase` |
| `mpc_open_bag.sh` | `bag/mpc_open_<timestamp>` | `/trigger`, `/imu`, `/motor/command`, `/motor/state`, `/force/state`, `/odometry/legacy/*`, `/walk/swing_phase` |