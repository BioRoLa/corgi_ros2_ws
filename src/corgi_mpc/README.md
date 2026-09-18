# corgi_mpc

ROS 2 controllers for the Corgi leg-wheel robot. The closed-loop walking controllers use model predictive control (MPC) to compute contact-force references and publish impedance commands. The package also contains open-loop H20/V10, trot, and WLW gait controllers that publish motor commands directly.

## Research basis and architecture

The closed-loop MPC algorithm is based on Yi-Syuan Shen's 2025 master's thesis, [輪腳複合機器人上可變多觸地點之全機力控制架構開發](https://tdr.lib.ntu.edu.tw/jspui/handle/123456789/99482?mode=full) (National Taiwan University, DOI: [10.6342/NTU202502013](https://doi.org/10.6342/NTU202502013)). The thesis describes a whole-body force-control architecture for a leg-wheel robot with varying ground contact points. This package implements the MPC controller and the launch integrations documented below; the open-loop gait executables serve different control paths.

![MPC control architecture](docs/images/mpc_architecture_white.png)

*System-level MPC architecture. Gait references and estimated body/contact states feed the MPC. Its force references pass through force control and the motor driver. The frequencies in the supplied diagram describe that architecture; this README does not assert that every current ROS node runs at the frequency shown.*

## Launch files

| File | Purpose |
|---|---|
| `walk_closed.launch.py` | Selects the closed-loop controller and its state-estimation and force-control nodes |
| `walk_open.launch.py` | Selects the H20/V10 or WLW open-loop gait |
| `esekf_stack.launch.py` | Support stack included automatically by `walk_closed.launch.py` when `state_source:=esekf` |

## Quick start

Run these commands from a sourced ROS 2 workspace containing the required packages. Start the simulator separately for simulation modes. Start the motor driver separately before running a real-robot mode. The controller currently loads `config/config.yaml` from `~/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml`; check this path if the workspace is elsewhere.

```bash
# Simulation: closed-loop walking with legacy odometry; stop after a set time.
ros2 launch corgi_mpc walk_closed.launch.py environment:=sim stop_mode:=time state_source:=odom_legacy

# Real robot: closed-loop walking with legacy odometry; stop near stop_x.
ros2 launch corgi_mpc walk_closed.launch.py environment:=real stop_mode:=distance state_source:=odom_legacy

# Real robot: ESEKF state, GMO contact, and distance-based stopping.
ros2 launch corgi_mpc walk_closed.launch.py environment:=real stop_mode:=distance state_source:=esekf contact_source:=gmo

# Real robot: open-loop H20/V10 walking.
ros2 launch corgi_mpc walk_open.launch.py environment:=real gait:=h20_v10

# Simulation: open-loop WLW gait.
ros2 launch corgi_mpc walk_open.launch.py environment:=sim gait:=wlw
```

Real-robot closed-loop and H20/V10 launches record a bag by default. WLW and simulation do not; add `record_bag:=true` to enable recording. ESEKF starts its bag after 15 seconds, while other modes start after 3 seconds.

## Controllers and operating modes

| Executable | Control path | Output | Stop or duration rule |
|---|---|---|---|
| `walk_closed_time` | Closed-loop MPC | `/impedance/command` | `common.walk.target_loop` control cycles |
| `walk_closed_dist` | Closed-loop MPC | `/impedance/command` | Decelerate near `common.walk.stop_x` |
| `walk_h20_v10_open` | Open-loop H20/V10 gait | `/motor/command` | Controller's internal `target_loop` |
| `wlw_open` | Open-loop WLW Hybrid gait | `/motor/command` | `common.wlw.target_loop` duration setting |
| `trot_open` | Open-loop trot gait | `/motor/command` | `common.trot.target_loop` duration setting |

All five executables publish `/walk/swing_phase`. `trot_open` is built and installed but has no entry in the consolidated launch files; run it only with the supporting system required by that controller. The open-loop controllers do not use the `state_source` argument and do not launch force control.

### Closed-loop launch arguments

| Argument | Default | Purpose |
|---|---|---|
| `environment` | `sim` | `sim` or `real`; selects supporting nodes and automatic defaults |
| `stop_mode` | `time` | `time` selects `walk_closed_time`; `distance` selects `walk_closed_dist` |
| `state_source` | `odom_legacy` | `odom_legacy`, `sim_driver`, or `esekf` |
| `contact_source` | `gait` | `gait` or `gmo`; GMO requires ESEKF |
| `use_sim_time` | `auto` | `auto` follows `environment`, or set `true`/`false` |
| `config_profile` | `auto` | `auto` follows `environment`, or set `sim`/`real` |
| `record_bag` | `auto` | `auto` records on the real robot only, or set `true`/`false` |
| `record_raw_lidar` | `false` | Include raw Livox topics in an ESEKF bag |

The selected state source determines which support nodes start:

| State source | Supported environment | State inputs | Launched support nodes |
|---|---|---|---|
| `odom_legacy` | Simulation or real robot | Legacy position/velocity and `/imu` | Legacy odometry and height estimator; real robot also starts `imu_node` |
| `sim_driver` | Simulation only | `/tf` (`odom` to `base_link`) and `/sim/body/velocity` | Legacy odometry and height estimator for controller fallback; the simulator supplies primary state and IMU |
| `esekf` | Real robot only | `/ekf`, with `/imu_raw` available before EKF data | `esekf_stack.launch.py`: raw IMU, leg ESEKF, Livox, FAST-LIO, odometry relay, fusion, and static TF |

Every closed-loop mode also launches force estimation and force control. The controller publishes `/impedance/command`; force control converts it to `/motor/command`. `contact_source:=gmo` selects measured contact when GMO data is available. The controller's default is `gait`: the `real.contact_source` value in `config.yaml` does not set this ROS parameter.

### Open-loop launch arguments

`walk_open.launch.py` accepts `environment:=sim/real`, `gait:=h20_v10/wlw`, `use_sim_time:=auto/true/false`, `config_profile:=auto/sim/real`, and `record_bag:=auto/true/false`. Real H20/V10 also launches IMU, force estimation, and legacy odometry nodes for experiment data. WLW launches only its controller node. Both publish `/motor/command` directly.

## ROS interfaces

| Controller | Subscribed topics | Published topics |
|---|---|---|
| Closed-loop, common | `/trigger`, `/motor/state` | `/impedance/command`, `/walk/swing_phase` |
| Closed-loop, legacy state | `/imu`, `/odometry/legacy/position`, `/odometry/legacy/velocity`, `/odometry/legacy/z_position_hip` | Same closed-loop outputs |
| Closed-loop, simulator state | `/tf`, `/sim/body/velocity`, `/imu`; legacy topics support fallback | Same closed-loop outputs |
| Closed-loop, ESEKF state | `/ekf`, `/imu_raw`; `/gmo/contact_state` when GMO contact is selected | Same closed-loop outputs |
| Open-loop H20/V10, trot, or WLW | `/trigger` | `/motor/command`, `/walk/swing_phase` |

The closed-loop executables subscribe to all supported state topics; `state_source` selects which incoming data drives the controller. Topic names above are shown from the default root namespace.

## Configuration

The controllers read [config/config.yaml](config/config.yaml) when they start. Edit the source file, then restart the affected controller or launch; changing the YAML while a node is running does not update its active values. The current C++ code uses the fixed path `~/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml`. If the workspace is elsewhere, make that path resolve to the file or update the source path.

### Select a profile and find the effective value

`config_profile:=sim` selects the `sim:` section; `config_profile:=real` selects `real:`. The consolidated launch files use `config_profile:=auto` by default, which selects the profile named by `environment`. You can override it explicitly, for example `environment:=sim config_profile:=real`, if you intend to test real-robot gains in simulation.

| Controller | YAML lookup for gait settings | Other YAML settings |
|---|---|---|
| `walk_closed_time`, `walk_closed_dist` | Selected profile first, then `common.walk` | MPC weights, gains, mass and force bounds: selected profile first, then `common` |
| `trot_open` | Selected profile first, then `common.trot`; `init_eta` must be in the selected profile | Loads MPC model and gain settings from the selected profile and `common` |
| `wlw_open` | `common.wlw` only | `config_profile` selects simulation or real behavior; it does not override `common.wlw` values |
| `walk_h20_v10_open` | Gait timing and geometry are still set in its C++ source | Loads MPC model and gain settings from the selected profile and `common` |

For example, adding `cruise_velocity: 0.08` directly under `real:` overrides `common.walk.cruise_velocity` only when the closed-loop controller runs with `config_profile:=real`. Keep all required YAML keys when editing the file; the controllers fail at startup if required values are missing or have the wrong type.

### Closed-loop walking (`common.walk`)

| Key | Current value | Use |
|---|---:|---|
| `stand_height` | 0.20 m | Target body height |
| `cruise_velocity` | 0.10 m/s | Forward cruise speed |
| `step_length` | 0.20 m | Nominal foot step length |
| `step_height` | 0.08 m | Nominal swing clearance |
| `ramp_loops` | 100 cycles | Acceleration and deceleration duration; 1 s at 100 Hz |
| `target_loop` | 2200 cycles | Time-stop mode only; 22 s at 100 Hz |
| `stop_x` | 3.0 m | Distance-stop mode only; target X position in the controller's odometry frame |
| `decel_margin` | 1.2 | Distance-stop braking-distance multiplier |

`walk_closed_time` uses `target_loop`; `walk_closed_dist` uses `stop_x` and `decel_margin`. With the current values, the distance controller begins deceleration at `stop_x - 0.5 × cruise_velocity × (ramp_loops / 100) × decel_margin`, or about **2.94 m**. `stop_x` is an absolute estimated X position, not a distance measured from the trigger point.

To change the real-robot distance target, edit the existing `common.walk.stop_x` value in `config/config.yaml` (for example, from `3.0` to `2.0`), then restart `walk_closed.launch.py` with `environment:=real stop_mode:=distance`. No `stop_x` launch argument exists. A value placed directly under `real:` takes precedence over `common.walk.stop_x` for the real profile.

### MPC and open-loop settings

`common` contains shared physical constants such as `m` and the force bounds. `sim` and `real` contain the 13-element `Q_diagonal`, impedance stiffness (`Kx`, `Ky`) and damping (`Bx`, `By`) values for stance and swing, `fz_lower_bound`, and `init_eta`. The eight `init_eta` entries are the initial joint angles in the order `[theta_A, beta_A, theta_B, beta_B, theta_C, beta_C, theta_D, beta_D]`, in radians. Closed-loop walking and `trot_open` read `init_eta` from the selected profile; H20/V10 uses initial angles in its source, while WLW initializes from its Hybrid gait. Choose the profile with `config_profile`, then edit that profile's values to change the settings that its controller reads.

`common.trot` controls the open-loop trot's velocity, body height, step geometry, ramp lengths, and duration. Its `target_loop` uses a legacy factor of 10: the current value `600` gives `600 × 10 / 1000 = 6` seconds at 1 kHz. `common.wlw` controls WLW velocity, body height, step length, first swinging leg (`swing_index`), ramp, and duration. Its current `target_loop` of `3000` gives 30 seconds by the same 1 kHz convention. WLW reads this block directly; changing `sim` or `real` gain values does not change its `common.wlw` gait settings.

`contact_source` under `sim:` or `real:` is not loaded as the closed-loop controller's ROS parameter. To use GMO contact, pass `contact_source:=gmo` to `walk_closed.launch.py` with `state_source:=esekf`; otherwise the effective controller default is `gait`.

## Bag recording

`script/record_mpc_bag.py` selects the topic set from the controller mode. Every set includes `/trigger`, `/motor/state`, `/motor/command`, and `/walk/swing_phase`.

| Mode | Additional recorded topics |
|---|---|
| Closed, `odom_legacy` | `/force/state`, `/impedance/command`, `/imu`, and `/odometry/legacy/{position,velocity,contact,z_position_hip}` |
| Closed, `sim_driver` | Closed legacy set plus `/tf` and `/sim/body/velocity` |
| Closed, `esekf` | `/force/state`, `/impedance/command`, `/imu_raw`, `/ekf`, `/gmo/contact_state`, `/lidar_odom`, `/odom_mapping`, `/fusion/bv` |
| Open, `h20_v10` | `/force/state`, `/imu`, and legacy odometry topics |
| Open, `wlw` | No additional topics |

Set `record_raw_lidar:=true` with ESEKF to add `/livox/lidar`, `/livox/imu`, and `/Odometry`. ESEKF recording starts 15 seconds after launch, so that initial period is not in the bag. The default output is `bag/mpc_<state_source_or_gait>_<timestamp>` in the source package when it is available. The older shell recorders remain available for manual use.

To inspect a topic list without recording:

```bash
python3 src/corgi_mpc/script/record_mpc_bag.py --controller closed --state-source esekf --print-topics
```
