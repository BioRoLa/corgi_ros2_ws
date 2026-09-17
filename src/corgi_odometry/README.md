# corgi_odometry

`corgi_odometry` is the state-estimation package for the CORGI leg-wheel robot. It provides:

- Contact-state detection using a Generalized Momentum Observer (GMO).
- An inner error-state extended Kalman filter (ES-EKF) with IMU propagation and leg-velocity constraints.
- An outer ES-EKF that fuses the inner estimate with LiDAR odometry.
- Simulation helpers for IMU noise, fake LiDAR odometry, and ground-truth state estimation.

## Method and Code Sources

The contact-state and contact-point estimation method is based on:

> “Proprioceptive Contact State and Contact Point Estimation for a Leg-Wheel
> Transformable Robot,” in *2026 IEEE International Conference on Robotics and
> Automation (ICRA)*, Vienna, Austria, 2026.

The GMO dynamics derivation and original implementation are available from:

- Repository: <https://github.com/hiho817/ContactLegEstimator>
- SSH: `git@github.com:hiho817/ContactLegEstimator.git`

The two-layer state-estimation diagram is adapted from the master's thesis 《輪腳複合機器人之雙層狀態估測與速度偏差修正》:

![CORGI two-layer state-estimation architecture](docs/estimator_architecture.png)

## Estimation Architecture

### Inner ES-EKF

The inner ES-EKF performs prediction with IMU measurements and uses leg-kinematic velocity as an observation. The GMO determines whether each leg is in contact using IMU data, motor states, and the estimated state. Only leg observations that pass both the contact-state and innovation-reliability checks are used for filter updates.

```text
/imu + /motor/state
        │
        ├──► GMO ──► contact state
        │                 │
        └──► IMU predict  ├──► leg observation ──► innovation gate
                          │                           │
                          └───────────────────────────┴──► /ekf
```

### Outer ES-EKF

The outer ES-EKF uses the inner `/ekf` output for prediction and `/lidar_odom` to update the map-to-odom correction and velocity bias:

```text
/ekf ──────────► outer predict ─────┐
                                    ├──► /odom_mapping
/lidar_odom ───► outer update ──────┘
                         │
                         └──► /fusion/bv ──► inner ES-EKF velocity correction
```

`/odom_mapping` is the outer-fusion result, while `/ekf` is the inner leg ES-EKF result. They are outputs from different estimator layers and should not be treated as the same estimate.

## Nodes

| Executable | Purpose | Main inputs | Main outputs |
|---|---|---|---|
| `corgi_leg_odom` | Inner ES-EKF, GMO, and leg updates | `/imu`, `/motor/state`, `/trigger`, `/fusion/bv` | `/ekf`, `/gmo/contact_state`, `/ekf/ba`, `/ekf/bw` |
| `corgi_fusion_node` | Outer ES-EKF | `/ekf`, `/lidar_odom`, `/trigger` | `/odom_mapping`, `/fusion/bv`, TF `map → odom` |
| `corgi_contact_leg_est` | Standalone GMO contact estimator for simulation and diagnostics | `/imu`, `/motor/state`, `/trigger`, `/sim/position`, `/sim/velocity` | `/gmo/contact_state` |
| `velocity_estimator` | Converts simulation TF into ground-truth states | TF `odom → base_link` | `/sim/position`, `/sim/velocity`, `/sim/body_velocity` |
| `imu_noise_sim` | Injects reproducible IMU noise and bias | `/imu` | `/imu_noisy` |
| `fake_lidar_odom` | Generates simulated LiDAR odometry | `/sim/position` | `/lidar_odom` |
| `odom_tf_relay.py` | Converts the FAST-LIO `body` pose into a `base_link` pose | `/Odometry` | `/lidar_odom` |

### Why the Standalone Contact Estimator Is Simulation-Only

`corgi_contact_leg_est` requires position and velocity before it can start processing. In simulation, `velocity_estimator` derives both from the ground-truth TF. A real robot does not provide independent ground-truth position and velocity, so the standalone node is not a valid real-robot contact-estimation entry point.

Use `corgi_leg_odom` on the real robot. It supplies the GMO with position and velocity from its own ES-EKF state and does not depend on simulation ground truth.

## Launch Files

The package provides five launch entry points. It does not provide auto-trigger or bag-replay launches. `/trigger` must be published by the motor driver, an experiment controller, or another external node.

| Launch file | Environment | Purpose |
|---|---|---|
| `contact_leg_estimator_sim.launch.py` | Simulation | Ground-truth velocity estimator and standalone GMO contact estimator |
| `leg_odom_real.launch.py` | Real robot | Raw IMU and inner ES-EKF, with optional bag recording |
| `leg_odom_sim.launch.py` | Simulation | Ground truth, deterministic IMU noise, and inner ES-EKF |
| `odom_fusion_real.launch.py` | Real robot | Inner ES-EKF, Livox, FAST-LIO, and outer fusion, with optional bag recording |
| `odom_fusion_sim.launch.py` | Simulation | Inner ES-EKF, fake LiDAR, and outer fusion |

### 1. Standalone Contact Estimation in Simulation

```bash
ros2 launch corgi_odometry contact_leg_estimator_sim.launch.py
```

Starts:

- `velocity_estimator`
- `corgi_contact_leg_est`

Data flow:

```text
simulator TF odom→base_link
             │
             ▼
    velocity_estimator
      ├─► /sim/position
      └─► /sim/velocity
             │
/imu + /motor/state + /trigger
             │
             ▼
   corgi_contact_leg_est ──► /gmo/contact_state
```

This launch is simulation-only. If TF `odom → base_link` is unavailable, `velocity_estimator` cannot generate position and velocity, and the contact estimator will continue waiting for data.

### 2. Inner ES-EKF on the Real Robot

```bash
ros2 launch corgi_odometry leg_odom_real.launch.py
```

Starts:

- `corgi_imu/imu_raw_node`
- `corgi_odometry/corgi_leg_odom`

External requirements:

- A motor driver publishing `/motor/state`.
- An external node publishing `/trigger`.
- A working IMU driver publishing `/imu_raw`.

The estimator's `/imu` input is remapped to `/imu_raw`.

Available arguments:

| Argument | Default | Description |
|---|---:|---|
| `imu_only` | `false` | Run IMU prediction only; disable the GMO, leg updates, ZUPT, and fusion feedback |
| `record_bag` | `false` | Start `leg_odom_bag.sh` |
| `record_delay` | `3.0` | Delay before starting bag recording, in seconds |

Example:

```bash
ros2 launch corgi_odometry leg_odom_real.launch.py \
  record_bag:=true record_delay:=3.0
```

IMU-only ablation:

```bash
ros2 launch corgi_odometry leg_odom_real.launch.py imu_only:=true
```

### 3. Inner ES-EKF in Simulation

```bash
ros2 launch corgi_odometry leg_odom_sim.launch.py
```

Starts:

- `velocity_estimator`: publishes simulation ground truth for analysis or other simulation nodes.
- `imu_noise_sim`: converts `/imu` into `/imu_noisy`.
- `corgi_leg_odom`: consumes `/imu_noisy`.

Available arguments:

| Argument | Default | Description |
|---|---:|---|
| `imu_only` | `false` | Enable the IMU-only ablation mode |
| `imu_seed` | `42` | Random seed for IMU noise and bias; a fixed value reproduces the same noise realization |

### 4. Inner and Outer Fusion on the Real Robot

```bash
ros2 launch corgi_odometry odom_fusion_real.launch.py
```

Starts:

1. `imu_raw_node`
2. `corgi_leg_odom`
3. Livox MID-360 driver
4. FAST-LIO
5. `odom_tf_relay.py`
6. `corgi_fusion_node`
7. Static TF `base_link → mid360_optical`
8. Optional bag recorder

FAST-LIO publishes `/Odometry` as `camera_init → body`. `odom_tf_relay.py` applies the known sensor extrinsics and republishes the pose as `/lidar_odom` in the `base_link` child frame for the outer fusion filter.

Available arguments:

| Argument | Default | Description |
|---|---:|---|
| `imu_only` | `false` | Run the inner estimator in IMU-only mode |
| `record_bag` | `false` | Record odometry and fusion topics without point clouds |
| `record_delay` | `15.0` | Delay recording until FAST-LIO has initialized, in seconds |

The real-robot motor driver and `/trigger` publisher must still be started separately.

### 5. Inner and Outer Fusion in Simulation

```bash
ros2 launch corgi_odometry odom_fusion_sim.launch.py
```

Data flow:

```text
/imu ─► imu_noise_sim ─► /imu_noisy ─► corgi_leg_odom ─► /ekf
                                                        │
sim TF ─► velocity_estimator ─► /sim/position            ├─► fusion
                                  │                      │
                                  └─► fake_lidar ─► /lidar_odom
```

Available arguments:

| Argument | Default | Description |
|---|---:|---|
| `imu_only` | `false` | Run the inner estimator in IMU-only mode |
| `imu_seed` | `42` | IMU noise seed |
| `lidar_seed` | `12345` | Fake-LiDAR noise seed |
| `lidar_event_driven` | `false` | Publish from simulation timestamp events when `true`; use a wall timer when `false` |

## Configuration

```text
config/
├── leg_odom/
│   └── config_online.yaml
└── fusion/
    └── config_fusion.yaml
```

The online nodes load these files at startup:

- Inner ES-EKF and GMO: `config/leg_odom/config_online.yaml`
- Outer fusion: `config/fusion/config_fusion.yaml`

Restart the node after modifying a YAML file. When not using a symlink install, rebuild the package so that the updated YAML is copied into the package share directory.

### Inner ES-EKF and GMO

Edit:

```text
config/leg_odom/config_online.yaml
```

#### `esekf`

| Parameter | Meaning | Effect of increasing the value |
|---|---|---|
| `sigma_a` | Per-axis accelerometer noise standard deviation | Reduces trust in IMU acceleration propagation |
| `sigma_w` | Per-axis gyroscope noise standard deviation | Reduces trust in IMU angular-rate propagation |
| `sigma_ba` | Accelerometer-bias random walk | Allows accelerometer bias to change faster |
| `sigma_bw` | Gyroscope-bias random walk | Allows gyroscope bias to change faster |
| `sigma_leg_vec` | Leg-velocity observation noise standard deviation `[x,y,z]` | Reduces the weight of leg-velocity observations on that axis |
| `mahalanobis_threshold` | Leg-innovation rejection threshold | Relaxes the outlier gate, so fewer observations are rejected |

`sigma_leg_vec` contains standard deviations, not variances. The implementation squares these values when constructing the observation covariance. `mahalanobis_threshold: 16.27` corresponds to a 99.9% chi-squared gate for a three-dimensional innovation. A very large value approximately disables rejection, but this is not recommended for normal operation.

#### `observer`

| Parameter | Meaning | Tuning guidance |
|---|---|---|
| `cutoff_freq` | GMO disturbance-observer LPF cutoff | A higher value responds faster but admits more noise; a lower value is smoother but delays contact detection |

#### `contact`

Contact state uses a Schmitt trigger:

```text
Currently not in contact: |rm| > rm_high or |beta| > beta_high → contact
Currently in contact:     |rm| < rm_low  and |beta| < beta_low → no contact
```

The thresholds must satisfy:

```text
rm_threshold_high > rm_threshold_low
beta_threshold_high > beta_threshold_low
```

Use recorded `/gmo/contact_state` data to inspect the stance and swing distributions of `rm_force` and `beta_torque` before tuning. Avoid changing the observer cutoff and contact thresholds at the same time, or the source of an improvement will be unclear.

#### `static_init`

| Parameter | Description |
|---|---|
| `window_ms` | IMU window before the trigger used to initialize bias and attitude |
| `motion_gyro_thresh` | Warn if the average angular rate in the initialization window exceeds this value |
| `initial_z` | Initial body height of the filter |

Keep the robot stationary during initialization. Set `initial_z` to the actual standing height instead of blindly retaining the default `0.2 m`.

#### `zupt`

| Parameter | Description |
|---|---|
| `enabled` | Enable the zero-velocity pseudo-measurement |
| `sigma_vec` | Per-axis ZUPT velocity-noise standard deviation; smaller values impose a stronger constraint |
| `gyro_thresh` | Skip ZUPT when the bias-corrected gyroscope norm exceeds this value |

#### Logic Switches

| Parameter | Description |
|---|---|
| `use_dynamic_dt` | Compute propagation `dt` from IMU timestamps; enabled by default online |
| `use_bv_feedback` | Accept velocity-bias feedback from outer `/fusion/bv` |

Online `corgi_leg_odom` always uses its ES-EKF state for the GMO. The real-robot pipeline does not consume external `/sim/position` or `/sim/velocity` data.

### Outer Fusion

Edit:

```text
config/fusion/config_fusion.yaml
```

| Parameter | Meaning | Effect of increasing the value |
|---|---|---|
| `q_p` | Map-to-odom position process noise | Allows the position correction to drift faster |
| `q_th` | Map-to-odom orientation process noise | Allows the orientation correction to drift faster |
| `q_bv` | Velocity-bias process noise | Allows velocity bias to change faster |
| `r_p` | LiDAR position measurement variance | Reduces trust in LiDAR position |
| `r_th` | LiDAR orientation measurement variance | Reduces trust in LiDAR orientation |
| `map_frame` | Fusion global frame | Normally remains `map` |
| `odom_frame` | Inner odometry frame | Normally remains `odom` |

The `q_*` and `r_*` values are variances, not standard deviations. Real FAST-LIO and simulated fake-LiDAR measurements have different noise characteristics, so do not reuse the same `r_p` and `r_th` values without validation.

### Recommended Tuning Procedure

1. Fix the dataset, gait, speed, and random seeds.
2. Verify IMU, motor-state, trigger, and TF timing first.
3. Tune the GMO cutoff and contact thresholds.
4. Tune `sigma_leg_vec` and the Mahalanobis gate.
5. Validate the inner `/ekf` output before tuning the outer `q_*` and `r_*` values.
6. Change only one parameter group at a time, and retain the bag, configuration snapshot, and evaluation metrics.

## Build

```bash
cd ~/corgi_ws/corgi_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select corgi_odometry --symlink-install
source install/setup.bash
```

Inspect launch arguments with:

```bash
ros2 launch corgi_odometry leg_odom_real.launch.py --show-args
ros2 launch corgi_odometry leg_odom_sim.launch.py --show-args
ros2 launch corgi_odometry odom_fusion_real.launch.py --show-args
ros2 launch corgi_odometry odom_fusion_sim.launch.py --show-args
```

## Troubleshooting

### The Node Keeps Printing `Waiting for trigger`

The package does not publish a trigger automatically. Confirm that the experiment controller or motor driver publishes `/trigger`.

### The Contact Estimator Keeps Waiting for Position or Velocity

Confirm that you are using `contact_leg_estimator_sim.launch.py` and that the simulator provides TF `odom → base_link`:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /sim/position --verbose
ros2 topic info /sim/velocity --verbose
```

### The Inner ES-EKF Produces No Output

Check:

```bash
ros2 topic hz /imu_raw
ros2 topic hz /motor/state
ros2 topic echo /trigger --once
```

### Fusion Produces No Output

Fusion requires a trigger, inner `/ekf` messages, and `/lidar_odom` messages with nearby timestamps. On the real robot, also wait for FAST-LIO to finish initialization.

### Simulation Results Are Not Reproducible

Fix `imu_seed` and `lidar_seed`, and ensure that the source bag or simulation initial conditions and configuration are identical.
