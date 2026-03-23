# corgi_odometry

State estimation package for the **Corgi** quadruped robot.  
Implements leg-kinematics-based odometry using a General Momentum Observer (GMO) for contact detection combined with an Error-State Extended Kalman Filter (ES-EKF) for full-state estimation (position, velocity, attitude, IMU biases).

---

## Table of Contents

1. [Package Overview](#package-overview)
2. [File Structure](#file-structure)
3. [Dependencies](#dependencies)
4. [Building](#building)
5. [Online Usage (Hardware & Simulation)](#online-usage)
   - [Hardware — Contact Estimator Only](#hardware--contact-estimator-only)
   - [Simulation — Contact Estimator](#simulation--contact-estimator)
   - [Simulation — Full Leg Odometry (ES-EKF)](#simulation--full-leg-odometry-es-ekf)
6. [Offline Usage](#offline-usage)
   - [Running `offline_test`](#running-offline_test)
   - [CLI Arguments](#cli-arguments)
7. [Parameter Reference](#parameter-reference)
   - [config_online.yaml](#config_onlineyaml)
   - [config_test.yaml](#config_testyaml)
8. [ROS 2 Interface](#ros-2-interface)
   - [corgi_contact_leg_est](#corgi_contact_leg_est-node)
   - [corgi_leg_odom](#corgi_leg_odom-node)
   - [velocity_estimator](#velocity_estimator-node-simulation-only)
   - [imu_noise_sim](#imu_noise_sim-node-simulation-only)

---

## Package Overview

The package provides two complementary estimation pipelines:

| Pipeline | Node | Use case |
|---|---|---|
| Contact detection only | `corgi_contact_leg_est` | Lightweight online contact state | 
| Full odometry (GMO + ES-EKF) | `corgi_leg_odom` | Position, velocity & attitude estimation |
| Offline evaluation | `offline_test` | Replay CSV recordings, compute RMSE |

Helper nodes for simulation:

| Node | Purpose |
|---|---|
| `velocity_estimator` | Derive velocity from simulation ground-truth position |
| `imu_noise_sim` | Inject realistic IMU noise into clean simulation data |

---

## File Structure

```
corgi_odometry/
├── CMakeLists.txt               # Top-level build configuration
├── package.xml                  # ROS 2 package manifest
│
├── config/
│   ├── config_online.yaml       # Runtime parameters for online nodes
│   └── config_test.yaml         # Parameters for offline_test evaluation
│
├── data/
│   └── *.csv                    # Recorded robot data for offline replay
│
├── include/
│   ├── common/
│   │   ├── Config.hpp           # Compile-time constants (robot geometry, topics, DT)
│   │   ├── Params.hpp           # Runtime-tunable parameter struct
│   │   └── ParamsIO.hpp         # YAML ↔ Params loader (yaml-cpp)
│   │
│   ├── es_ekf/
│   │   └── ESEKF.hpp            # Error-State Extended Kalman Filter
│   │
│   ├── estimation/
│   │   ├── EstimationPipeline.hpp   # GMO → Contact → ES-EKF pipeline
│   │   └── SchmittTrigger.hpp       # Hysteresis contact detector
│   │
│   ├── general_momentum_observer/
│   │   ├── DataProcessor.hpp        # Encoder pre-processing & LPF
│   │   ├── DisturbanceObserver.hpp  # GMO implementation
│   │   └── simplify_dynamics.hpp    # Simplified robot dynamics
│   │
│   ├── kinematic/
│   │   ├── ContactMap.hpp       # Maps contact flags → active legs for ESEKF
│   │   ├── Leg.hpp              # Forward kinematics for one leg
│   │   └── LinkLegModel.hpp     # Link-leg geometric model
│   │
│   ├── node/
│   │   └── LegOdometryNode.hpp  # ROS 2 node class for corgi_leg_odom
│   │
│   └── offline/
│       ├── CSVReader.hpp        # CSV file parser
│       ├── ImuNoiseSimulator.hpp # Simulates 3DM-CX5-AHRS IMU noise
│       ├── OfflineTestNode.hpp  # Offline evaluation pipeline
│       └── RmseAccumulator.hpp  # RMSE bookkeeping helper
│
├── launch/
│   ├── contact_leg_estimator.launch.py        # Hardware — contact estimator only
│   ├── contact_leg_estimator_sim.launch.py    # Simulation — contact estimator
│   └── contact_leg_odom_sim.launch.py         # Simulation — full leg odometry
│
└── src/
    ├── CMakeLists.txt               # Sub-directory build targets
    │
    ├── es_ekf/
    │   └── ESEKF.cpp
    ├── estimation/
    │   └── EstimationPipeline.cpp
    ├── general_momentum_observer/
    │   ├── DataProcessor.cpp
    │   ├── DisturbanceObserver.cpp
    │   └── simplify_dynamics.cpp
    ├── kinematic/
    │   ├── Leg.cpp
    │   └── LinkLegModel.cpp
    │
    ├── node/
    │   ├── corgi_contact_leg_est.cpp  # Contact estimator node entry point
    │   └── corgi_leg_odom.cpp         # Leg odometry node entry point
    │
    ├── offline/
    │   ├── offline_test.cpp           # Offline test entry point
    │   └── OfflineTestNode.cpp        # Offline evaluation logic
    │
    └── sim/
        ├── imu_noise_sim.cpp          # IMU noise injection node
        └── velocity_estimator.cpp     # Simulation velocity estimator
```

---

## Dependencies

| Dependency | Version | Notes |
|---|---|---|
| ROS 2 | Humble or later | `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs` |
| `corgi_msgs` | same workspace | Custom message types |
| Eigen3 | ≥ 3.3 | Linear algebra |
| yaml-cpp | any | Runtime config loading |
| `tf2_ros` | ROS 2 | Used by `velocity_estimator` |
| `ament_index_cpp` | ROS 2 | Locates installed config files |

---

## Building

```bash
# From the workspace root
colcon build --packages-select corgi_odometry
source install/setup.bash
```

---

## Online Usage

All tunable parameters (observer cutoff frequency, contact thresholds, ES-EKF noise) are loaded at node startup from the installed `config_online.yaml` — **no recompilation required** after editing that file.

### Hardware — Contact Estimator Only

Runs `corgi_contact_leg_est` on the physical robot.

```bash
ros2 launch corgi_odometry contact_leg_estimator.launch.py
```

### Simulation — Contact Estimator

Runs `velocity_estimator` (to produce ground-truth velocity from simulation position) and `corgi_contact_leg_est`.

```bash
ros2 launch corgi_odometry contact_leg_estimator_sim.launch.py
```

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use `/clock` from simulator |
| `sample_rate` | `1000.0` | Estimator loop rate [Hz] |
| `position_topic` | `sim/data` | Source topic for ground-truth position |
| `velocity_topic` | `odometry/velocity` | Output velocity topic |
| `position_output_topic` | `odometry/position` | Output position topic |

### Simulation — Full Leg Odometry (ES-EKF)

Runs three nodes together:
1. `velocity_estimator` — derives velocity from simulation ground-truth
2. `imu_noise_sim` — injects realistic 3DM-CX5-AHRS noise into clean simulation IMU
3. `corgi_leg_odom` — full ES-EKF odometry (subscribes to the noisy IMU)

```bash
ros2 launch corgi_odometry contact_leg_odom_sim.launch.py
```

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use `/clock` from simulator |
| `seed` (imu_noise_sim) | `0` | Random seed for IMU noise |
| `sample_rate` (imu_noise_sim) | `1000.0` | Sampling rate [Hz] |
| `input_topic` (imu_noise_sim) | `imu_raw` (remapped to `imu`) | Clean IMU source |
| `output_topic` (imu_noise_sim) | `imu_noisy` | Noisy IMU output |

The `corgi_leg_odom` node is remapped to subscribe to `imu_noisy` so it receives the noisy signal, matching real-robot conditions.

---

## Offline Usage

`offline_test` replays a recorded CSV file through the full estimation pipeline (GMO + ES-EKF) without running ROS 2. It prints RMSE metrics and (optionally) writes per-step result logs.

### Running `offline_test`

After building, the executable is installed to `install/corgi_odometry/lib/corgi_odometry/offline_test`.

```bash
# Run with default config_test.yaml
ros2 run corgi_odometry offline_test

# Run with a custom YAML file
ros2 run corgi_odometry offline_test --config /path/to/my_config.yaml
```

The default YAML path is resolved relative to the source tree:  
`<package_source>/config/config_test.yaml`

Place CSV data files in:  
`<package_source>/data/<csv_filename>.csv`

The `csv_filename` key in `config_test.yaml` (without `.csv` extension) selects the recording to replay.

### CLI Arguments

For quick parameter sweeps, positional CLI arguments override values from the YAML file:

```
offline_test [sigma_a_x] [sigma_leg_x] [sigma_leg_z] [mahalanobis_threshold] [simulate_imu_noise] [quiet] [use_esekf_state]
```

| Position | Parameter | Type | Example |
|---|---|---|---|
| 1 | `sigma_a.x` (accel noise X) | float | `5.0` |
| 2 | `sigma_leg_vec.x` (leg noise X) | float | `0.05` |
| 3 | `sigma_leg_vec.z` (leg noise Z) | float | `1.2` |
| 4 | `mahalanobis_threshold` | float | `16.27` |
| 5 | `simulate_imu_noise` | `0`/`1` | `1` |
| 6 | `quiet` | `0`/`1` | `0` |
| 7 | `use_esekf_state` | `0`/`1` | `1` |

**Example — custom noise + quiet mode:**

```bash
ros2 run corgi_odometry offline_test 3.0 0.1 1.0 16.27 0 1 0
```

---

## Parameter Reference

### config_online.yaml

Used by both `corgi_contact_leg_est` and `corgi_leg_odom` at startup.  
Path: `share/corgi_odometry/config/config_online.yaml`

```yaml
observer:
  cutoff_freq:         15.0    # Disturbance observer LPF cutoff [Hz]
  encoder_cutoff_freq: 30.0    # Encoder velocity LPF cutoff [Hz]

contact:
  rm_threshold_high:   25.0    # Schmitt-trigger high threshold for Rm
  rm_threshold_low:    15.0    # Schmitt-trigger low  threshold for Rm
  beta_threshold_high: 10.0    # Schmitt-trigger high threshold for β
  beta_threshold_low:   1.0    # Schmitt-trigger low  threshold for β
```

| Key | Default | Description |
|---|---|---|
| `observer.cutoff_freq` | `15.0` | Low-pass filter cutoff for the disturbance observer [Hz]. Lower values remove more noise but increase lag. |
| `observer.encoder_cutoff_freq` | `30.0` | Low-pass filter cutoff applied to encoder velocity signals [Hz]. |
| `contact.rm_threshold_high` | `25.0` | Schmitt-trigger **upper** threshold on the Rm signal for contact detection. Leg enters contact when Rm exceeds this value. |
| `contact.rm_threshold_low` | `15.0` | Schmitt-trigger **lower** threshold on Rm. Leg leaves contact when Rm drops below this value. |
| `contact.beta_threshold_high` | `10.0` | Schmitt-trigger **upper** threshold on the β (beta) signal. |
| `contact.beta_threshold_low` | `1.0` | Schmitt-trigger **lower** threshold on β. |

### config_test.yaml

Used exclusively by `offline_test`.  
Path: `share/corgi_odometry/config/config_test.yaml`

```yaml
esekf:
  sigma_a:       [5.0, 1.0, 1.0]
  sigma_w:       [0.001, 0.01, 0.001]
  sigma_ba:      [1.0e-5, 1.0e-5, 1.0e-5]
  sigma_bw:      [1.0e-8, 1.0e-8, 1.0e-8]
  sigma_bv:      [1.0e-6, 1.0e-6, 1.0e-6]
  sigma_leg_vec: [0.05, 1.5, 1.2]
  mahalanobis_threshold: 16.27

observer:
  cutoff_freq:         15.0
  encoder_cutoff_freq: 30.0

contact:
  rm_threshold_high:   25.0
  rm_threshold_low:    15.0
  beta_threshold_high: 10.0
  beta_threshold_low:   1.0

simulate_imu_noise: false
use_esekf_state:    false
quiet:              false

offline:
  csv_filename:   walk_2m_01
  start_index:    0
  max_processed:  12000
  rmse_skip:      0
  enable_logging: true
  log_details:    false
  imu_noise_seed: 42

gt_velocity_lpf_cutoff: 10.0
```

#### ES-EKF noise parameters

| Key | Default | Description |
|---|---|---|
| `esekf.sigma_a` | `[5.0, 1.0, 1.0]` | Accelerometer noise standard deviation [x, y, z] in m/s². |
| `esekf.sigma_w` | `[0.001, 0.01, 0.001]` | Gyroscope noise standard deviation [x, y, z] in rad/s. |
| `esekf.sigma_ba` | `[1e-5, 1e-5, 1e-5]` | Accelerometer bias random-walk std [m/s²]. |
| `esekf.sigma_bw` | `[1e-8, 1e-8, 1e-8]` | Gyroscope bias random-walk std [rad/s]. |
| `esekf.sigma_bv` | `[1e-6, 1e-6, 1e-6]` | Velocity bias random-walk std [m/s]. |
| `esekf.sigma_leg_vec` | `[0.05, 1.5, 1.2]` | Leg velocity measurement noise std [x, y, z] in m/s. Y is set loose due to lateral FK error. |
| `esekf.mahalanobis_threshold` | `16.27` | χ²(3) outlier-rejection threshold (99.9 % confidence). Increase to accept more leg updates; decrease to be stricter. |

#### Logic switches

| Key | Default | Description |
|---|---|---|
| `simulate_imu_noise` | `false` | Inject simulated 3DM-CX5-AHRS noise onto the IMU signal before processing. |
| `use_esekf_state` | `false` | Feed ES-EKF estimated position/velocity back into the GMO instead of ground-truth values. |
| `quiet` | `false` | Suppress per-step console output. |

#### Offline processing

| Key | Default | Description |
|---|---|---|
| `offline.csv_filename` | `walk_2m_01` | Name of the CSV data file (without `.csv`) located in the `data/` directory. |
| `offline.start_index` | `0` | First row index to process (skip warm-up data). |
| `offline.max_processed` | `12000` | Maximum number of rows to process. |
| `offline.rmse_skip` | `0` | Number of initial steps to exclude from RMSE computation. |
| `offline.enable_logging` | `true` | Write per-step result log files. |
| `offline.log_details` | `false` | Write verbose per-leg diagnostic logs. |
| `offline.imu_noise_seed` | `42` | Random seed used when `simulate_imu_noise` is enabled. |
| `gt_velocity_lpf_cutoff` | `10.0` | Cutoff frequency [Hz] of the IIR low-pass filter applied to ground-truth velocity (derived from simulation positions) before RMSE computation. |

---

## ROS 2 Interface

### `corgi_contact_leg_est` Node

**Subscribed Topics**

| Topic | Message Type | Description |
|---|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` | Joint positions, velocities, and torques for all legs |
| `imu` | `corgi_msgs/ImuStamped` | IMU orientation, angular velocity, linear acceleration |
| `odometry/position` | `geometry_msgs/Vector3` | Current robot base position [m] (world frame) |
| `odometry/velocity` | `geometry_msgs/Vector3` | Current robot base velocity [m/s] (world frame) |
| `trigger` | `corgi_msgs/TriggerStamped` | External trigger signal |

**Published Topics**

| Topic | Message Type | Description |
|---|---|---|
| `contact_state` | `corgi_msgs/ContactStateStamped` | Per-leg contact flags (4 legs: LF, RF, RH, LH) |

---

### `corgi_leg_odom` Node

Runs the full pipeline (GMO → contact detection → ES-EKF) and additionally publishes odometry.

**Subscribed Topics**

| Topic | Message Type | Description |
|---|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` | Joint positions, velocities, and torques for all legs |
| `imu` | `corgi_msgs/ImuStamped` | IMU orientation, angular velocity, linear acceleration |
| `odometry/position` | `geometry_msgs/Vector3` | Base position (used by GMO) |
| `odometry/velocity` | `geometry_msgs/Vector3` | Base velocity (used by GMO) |
| `trigger` | `corgi_msgs/TriggerStamped` | External trigger signal |

**Published Topics**

| Topic | Message Type | Description |
|---|---|---|
| `contact_state` | `corgi_msgs/ContactStateStamped` | Per-leg contact flags |
| `odometry` | `nav_msgs/Odometry` | Full 6-DOF odometry from ES-EKF (position + orientation + velocity) |

---

### `velocity_estimator` Node (Simulation only)

Differentiates simulation position data to produce velocity and re-publishes position.

**ROS 2 Parameters**

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `sample_rate` | `1000.0` | Filter update rate [Hz] |
| `position_topic` | `sim/data` | Input topic carrying ground-truth position |
| `velocity_topic` | `odometry/velocity` | Output velocity topic name |
| `position_output_topic` | `odometry/position` | Output position topic name |

---

### `imu_noise_sim` Node (Simulation only)

Subscribes to a clean simulation IMU topic and republishes with realistic sensor noise added.

**ROS 2 Parameters**

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `seed` | `0` | Random seed for repeatable noise |
| `sample_rate` | `1000.0` | Sampling rate [Hz] |
| `input_topic` | `imu_raw` | Clean IMU input topic |
| `output_topic` | `imu_noisy` | Noisy IMU output topic |
