# corgi_odometry

A ROS 2 package that provides leg-contact detection and error-state extended Kalman filter (ES-EKF) odometry for the **Corgi** quadruped robot.
The estimator fuses IMU data with per-leg kinematics (4-bar linkage) and a disturbance-observer-based contact detector to produce accurate pose and velocity estimates at **1 000 Hz**.

---

## Table of Contents

1. [Overview](#overview)
2. [File Structure](#file-structure)
3. [Dependencies](#dependencies)
4. [Building the Package](#building-the-package)
5. [Online Usage](#online-usage)
   - [Hardware Mode — Contact Detection Only](#hardware-mode--contact-detection-only)
   - [Simulation Mode — Contact + Velocity Estimation](#simulation-mode--contact--velocity-estimation)
   - [Simulation Mode — Full Odometry with IMU Noise](#simulation-mode--full-odometry-with-imu-noise)
6. [Offline Usage](#offline-usage)
7. [Parameter Tuning](#parameter-tuning)
   - [config_online.yaml — Online / Hardware Parameters](#config_onlineyaml--online--hardware-parameters)
   - [config_test.yaml — Offline Test Parameters](#config_testyaml--offline-test-parameters)
8. [ROS 2 Interfaces](#ros-2-interfaces)
   - [Subscribed Topics](#subscribed-topics)
   - [Published Topics](#published-topics)
9. [Algorithm Overview](#algorithm-overview)

---

## Overview

`corgi_odometry` implements three complementary estimators that can be used independently or together:

| Estimator | Node / Executable | Purpose |
|---|---|---|
| **Contact Detector** | `corgi_contact_leg_est` | Generalised momentum observer (GMO) + Schmitt-trigger hysteresis to detect which legs are in contact with the ground |
| **Leg Odometry** | `corgi_leg_odom` | Full ES-EKF: propagates via IMU, updates via per-leg no-slip constraint, outputs `nav_msgs/Odometry` |
| **Offline Tester** | `offline_test` | Standalone (no ROS 2) tool that replays a CSV data file, evaluates RMSE against ground truth, and writes diagnostic CSVs |

Two simulation helpers are also provided:

| Helper | Node / Executable | Purpose |
|---|---|---|
| **Velocity Estimator** | `velocity_estimator` | Derives body velocity from the `odom → base_link` TF transform and publishes it as `odometry/velocity` |
| **IMU Noise Simulator** | `imu_noise_sim` | Injects realistic 3DM-CX5-AHRS noise into a clean simulator IMU topic |

---

## File Structure

```
corgi_odometry/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   ├── config_online.yaml      # Runtime-tunable parameters for online nodes
│   └── config_test.yaml        # Comprehensive parameters for offline testing
│
├── data/
│   └── cpp_test.csv            # Sample offline test dataset (~12 000 frames @ 1 kHz)
│
├── include/
│   ├── common/
│   │   ├── Config.hpp          # Compile-time constants (DOF, topics, robot geometry)
│   │   ├── Params.hpp          # Runtime parameter struct (loaded from YAML)
│   │   └── ParamsIO.hpp        # YAML loader utility
│   ├── es_ekf/
│   │   └── ESEKF.hpp           # 18-dim error-state EKF
│   ├── estimation/
│   │   ├── EstimationPipeline.hpp  # GMO → contact detection → ES-EKF pipeline
│   │   └── SchmittTrigger.hpp      # Hysteresis-based contact detector
│   ├── general_momentum_observer/
│   │   ├── DisturbanceObserver.hpp # Discrete-time disturbance observer
│   │   ├── DataProcessor.hpp       # ROS message → generalised coordinates
│   │   └── simplify_dynamics.hpp   # 12-DOF robot dynamics (M, C, G matrices)
│   ├── kinematic/
│   │   ├── Leg.hpp             # 4-bar linkage leg model
│   │   ├── LinkLegModel.hpp    # 4-bar geometry kinematics
│   │   └── ContactMap.hpp      # (θ, β) → contact rim lookup table
│   ├── node/
│   │   └── LegOdometryNode.hpp # Main ROS 2 node definition
│   └── offline/
│       ├── CSVReader.hpp       # 42-column offline CSV parser
│       ├── ImuNoiseSimulator.hpp  # Seedable 3DM-CX5-AHRS noise model
│       ├── OfflineTestNode.hpp    # Offline pipeline coordinator
│       └── RmseAccumulator.hpp   # RMSE computation vs ground truth
│
├── launch/
│   ├── contact_leg_estimator.launch.py          # Online hardware mode
│   ├── contact_leg_estimator_sim.launch.py      # Simulation + velocity estimator
│   └── contact_leg_odom_sim.launch.py           # Full simulation with noise + ES-EKF
│
└── src/
    ├── CMakeLists.txt
    ├── es_ekf/
    │   └── ESEKF.cpp
    ├── estimation/
    │   └── EstimationPipeline.cpp
    ├── general_momentum_observer/
    │   ├── DisturbanceObserver.cpp
    │   ├── DataProcessor.cpp
    │   └── simplify_dynamics.cpp
    ├── kinematic/
    │   ├── Leg.cpp
    │   └── LinkLegModel.cpp
    ├── node/
    │   ├── corgi_leg_odom.cpp          # corgi_leg_odom executable
    │   └── corgi_contact_leg_est.cpp   # corgi_contact_leg_est executable
    ├── offline/
    │   ├── offline_test.cpp            # offline_test executable entry point
    │   └── OfflineTestNode.cpp
    └── sim/
        ├── velocity_estimator.cpp      # velocity_estimator executable
        └── imu_noise_sim.cpp           # imu_noise_sim executable
```

---

## Dependencies

| Dependency | Notes |
|---|---|
| ROS 2 (Humble or later) | `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros` |
| `corgi_msgs` | Custom messages: `ImuStamped`, `MotorStateStamped`, `ContactStateStamped`, `TriggerStamped` |
| Eigen 3 | Linear algebra |
| yaml-cpp | Runtime config loading |
| ament_index_cpp | Package share directory lookup |

---

## Building the Package

```bash
# From the workspace root
colcon build --packages-select corgi_odometry
source install/setup.bash
```

> All parameters in `config_online.yaml` and `config_test.yaml` are loaded at runtime by the nodes themselves via yaml-cpp.
> You do **not** need to recompile after editing those files.

---

## Online Usage

"Online" means the estimator runs as a live ROS 2 node receiving data from real hardware or a simulator.

### Hardware Mode — Contact Detection Only

```bash
ros2 launch corgi_odometry contact_leg_estimator.launch.py
```

**What it starts:**

| Node | Executable | `use_sim_time` |
|---|---|---|
| `corgi_contact_leg_est` | `corgi_contact_leg_est` | `false` |

**Required topics from external sources:**

| Topic | Type |
|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` |
| `imu` | `corgi_msgs/ImuStamped` |
| `odometry/position` | `geometry_msgs/Vector3Stamped` |
| `odometry/velocity` | `geometry_msgs/Vector3Stamped` |
| `trigger` | `corgi_msgs/TriggerStamped` |

**Output:** `contact_state` (`corgi_msgs/ContactStateStamped`) at 1 000 Hz.

---

### Simulation Mode — Contact + Velocity Estimation

```bash
ros2 launch corgi_odometry contact_leg_estimator_sim.launch.py
```

**What it starts:**

| Node | Executable | `use_sim_time` |
|---|---|---|
| `velocity_estimator` | `velocity_estimator` | `true` |
| `corgi_contact_leg_est` | `corgi_contact_leg_est` | `true` |

The `velocity_estimator` node reads the simulator's `odom → base_link` TF at 1 000 Hz,
differentiates position numerically, applies a 30 Hz low-pass filter, and publishes
`odometry/velocity` and `odometry/position`.

**Required topics from simulator:**

| Topic | Type | Note |
|---|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` | |
| `imu` | `corgi_msgs/ImuStamped` | |
| `sim/data` | `geometry_msgs/Vector3Stamped` | Simulator position (read by velocity estimator) |
| `trigger` | `corgi_msgs/TriggerStamped` | |

**Output:** `contact_state` at 1 000 Hz.

---

### Simulation Mode — Full Odometry with IMU Noise

```bash
ros2 launch corgi_odometry contact_leg_odom_sim.launch.py
```

**What it starts:**

| Node | Executable | `use_sim_time` | Notable Parameters |
|---|---|---|---|
| `velocity_estimator` | `velocity_estimator` | `true` | `sample_rate=1000`, `position_topic=sim/data` |
| `imu_noise_sim` | `imu_noise_sim` | `true` | `seed=0`, `sample_rate=1000` |
| `corgi_leg_odom` | `corgi_leg_odom` | `true` | subscribes to `imu_noisy` (remapped from `imu`) |

**Topic wiring:**

```
simulator  →  "imu"  ──(remap imu_raw→imu)──▶  imu_noise_sim  ──▶  "imu_noisy"
                                                                         │
                                                                         ▼
                                                               corgi_leg_odom (remapped imu ← imu_noisy)
```

**Required topics from simulator:**

| Topic | Type |
|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` |
| `imu` | `corgi_msgs/ImuStamped` |
| `sim/data` | `geometry_msgs/Vector3Stamped` |
| `trigger` | `corgi_msgs/TriggerStamped` |

**Outputs:**

| Topic | Type |
|---|---|
| `contact_state` | `corgi_msgs/ContactStateStamped` |
| `ekf` | `nav_msgs/Odometry` |

---

## Offline Usage

The `offline_test` executable replays a CSV data file through the full estimation pipeline without starting any ROS 2 nodes.
It is useful for parameter sweeps, algorithm validation, and RMSE evaluation against ground truth.

### Running with Default Settings

```bash
# Uses config/config_test.yaml and data/cpp_test.csv
./install/corgi_odometry/lib/corgi_odometry/offline_test
```

### Specifying a Custom Config File

```bash
./install/corgi_odometry/lib/corgi_odometry/offline_test --config /path/to/my_config.yaml
```

### Positional CLI Overrides

Positional arguments override the corresponding YAML values without modifying the file:

```bash
OFFLINE_TEST=./install/corgi_odometry/lib/corgi_odometry/offline_test

# Positional arguments
$OFFLINE_TEST [sigma_a_x] [sigma_leg_x] [sigma_leg_z] [mahalanobis_threshold] [simulate_imu_noise] [quiet] [use_esekf_state]
```

| Position | Parameter | Example | Type |
|---|---|---|---|
| 1 | `sigma_a` (x component) | `5.0` | float |
| 2 | `sigma_leg_vec` (x component) | `0.05` | float |
| 3 | `sigma_leg_vec` (z component) | `1.2` | float |
| 4 | `mahalanobis_threshold` | `16.27` | float |
| 5 | `simulate_imu_noise` | `0` or `1` | bool |
| 6 | `quiet` | `0` or `1` | bool |
| 7 | `use_esekf_state` | `0` or `1` | bool |

**Example — tune noise parameters and enable IMU noise simulation:**

```bash
$OFFLINE_TEST 5.0 0.05 1.2 16.27 1 0 0
```

**Example — combine YAML config with selective overrides:**

```bash
$OFFLINE_TEST --config my_config.yaml 3.0 0.03 1.0
```

### Output

- RMSE metrics (position, velocity, orientation) printed to stdout.
- Per-frame diagnostic CSV files written to the working directory when `offline.enable_logging: true`.

---

## Parameter Tuning

### `config_online.yaml` — Online / Hardware Parameters

Located at `config/config_online.yaml`.
Loaded at node startup; **no recompile required** after editing.

```yaml
# Disturbance observer / encoder filter
observer:
  cutoff_freq:         15.0    # Disturbance observer low-pass filter [Hz]
  encoder_cutoff_freq: 30.0    # Encoder velocity low-pass filter [Hz]

# Schmitt-trigger contact detection thresholds
contact:
  rm_threshold_high:   25.0   # Activate contact when |Rm| exceeds this value
  rm_threshold_low:    15.0   # Deactivate contact when |Rm| falls below this value
  beta_threshold_high: 10.0   # Activate contact when |β| exceeds this value
  beta_threshold_low:   1.0   # Deactivate contact when |β| falls below this value
```

**Tuning guidelines:**

| Parameter | Effect of Increasing | Effect of Decreasing |
|---|---|---|
| `cutoff_freq` | Faster disturbance response, more noise | Smoother signal, slower response |
| `encoder_cutoff_freq` | Less encoder lag, more noise | Smoother encoder velocity |
| `rm_threshold_high` | Fewer false contact detections | More sensitive contact detection |
| `rm_threshold_low` | Contact stays active longer | Contact drops out more quickly |
| `beta_threshold_high` | Fewer false contacts from β | More sensitive β-based detection |
| `beta_threshold_low` | Contact from β stays active longer | Contact from β drops sooner |

---

### `config_test.yaml` — Offline Test Parameters

Located at `config/config_test.yaml`.
Contains all parameters from `config_online.yaml` plus ES-EKF noise tuning and offline-specific settings.

```yaml
# ES-EKF noise standard deviations (diagonal of process/measurement noise matrices)
esekf:
  sigma_a:       [5.0, 1.0, 1.0]         # Accelerometer noise [m/s²], per axis (x, y, z)
  sigma_w:       [0.001, 0.01, 0.001]     # Gyroscope noise [rad/s], per axis
  sigma_ba:      [1.0e-5, 1.0e-5, 1.0e-5] # Accel bias random-walk std
  sigma_bw:      [1.0e-8, 1.0e-8, 1.0e-8] # Gyro bias random-walk std
  sigma_bv:      [1.0e-6, 1.0e-6, 1.0e-6] # Velocity bias random-walk std
  sigma_leg_vec: [0.05, 1.5, 1.2]         # Leg no-slip measurement noise [m/s], per axis
  mahalanobis_threshold: 16.27             # χ²(3) outlier rejection threshold (99.9 %)

# Disturbance observer (same as config_online.yaml)
observer:
  cutoff_freq:        15.0
  encoder_cutoff_freq: 30.0

# Contact thresholds (same as config_online.yaml)
contact:
  rm_threshold_high:   25.0
  rm_threshold_low:    15.0
  beta_threshold_high: 10.0
  beta_threshold_low:   1.0

# Logic switches
simulate_imu_noise: false   # Inject 3DM-CX5-AHRS noise model into CSV IMU data
use_esekf_state:    false   # Use ES-EKF state for evaluation (vs ground truth)
quiet:              false   # Suppress per-frame log output

# Offline replay settings
offline:
  csv_filename:   walk_2m_01  # Stem of the CSV file to load (without .csv extension).
                              # The bundled sample file is data/cpp_test.csv — change this
                              # to "cpp_test" to use it, or point to your own dataset.
  start_index:    0           # First frame to process
  max_processed:  12000       # Maximum number of frames to process
  rmse_skip:      0           # Skip this many frames before accumulating RMSE
  enable_logging: true        # Write per-frame diagnostic CSV files
  log_details:    false       # Include detailed per-axis diagnostics
  imu_noise_seed: 42          # RNG seed for reproducible IMU noise

# Ground-truth velocity low-pass filter (used when computing RMSE)
gt_velocity_lpf_cutoff: 10.0  # [Hz]
```

**ES-EKF noise tuning guidelines:**

| Parameter | Effect of Increasing | Effect of Decreasing |
|---|---|---|
| `sigma_a` | Trust IMU accel less → larger predict uncertainty | Trust IMU accel more |
| `sigma_w` | Trust IMU gyro less | Trust IMU gyro more |
| `sigma_ba` / `sigma_bw` | Accel / gyro bias drifts faster | Bias tracked more tightly |
| `sigma_leg_vec` | Trust leg measurements less → smoother but slower correction | Leg measurements dominate |
| `mahalanobis_threshold` | Accept more leg measurement updates | Reject more outliers |

> **Note:** `sigma_leg_vec` is **anisotropic**: the x-component (forward) is typically much tighter than y (lateral) and z (vertical) to reflect the physical constraints of leg-ground contact.

---

## ROS 2 Interfaces

### Subscribed Topics

| Topic | Type | Node | Description |
|---|---|---|---|
| `motor/state` | `corgi_msgs/MotorStateStamped` | `leg_odom`, `contact_leg_est` | Per-leg joint angles (θ, β) and velocities |
| `imu` | `corgi_msgs/ImuStamped` | `leg_odom`, `contact_leg_est` | Accelerometer, gyroscope, orientation quaternion |
| `odometry/position` | `geometry_msgs/Vector3Stamped` | `leg_odom`, `contact_leg_est` | Base position [x, y, z] from external source |
| `odometry/velocity` | `geometry_msgs/Vector3Stamped` | `leg_odom`, `contact_leg_est` | Base velocity [vx, vy, vz] from external source |
| `trigger` | `corgi_msgs/TriggerStamped` | `leg_odom`, `contact_leg_est` | ES-EKF initialisation trigger |
| `imu_raw` | `corgi_msgs/ImuStamped` | `imu_noise_sim` | Clean IMU input for noise injection |
| `sim/data` | `geometry_msgs/Vector3Stamped` | `velocity_estimator` | Simulator position (for velocity differentiation) |

### Published Topics

| Topic | Type | Node | Description |
|---|---|---|---|
| `contact_state` | `corgi_msgs/ContactStateStamped` | `leg_odom`, `contact_leg_est` | Contact boolean per leg [LF, RF, RH, LH] @ 1 000 Hz |
| `ekf` | `nav_msgs/Odometry` | `leg_odom` | Full pose + velocity + covariance from ES-EKF |
| `imu_noisy` | `corgi_msgs/ImuStamped` | `imu_noise_sim` | IMU with 3DM-CX5-AHRS noise applied |
| `odometry/velocity` | `geometry_msgs/Vector3Stamped` | `velocity_estimator` | Differentiated + low-pass filtered velocity |
| `odometry/position` | `geometry_msgs/Vector3Stamped` | `velocity_estimator` | Position re-published from TF |
| `debug/leg_obs` | `std_msgs/Float64MultiArray` | `leg_odom` | Per-leg observations (θ, β, θ̇, β̇, rim, α) |
| `debug/z_leg` | `std_msgs/Float64MultiArray` | `leg_odom` | Per-leg measurement innovation |
| `debug/imu` | `std_msgs/Float64MultiArray` | `leg_odom` | Raw IMU data stream |

> **Topic remapping:** Topic names can be remapped at launch time using standard ROS 2 remapping syntax or by editing the launch files.

---

## Algorithm Overview

The GMO (Disturbance Observer) requires world-frame position and velocity as inputs.
In the intended steady-state design these are supplied by the **ES-EKF output** (closed feedback loop).
External `odometry/position` and `odometry/velocity` topics are currently used as a **temporary substitute** while the ES-EKF output is being validated; once validated they will be removed and the node will become fully self-contained.

```
Subscribed Topics (1 000 Hz)
  motor/state, imu, trigger
  odometry/position  ◄──(*)  world-frame p from ES-EKF
  odometry/velocity  ◄──(*)  world-frame v from ES-EKF   <── feedback (see below)
                     │
                     ▼
┌─────────────────────────────────────────┐
│ DataProcessor                           │
│ msgs → (q, q̇, τ, Ic)                   │
│ low-pass filter on encoder velocity     │
└────────────────────┬────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────┐
│ DisturbanceObserver (GMO)               │
│ τ̂_d = β·p_k − LPF(β·p + Sᵀτ           │
│                + Cᵀq̇ − g)              │
│ estimates per-leg contact force         │
└────────────────────┬────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────┐
│ SchmittTrigger × 4 legs                 │
│ activate:   |Rm| > rm_high  OR          │
│             |β|  > beta_high            │
│ deactivate: |Rm| < rm_low   AND         │
│             |β|  < beta_low             │
└────────────────────┬────────────────────┘
                     │
                     ▼  (corgi_leg_odom only)
┌─────────────────────────────────────────┐
│ ES-EKF                                  │
│  predict: IMU propagation               │
│           (bias-corrected)              │
│  update:  per-leg no-slip constraint    │
│  reject:  Mahalanobis outliers          │
└────────────────────┬────────────────────┘
          ┌──────────┘
          │  (*) world-frame p, v fed back into DataProcessor / GMO
          │      (replaces external odometry topics once validated)
          ▼
Published Topics
  contact_state  (all nodes)
  ekf            (corgi_leg_odom)
```

The **ES-EKF** maintains a 19-dimensional nominal state (position, velocity, attitude quaternion, accel bias, gyro bias, velocity bias) and an 18-dimensional error state (substituting a rotation vector for the quaternion error).
At each update step the per-leg no-slip constraint provides a velocity observation:

```
z_leg = v_body + b_v + noise
```

Observations whose Mahalanobis distance exceeds `mahalanobis_threshold` are discarded to guard against slip events.

The **4-bar linkage** leg model maps joint angles (θ, β) to a contact-rim lookup table (upper rim left/right, lower rim left/right, G-point, no contact) and computes the contact-point velocity used in the EKF update.
