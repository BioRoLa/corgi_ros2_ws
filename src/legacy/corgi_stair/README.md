# corgi_stair

**Stair climbing algorithms with point cloud processing for the Corgi quadruped robot.**

This package provides advanced stair detection and climbing capabilities using 3D point cloud data from a ZED camera, combined with adaptive gait control for both indoor and outdoor environments.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Nodes](#nodes)
- [Topics](#topics)
- [Building](#building)
- [Usage](#usage)
- [Configuration](#configuration)
- [Algorithm Details](#algorithm-details)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `corgi_stair` package enables the Corgi robot to autonomously detect and climb stairs by:

1. Processing 3D point cloud data to segment horizontal and vertical planes
2. Estimating stair geometry (depth, height, orientation)
3. Planning optimal foothold positions for each leg
4. Generating adaptive gaits that transition from walking to stair climbing
5. Providing open-loop and closed-loop control variants for different environments

The package is designed for ROS2 Humble and integrates with:

- **ZED Camera** for depth sensing and point cloud generation
- **corgi_walk** library for gait generation and leg kinematics
- **corgi_msgs** for motor commands and sensor data
- **TF2** for coordinate frame transformations

---

## Features

### Stair Detection

- **Real-time plane segmentation** from organized point clouds
- **RANSAC-based** horizontal and vertical plane detection
- **Plane tracking** for robust stair edge identification
- **Normal vector estimation** for stair orientation

### Adaptive Control

- **Indoor variants** optimized for controlled environments
- **Outdoor variants** with enhanced robustness for uneven surfaces
- **Open-loop modes** for demonstration and testing without feedback
- **Closed-loop modes** using camera feedback for precise positioning

### Gait Planning

- **Smooth transitions** from walking to stair climbing
- **Optimal foothold calculation** based on stair geometry
- **Variable step length adjustment** to align with stair edges
- **Pitch compensation** to maintain body orientation on slopes

### Utility Tools

- **Motor command recording** (`save_motor_cmd`) for trajectory analysis
- **Motor command playback** (`play_motor_cmd`) for replaying demonstrations
- **Plane visualization** for debugging stair detection

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      ZED Camera                             │
│              (Point Cloud + TF Publisher)                   │
└────────────────────────┬────────────────────────────────────┘
                         │ PointCloud2
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                   stair_estimator                           │
│  • Plane segmentation (horizontal/vertical)                 │
│  • Plane tracking and averaging                             │
│  • Stair geometry estimation                                │
└────────────────────────┬────────────────────────────────────┘
                         │ StairPlanes
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              stair_main_* (Control Node)                    │
│  • State machine: INIT → WALK → STAIR → END                │
│  • Gait generation (walk_gait, stair_climb)                 │
│  • Foothold planning                                        │
│  • Pitch/CoM compensation                                   │
└────────────────────────┬────────────────────────────────────┘
                         │ MotorCmdStamped
                         ↓
┌─────────────────────────────────────────────────────────────┐
│               Motor Control System                          │
│           (corgi_ros_bridge → FPGA → Motors)                │
└─────────────────────────────────────────────────────────────┘
```

---

## Nodes

### Main Control Nodes

#### `stair_main_indoor`

Primary stair climbing controller for indoor environments.

**Features:**

- Closed-loop control with camera feedback
- Precise stair edge detection
- Step length adaptation for alignment
- Suitable for regular, well-defined stairs

**Topics:**

- Publishes: `/motor/command` (MotorCmdStamped)
- Subscribes: `/trigger` (TriggerStamped), `/stair_planes` (StairPlanes)

#### `stair_main_outdoor`

Enhanced controller for outdoor stair climbing.

**Features:**

- Robust to irregular stair geometry
- Additional filtering for noisy point clouds
- Adaptive thresholds for plane detection
- Better handling of lighting variations

#### `stair_main_outdoor2` / `stair_main_outdoor3`

Alternative outdoor variants with different tuning parameters for various stair types and conditions.

#### `stair_main_indoor_open` / `stair_main_outdoor_open`

Open-loop variants that execute pre-planned trajectories without real-time feedback.

**Use cases:**

- Testing and demonstrations
- Environments without camera access
- Debugging gait generation
- Performance benchmarking

### Perception Node

#### `stair_estimator`

Point cloud processing node for stair detection.

**Algorithm:**

1. Receives organized point cloud from ZED camera
2. Applies ROI filtering (x: 0.2-1.5m, y: ±0.4m, z: -0.5-1.0m)
3. Performs integral image normal estimation
4. Segments horizontal and vertical planes using RANSAC
5. Tracks planes across frames with clustering
6. Publishes averaged plane distances and normals

**Topics:**

- Publishes: `/stair_planes` (StairPlanes)
- Subscribes: `/zedxm/zed_node/point_cloud/cloud_registered` (PointCloud2), `/trigger` (TriggerStamped)

**Output CSV files:**

- `plane_distances.csv` - Raw plane detections per frame
- `stair_planes.csv` - Averaged plane data with camera pose

### Utility Nodes

#### `save_motor_cmd`

Records motor commands to CSV file.

**Topics:**

- Subscribes: `/motor/command` (MotorCmdStamped), `/trigger` (TriggerStamped)

**Output:**

- `motor_commands.csv` - Time-stamped motor commands (theta, beta, PID gains, torques)

#### `play_motor_cmd`

Replays recorded motor commands from CSV file.

**Topics:**

- Publishes: `/motor/command` (MotorCmdStamped)
- Subscribes: `/trigger` (TriggerStamped)

**Input:**

- `motor_commands.csv` - Previously recorded trajectory

---

## Topics

### Published

| Topic            | Message Type                     | Description                                       |
| ---------------- | -------------------------------- | ------------------------------------------------- |
| `/motor/command` | `corgi_msgs/msg/MotorCmdStamped` | Motor position commands in theta-beta coordinates |
| `/stair_planes`  | `corgi_msgs/msg/StairPlanes`     | Detected stair plane distances and normals        |

### Subscribed

| Topic                                          | Message Type                    | Description                                  |
| ---------------------------------------------- | ------------------------------- | -------------------------------------------- |
| `/trigger`                                     | `corgi_msgs/msg/TriggerStamped` | Enable/disable signal for control activation |
| `/stair_planes`                                | `corgi_msgs/msg/StairPlanes`    | Stair geometry from perception node          |
| `/zedxm/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2`   | Registered point cloud from ZED camera       |

### TF Frames

| Frame                 | Description               |
| --------------------- | ------------------------- |
| `map`                 | World reference frame     |
| `zedxm_camera_center` | ZED camera optical center |
| `base_link`           | Robot body center of mass |

---

## Building

### Prerequisites

**ROS2 Dependencies:**

```bash
sudo apt install ros-humble-rclcpp \
                 ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs \
                 ros-humble-visualization-msgs \
                 ros-humble-tf2-ros \
                 ros-humble-tf2-eigen \
                 ros-humble-pcl-ros \
                 ros-humble-pcl-conversions
```

**System Dependencies:**

```bash
# Eigen3 (version 3.4.0 required)
cd ~/corgi_ws/install
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar xf eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
make install

# PCL (Point Cloud Library) - if not installed
sudo apt install libpcl-dev
```

**Workspace Dependencies:**

- `corgi_msgs` - Custom message definitions
- `corgi_walk` - Gait generation library
- `corgi_utils` - Shared utilities

### Build Commands

```bash
cd ~/corgi_ws/corgi_ros_ws

# Build with local dependencies
colcon build --packages-select corgi_stair \
  --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install

# Source the workspace
source install/setup.bash
```

### Verify Installation

```bash
# Check executables are available
ros2 pkg executables corgi_stair

# Expected output:
# corgi_stair play_motor_cmd
# corgi_stair save_motor_cmd
# corgi_stair stair_estimator
# corgi_stair stair_main_indoor
# corgi_stair stair_main_indoor_open
# corgi_stair stair_main_outdoor
# corgi_stair stair_main_outdoor2
# corgi_stair stair_main_outdoor3
# corgi_stair stair_main_outdoor_open
```

---

## Usage

### Basic Stair Climbing (Indoor)

**Terminal 1 - Stair Detection:**

```bash
ros2 run corgi_stair stair_estimator
```

**Terminal 2 - Control:**

```bash
ros2 run corgi_stair stair_main_indoor
```

**Terminal 3 - Enable (when ready):**

```bash
ros2 topic pub /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

### Recording a Trajectory

**Terminal 1 - Record:**

```bash
ros2 run corgi_stair save_motor_cmd
```

**Terminal 2 - Execute motion:**

```bash
ros2 run corgi_stair stair_main_indoor
```

**Terminal 3 - Trigger:**

```bash
ros2 topic pub /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

Output saved to: `motor_commands.csv`

### Playing Back a Trajectory

```bash
# Place motor_commands.csv in current directory
ros2 run corgi_stair play_motor_cmd

# Enable playback
ros2 topic pub /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

### Outdoor Mode

For outdoor environments with irregular stairs:

```bash
ros2 run corgi_stair stair_estimator
ros2 run corgi_stair stair_main_outdoor3  # Most robust variant
```

### Open-Loop Demo (No Camera)

For demonstrations without sensor feedback:

```bash
ros2 run corgi_stair stair_main_indoor_open
ros2 topic pub /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

---

## Configuration

### Key Parameters (in source code)

Located in `src/stair_main_*.cpp`:

```cpp
// Gait Parameters
const double velocity = 0.1;           // m/s - Walking velocity
const double stand_height = 0.25;      // m - Body height above ground
const double step_length = 0.3;        // m - Nominal step length
const double step_height = 0.04;       // m - Foot lift height

// Stair Detection
const double max_step_length = 0.3;    // m - Maximum step for adjustment
const double min_step_length = 0.2;    // m - Minimum step for alignment

// Camera Calibration
const std::array<double, 2> CoM2camera = {0.32075, 0.099};  // m - CoM to camera offset
const std::array<double, 3> camera_bias = {-0.01, 0.25, 0.032};  // m - Initial camera position

// Control Gains (PID)
motor_cmd.kp_r = 150.0;   // Position gain
motor_cmd.kd_r = 1.75;    // Derivative gain
motor_cmd.ki_r = 0.0;     // Integral gain (disabled)
```

### Plane Segmentation Tuning

Located in `src/stair_estimator.cpp`:

```cpp
// ROI Filtering
pass.setFilterFieldName("x");
pass.setFilterLimits(0.20, 1.5);   // Forward distance range
pass.setFilterFieldName("y");
pass.setFilterLimits(-0.4, 0.4);   // Lateral range
pass.setFilterFieldName("z");
pass.setFilterLimits(-0.5, 1.0);   // Height range
```

---

## Algorithm Details

### State Machine

All `stair_main_*` nodes implement a finite state machine:

```
INIT → TRANSFORM → WAIT → WALK → STAIR → END
```

**States:**

- **INIT**: Motor initialization, move to stand position
- **TRANSFORM**: Transition from stand to walk gait (5 seconds)
- **WAIT**: Wait for trigger signal
- **WALK**: Normal walking gait with stair detection
- **STAIR**: Active stair climbing with foothold planning
- **END**: Graceful shutdown

### Foothold Planning

When stair edge is detected:

1. **Measure stair geometry**: Depth `D` and height `H` from plane distances
2. **Calculate optimal foothold**: Position on stair step for maximum stability
3. **Adjust step length**: Dynamically modify walking gait to align with stair edge
4. **Transition criteria**: Switch to STAIR mode when front leg reaches optimal position

### Plane Tracking

The `stair_estimator` maintains running averages of detected planes:

- **Horizontal planes**: Ground, stair treads (sorted by height)
- **Vertical planes**: Stair risers, walls (sorted by distance)
- **Exponential smoothing**: Weighted average with decay to reduce noise
- **Outlier rejection**: Discard planes outside expected ranges

---

## Output Files

When nodes run, they generate CSV files in the current directory:

| File                    | Node            | Contents                                                      |
| ----------------------- | --------------- | ------------------------------------------------------------- |
| `motor_commands.csv`    | save_motor_cmd  | Time, motor positions (theta/beta), PID gains, torques        |
| `plane_distances.csv`   | stair_estimator | Time, trigger, 10 horizontal distances, 10 vertical distances |
| `stair_planes.csv`      | stair_estimator | Time, trigger, camera pose, averaged plane distances          |
| `stair_size.csv`        | stair*main*\*   | Time, trigger, stair depth (D), stair height (H)              |
| `command_pitch_CoM.csv` | stair*main*\*   | Time, trigger, body pitch, CoM position (x, z)                |

**Note:** CSV files are created in the directory where `ros2 run` is executed.

---

## Related Packages

- **corgi_msgs**: Message definitions (MotorCmd, StairPlanes, TriggerStamped)
- **corgi_walk**: Walking gait generation and leg kinematics (walk_gait, stair_climb)
- **corgi_utils**: Shared utilities (leg_model, bezier)
- **corgi_transform**: Coordinate transformations between locomotion modes
- **corgi_ros_bridge**: ROS ↔ FPGA communication for motor control

---

## References

### Papers & Algorithms

- **Point Cloud Segmentation**: Integral Image Normal Estimation + RANSAC
- **Gait Planning**: Bezier curve-based trajectory generation
- **Leg Kinematics**: Theta-beta coordinate system for dual-motor legs

### Hardware

- **ZED Camera**: Stereo depth camera for point cloud generation
- **Corgi Robot**: 4-module wheeled-legged hybrid quadruped
- **FPGA Controller**: NI sbRIO-9629 for real-time motor control

---

## License

BSD-3-Clause

For more information, see the [project documentation](../../README.md).
