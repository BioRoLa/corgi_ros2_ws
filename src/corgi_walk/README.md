# corgi_walk

Walk and trot gait generation algorithms for Corgi quadruped robot.

## ROS 2 Status: ✅ FULLY CONVERTED

This package has been completely converted to ROS 2 Humble, including all libraries and executable nodes.

## Overview

`corgi_walk` provides quadruped locomotion algorithms for walking and trotting gaits. The package includes both reusable C++ libraries for gait generation and standalone ROS 2 nodes for testing and demonstration.

## Features

### WalkGait Class
Generates stable walking gait trajectories using Bezier curve interpolation.

**Characteristics:**
- **Default velocity**: 0.1 m/s
- **Swing time**: 0.2 seconds
- **Default step length**: 0.3 m
- **Default step height**: 0.04 m
- **Stand height**: 0.25 m (adjustable)
- **Gait pattern**: Sequential leg lifting for stable walking
- **Turning**: Supports curvature-based turning (+ left, - right)

**Key Methods:**
```cpp
WalkGait(bool sim=true, double CoM_bias=0.0, int rate=1000, 
         double BL=0.444, double BW=0.4, double BH=0.2);
void initialize(double init_eta[8], double step_length=0.3);
std::array<std::array<double, 4>, 2> step();  // Returns [theta, beta] for 4 legs
void set_velocity(double velocity);
void set_stand_height(double height);
void set_step_length(double length);
void set_step_height(double height);
void set_curvature(double curvature);  // Turning radius control
bool if_touchdown();  // Check if leg is touching ground
```

### TrotGait Class
Generates faster trotting gait with diagonal leg pairs moving together.

**Characteristics:**
- **Default velocity**: 0.4 m/s (faster than walk)
- **Swing time**: 0.5 seconds
- **Default step length**: 0.2 m
- **Default step height**: 0.04 m
- **Stand height**: 0.25 m (adjustable)
- **Gait pattern**: Diagonal pairs (LF+RH, RF+LH) alternate
- **Turning**: Supports curvature-based turning

**Key Methods:**
Same interface as WalkGait for consistent API.

## Dependencies

**ROS 2 Packages:**
- `rclcpp` - ROS 2 C++ client library
- `std_msgs`, `geometry_msgs` - Standard message types
- `corgi_msgs` - Custom Corgi robot messages (`MotorCmdStamped`)
- `corgi_utils` - Shared utilities (`leg_model`, `bezier`)

**External:**
- Eigen3 - Linear algebra library

## Building

```bash
cd ~/corgi_ws/corgi_ros_ws
colcon build --packages-select corgi_walk
source install/setup.bash
```

## Usage

### As a Library

Include the gait headers in your ROS 2 node:

```cpp
#include "walk_gait.hpp"
#include "trot_gait.hpp"

// Initialize walk gait generator
bool is_simulation = true;
double CoM_bias = 0.0;
int sampling_rate = 1000;  // Hz (1 kHz control loop)
double body_length = 0.444;  // meters
double body_width = 0.4;
double body_height = 0.2;

WalkGait walk_gait(is_simulation, CoM_bias, sampling_rate, 
                   body_length, body_width, body_height);

// Initialize with starting leg positions [theta0, beta0, theta1, beta1, ...]
double init_eta[8] = {1.857, 0.479, 1.605, 0.129, 1.605, -0.129, 1.857, -0.479};
walk_gait.initialize(init_eta);

// Configure gait parameters
walk_gait.set_velocity(0.2);      // m/s
walk_gait.set_stand_height(0.25);  // m
walk_gait.set_step_length(0.3);    // m
walk_gait.set_step_height(0.05);   // m
walk_gait.set_curvature(0.5);      // rad/m (positive = turn left)

// Generate trajectory at each timestep (1 kHz)
auto eta_list = walk_gait.step();  // Returns [[theta×4], [beta×4]]

// Extract theta-beta coordinates for each leg
for (int i = 0; i < 4; i++) {
    double theta = eta_list[0][i];
    double beta = eta_list[1][i];
    // Convert to motor commands and publish
}
```

### Standalone Executables

Three test nodes are provided for demonstration:

#### 1. walk_main - Basic Walking Demo
Simple walking gait with sinusoidal parameter variation:

```bash
ros2 run corgi_walk walk_main
```

**Behavior:**
- Velocity oscillates: `0.2 * cos(t)`
- Stand height oscillates: `0.25 + 0.05 * cos(t)`
- Step length alternates: 0.3 m / 0.1 m
- Step height alternates: 0.08 m / 0.04 m
- Curvature oscillates: `1.0 * sin(t)` (turning)
- Publishes to `/motor/command` at 1 kHz

#### 2. trot_main - Basic Trotting Demo
Similar to walk_main but uses TrotGait class for faster locomotion:

```bash
ros2 run corgi_walk trot_main
```

**Behavior:**
- Uses diagonal leg pair coordination
- Same parameter variation pattern as walk_main
- Publishes to `/motor/command` at 1 kHz

#### 3. walk_exp - State Machine Walking
Advanced walking demo with state machine control and trigger support:

```bash
ros2 run corgi_walk walk_exp
```

**Features:**
- Subscribes to `/trigger` topic for external control
- State machine: INIT → TRANSFORM → WAIT → WALK → END
- Transform phase: 5-second transition to legged mode
- Configurable initial leg positions for different swing times
- Higher PID gains (kp=150 vs 90)

**Topics:**
- Subscribes: `/trigger` (`corgi_msgs/TriggerStamped`)
- Publishes: `/motor/command` (`corgi_msgs/MotorCmdStamped`)

## Coordinate System

The robot uses **theta-beta coordinates** for leg control:

- **theta (θ)**: Leg spread angle (17° to 160°)
- **beta (β)**: Leg rotation angle
- **Module naming**: 
  - module_a: Left Front (LF)
  - module_b: Right Front (RF)
  - module_c: Right Hind (RH)
  - module_d: Left Hind (LH)

**Beta sign convention:**
- Modules B and C (right side): positive beta
- Modules A and D (left side): negative beta

## Integration Examples

This package is used by:

- **corgi_stair**: Stair climbing with walk gait trajectories
- **corgi_mpc**: Model Predictive Control with gait planning
- **corgi_sim**: Webots simulation integration
- Custom locomotion controllers requiring gait generation

## Tuning Guidelines

### Walking Gait Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| velocity | 0.1 m/s | 0.05-0.3 | Walking speed |
| stand_height | 0.25 m | 0.20-0.30 | Ground clearance |
| step_length | 0.3 m | 0.1-0.4 | Stride distance |
| step_height | 0.04 m | 0.02-0.08 | Foot clearance |
| curvature | 0.0 | -2.0 to 2.0 | Turning radius |

### Trotting Gait Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| velocity | 0.4 m/s | 0.2-0.6 | Trotting speed |
| stand_height | 0.25 m | 0.20-0.30 | Ground clearance |
| step_length | 0.2 m | 0.1-0.3 | Stride distance |
| step_height | 0.04 m | 0.02-0.08 | Foot clearance |

**Note:** Larger step lengths require higher velocities to maintain stability.

## Performance

- **Library size**: 37 KB (static library)
- **Control frequency**: 1000 Hz (1 ms timestep)
- **Memory footprint**: Minimal, suitable for real-time control
- **Computational cost**: < 0.1 ms per step() call on typical hardware

## Exported Files

**Headers:**
- `install/corgi_walk/include/walk_gait.hpp`
- `install/corgi_walk/include/trot_gait.hpp`

**Library:**
- `install/corgi_walk/lib/libcorgi_walk.a`

**Executables:**
- `install/corgi_walk/lib/corgi_walk/walk_main`
- `install/corgi_walk/lib/corgi_walk/trot_main`
- `install/corgi_walk/lib/corgi_walk/walk_exp`
