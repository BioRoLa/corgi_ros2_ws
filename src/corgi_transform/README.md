# corgi_transform

Coordinate transformations between wheeled and legged locomotion modes for the Corgi quadruped robot.

## Overview

The `corgi_transform` package implements the critical transition algorithms that enable the Corgi robot to physically transform between its wheeled and legged locomotion modes. This is a unique capability of the Corgi's hybrid design, where each of the four modules can reconfigure from a wheeled stance to a legged stance and vice versa.

## Features

- **Wheel-to-Leg Transformation**: Multi-stage state machine for safe transition from wheeled to legged mode
- **Bezier Trajectory Generation**: Smooth swing trajectories during transformation
- **Hybrid Mode Support**: Intermediate locomotion combining wheeled and legged gaits
- **Theta-Beta Coordinate System**: Uses robot-specific motor control coordinates
- **Real-time Execution**: Publishes motor commands at 1000 Hz

## Package Information

- **ROS Version**: ROS 2 Humble
- **Language**: C++17
- **Build System**: ament_cmake
- **License**: BSD-3-Clause

## Dependencies

### ROS 2 Packages

- `rclcpp` - ROS 2 C++ client library
- `corgi_msgs` - Custom message definitions for Corgi robot
- `corgi_utils` - Shared utilities (LegModel, SwingProfile, Bezier curves)

### External Libraries

- `Eigen3` - Linear algebra library
- `eigen3_cmake_module` - CMake support for Eigen3

## Installation

```bash
cd ~/corgi_ws/corgi_ros_ws/
colcon build --packages-select corgi_transform --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install
source install/setup.bash
```

## Nodes

### transform_main

Main transformation node that executes the wheel-to-leg transformation sequence.

**Published Topics:**

- `/motor/command` (`corgi_msgs/msg/MotorCmdStamped`) - Motor commands in theta-beta coordinates at 1000 Hz

**Parameters:**

- None (currently hardcoded in source)

**Usage:**

```bash
ros2 run corgi_transform transform_main
```

## Transformation Algorithm

The wheel-to-leg transformation follows a 7-stage state machine:

### Stage 0: Initial Stay

- Robot maintains current position
- Duration: `stay_time_step` (configurable)

### Stage 1: Front Preparation

- Rotates wheels until right front (RF) module beta angle reaches ~45°
- Calculates required body angle based on stance height
- Plans hybrid mode step sequence

### Stage 2: Front Module Transform

- Transforms front modules (LF and RF) to legged stance
- RF transitions to target theta-beta position
- LF prepares for hybrid walking

### Stage 3: Hybrid Mode Walking

- Alternating front leg swings while hind modules remain wheeled
- Uses Bezier curves for smooth swing trajectories
- Number of steps determined by geometry constraints
- Maintains body stability throughout

### Stage 4: Stability Maintenance

- Brief stabilization period
- All legs prepare for final hind transformation

### Stage 5: Hind Module Transform

- Transforms rear modules (RH and LH) to legged stance
- Dynamically adjusts body angle based on hind leg contact
- Front legs maintain stance and compensate for body motion

### Stage 6: Final Stabilization

- Short stabilization period with all four legs in stance
- Ensures balanced legged configuration

### Stage 7: Transformation Complete

- Sets `transform_finished = true`
- Node terminates

## Key Classes

### WheelToLegTransformer

Primary class implementing the transformation logic.

**Constructor:**

```cpp
WheelToLegTransformer(bool sim = true)
```

- `sim`: Whether running in simulation (affects leg model parameters)

**Public Methods:**

```cpp
void initialize(double init_eta[8])
```

Initialize transformation with current motor positions (8 values for 4 modules × 2 motors).

```cpp
std::array<std::array<double, 4>, 2> step()
```

Execute one timestep of transformation. Returns current [theta, beta] arrays for all 4 modules.

**Public Members:**

- `int stage` - Current transformation stage (0-7)
- `bool transform_finished` - Whether transformation is complete
- `int stay_time_step` - Duration to stay in initial position

## Configuration Parameters

Key parameters defined in `WheelToLegTransformer` class:

| Parameter               | Value   | Description                  |
| ----------------------- | ------- | ---------------------------- |
| `BL`                    | 0.444 m | Body length                  |
| `BW`                    | 0.4 m   | Body width                   |
| `BH`                    | 0.2 m   | Body height                  |
| `body_vel`              | 0.1 m/s | Transformation velocity      |
| `stance_height`         | 0.2 m   | Target height in legged mode |
| `step_height`           | 0.05 m  | Swing trajectory clearance   |
| `dt`                    | 0.001 s | Control timestep (1 kHz)     |
| `last_transform_step_x` | 0.15 m  | Final step forward distance  |

## Motor Control Details

### Theta-Beta Coordinate System

The robot uses a specialized coordinate system for its 2-motor-per-leg design:

- **theta (θ)**: Leg spread angle - controls leg width
- **beta (β)**: Leg rotation angle - controls leg position around body
- **theta_0**: 17° offset for physical leg geometry

For modules B (RF) and C (RH), beta is negated due to motor mounting orientation.

### Module Naming Convention

- **module_a**: Left Front (LF)
- **module_b**: Right Front (RF)
- **module_c**: Right Hind (RH)
- **module_d**: Left Hind (LH)

## Integration with Corgi System

This package is part of the broader Corgi locomotion stack:

```
corgi_algo (high-level planning)
    ↓
corgi_gait_selector (mode selection)
    ↓
corgi_transform ← YOU ARE HERE
    ↓
corgi_ros_bridge (hardware interface)
```

The package can be used:

1. **Standalone**: Run transformation sequence and exit
2. **Integrated**: Called by higher-level controllers during mode switching

## Future Development

### Leg-to-Wheel Transformation

The reverse transformation (leg-to-wheel) is currently not implemented:

- `leg_to_wheel.cpp` and `leg_to_wheel.hpp` are empty stub files
- Future work should implement the reverse state machine

### Dynamic Parameter Configuration

Consider adding ROS 2 parameters for:

- Transformation velocity
- Stance height
- Step height
- Hybrid mode step constraints

## Troubleshooting

### Build Issues

If `corgi_utils` library is not found:

```bash
# Build corgi_utils first
colcon build --packages-select corgi_utils
```

### Runtime Issues

**Transformation doesn't complete:**

- Check initial motor positions match expected wheeled configuration
- Verify `stay_time_step` is appropriate for your hardware

**Motor commands seem incorrect:**

- Ensure theta-beta conversions account for module orientation (RF/RH have negated beta)
- Verify initial eta array matches current robot state

## References

- [Corgi Workspace Documentation](../../README.md)
- [corgi_utils Package](../corgi_utils/README.md) - Leg model and trajectory utilities
- [corgi_msgs Package](../corgi_msgs/README.md) - Message definitions

## Contributors

- Original Author: yisyuan (r12522823@ntu.edu.tw)
- Maintainer: Bio-Inspired Robotic Laboratory (BioRoLa), NTU

## License

BSD-3-Clause License. See workspace root for full license text.
