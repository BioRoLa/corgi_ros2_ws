# corgi_gait_selector

**Finite State Machine (FSM) for gait selection and locomotion mode management in the Corgi quadruped robot.**

This package provides the `GaitSelector` class - a comprehensive state machine that manages transitions between different locomotion modes (wheeled, legged, hybrid) and coordinates motor commands for smooth gait execution.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [API Reference](#api-reference)
- [Building](#building)
- [Usage](#usage)
- [Configuration](#configuration)
- [Integration](#integration)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `corgi_gait_selector` package implements a finite state machine that:

1. Manages gait state transitions (WHEELED → LEGGED → HYBRID → TRANSFORM)
2. Provides smooth interpolation between motor positions during transitions
3. Publishes motor commands in theta-beta coordinates
4. Tracks motor state feedback for closed-loop control
5. Supports configurable gait parameters (velocity, step length, duty cycle, etc.)

This is a **library package** - it provides the `GaitSelector` class to be instantiated by other packages (corgi_legged, corgi_wheeled, corgi_hybrid) rather than standalone executables.

---

## Features

### Gait State Management

- **Four gait modes**: WHEELED, LEGGED, HYBRID, TRANSFORM
- **State enumeration** for type-safe gait selection
- **Automatic state tracking** with current and target gait states

### Motor Control

- **Theta-beta coordinate system** for dual-motor legs
- **Position command publishing** at configurable rates (default 1000 Hz)
- **Motor state feedback** subscription for closed-loop control
- **PID gain configuration** per motor (default: kp=150, kd=1.75, ki=0)

### Smooth Transitions

- **Transfer() method** for interpolated position changes
- **Configurable transfer duration** (default 5 seconds)
- **Wait period support** for settling time (default 3 seconds)
- **Linear interpolation** between current and target positions

### Configurable Parameters

All key gait parameters are static members, configurable at runtime:

- Velocity, step length, step height, stand height
- Duty cycle per leg
- Swing phase and timing
- Curvature for turning maneuvers
- Differential step length for rotation

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    External Packages                         │
│         (corgi_legged, corgi_wheeled, corgi_hybrid)         │
└────────────────────────┬────────────────────────────────────┘
                         │ Creates GaitSelector instance
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                   GaitSelector (FSM)                        │
│  • State management (currentGait, newGait)                  │
│  • Motor command generation                                 │
│  • Interpolation (Transfer)                                 │
│  • Parameter storage (static variables)                     │
└───────────┬─────────────────────────────┬───────────────────┘
            │ Publishes                   │ Subscribes
            ↓                             ↓
    /motor/command              /motor/state, /trigger
   (MotorCmdStamped)            (MotorStateStamped, TriggerStamped)
```

### Message Flow

```
Gait Generator (user code)
    ↓ Sets parameters
GaitSelector
    ↓ Calls Send()
Motor Commands → /motor/command
    ↓
Motor Controllers (corgi_ros_bridge)
    ↓
Motor State ← /motor/state
    ↓
GaitSelector (motor_state_cb)
    ↓ Feedback for next iteration
```

---

## API Reference

### Constructor

```cpp
GaitSelector(rclcpp::Node::SharedPtr node,
             bool sim = true,
             double CoM_bias = 0.0,
             int pub_rate = 1000,
             double BL = 0.444,
             double BW = 0.4,
             double BH = 0.2);
```

**Parameters:**

- `node` - ROS2 node shared pointer (required for pub/sub)
- `sim` - Simulation mode flag (passed to LegModel)
- `CoM_bias` - Center of Mass bias offset (meters)
- `pub_rate` - Publishing rate in Hz (default: 1000)
- `BL` - Body Length in meters (default: 0.444)
- `BW` - Body Width in meters (default: 0.4)
- `BH` - Body Height in meters (default: 0.2)

### Core Methods

#### `void Send()`

Publishes current motor commands and maintains control loop timing.

**Usage:**

```cpp
gait_selector.Send();  // Call at your control rate (e.g., 1000 Hz)
```

**Behavior:**

- Copies `next_eta[i][j]` to motor command messages
- Applies theta-beta to motor module mapping (accounts for left/right symmetry)
- Sets PID gains (kp=150, ki=0, kd=1.75)
- Publishes to `/motor/command`
- Sleeps to maintain configured rate

#### `void Transfer(int transfer_sec, int wait_sec)`

Smoothly interpolates from current position to target position.

**Parameters:**

- `transfer_sec` - Duration for interpolation (seconds)
- `wait_sec` - Hold time at target position (seconds)

**Usage:**

```cpp
// Set target position in next_eta[leg_index][theta/beta]
gait_selector.next_eta[0][0] = target_theta;
gait_selector.next_eta[0][1] = target_beta;

// Execute smooth transition over 5 seconds, then wait 3 seconds
gait_selector.Transfer(5, 3);
```

**Behavior:**

- Reads current motor state from feedback
- Generates linear trajectory (linspace)
- Steps through trajectory calling Send() at each step
- Holds final position for wait period

#### `void Receive()`

Updates `next_eta` with current motor state from feedback.

**Usage:**

```cpp
gait_selector.Receive();  // Sync with actual motor positions
```

### Callbacks

#### `void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr state)`

Updates static `motor_state` variable with latest feedback.

**Subscribed Topic:** `/motor/state`

#### `void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)`

Updates `trigger_msg` for enable/disable control.

**Subscribed Topic:** `/trigger`

---

## Building

### Prerequisites

**ROS2 Dependencies:**

```bash
sudo apt install ros-humble-rclcpp \
                 ros-humble-std-msgs \
                 ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs \
                 ros-humble-visualization-msgs
```

**Workspace Dependencies:**

- `corgi_msgs` - Custom message definitions
- `corgi_utils` - Shared utilities (LegModel, leg kinematics)
- `Eigen3` (version 3.4.0)

### Build Commands

```bash
cd ~/corgi_ws/corgi_ros_ws

# Build with dependencies
colcon build --packages-select corgi_gait_selector

# Source the workspace
source install/setup.bash
```

### Verify Installation

```bash
# Check library installation
ls -la install/corgi_gait_selector/lib/

# Expected output:
# libcorgi_gait_selector.a (static library)

# Check header installation
ls install/corgi_gait_selector/include/

# Expected output:
# Simple_fsm.hpp
```

---

## Usage

### Basic Integration Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include "Simple_fsm.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_gait_controller");

    // Create GaitSelector instance
    GaitSelector gait_selector(node,
                               true,    // simulation mode
                               0.0,     // CoM bias
                               1000,    // 1kHz control rate
                               0.444,   // body length
                               0.4,     // body width
                               0.2);    // body height

    // Set gait parameters
    GaitSelector::velocity = 0.15;       // m/s
    GaitSelector::step_length = 0.4;     // m
    GaitSelector::stand_height = 0.20;   // m
    GaitSelector::step_height = 0.05;    // m

    // Control loop
    rclcpp::WallRate rate(1000);  // 1kHz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Your gait generation logic here
        // Update next_eta[leg][theta/beta] with desired positions

        gait_selector.Send();  // Publish commands
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
```

### Setting Target Positions

```cpp
// Leg indices: 0=Left Front, 1=Right Front, 2=Right Hind, 3=Left Hind
// Coordinate indices: 0=theta, 1=beta

// Example: Set Left Front leg to specific theta-beta
gait_selector.next_eta[0][0] = 0.5;  // theta (rad)
gait_selector.next_eta[0][1] = 0.3;  // beta (rad)

// Send immediately
gait_selector.Send();

// Or smoothly transition
gait_selector.Transfer(5, 2);  // 5s transfer, 2s wait
```

### Gait State Management

```cpp
// Check current gait
if (gait_selector.currentGait == Gait::WHEELED) {
    // Switch to legged mode
    gait_selector.newGait = Gait::LEGGED;
    // Implement transition logic...
}

// Available states
enum class Gait {
    WHEELED = 0,
    LEGGED = 1,
    HYBRID = 2,
    TRANSFORM = 3
};
```

---

## Configuration

### Static Gait Parameters

All parameters are public static members - modify before or during execution:

```cpp
// Timing and Speed
GaitSelector::velocity = 0.15;        // m/s - Forward velocity
GaitSelector::swing_time = 0.25;      // s - Swing phase duration

// Geometry
GaitSelector::stand_height = 0.20;    // m - Body height above ground
GaitSelector::step_length = 0.4;      // m - Step length during gait
GaitSelector::step_height = 0.05;     // m - Foot clearance height

// Per-Leg Duty Cycle (0.0-1.0)
GaitSelector::duty[0] = 0.5;  // Left Front
GaitSelector::duty[1] = 0.5;  // Right Front
GaitSelector::duty[2] = 0.5;  // Right Hind
GaitSelector::duty[3] = 0.5;  // Left Hind

// Swing Phase (leg timing offsets)
GaitSelector::swing_phase[0] = 0;   // LF
GaitSelector::swing_phase[1] = 1;   // RF
GaitSelector::swing_phase[2] = 2;   // RH
GaitSelector::swing_phase[3] = 3;   // LH

// Turning
GaitSelector::curvature = 0.0;   // rad/m: + left, - right, 0 straight
```

### Per-Leg Dynamic Parameters

```cpp
// Step length per leg (for uneven terrain)
GaitSelector::next_step_length[0] = 0.35;  // LF
GaitSelector::next_step_length[1] = 0.40;  // RF
// ... etc

// Stand height per leg (for slope adaptation)
GaitSelector::next_stand_height[0] = 0.18;
GaitSelector::next_stand_height[1] = 0.22;
// ... etc

// Body and foothold positions
GaitSelector::next_body[0] = x;  // x position
GaitSelector::next_body[1] = y;  // y position
GaitSelector::next_body[2] = z;  // z position (height)
```

### PID Gains

Default gains are set in `Send()`:

```cpp
motor_cmd_modules[i]->kp_r = 150.0;   // Proportional gain
motor_cmd_modules[i]->kd_r = 1.75;    // Derivative gain
motor_cmd_modules[i]->ki_r = 0.0;     // Integral gain (disabled)
// Same for _l (left motor)
```

To customize, modify after creating GaitSelector or override in `Send()`.

---

## Integration

### With Gait Generators

Typical usage pattern in gait generation packages:

```cpp
// In corgi_legged/src/trot_gait.cpp
class TrotGait {
public:
    TrotGait(rclcpp::Node::SharedPtr node) {
        gait_selector_ = std::make_shared<GaitSelector>(node, ...);
        gait_selector_->currentGait = Gait::LEGGED;
    }

    void update() {
        // Generate trot gait trajectory
        for (int leg = 0; leg < 4; leg++) {
            // Calculate theta-beta for trot pattern
            gait_selector_->next_eta[leg][0] = calculate_theta(leg);
            gait_selector_->next_eta[leg][1] = calculate_beta(leg);
        }
        gait_selector_->Send();
    }

private:
    std::shared_ptr<GaitSelector> gait_selector_;
};
```

### With Trigger System

```cpp
// Enable/disable gait execution
rclcpp::WallRate rate(1000);
while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (gait_selector.trigger_msg.enable) {
        // Generate and send commands only when enabled
        update_gait_trajectory();
        gait_selector.Send();
    }

    rate.sleep();
}
```

### Coordinate System Notes

**Important:** The GaitSelector handles theta-beta symmetry automatically:

- Modules A (LF) and D (LH): `beta = -next_eta[i][1]` (left side)
- Modules B (RF) and C (RH): `beta = +next_eta[i][1]` (right side)

User code should work in the **robot's coordinate frame** - the class handles motor-specific transformations.

---

## Troubleshooting

### Build Errors

**Error: "leg_model.hpp: No such file"**

```bash
# Solution: Ensure corgi_utils is built first
colcon build --packages-select corgi_utils
colcon build --packages-select corgi_gait_selector
```

**Error: "corgi_utils target not found"**

```bash
# Solution: corgi_utils must be ROS2-compatible
# Check: colcon list --packages-select corgi_utils
# Should show: (ros.ament_cmake)
```

### Runtime Issues

**Motors not responding**

1. Check `/motor/command` is being published:
   ```bash
   ros2 topic hz /motor/command
   # Should show ~1000 Hz
   ```
2. Verify trigger is enabled:
   ```bash
   ros2 topic echo /trigger
   # enable should be true
   ```
3. Check motor state feedback:
   ```bash
   ros2 topic hz /motor/state
   ```

**Jerky motion during Transfer()**

- Increase `transfer_sec` for smoother interpolation
- Check that `motor_state` is being updated (verify callback)
- Ensure control loop rate matches `pub_rate`

**Unexpected leg positions**

- Verify theta-beta coordinate system understanding
- Check `next_eta` values are in radians, not degrees
- Remember: theta includes theta_0 offset (17° = 0.2967 rad)

### Common Pitfalls

❌ **Don't** forget to call `rclcpp::spin_some(node)` in your loop  
❌ **Don't** modify `eta` directly - use `next_eta`  
❌ **Don't** assume Send() is non-blocking - it sleeps to maintain rate  
❌ **Don't** create multiple GaitSelector instances per node

✅ **Do** call Send() at regular intervals (e.g., 1000 Hz)  
✅ **Do** use Transfer() for smooth state changes  
✅ **Do** check trigger_msg.enable before execution  
✅ **Do** monitor motor state feedback

---

## Topics

### Published

| Topic             | Type                             | Rate    | Description                          |
| ----------------- | -------------------------------- | ------- | ------------------------------------ |
| `/motor/command`  | `corgi_msgs/msg/MotorCmdStamped` | 1000 Hz | Motor position commands (theta-beta) |
| `stable_triangle` | `visualization_msgs/msg/Marker`  | 1000 Hz | Visualization marker for stability   |

### Subscribed

| Topic          | Type                               | Description            |
| -------------- | ---------------------------------- | ---------------------- |
| `/motor/state` | `corgi_msgs/msg/MotorStateStamped` | Motor encoder feedback |
| `/trigger`     | `corgi_msgs/msg/TriggerStamped`    | Enable/disable signal  |

---

## Dependencies

### Required Packages

- **rclcpp** - ROS2 C++ client library
- **corgi_msgs** - Custom message types (MotorCmd, MotorState, Trigger)
- **corgi_utils** - Leg kinematics (LegModel class)
- **std_msgs**, **geometry_msgs**, **sensor_msgs**, **visualization_msgs** - Standard ROS messages
- **Eigen3** - Linear algebra (version 3.4.0+)

### Reverse Dependencies

Packages that use corgi_gait_selector:

- `corgi_legged` - Legged gait generators (walk, trot, pace, bound)
- `corgi_wheeled` - Wheeled locomotion control
- `corgi_hybrid` - Hybrid mode transitions
- `corgi_algo` - High-level behavior control

---

## Examples

### Example 1: Standing Position

```cpp
auto node = std::make_shared<rclcpp::Node>("stand_controller");
GaitSelector gs(node);

// Set all legs to standing configuration
for (int i = 0; i < 4; i++) {
    gs.next_eta[i][0] = M_PI * 17.0 / 180.0;  // theta = 17° (init)
    gs.next_eta[i][1] = 0.0;                   // beta = 0° (vertical)
}

// Smoothly move to standing
gs.Transfer(3, 2);  // 3s transition, 2s hold
```

### Example 2: Simple Walk Cycle

```cpp
rclcpp::WallRate rate(1000);
double phase = 0.0;

while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    // Simple sinusoidal leg motion
    for (int leg = 0; leg < 4; leg++) {
        double leg_phase = phase + leg * M_PI / 2;  // 90° offset per leg

        gs.next_eta[leg][0] = 0.5 + 0.2 * std::sin(leg_phase);  // theta
        gs.next_eta[leg][1] = 0.1 * std::cos(leg_phase);        // beta
    }

    gs.Send();
    phase += 0.001;  // Increment phase
    rate.sleep();
}
```

---

## License

BSD-3-Clause

---

## Maintainer

- **Author**: Yatinghsu000627
- **Email**: r12522806@ntu.edu.tw
- **Lab**: Bio-Inspired Robotic Laboratory (BioRoLa), NTU

---

## Version History

- **v0.1.0** (Nov 2025): ROS2 Humble conversion
  - Converted from ROS1 Noetic to ROS2 Humble
  - Changed constructor signature (NodeHandle → Node::SharedPtr)
  - Updated callbacks to use SharedPtr
  - Builds and integrates successfully

---

### Papers & Algorithms

- Finite State Machine design for quadruped locomotion
- Theta-beta dual-motor leg control
- Smooth trajectory interpolation techniques

---

For more information about the Corgi robot system, see the [main workspace documentation](../../README.md).
