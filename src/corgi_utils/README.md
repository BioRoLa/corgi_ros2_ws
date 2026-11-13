# corgi_utils

Utility library for the Corgi quadruped robot, providing core mathematical and kinematic functions used across the control stack.

## Overview

`corgi_utils` is a ROS2 package that provides essential utility classes and functions for the Corgi robot, including:
- **Leg kinematics**: Forward and inverse kinematics for the unique theta-beta coordinate system
- **Bezier curves**: Trajectory generation for smooth leg swing profiles
- **Trajectory planning**: Linear interpolation with parabolic blending for smooth motion

This is a pure C++ library with minimal ROS dependencies, making it reusable across different control packages.

## Features

### 1. Leg Model (`LegModel`)

The leg model implements kinematics for Corgi's unique wheeled-legged hybrid design using theta-beta coordinates.

**Key Functions:**
- `forward(theta, beta)` - Forward kinematics to compute joint positions
- `inverse(pos, joint)` - Inverse kinematics to find theta-beta from Cartesian position
- `contact_map(theta, beta, slope)` - Determine contact point on wheel rim
- `move(theta, beta, move_vec)` - Compute theta-beta to achieve desired displacement

**Coordinate System:**
- **theta (θ)**: Leg spread angle (half the difference between left/right motor angles)
- **beta (β)**: Leg rotation angle (average of left/right motor angles)
- **theta_0**: 17° offset for physical leg geometry

### 2. Bezier Curves (`Bezier`, `SwingProfile`)

**Bezier Class:**
- Generates points on Bezier curves from control points
- Supports arbitrary degree curves
- `getBzPoint(t, offset_x, offset_y)` - Get point at parameter t ∈ [0,1]

**SwingProfile Class:**
- Creates smooth swing trajectories for leg motion
- Automatically generates control points for foot swing
- `getFootendPoint(t_duty)` - Get foot position at duty cycle t

### 3. Trajectory Planning (`LinearParaBlend`)

Linear interpolation with parabolic blending for smooth acceleration/deceleration.

**Features:**
- Via points with specified timing
- Configurable blend time for smooth transitions
- Optional initial and final velocity constraints
- `get_point(t)` - Get position at time t ∈ [0,1]

## Installation

### Prerequisites

- ROS2 Humble or later
- Eigen 3.3+ (install to `~/corgi_ws/install/` for LOCAL_PACKAGE_PATH support)
- C++17 compatible compiler

### Build

```bash
cd ~/corgi_ws/corgi_ros_ws
colcon build --packages-select corgi_utils --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install
source install/setup.bash
```

## Usage

### As a Library Dependency

In your `package.xml`:
```xml
<depend>corgi_utils</depend>
```

In your `CMakeLists.txt`:
```cmake
find_package(corgi_utils REQUIRED)
ament_target_dependencies(your_target corgi_utils)
```

### Example Code

```cpp
#include "leg_model.hpp"
#include "bezier.hpp"
#include "trajectory_plan.hpp"

// Forward kinematics
LegModel leg(true);  // true = simulation mode
leg.forward(M_PI * 130.0 / 180.0, M_PI * 50.0 / 180.0);
std::cout << "Foot position: " << leg.G[0] << ", " << leg.G[1] << std::endl;

// Inverse kinematics
std::array<double, 2> target = {0.05, -0.25};
auto theta_beta = leg.inverse(target, "G");
std::cout << "Theta: " << theta_beta[0] << ", Beta: " << theta_beta[1] << std::endl;

// Bezier curve
std::vector<std::array<double, 2>> control_pts = {{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.0}};
Bezier bezier(control_pts);
auto point = bezier.getBzPoint(0.5);

// Swing profile
std::array<double, 2> lift_point = {0.0, 0.0};
std::array<double, 2> touch_point = {0.5, 0.0};
SwingProfile swing(lift_point, touch_point, 0.1, 1);  // 0.1m height, forward
auto foot_pos = swing.getFootendPoint(0.5);

// Trajectory planning
std::vector<double> via_points = {0.0, 1.0, 0.5, 1.5};
std::vector<double> via_times = {0.0, 0.3, 0.7, 1.0};
LinearParaBlend traj(via_points, via_times, 0.1);  // 0.1s blend time
double position = traj.get_point(0.5);
```

### Demo Executable

Run the included demo to test kinematics:

```bash
ros2 run corgi_utils leg_model
```

**Output:**
- Forward kinematics timing (1M iterations)
- Inverse kinematics accuracy test
- Contact mapping demonstration
- Move function example

## Testing

Run the comprehensive test suite:

```bash
bash src/corgi_utils/test_ros2_conversion.sh
```

**Tests include:**
- Package build verification
- Library installation check
- Executable functionality
- Integration with dependent packages

## Package Structure

```
corgi_utils/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package metadata
├── README.md               # This file
├── CONVERSION_STATUS.md    # ROS2 conversion documentation
├── test_ros2_conversion.sh # Automated test script
├── include/
│   ├── leg_model.hpp       # Leg kinematics
│   ├── bezier.hpp          # Bezier curves
│   ├── trajectory_plan.hpp # Trajectory planning
│   ├── config.hpp          # Configuration structures
│   └── fitted_coefficient.hpp # Polynomial coefficients
└── src/
    ├── leg_model.cpp       # Leg model implementation
    ├── leg_model_main.cpp  # Demo executable
    ├── bezier.cpp          # Bezier implementation
    ├── trajectory_plan.cpp # Trajectory implementation
    └── config.cpp          # Config loader (not compiled)
```

## Dependencies

**Runtime:**
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry message types
- `corgi_msgs` - Custom Corgi message types
- `eigen` - Linear algebra library

## Used By

This library is a dependency for many Corgi packages:
- `corgi_walk` - Legged locomotion algorithms
- `corgi_mpc` - Model predictive control
- `corgi_force_control` - Force control
- `corgi_force_estimation` - Contact force estimation
- `corgi_odometry` - State estimation
- `corgi_stair` - Stair climbing
- `corgi_legged` - Legged gait generation
- `corgi_wheeled` - Wheeled locomotion
- `corgi_hybrid` - Hybrid mode transitions

## Important Notes

1. **Theta-Beta Coordinates**: Always use the theta-beta coordinate system (not direct motor angles) when working with leg kinematics.

2. **theta_0 Offset**: The 17° offset is physically required for the leg geometry - never omit it.

3. **Simulation vs Real**: Pass `true` to `LegModel` constructor for simulation (no tire), `false` for real robot (with tire).

4. **Angle Limits**: Theta is constrained to [16.9°, 160°] to prevent mechanical damage.

## Performance

- Forward kinematics: ~200 µs per call (1M iterations in 200ms)
- Inverse kinematics: Converges in <10 iterations, accurate to <0.0001 units
- Bezier curves: ~1 µs per point evaluation

## ROS2 Conversion Status

✅ **Fully converted to ROS2 and operational**

- Build system: `ament_cmake`
- All code is ROS-agnostic (pure C++)
- Successfully builds and runs in ROS2 Humble
- Integration tested with dependent packages

See `CONVERSION_STATUS.md` for detailed conversion information.

## Contributing

This package is part of the Corgi quadruped robot project at Bio-Inspired Robotic Laboratory (BioRoLa), NTU.

## License

TODO

## Repository

Part of [corgi_ros_ws](https://github.com/BioRoLa/corgi_ros_ws.git).


## Module frame (`module_frame.hpp`)

`corgi::ModuleFrame` converts between the common ROS joint convention and one leg module's own wiring: positions, r/l torque and velocity pairs (swap and negate on a mirrored-beta leg), r/l gain pairs (swap, never negate) and the ABAD torque and velocity. Every map is self-inverse. The robot bridge is its user; the sim driver applies the same algebra in Python. Definitions and derivation: [`CONVENTIONS.md`](../../CONVENTIONS.md) section 5. Tests: `colcon test --packages-select corgi_utils`.

## Direction table (`config/module_directions.yaml`, `module_directions.hpp`)

The one per-leg table of θ, β and γ directions, installed to `share/corgi_utils/config/`. The robot bridge reads it through `corgi::load_module_directions()`; the sim driver reads the same file in Python (`corgi_driver_pkg/module_directions.py`) with the same rules: every leg and key required, no other keys, each value ±1, θ = +1. `CORGI_MODULE_DIRECTIONS` overrides the path for the bridge, the sim and the control panel. Rebuild `corgi_utils` after an edit.

## Leg-model frame (`leg_frame.hpp`)

`LegModel` and `calculate_jacobian_3d` work in a per-leg frame: x backward, y outward, z up. `corgi::leg_model_to_body()` / `body_to_leg_model()` convert to and from the body frame (x forward, y left, z up) with `diag(−1, s, +1)`, s = +1 on the left legs (A, D) and −1 on the right (B, C). `force_control` converts its body-frame command with it, and `force_estimation` its estimate (CONVENTIONS.md section 4).
