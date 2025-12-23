# corgi_sim - Corgi Quadruped Robot Webots Simulation

ROS2 simulation package for the Corgi quadruped robot using Webots.

## Overview

This package provides a complete simulation environment for the Corgi robot with:

- **Embedded Webots controller** - Runs directly inside Webots for reliable communication
- **Topic-based motor control** - Standard ROS2 pattern using `std_msgs/Float64`
- **Theta-beta coordinate system** - Custom leg control coordinates for 2-motor-per-leg design
- **Multiple simulation nodes** - Position and torque control modes
- **CSV trajectory playback** - Replay pre-recorded motions

## Architecture

```
Webots Simulator
  └─ corgi_ros2 (Python controller, runs inside Webots)
      ├─ Publishes: /motor_name_sensor/value (encoder feedback)
      └─ Subscribes: /motor_name/set_position (motor commands)
          ↕
corgi_sim_trq (C++ ROS2 node, runs externally)
  ├─ Subscribes: /motor/command (MotorCmdStamped - theta/beta coords)
  ├─ Publishes: /motor/state (MotorStateStamped - feedback)
  └─ Converts theta-beta ↔ motor angles (phi_r, phi_l)
      ↕
Controller (e.g., corgi_csv_control)
  └─ Publishes: /motor/command
```

## Quick Start

### 1. Build the Package

```bash
cd ~/corgi_ws/corgi_ros_ws
colcon build --packages-select corgi_sim
source install/setup.bash
```

### 2. Launch Simulation

```bash
# Start Webots and simulation node
ros2 launch corgi_sim run_simulation.launch.py
```

This will:

- Launch Webots with the Corgi robot world
- Start the `corgi_sim_trq` node (3 second delay to allow Webots to initialize)
- Set up all topics for motor control and feedback

### 3. Run a Controller

In a separate terminal:

```bash
source ~/corgi_ws/corgi_ros_ws/install/setup.bash

# Play a pre-recorded CSV trajectory
ros2 run corgi_csv_control corgi_csv_control  demo_transform_sim --ros-args -p use_sim_time:=True

# Or run other gaits
ros2 run corgi_csv_control corgi_csv_control demo_walk_sim --ros-args -p use_sim_time:=True
```
#### Note: 
* must add **--ros-args -p use_sim_time:=True** to sync Node timer with Webots

## Files and Directories

### Launch Files

- `launch/run_simulation.launch.py` - Main launch file for simulation

### Executables

- `corgi_sim_trq` - Main simulation node with torque/position control
- `corgi_sim_pos` - Alternative simulation node (functionally identical)

### Controllers (Webots-side)

- `controllers/corgi_ros2/corgi_ros2.py` - Embedded Python controller running inside Webots
- `controllers/force_plate/force_plate.py` - Force plate controller for specialized worlds

### Worlds

- `worlds/corgi_origin.wbt` - Main simulation world (default)
- `worlds/corgi_force_plate_proto.wbt` - World with ground force sensors
- `worlds/corgi_uneven*.wbt` - Uneven terrain worlds
- `worlds/corgi_stair_proto.wbt` - Stair climbing world

### Input Data

- `input_csv/` - Pre-recorded CSV trajectories for replay
  - `demo_transform_sim.csv` - Mode transformation (legged ↔ wheeled)
  - `demo_walk_sim.csv` - Walking gait
  - `demo_*_real.csv` - Real robot trajectories (for reference)

## Theta-Beta Coordinate System

The Corgi robot uses a unique **theta-beta** control system for its 4-module, 2-motor-per-leg design:

- **theta (θ)**: Leg spread angle - controls how wide the leg spreads
- **beta (β)**: Leg rotation angle - controls leg rotation around the hip
- **theta_0**: Physical offset constant (17° or 0.2967 rad)

### Conversion Functions

```cpp
// Motor angles to theta-beta
void phi2tb(double phi_r, double phi_l, double &theta, double &beta) {
    theta = (phi_l - phi_r) / 2.0 + theta_0;
    beta = (phi_l + phi_r) / 2.0;
}

// Theta-beta to motor angles
void tb2phi(double theta, double beta, double &phi_r, double &phi_l) {
    phi_r = beta - theta + theta_0;
    phi_l = beta + theta - theta_0;
}
```

**Motor naming convention:**

- Module A (LF): AR (left), AL (right)
- Module B (RF): BR (left), BL (right)
- Module C (RH): CR (left), CL (right)
- Module D (LH): DR (left), DL (right)

## ROS2 Topics

### Published by corgi_sim_trq

- `/motor/state` (corgi_msgs/MotorStateStamped) - Current motor positions in theta-beta
- `/imu/filtered` (sensor_msgs/Imu) - Filtered IMU data (gravity compensated)
- `/trigger` (corgi_msgs/TriggerStamped) - Simulation trigger/enable status

### Subscribed by corgi_sim_trq

- `/motor/command` (corgi_msgs/MotorCmdStamped) - Desired motor positions in theta-beta
- `/imu` (sensor_msgs/Imu) - Raw IMU data from Webots

### Internal Topics (Webots controller ↔ sim node)

- `/lf_left_motor/set_position` through `/lh_right_motor/set_position` - Motor position commands (std_msgs/Float64)
- `/lf_left_motor_sensor/value` through `/lh_right_motor_sensor/value` - Encoder feedback (std_msgs/Float64)

## Parameters

### corgi_sim_trq

- `output_filename` (string, default: "") - CSV filename for data logging
  - If empty, no data is recorded
  - If specified, logs to `output_data/<filename>.csv`

Example:

```bash
ros2 run corgi_sim corgi_sim_trq --ros-args -p output_filename:="my_experiment.csv"
```

## Development

### Creating New Controllers

To create a custom controller that commands the robot:

1. Subscribe to `/motor/state` to get current positions
2. Publish to `/motor/command` with desired theta-beta positions
3. Use the motor naming convention (module_a, module_b, module_c, module_d)

Example minimal controller:

```cpp
auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);

corgi_msgs::msg::MotorCmdStamped cmd;
cmd.module_a.theta = 0.5;  // 0.5 rad spread
cmd.module_a.beta = 0.0;   // 0 rad rotation
// Set module_b, module_c, module_d similarly...

motor_cmd_pub->publish(cmd);
```

### Modifying the Webots Controller

The embedded controller is at `controllers/corgi_ros2/corgi_ros2.py`. Key features:

- Runs inside Webots process (no network communication)
- Publishes encoder values at simulation rate
- Subscribes to motor position commands
- Handles NaN guards for safety

### Adding New World Files

1. Create `.wbt` file in `worlds/`
2. Ensure robot uses `controller "corgi_ros2"`
3. Update launch file if you want a dedicated launcher

## Troubleshooting

### Webots doesn't start

- Check `DISPLAY` variable: `export DISPLAY=:0`
- Verify Webots installation: `/usr/local/bin/webots --version`
- Check X server is running (if on WSL)

### Robot doesn't move

- Check topics: `ros2 topic list`
- Verify controller is publishing: `ros2 topic echo /motor/command`
- Check simulation node is running: `ros2 node list`
- Look for NaN errors in console (indicates invalid commands)

### "NaN position" errors at startup

- These are harmless initialization messages
- Occur before `corgi_sim_trq` starts publishing
- Robot will move normally once controller begins

### Motor positions look wrong

- Verify you're using theta-beta coordinates, not direct motor angles
- Check theta_0 offset is applied (17° = 0.2967 rad)
- Ensure theta >= theta_0 (enforced minimum spread)

## Related Packages

- **corgi_csv_control** - CSV trajectory playback controller
- **corgi_msgs** - Custom message definitions (must be built first)
- **corgi_gait_generate** - Gait generation libraries
- **corgi_panel** - GUI control panel for real robot

## Citation

If you use this simulation in your research, please cite:

```
Corgi Quadruped Robot Simulation
Bio-Inspired Robotic Laboratory (BioRoLa), National Taiwan University
```

## License

[Add your license here]
