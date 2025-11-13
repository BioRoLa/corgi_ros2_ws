# corgi_panel

PyQt5 GUI control panel for the Corgi quadruped robot. This package is a ROS 2 (Humble) node that lets you power the system, select modes, trigger runs, and monitor status. It speaks the corgi_msgs message types and is compatible with simulation and real robot workflows.

## Features

- One-click power sequencing: Digital → Signal → Power
- Mode control: Rest, Set Zero, Hall Calibrate, Motor Mode, Steering Calibrate
- RT vs CSV control mode selection
- CSV playback with file picker (reads from `input_csv/`)
- Trigger start/stop with output filename for logging
- Status panel for power rails, mode, and per-motor status
- Optional hardware trigger on Jetson GPIO pin 16 (if Jetson.GPIO available)

## Requirements

- ROS 2 Humble (rclpy)
- Python 3 with:
  - PyQt5
  - numpy
- `corgi_msgs` built and sourced in the same workspace

These are declared in `package.xml` and resolved when building the workspace.

## Build

From the workspace root (`~/corgi_ws/corgi_ros_ws`):

```bash
colcon build --packages-select corgi_panel
source install/setup.bash
```

## Run

Recommended (launch):

```bash
ros2 launch corgi_panel corgi_control_panel.launch.py
```

Direct executable:

```bash
ros2 run corgi_panel corgi_control_panel.py
```

Notes:

- The GUI is a ROS 2 node. Some external tools (e.g., `corgi_ros_bridge`) may still be ROS 1; the panel attempts to start the bridge with ROS 2 first and falls back to ROS 1 if available.
- Always source the workspace before running.

## GUI Controls

- Run Bridge
  - Starts the ROS ↔ FPGA bridge. Prefers `ros2 run corgi_ros_bridge corgi_ros_bridge.sh`; falls back to `rosrun` if ROS 1 is present.
- Digital / Signal / Power
  - Follow the sequence: Digital → Signal → Power. The panel enforces button enablement to guide the order.
- Robot Mode Switch
  - Rest Mode: safe idle
  - Set Zero: calibrate home position
  - Hall Calibrate: perform hall sensor calibration
  - Motor Mode: enable control (choose RT or CSV)
  - Steering Calibrate: available in wheeled mode when allowed by state
- Motor Mode Selection
  - RT: real-time control path (external nodes)
  - CSV: playback of trajectories; enables CSV controls
- CSV Controls
  - Input File Name: choose a file from `input_csv/` via Select
  - Run: start/stop CSV playback (ROS 2 `corgi_csv_control` node)
- Output File Name (.csv)
  - Used by downstream loggers/recorders when Trigger is enabled
- Trigger
  - Toggles run/record state; also toggles Jetson GPIO pin 16 if available
- Reset
  - Returns panel to a safe default state and sends corresponding commands

## Topics

Publishers:

- `power/command` (corgi_msgs/PowerCmdStamped)
- `motor/command` (corgi_msgs/MotorCmdStamped)
- `trigger` (corgi_msgs/TriggerStamped)

Subscribers:

- `power/state` (corgi_msgs/PowerStateStamped)
- `motor/state` (corgi_msgs/MotorStateStamped)
- `steer/state` (corgi_msgs/SteeringStateStamped)

Header stamps use ROS 2 time (`node.get_clock().now().to_msg()`).

## CSV Control

- Place CSVs in `~/corgi_ws/corgi_ros_ws/input_csv/`
- Use the Select button to pick the file (the panel stores the basename without extension)
- Press Run to start/stop the `corgi_csv_control` node with the selected input

## Simulation vs Real Robot

- Simulation (ROS 2): Use `corgi_sim` and CSV/RT modes without the hardware bridge
- Real Robot (mixed ROS 1/2 during migration):
  - Start the FPGA driver on the robot
  - Use the panel sequence Digital → Signal → Power
  - Set Zero → Hall Calibrate → (optional) Steering Calibrate → Motor Mode
  - Choose RT or CSV and set output filename
  - Trigger to start motion + recording

## Notes & Known Issues

- Qt runtime dir warning: `QStandardPaths: wrong permissions on /run/user/1000/` is harmless in development; fix by setting directory perms to `0700` if desired.
- Thread safety: ROS callbacks use Qt signals to update the GUI on the main thread.
- Jetson GPIO is optional: if `Jetson.GPIO` is not importable, GPIO features are disabled automatically.

## Development

- Package type: `ament_cmake` + `ament_cmake_python`
- Executable installed to: `install/corgi_panel/lib/corgi_panel/corgi_control_panel.py`
- Launch file: `launch/corgi_control_panel.launch.py`

### Building just this package

```bash
colcon build --packages-select corgi_panel
```

### Running just the panel

```bash
source install/setup.bash
ros2 run corgi_panel corgi_control_panel.py
```

## License

BSD-3-Clause (see package.xml)
