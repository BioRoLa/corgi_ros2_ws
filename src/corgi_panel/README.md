# Corgi Panel - Build and Installation Guide

## Package Structure
```
corgi_panel/
├── package.xml          # ROS 2 package manifest
├── setup.py             # Python package setup
├── resource/
│   └── corgi_panel      # Package marker file
├── scripts/
│   ├── __init__.py
│   ├── run_control.py   # Control Panel entry point
│   ├── run_config.py    # Config Panel entry point
│   └── normalize_sequence_json.py  # Convert auto-generated JSON to sequence format
└── corgi_ui/            # Main package source
    ├── __init__.py
    ├── assets/
    │   └── theme.qss
    ├── core/
    │   ├── constants.py
    │   ├── motor_data.py
    │   ├── process_manager.py
    │   ├── ros_worker.py
    │   └── sequence_model.py   # Sequence data model and execution records
    └── gui/
        ├── control_panel.py
        ├── config_panel.py
        ├── custom_sequence_window.py  # Custom Command Sequence editor window
        └── widgets/
            └── log_widget.py
```

## Build Instructions

### 1. Build the package
```bash
cd ~/corgi_ws/corgi_ros2_ws
colcon build --packages-select corgi_panel
```

### 2. Source the workspace
```bash
source install/setup.bash
```

### 3. Run the panels

**Control Panel:**
```bash
ros2 run corgi_panel corgi_control_panel
```

**Configuration Panel:**
```bash
ros2 run corgi_panel corgi_config_panel
```

## Development Mode

For development without reinstalling after each change:

```bash
# Build with symlink install (Python changes take effect immediately)
colcon build --packages-select corgi_panel --symlink-install

# Source workspace
source install/setup.bash
```

## Dependencies

### ROS 2 Packages
- rclpy
- std_msgs
- sensor_msgs
- corgi_msgs (custom package)

### Python Packages
- PyQt5
- numpy

### System Requirements
- ROS 2 (Humble or later)
- Python 3.8+
- Qt5

## Troubleshooting

### 1. Import errors
If you see `ModuleNotFoundError: No module named 'corgi_ui'`:
```bash
# Make sure you've sourced the workspace
source install/setup.bash

# Verify package installation
ros2 pkg list | grep corgi_panel
```

### 2. Theme not loading
If the dark theme doesn't apply:
```bash
# Check if assets were installed
ls install/corgi_panel/share/corgi_panel/assets/
```

### 3. Entry points not found
If `ros2 run corgi_panel corgi_control_panel` fails:
```bash
# Rebuild and source again
colcon build --packages-select corgi_panel
source install/setup.bash
```

## Testing

Run basic smoke test:
```bash
# Test import
python3 -c "from corgi_ui.gui.control_panel import CorgiControlPanel; print('OK')"

# Test entry point
ros2 run corgi_panel corgi_control_panel --help
```

## Clean Build

If you encounter persistent issues:
```bash
# Clean build artifacts
rm -rf build/corgi_panel install/corgi_panel log/corgi_panel

# Rebuild
colcon build --packages-select corgi_panel
source install/setup.bash
```

## Architecture Overview

### MVC Pattern
- **Model**: `corgi_ui/core/` (constants, motor_data, ros_worker, process_manager)
- **View**: `corgi_ui/gui/` (control_panel, config_panel, widgets)
- **Controller**: ROS Worker (thread-safe communication layer)

### Key Features
- ✅ Thread-safe ROS 2 communication via `RosWorker`
- ✅ Centralized process management via `ProcessManager`
- ✅ Reusable UI components (LogWidget)
- ✅ Shared theme system (theme.qss)
- ✅ Modular architecture (easy to test and extend)
- ✅ Custom Command Sequence editor with per-node PD gains, joint targets, limit profiles

## Custom Command Sequence

The **Custom Command Sequence** feature allows you to define and execute multi-node position sequences on the robot.

### Enabling the feature

The feature is disabled by default. Pass the `enable_custom_sequence` parameter when launching the control panel:

```bash
ros2 run corgi_panel corgi_control_panel --ros-args -p enable_custom_sequence:=true
```

Or in a launch file:
```python
control_panel = Node(
    package='corgi_panel',
    executable='corgi_control_panel',
    parameters=[{
        'use_sim_time': True,
        'enable_custom_sequence': True,
    }],
    output='screen'
)
```

Once enabled, a **Custom Command Sequence** button appears in the tools sidebar.

### Features

- **Sequence editor**: Add, remove, reorder, duplicate nodes in a list
- **Per-node targets**: Set individual theta/beta/gamma angles (degrees) for each of the 4 legs (A/B/C/D)
- **Per-node PD gains**: Override global leg_kp/leg_kd/gamma_kp/gamma_kd per node
- **Duration**: Configurable time (seconds) for each node transition
- **Templates**: Built-in pose templates (Stand 90°, Squat 45°, Low 17.5°, Neutral 60°); save/load custom templates
- **Limit profiles**: Load/save joint angle and speed limits per leg per joint
- **Smooth interpolation**: Speed-limited interpolation from actual start pose (no jump on first node)
- **Dry-run mode**: Validate and preview without sending commands
- **Progress tracking**: Progress bar with ETA during execution
- **Auto-save execution record**: Results saved to `output_data/` as JSON

### JSON format

Sequences are saved/loaded as JSON files. The canonical format:
```json
{
  "version": 1,
  "name": "my_sequence",
  "nodes": [
    {
      "name": "Stand",
      "duration_sec": 2.0,
      "targets": {
        "A": {"theta": 90.0, "beta": 0.0, "gamma": 0.0},
        "B": {"theta": 90.0, "beta": 0.0, "gamma": 0.0},
        "C": {"theta": 90.0, "beta": 0.0, "gamma": 0.0},
        "D": {"theta": 90.0, "beta": 0.0, "gamma": 0.0}
      },
      "gains": {
        "leg_kp": 60.0, "leg_kd": 1.0,
        "gamma_kp": 20.0, "gamma_kd": 0.5
      },
      "notes": ""
    }
  ]
}
```

Auto-generated execution records from `output_data/` can also be loaded directly.

### Converting auto-generated JSON

Use the provided script to convert auto-generated JSON to canonical sequence format:
```bash
python3 scripts/normalize_sequence_json.py input.json output.json
```

### Timing and simulation

- **Header timestamps** (`msg.header.stamp`): use ROS clock → respects `use_sim_time`
- **Node duration timing**: also uses ROS clock → durations match simulated time when `use_sim_time=true`

## Next Steps

1. **Build the package**: `colcon build --packages-select corgi_panel`
2. **Source workspace**: `source install/setup.bash`
3. **Launch Control Panel**: `ros2 run corgi_panel corgi_control_panel`
4. **Test functionality**: Start ROS Bridge, control robot FSM, monitor status

Enjoy your refactored Corgi control system! 🐕🤖
