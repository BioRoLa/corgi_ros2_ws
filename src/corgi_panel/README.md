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
│   └── run_config.py    # Config Panel entry point
└── corgi_ui/            # Main package source
    ├── __init__.py
    ├── assets/
    │   └── theme.qss
    ├── core/
    │   ├── constants.py
    │   ├── motor_data.py
    │   ├── process_manager.py
    │   └── ros_worker.py
    └── gui/
        ├── control_panel.py
        ├── config_panel.py
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

## Next Steps

1. **Build the package**: `colcon build --packages-select corgi_panel`
2. **Source workspace**: `source install/setup.bash`
3. **Launch Control Panel**: `ros2 run corgi_panel corgi_control_panel`
4. **Test functionality**: Start ROS Bridge, control robot FSM, monitor status

Enjoy your refactored Corgi control system! 🐕🤖
