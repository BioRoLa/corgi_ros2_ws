# Corgi Robot ROS 2 Workspace

![ROS Version](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-yellowgreen)

This is the central ROS 2 workspace for the Corgi quadruped robot, developed at the Bio-Inspired Robotics Laboratory (BioRoLa), NTU. It contains all necessary packages for control, simulation, and hardware interfacing.

<!-- <p align="center">
  <img src="[INSERT_IMAGE_OR_GIF_URL_HERE]" alt="Corgi Robot Demo" width="600"/>
</p> -->

## Table of Contents

- [**_System Architecture_**](#system-architecture)
- [**_System Requirements & Dependencies_**](#system-requirements--dependencies)
  - [_Hardware_](#hardware)
  - [_Software_](#software)
- [**_Installation and Build Instructions_**](#installation-and-build-instructions)
- [**_Conventions_**](#conventions)
- [**_Usage Guide_**](#usage-guide)
  - [_Running the Simulation_](#running-the-simulation)
  - [_Running on the Real Robot_](#running-on-the-real-robot)
  - [_Force calibration and force-plate tools_](#force-calibration-and-force-plate-tools)
- [**_Workspace Structure & Key Packages_**](#workspace-structure--key-packages)
- [**_Notes_**](#notes)

## System Architecture
The system uses ROS2 on a high-level computer (PC/Jetson) to communicate with a low-level FPGA driver (NI sbRIO) via gRPC.

## System Requirements & Dependencies

### Hardware
* **Main Controller:**
    * Real Robot: NVIDIA Jetson AGX Orin
    * Panel/Simulation/Dev: PC with Ubuntu 22.04
* **Low-Level Controller:**
    * NI sbRIO-9629

### Software

- **NI sbRIO-9629**

  - [**fpga_driver**](https://github.com/Yatinghsu000627/fpga_driver)
  - [**grpc_core**](https://github.com/kyle1548/grpc_core)

- **PC / Nvidia Jetson**
  - **OS**: Ubuntu 22.04
  - **ROS**: [**ROS Humble**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
  - **Simulator**: [**Webots**](https://cyberbotics.com/) (for simulation only)
  - **Python Dependencies**:
    - `pip install numpy`
    - `pip install PyQt5`
    - `pip install Jetson.GPIO`
  - **C++ Dependencies**:
    - [**Eigen 3.4.0**](https://eigen.tuxfamily.org/index.php?title=Main_Page)
    - [**yaml-cpp**](https://github.com/jbeder/yaml-cpp)
    - [**mip_sdk (v2.0.0)**](https://github.com/hiho817/mip_sdk/tree/fix-install-interface)
    - [**osqp (v0.6.3)**](https://github.com/osqp/osqp/tree/v0.6.3)
    - [**osqp-eigen**](https://github.com/robotology/osqp-eigen)

> **_CRITICAL INSTALLATION NOTE_**
>
> To avoid build errors, it is **strongly recommended** to install all C++ dependencies listed above into the `~/corgi_ws/install/` directory. If you install them elsewhere, you **must** manually update the compile commands or the paths in the `CMakeLists.txt` files.

## Installation and Build Instructions

> Docker image is created, see [Corgi_ws_-Docker_Version-
](https://github.com/BioRoLa/Corgi_ws_-Docker_Version-.git)

1.  **Create the Workspace Directory**

    Use this structure to keep all related projects organized.

    ```
    mkdir ~/corgi_ws/
    ```

2.  **Clone the Repository**

    ```
    cd ~/corgi_ws/
    git clone https://github.com/BioRoLa/corgi_ros2_ws.git
    ```

3.  **Install All Dependencies**

    Follow the list in the [**_Software_**](#software) section to install all required libraries.

    ```
    cd ~/corgi_ws/
    mkdir install/
    ```
    1. **yaml-cpp**
    Clone the `yaml-cpp` repository and follow the installation instructions:
      ```bash
      cd ~/corgi_ws/install
      git clone https://github.com/jbeder/yaml-cpp.git
      cd yaml-cpp
      mkdir build && cd build
      cmake ..
      make -j16
      sudo make install
      ```
    2. **Eigen**
    Clone the `Eigen` repository and follow the installation instructions:
      ```bash
      cd ~/corgi_ws/install
      git clone https://gitlab.com/libeigen/eigen.git
      cd eigen
      mkdir build && cd build
      cmake ..
      make -j16
      sudo make install
      ```
    3. **grpc_core**
    Install gRPC & grpc_core as follows: [**grpc_core**](https://github.com/BioRoLa/grpc_core)

    4. **osqp**
    Install osqp as follow: [**osqp**](https://osqp.org/docs/get_started/sources.html)
      ```bash
      git clone https://github.com/osqp/osqp
      cd osqp
      mkdir build && cd build
      cmake -G "Unix Makefiles" ..
      cmake --build .
      make -j16
      sudo make install
      ```
    
    5. **osqp_eigen**
    Install osqp_eigen as follows: [**osqp_eigen**](https://github.com/robotology/osqp-eigen)

    6. **MIP_SDK**
    Clone the `mip_sdk` repository and follow the installation instructions:
      ```bash
      cd corgi_ws
      git clone --branch fix-install-interface --single-branch https://github.com/hiho817/mip_sdk.git
      cd mip_sdk
      mkdir build && cd build
      cmake .. -DMIP_USE_SERIAL=ON
      make -j16
      sudo make install
      ```

    7. **pcl_ros**
    PCL (Point Cloud Library) ROS interface stack.
      ```bash
      sudo apt install ros-humble-pcl*
      ```   

    8. **joy**
    The joy package contains joy_node, a node that interfaces a generic joystick to ROS 2.
      ```bash
      sudo apt install ros-humble-joy
      ```
    
5.  **Build the ROS Workspace**

    If you installed C++ dependencies to the recommended path, use the following command.

    ```bash
    cd ~/corgi_ws/corgi_ros2_ws/
    colcon build --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install
    ```

6.  **Source the Environment (ROS 2)**

    You can also add this to your own `~/.bashrc`:

    ```bash
    source /opt/ros/humble/setup.bash
    source $HOME/corgi_ws/corgi_ros2_ws/install/setup.bash
    ```
## Conventions

**Read [`CONVENTIONS.md`](CONVENTIONS.md) before touching anything that sends or reads joint torques, forces or angles.** It defines the body frame, the joint and motor angles, the force sign, and the one place per plant where each leg module's own wiring is converted:

- robot: [`corgi_ros_bridge`](src/corgi_ros_bridge) with `config/real_motor_config.yaml`
- sim: `corgi_sim/corgi_driver_pkg/corgi_driver.py` with `motor_config.yaml`

Above that boundary every node treats the four legs the same. The conversion itself is [`corgi_utils/module_frame.hpp`](src/corgi_utils/include/corgi_utils/module_frame.hpp), with unit tests (`colcon test --packages-select corgi_utils`).

## Usage Guide

### Running the Simulation

Webots runs on Windows and the ROS side in WSL; the driver finds Windows through the first `nameserver` in `/etc/resolv.conf`.

```bash
ros2 launch corgi_sim Corgi_launch.py                     # sim + control panel
ros2 launch corgi_sim Corgi_experiment_launch.py world:=Corgi_ABAD.wbt   # headless experiments
```

The sim's plant switches are environment variables read by the driver. The driver converts torques and gains with the positions by default (`CORGI_DIRBETA_TRANSFORM=0` restores the legacy half-conversion); `CORGI_THETA_STOP=1` is part of the plant of record; `CORGI_G_JOINT_MODE` selects the foot-arc gear model (default `gear`). `Corgi_ABAD_force_plate.wbt` puts the robot on four force plates laid out like the lab's Bertecs.

### Running on the Real Robot

Bring-up order and the exact commands live in the session launch crib (vault: *Session Launch Crib*). In short: bench supply, FPGA driver on the sbRIO, `ros2 run corgi_homing homing`, `ros2 launch corgi_ros_bridge corgi_ros_bridge.launch.py`, `ros2 launch corgi_imu imu.launch.py`, then the panel (`ros2 run corgi_panel corgi_control_panel`).

The bridge refuses to start if its direction table is missing or invalid. `legacy_torque_frame:=true` restores the pre-2026-11-13 behaviour (positions converted, torques not) for comparison only.

### Force calibration and force-plate tools

In [`corgi_force_control/scripts/diag`](src/corgi_force_control/scripts/diag) (see its README):

| tool | what it does |
|---|---|
| `measure_kt.py` | a motor's true torque constant from a lever and known masses |
| `measure_friction.py` | per-motor friction from a slow beta roll, and whether the friction feedforward has the right sign |
| `rock_cycle.sh` | one scored force rock on the simulated force plates |
| `force_plate_track.py` | scores a rock: command vs estimate vs plates (sim topics or a Vicon `.c3d`) |
| `pitch_sign_check.py` | whether the estimator's per-leg pitch sign is right, from a pitched stance |

`exp_h24` (in `corgi_force_control`) stands the robot up and runs the rocks (`rock_mode:=diag_fz|pitch_fz|roll_fz|fore_fx|lateral_fy`).

## Workspace Structure & Key Packages

For more details on a package, see its own `README.md`.

- **Core**
  - [`corgi_msgs`](src/corgi_msgs): all custom messages and services.
  - [`corgi_ros_bridge`](src/corgi_ros_bridge): gRPC link between ROS and the FPGA driver; the robot's frame boundary.
  - [`corgi_utils`](src/corgi_utils): leg kinematics (`leg_model`), trajectory helpers, and the module-frame conversion.
  - [`corgi_panel`](src/corgi_panel): the PyQt5 control and configuration panel.
  - [`corgi_data_recorder`](src/corgi_data_recorder): logs topics to CSV.
  - [`corgi_homing`](src/corgi_homing): homes the legs to the reference pose.

- **Control & estimation**
  - [`corgi_force_control`](src/corgi_force_control): impedance / force control, the G-SLIP pronk controller, `exp_h24`, and the diagnostics.
  - [`corgi_force_estimation`](src/corgi_force_estimation): ground-reaction force estimate from motor torques (`/force/state`).
  - [`corgi_odometry`](src/corgi_odometry): leg odometry and state estimation.
  - [`corgi_csv_control`](src/corgi_csv_control): plays motor commands from a CSV file.
  - [`corgi_virtual_agent`](src/corgi_virtual_agent): virtual agent for testing.

- **Simulation**
  - [`corgi_sim`](src/corgi_sim) (submodule): Webots worlds, the robot proto and the driver plugin.

- **Sensors** (submodules): `fast_lio`, `livox_ros_driver2`, `zed-ros2-wrapper`.

- **Legacy** ([`src/legacy`](src/legacy)): earlier walking, stair, MPC and gait packages, kept for reference.

## Notes

Research notes, the implementation log and session runsheets live in the project's Obsidian vault, not in this repository.
