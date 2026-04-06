# Corgi Robot ROS 2 Workspace

![ROS Version](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-yellowgreen)

This is the central ROS 2 workspace for the Corgi quadruped robot, developed at the Bio-Inspired Robotics Laboratory (BioRoLa), NTU. It contains all necessary packages for control, simulation, and hardware interfacing.

<!-- <p align="center">
  <img src="[INSERT_IMAGE_OR_GIF_URL_HERE]" alt="Corgi Robot Demo" width="600"/>
</p> -->

## Table of Contents

- [Corgi Robot ROS 2 Workspace](#corgi-robot-ros-2-workspace)
  - [Table of Contents](#table-of-contents)
  - [System Architecture](#system-architecture)
  - [System Requirements \& Dependencies](#system-requirements--dependencies)
    - [Hardware](#hardware)
    - [Software](#software)
  - [Installation and Build Instructions](#installation-and-build-instructions)
  - [Usage Guide](#usage-guide)
    - [Running on the Real Robot](#running-on-the-real-robot)
  - [Workspace Structure \& Key Packages](#workspace-structure--key-packages)
    - [Key Packages in `src/`](#key-packages-in-src)

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
  - [**grpc_core**](https://github.com/BioRoLa/grpc_core)

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

> **_NOTE_**
>
> All C++ dependencies are installed system-wide via `sudo make install` (to `/usr/local/`), except for `grpc_core` and `gRPC`, which are installed in `~/corgi_ws/install/`.

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

    4. **[osqp](https://osqp.org/docs/get_started/sources.html)**
      ```bash
      git clone https://github.com/osqp/osqp
      cd osqp
      mkdir build && cd build
      cmake -G "Unix Makefiles" ..
      cmake --build .
      make -j16
      sudo make install
      ```
    
    5. **[osqp_eigen](https://github.com/robotology/osqp-eigen)**
      ```bash
      git clone https://github.com/gbionics/osqp-eigen.git
      cd osqp-eigen
      mkdir build
      cd build
      cmake ..
      make
      make install
      ```

    6. **[MIP_SDK](https://github.com/hiho817/mip_sdk.git)**
    
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

    9.  **Livox-SDK2 (required by livox_ros_driver2)**
    Install Livox-SDK2 to provide `/usr/local/lib/liblivox_lidar_sdk_shared.so`:
      ```bash
      cd ~/corgi_ws/
      git clone https://github.com/Livox-SDK/Livox-SDK2.git
      cd Livox-SDK2
      mkdir -p build && cd build
      cmake ..
      make -j$(nproc)
      sudo make install
      ```
    
4.  **Build the ROS Workspace**

    Per-package CMake arguments (e.g., `LOCAL_PACKAGE_PATH`, `ROS_EDITION`, `yaml-cpp_DIR`) are configured in [`colcon.meta`](colcon.meta), so you only need:

    ```bash
    cd ~/corgi_ws/corgi_ros2_ws/
    source /opt/ros/humble/setup.bash
    colcon build
    ```

    > **Note:** If you installed C++ dependencies to a non-default path, update the paths in `colcon.meta` accordingly.

    - Do **not** run `src/livox_ros_driver2/build.sh` inside this combined workspace. That script deletes `build/` and `install/` which affects all packages.

5.  **Source the Environment (ROS 2)**

    You can also add this to your own `~/.bashrc`:

    ```bash
    source /opt/ros/humble/setup.bash
    source $HOME/corgi_ws/corgi_ros2_ws/install/setup.bash
    ```

## Usage Guide

### Running on the Real Robot

<!-- **Safety First:** Always have the robot on a stand before powering the motors. -->

1.  **Start Low-Level Driver**

    Before anything else, ensure the **`fpga_driver`** is running on the NI sbRIO.

2.  **Launch ROS Control Panel**

    On the Jetson/PC, run the main launch file. This will start the **_panel_**, **_data recorder_**, **_force estimation_**, and **_IMU_** nodes.

    ```
    ros2 launch corgi_panel corgi_control_panel_dev.launch
    ```

3.  **Control Panel Operation Sequence** 

## Workspace Structure & Key Packages

### Key Packages in `src/`

For more details on a specific package, please see its respective `README.md` file.

- **Core**
  - [`corgi_msgs`](src/corgi_msgs): Defines all custom ROS messages used in the project.
  - [`corgi_ros_bridge`](src/corgi_ros_bridge): The gRPC client/server that connects ROS to the FPGA driver.
  - [`corgi_data_recorder`](src/corgi_data_recorder): A flexible node to subscribe to topics and log data to CSV.
  - [`corgi_panel`](src/corgi_panel): The PyQt5-based GUI for robot control and monitoring.
  - [`corgi_utils`](src/corgi_utils): Shared utility functions, constants, and helper classes.

- **Sensing & Estimation**

  - [`corgi_imu`](src/corgi_imu): Driver and interface for the LORD MicroStrain IMU.
  - [`corgi_odometry_legacy`](src/corgi_odometry_legacy): Legacy odometry estimation node.
  - [`corgi_force_estimation`](src/corgi_force_estimation): Estimates contact forces and ground reaction forces during locomotion.
  - [`livox_ros_driver2`](src/livox_ros_driver2): ROS 2 driver for Livox MID-360 LiDAR (fork of [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)).

- **Control & Planning**
  - [`corgi_mpc`](src/corgi_mpc): Model predictive control solver for gait optimization.
  - [`corgi_force_control`](src/corgi_force_control): Impedance and force control for FPGA low-level commands.
  - [`corgi_walk`](src/corgi_walk): General legged locomotion and gait planning.
  - [`corgi_stair`](src/corgi_stair): Algorithms specifically for stair climbing locomotion.
  - [`corgi_gait_generate`](src/corgi_gait_generate): Procedural gait pattern generation.
  - [`corgi_gait_selector`](src/corgi_gait_selector): Selects the appropriate gait based on robot state or command.
  - [`corgi_csv_control`](src/corgi_csv_control): Publishes motor commands from a pre-defined CSV file.
  - [`corgi_set_zero`](src/corgi_set_zero): Adjust all motors to the standard zero position.

- **Simulation**
  - [`corgi_sim`](src/corgi_sim): Webots simulator integration for virtual robot testing and validation.
