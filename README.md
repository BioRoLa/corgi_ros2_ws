# Corgi Robot ROS Workspace

This repository contains the ROS workspace for Corgi quadruped robot in BioRoLa at NTU.

## Table of Contents

- [**About The Project**](#about-the-project)
- [**Prerequisites**](#prerequisites)
- [**Project Structure**](#project-structure)
- [**Getting Started**](#getting-started)
- [**Usage**](#usage)
- [**Notes**](#notes)


## About The Project

Refers to R12 Master's Thesis.

## Prerequisites

### NI sbRIO-9629
- [**fpga_driver**](https://github.com/Yatinghsu000627/fpga_driver)
- [**grpc_core**](https://github.com/kyle1548/grpc_core)

### PC (Simulation) / Nvidia Jetson AGX Orin (Real Robot)
- [**ROS Noetic**](https://wiki.ros.org/noetic/Installation/Ubuntu) (Including ```catkin``` tools)
- (Simulation) [**Webots**](https://cyberbotics.com/)
- (Python) [**NumPy**](https://pypi.org/project/numpy/) (```pip install numpy```)
- (Python) [**PyQt5**](https://pypi.org/project/PyQt5/) (```pip install PyQt5```)
- (Python) [**Jetson.GPIO**](https://github.com/NVIDIA/jetson-gpio) (```pip install Jetson.GPIO```)
- (C++) [**Eigen 3.4.0**](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- (C++) [**yamp-cpp**](https://github.com/jbeder/yaml-cpp)
- (C++) [**mip_sdk**](https://github.com/LORD-MicroStrain/mip_sdk/tree/v2.0.0) (CMakeLists.txt:241 => modify ```src``` to ```include```)
- (C++) [**osqp v0.6.3**](https://github.com/osqp/osqp/tree/v0.6.3)
- (C++) [**osqp-eigen**](https://github.com/robotology/osqp-eigen)

***Notes:*** Recommand to install all the C++ packages in ```~/corgi_ws/install/```, or you have to change ROS compile commands (will be mentioned later).

## Project Structure

The following packages in ```src/``` directory are basic modules of the main system.

```
corgi_ros_ws/
├── input_csv/
├── output_data/
└── src/
    ├── corgi_ros_bridge     # Establish connection between ROS and gRPC
    ├── corgi_panel          # The Panel for power and motor control
    ├── corgi_data_recorder  # Subscribe and record data to csv files
    ├── corgi_csv_control    # Publish motor commands to drive the robot
    ├── corgi_msgs           # Define all needed ROS message types
    ├── corgi_sim            # The simulation environment in Webots
    └── ... (Other Packages)
```

Recommanded Environment Setup:

```
~/corgi_ws/
├── corgi_ros_ws/
├── grpc_core/
└── install/
```

## Getting Started



## Usage



## Notes








