#pragma once
/******* std lib *******/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry>
#include <algorithm>
#include "ros/ros.h"

/******* ROS msg *******/

#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/ContactStateStamped.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float64.h>

/******* Leg kinematic *******/ 
#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

/******* filter *******/  
#include "KLD_estimation/csv_reader.hpp"
#include "KLD_estimation/InformationFilter.hpp"

/*******  simulation switch *******/ 
inline constexpr bool SimMode = true; // true → simulation, false → real robot

/******* mechanism *******/ 
inline constexpr double MotorOffsetX = 0.222;   // [m]
inline constexpr double MotorOffsetY = 0.193;   // [m]
inline constexpr double MotorOffsetZ = 0.0;     // [m]

inline constexpr double WheelRadius = 0.10;     // [m]
inline constexpr double WheelWidthSim  = 0.012; // [m]
inline constexpr double WheelWidthReal = 0.019; // [m]
inline constexpr double kWheelWidth = kSimMode ? kWheelWidthSim : kWheelWidthReal;

inline constexpr double Gravity        = 9.80665; // [m/s²]