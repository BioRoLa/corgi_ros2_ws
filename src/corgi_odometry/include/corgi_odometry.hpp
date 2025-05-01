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
inline constexpr double MOTOR_OFFSET_X = 0.222;   // [m]
inline constexpr double MOTOR_OFFSET_Y = 0.193;   // [m]
inline constexpr double MOTOR_OFFSET_Z = 0.0;     // [m]

inline constexpr double WHEEL_RADIUS = 0.10;     // [m]
inline constexpr double WHEEL_WIDTH_SIM  = 0.012; // [m]
inline constexpr double WHEEL_WIDTH_REAL = 0.019; // [m]
inline constexpr double WHEEL_WIDTH = SimMode ? WHEEL_WIDTH_SIM : WHEEL_WIDTH_REAL;

inline constexpr double GRAVITY        = 9.80665; // [m/s²]