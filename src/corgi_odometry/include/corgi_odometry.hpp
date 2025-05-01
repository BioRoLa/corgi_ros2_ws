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