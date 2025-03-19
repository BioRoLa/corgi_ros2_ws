#ifndef CORGI_MPC_HPP
#define CORGI_MPC_HPP

#include <iostream>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"

#include "corgi_msgs/SimDataStamped.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/ForceStateStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "walk_gait.hpp"
#include "bezier.hpp"

bool sim = true;
LegModel legmodel(sim);

// Constants
const int m = 22;
const double gravity = 9.81;
const double dt = 0.01;
const int N = 10;
const int n_x = 12;
const int n_u = 12;

double roll = 0;
double pitch = 0;
double yaw = 0;

double robot_vel[3] = {0, 0, 0};
double robot_pos[3] = {0, 0, 0};

#endif