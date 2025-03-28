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

int freq = 100;

const int m = 19;
const double gravity = 9.81;
const double dt = 1.0/(double)freq;
const int N = 10;
const int n_x = 12;
const int n_u = 12;

Eigen::MatrixXd A(n_x, n_x);
Eigen::MatrixXd B(n_x, n_u);
Eigen::MatrixXd Q(n_x, n_x);
Eigen::MatrixXd R(n_u, n_u);

double roll = 0;
double pitch = 0;
double yaw = 0;

double robot_vel[3] = {0, 0, 0};
double robot_pos[3] = {0, 0, 0};
double robot_ang_vel[3] = {0, 0, 0};
Eigen::Quaterniond robot_ang;

int fx_upper_bound = 50;
int fx_lower_bound = -50;
int fz_upper_bound = 150;
int fz_lower_bound = -150;

double friction_coef = 1.0;

double Mx = 0;
double My = 0;
double Bx = 30;
double By = 30;
double Kx_transform = 2500;
double Ky_transform = 3000;
double Kx_run = 2500;
double Ky_run = 1000;

std::array<std::array<double, 4>, 2> eta_list;

#endif