#ifndef WLW_HPP
#define WLW_HPP

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "leg_model.hpp"

using namespace std;
const double PI = M_PI;
double walking_distance, step_length, velocity, tilt_angle_deg, tilt_angle_rad;
vector<pair<double, double>> terrain_positions;

double deg2rad(double degrees);
double rad2deg(double radians);
void genTerrainPosition(double walking_distance, double tilt_angle_rad, vector<pair<double, double>> &terrain_positions);
void setting(double &walking_distance, double &step_length, double &velocity, double &tilt_angle_deg, double &tilt_angle_rad);
void setting_calculation(double walking_distance, double step_length, double tilt_angle_rad, const vector<pair<double, double>> &terrain_positions, bool output_details);
void findMinMaxHeight(const vector<pair<double, double>> &terrain_positions);
#endif
