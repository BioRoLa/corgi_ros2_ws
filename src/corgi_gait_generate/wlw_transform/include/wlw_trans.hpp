#ifndef WLW_TRANS_HPP
#define WLW_TRANS_HPP


#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "bezier.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <utility>

const double PI = M_PI;
double deg2rad(double degrees);
double rad2deg(double radians);


double body_vel      = 0.05;     
double dt            = 0.001;   
double body_length   = 0.444;  
double stance_height = 0.172; 

double find_closest_beta(double target_beta, double ref_beta);
double find_smaller_closest_beta(double target_beta, double ref_beta);
std::pair<int, double> find_hybrid_step(double RH_beta,
                                        double LH_beta,
                                        double body_angle,
                                        double radius);
void wlw_transform_main();

#endif 