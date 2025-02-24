// test2.hpp
#ifndef TEST2_HPP
#define TEST2_HPP

#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <iomanip>
#include "nlopt.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"

#include "leg_model.hpp"
#include "bezier.hpp"
#include "wlw.hpp"

class WLWGait {
public:
    WLWGait(ros::NodeHandle& nh,
            bool sim=true,
            double CoM_bias=0.0,
            int pub_rate=1000,
            double BL=0.444,
            double BW=0.4,
            double BH=0.2);
    ~WLWGait();
    void Initialize(int index, int pub_time, int swing_index, int do_pub);
    void setCmd(std::array<double, 2> send, int index, bool dir);
    void publish(int freq);
    std::array<double, 2> find_pose(double height, float shift, float steplength);
    void Send(int freq);
    void Step(int pub_time, int do_pub);

private:
    LegModel leg_model;
    const double CoM_bias;
    const double BL;
    const double BW;
    const double BH;
    const double swing_time = 0.2;

    int pub_rate;
    double dS;
    double incre_duty;
    double velocity     = 0.15; // m/s
    double stand_height = 0.159;
    double step_length  = 0.3;

    double eta[4][2];
    std::array<std::array<double, 2>, 4> foothold;
    std::array<std::array<double, 2>, 4> hip;
    std::array<std::array<double, 2>, 4> next_hip;
    std::array<double, 4> duty;
    std::array<int, 4> swing_phase = {0, 0, 0, 0};

    ros::Publisher motor_pub;
    ros::Rate* rate_ptr;

    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
};

#endif
