#ifndef HYBRID_GEN_HPP
#define HYBRID_GEN_HPP

#include "Simple_fsm.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include <corgi_msgs/MotorState.h>
#include <corgi_msgs/MotorStateStamped.h>
#include <corgi_msgs/MotorCmd.h>
#include <corgi_msgs/MotorCmdStamped.h>

#include "leg_model.hpp"

class Hybrid: public GaitSelector
{   
    public:
        Hybrid(ros::NodeHandle& nh);
        ~Hybrid()= default;

        void Initialize(int swing_index, int pub_time, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift); 
        std::array<double, 2> find_pose(double height, float shift, float steplength, double slope);
        void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg);
        void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio);
        void Step(int pub_time, int do_pub, double shift);
        void change_Height(double new_value);
        void change_Step_length(double new_value);
        void change_Velocity(double new_value);

        std::array<double, 2> swing_pose;
        std::array<double, 2> swing_variation;
    // private:
        
        
        
};

#endif