#ifndef SIMPLE_FSM_HPP
#define SIMPLE_FSM_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <thread>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"

#include "leg_model.hpp"

enum class Gait {
    WHEEL = 0,
    LEG = 1,   
    WLW = 2,   
    TRANSFORM = 3
};
class GaitSelector {
    public:
        GaitSelector(   ros::NodeHandle& nh,
                        bool sim=true,
                        double CoM_bias=0.0,
                        int pub_rate=1000,
                        double BL=0.444,
                        double BW=0.4,
                        double BH=0.2);
        ~GaitSelector();

        ros::Subscriber motor_state_sub_;
        ros::Publisher motor_cmd_pub_; 
        ros::Rate* rate_ptr;
        std::random_device rd;                     
        std::mt19937 rng;                          
        std::uniform_int_distribution<int> dist;
        /*    State or cmd messages      */ 
        corgi_msgs::MotorCmdStamped motor_cmd;
        corgi_msgs::MotorStateStamped motor_state;
        std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
            &motor_cmd.module_a,
            &motor_cmd.module_b,
            &motor_cmd.module_c,
            &motor_cmd.module_d
        };
        std::vector<corgi_msgs::MotorState*> motor_state_modules = {
            &motor_state.module_a,
            &motor_state.module_b,
            &motor_state.module_c,
            &motor_state.module_d
        };
        void motor_state_cb(const corgi_msgs::MotorStateStamped state);
        /*     Cooperate variables      */ 
        LegModel leg_model;
        int pub_rate;
        const double CoM_bias;
        const double BL;
        const double BW;
        const double BH;

        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};
        double swing_time;   // ratio
        double velocity;     // m/s
        double stand_height; // m
        double step_length;  // m
        double step_height;  // m

        double eta[4][2];
        double next_eta[4][2];
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> next_foothold;
        std::array<std::array<double, 2>, 4> body;
        std::array<std::array<double, 2>, 4> next_body;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> next_hip;

        // This thread continuously reads keyboard input and updates shared variables.
        void keyboardInputThread();
        /*     Cooperate functions      */ 
        void setCmd(std::array<double, 2> send, int index, bool dir);
        void publish(int freq);
        void Send(int freq);


    private:
        /*     Gait Selector     */ 
        Gait currentGait;
        Gait newGait;
        // Global shared variables and a mutex for thread safety
        std::mutex input_mutex;
        void changeGait(const std::string& command);
        void printCurrentGait() const;
        void Transform();

        
        };
#endif

// gaitselector setup add nodehandler and all initialize
// CoM_bias to 2D array(current 1D)