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
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"

#include "leg_model.hpp"

enum class Gait {
    WHEELED = 0,
    LEGGED = 1,   
    HYBRID = 2,   
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
        /*     Gait Selector     */ 
        Gait currentGait;
        Gait newGait; 
        
        ros::Subscriber motor_state_sub_;
        ros::Publisher motor_cmd_pub_; 
        ros::Publisher marker_pub_;
        ros::Rate* rate_ptr;
        std::random_device rd;                     
        std::mt19937 rng;                          
        std::uniform_int_distribution<int> dist;
        /*    State or cmd messages      */ 
        corgi_msgs::MotorCmdStamped motor_cmd;
        static corgi_msgs::MotorStateStamped motor_state;
        std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
            &motor_cmd.module_a,
            &motor_cmd.module_b,
            &motor_cmd.module_c,
            &motor_cmd.module_d
        };
        static std::vector<corgi_msgs::MotorState*> motor_state_modules;
        void motor_state_cb(const corgi_msgs::MotorStateStamped state);
        /*     Cooperate variables      */ 
        LegModel leg_model;
        int pub_rate;
        const double CoM_bias;
        const double BL;
        const double BW;
        const double BH;

        static std::array<double, 4> duty;
        static std::array<int, 4> swing_phase;
        static double swing_time;   // ratio
        static double velocity;     // m/s 
        static double stand_height; // m
        static double step_length;  // m
        static double step_height;  // m

        static std::array<double, 4> current_step_length;
        static std::array<double, 4> next_step_length;
        static double new_step_length ;

        static std::array<double, 4> current_shift;

        static double curvature;

        double dS;
        double incre_duty;

        static int pub_time;
        static int do_pub;
        static int transfer_state;
        static int transfer_sec;
        static int wait_sec;

        static double relative_foothold[4][2];
        static double eta[4][2];
        static double next_eta[4][2];
        static std::array<std::array<double, 3>, 4> foothold;
        static std::array<std::array<double, 3>, 4> next_foothold;
        static std::array<double, 3> body;
        static std::array<double, 3> next_body;
        static std::array<std::array<double, 3>, 4> hip;
        static std::array<std::array<double, 3>, 4> next_hip;

        // For turning 
        static int direction;
        static double outer_radius;
        static double inner_radius;
        static double diff_step_length;  // Differential step length 
        static double new_diff_step_length;  // New differential step length
        static double diff_dS;   // Differential dS
        static int sign_diff[4];   // Differential sign

        
        /*     Cooperate functions      */ 
        void setCmd(std::array<double, 2> send, int index, bool dir);
        void publish(int freq);
        void Send(int freq);
        void Transfer(int pub, int transfer_sec, int wait_sec);
        void Receive();
        // void changeGait(const std::string& command);
        // void process_and_visualize();
        // void publish_support_polygon(
        //     const std::vector<Eigen::Vector3d>& pts,
        //     const std::vector<int>& indices
        // );
        // void publish_com_marker();
        // bool is_point_inside_polygon(
        //     const std::vector<Eigen::Vector3d>& points,
        //     const std::vector<int>& indices,
        //     const Eigen::Vector2d& point_xy
        // ) ;
    private:
        // void printCurrentGait() const;
        // void GaitTransform();
        std::vector<double> linspace(double start, double end, int num_steps);
};



#endif

// gaitselector setup add nodehandler and all initialize
// CoM_bias to 2D array(current 1D)