#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "walk_gait.hpp"
#include "leg_model.hpp"
#include "bezier.hpp"

#define INIT_THETA M_PI*17.0/180.0
#define INIT_BETA 0.0

int main(int argc, char** argv) {
    ros::init(argc, argv, "stair_climb");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    // pid command
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.0;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.0;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    /* Setting variable */
    enum STATES {INIT, IDLE, TRANSFORM, WAIT, WALK, STAIR, END};
    const double CoM_bias = 0.0;
    const int sampling_rate = 1000;
    const int transform_count = 5000; // 5s
    ros::Rate rate(sampling_rate);
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    
    
    /* Initial variable */
    WalkGait walk_gait(true, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA, INIT_BETA, INIT_BETA, INIT_BETA}}};   // init eta (wheel mode)
    
    /* Other variable */
    STATES state = INIT;
    double transform_ratio;
    bool trigger;
    int count;

    /* Behavior loop */
    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
        // state machine
        switch (state) {
            case INIT:
                transform_ratio = 0.0;
                trigger = false;
                count = 0;
                break;
            case IDLE:
                state = WALK;
                break;
            case TRANSFORM:
                transform_ratio += 1.0 / transform_count;
                for (int i=0; i<4; i++) {
                    eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i*2]   - INIT_THETA);
                    eta_list[1][i] = INIT_BETA  + transform_ratio * (init_eta[i*2+1] - INIT_BETA);
                }//end for
                break;
            case WAIT:
                walk_gait.initialize(init_eta);
                trigger = true;
                break;
            case WALK:
                eta_list = walk_gait.step();
                count ++;
                break;
            case STAIR:
                break;
            default:
                break;
        }//end switch

        // next state
        switch (state) {
            case INIT:
                state = TRANSFORM;
                break;
            case IDLE:
                state = WALK;
                break;
            case TRANSFORM:
                if (transform_ratio > 1.0) {
                    state = WAIT;
                }//end if
                break;
            case WAIT:
                if (trigger) {
                    state = WALK;
                }//end if
                break;
            case WALK:
                if (count > 10000) {
                    state = END;
                }//end if
                break;
            case STAIR:
                break;
            default:
                break;
        }//end switch

        /* Publish motor commands */
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI*160.0/180.0;
            }//end if 
            if (eta_list[0][i] < M_PI*16.99/180.0) {
                std::cout << "Exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI*16.99/180.0;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? eta_list[1][i] : -eta_list[1][i];
        }//end for
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }//end while
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    
    ros::shutdown();
    return 0;
}//end main
