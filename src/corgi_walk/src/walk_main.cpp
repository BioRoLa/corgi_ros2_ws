#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include <array>
#include <string>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "walk_gait.hpp"
#include "leg_model.hpp"
#include "bezier.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    for (int i=0; i<4; i++) {
<<<<<<< HEAD
        motor_cmd_modules[i]->kp = 90;
        motor_cmd_modules[i]->ki = 0;
        motor_cmd_modules[i]->kd = 1.75;
=======
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            relative_foothold[i][0] = leg_model.U_l[0];
        } else if (current_rim == 2) {
            relative_foothold[i][0] = leg_model.L_l[0];
        } else if (current_rim == 3) {
            relative_foothold[i][0] = leg_model.G[0];
        } else if (current_rim == 4) {
            relative_foothold[i][0] = leg_model.L_r[0];
        } else if (current_rim == 5) {
            relative_foothold[i][0] = leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        relative_foothold[i][1] = -stand_height;
    }//end for
    // Get initial leg duty  
    int first_swing_leg = 0;
    for (int i=1; i<4; i++) {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }//end if
    }//end for 
    std::array<double, 4> duty;
    if (!use_init_conf || first_swing_leg == 0) {
        duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
    } else if (first_swing_leg == 1) {
        duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
    } else if (first_swing_leg == 2) {
        duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
    } else if (first_swing_leg == 3) {
        duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
    }//end if else
    // Get foothold in world coordinate
    double hip[4][2] = {{BL/2, stand_height} ,
                        {BL/2, stand_height} ,
                        {-BL/2, stand_height},
                        {-BL/2, stand_height}};
    std::array<std::array<double, 2>, 4> foothold;
    // initial leg configuration
    if (use_init_conf) { 
        foothold = {{{hip[0][0] + relative_foothold[0][0], hip[0][1] + relative_foothold[0][1]},   
                     {hip[1][0] + relative_foothold[1][0], hip[1][1] + relative_foothold[1][1]},
                     {hip[2][0] + relative_foothold[2][0], hip[2][1] + relative_foothold[2][1]},
                     {hip[3][0] + relative_foothold[3][0], hip[3][1] + relative_foothold[3][1]}}};
    } else {
        foothold = {{{hip[0][0] - step_length/2*(1-swing_time), hip[0][1] - stand_height},   
                     {hip[1][0] + step_length/8*(1-swing_time), hip[1][1] - stand_height},
                     {hip[2][0] - step_length/8*(1-swing_time), hip[2][1] - stand_height},
                     {hip[3][0] + step_length/2*(1-swing_time), hip[3][1] - stand_height}}};
    }//end if else

    // Initial stored data
    std::array<double, 4> current_theta;
    std::array<double, 4> current_beta;
    std::array<double, 4> next_theta;
    std::array<double, 4> next_beta;
    double dS = velocity / sampling;
    double incre_duty = dS / step_length;
    double traveled_distance = 0.0;
    std::vector<SwingProfile> sp(4);
    std::string touch_rim_list[3] = {"G", "L_l", "U_l"};
    int touch_rim_idx[3] = {3, 2, 1};

    // Initial teata, beta
    std::array<double, 2> result_eta;
    std::string rim_list[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
    int rim_idx[5] = {3, 2, 4, 1, 5};
    double contact_height_list[5] = {leg_model.r, leg_model.radius, leg_model.radius, leg_model.radius, leg_model.radius};
    for (int i=0; i<4; i++) {
        // calculate contact rim of initial pose
        for (int j=0; j<5; j++) {
            double contact_point[2] = {foothold[i][0] - hip[i][0], foothold[i][1] - hip[i][1] + contact_height_list[j]};
            result_eta = leg_model.inverse(contact_point, rim_list[j]);
            leg_model.contact_map(result_eta[0], result_eta[1]);
            if (leg_model.rim == rim_idx[j])
                break;
        }
        current_theta[i] = result_eta[0];
        current_beta[i]  = result_eta[1];
    }//end for
    // Check and update theta, beta
    for (int i=0; i<4; i++) {
        if (current_theta[i] > M_PI*160.0/180.0) {
            std::cout << "Exceed upper bound." << std::endl;
        }//end if 
        if (current_theta[i] < M_PI*17.0/180.0) {
            std::cout << "Exceed lower bound." << std::endl;
        }//end if 
        motor_cmd_modules[i]->theta = current_theta[i];
        if (i==1 || i==2) {
            motor_cmd_modules[i]->beta  = current_beta[i];
        } else {
            motor_cmd_modules[i]->beta  = -current_beta[i];
        }
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kd_l = 1.75;
>>>>>>> dev
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    double CoM_bias = 0.0;
    int sampling_rate = 1000;
    ros::Rate rate(sampling_rate);
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    WalkGait walk_gait(true, CoM_bias, sampling_rate);
    walk_gait.initialize(init_eta);
    std::array<std::array<double, 4>, 2> eta_list;
    double velocity     = 0.1;
    double stand_height = 0.2;
    double step_length  = 0.3;
    double step_height  = 0.05;
    double curvature = 0.0;
    int count = 0;

    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
    // for (int count=0; count<200000; count++){
        velocity = 0.2*cos((count++)/1711.0);
        walk_gait.set_velocity(velocity);
        stand_height = 0.25 + 0.05*cos((count++)/1211.0);
        walk_gait.set_stand_height(stand_height);
        step_length = (count++/3311)%2 == 0? 0.3 : 0.1;
        walk_gait.set_step_length(step_length);
        step_height = (count++/2311)%2 == 0? 0.08 : 0.04;
        walk_gait.set_step_height(step_height);
        curvature = 1.0*sin((count++)/3911.0);
        walk_gait.set_curvature(curvature);
        eta_list = walk_gait.step();
        // Publish motor commands
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Exceed upper bound." << std::endl;
            }//end if 
            if (eta_list[0][i] < M_PI*17.0/180.0) {
                std::cout << "Exceed lower bound." << std::endl;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
        }
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    
    ros::shutdown();
    return 0;
}//end main
