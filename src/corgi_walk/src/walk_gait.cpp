#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "walk_gait.hpp"
#include "bezier.hpp"

#include <array>
#include <string>
#include <fstream>

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "walk_pub");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    ros::Rate rate(1000);

    bool sim = true;
    LegModel leg_model(sim);

    // User-defined parameters
    double BL = 0.444;
    double BH = 0.2;
    double CoM_bias = 0.0;
    double velocity = 0.1;
    int sampling = 1000;
    double stand_height = 0.2;
    double step_length = 0.4;
    double step_height = 0.06;
    double forward_distance = 2.0;

    double init_eta[8] = {1.7908786895256839,0.7368824288764617,1.1794001564068406,-0.07401410141135822,1.1744876957173913,-1.8344700758454735e-15,1.790992783013031,5.5466991499313485};
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4] = {init_eta[1], init_eta[3], init_eta[5], init_eta[7]};
    init_beta[0] *= -1.0;
    init_beta[3] *= -1.0;

    bool use_init_conf = false;

    /* Dependent parameters */
    double swing_time = 0.2;
    int swing_phase[4] = {0, 0, 0, 0};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    for (int i=0; i<4; i++) {
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
    std::array<std::array<double, 2>, 4> next_hip;
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
        motor_cmd_modules[i]->kp = 90;
        motor_cmd_modules[i]->ki = 0;
        motor_cmd_modules[i]->kd = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
        std::cout << "current_theta " << i << ": "<< current_theta[i]*180.0/M_PI << std::endl;
        std::cout << "current_beta " << i << ": "<< current_beta[i]*180.0/M_PI << std::endl;
    }//end for
    for (int i=0; i<1000; i++) {
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }
    
    // Start walking
    while (traveled_distance <= forward_distance) {
        for (int i=0; i<4; i++) {
            if (swing_phase[i] == 0) { // Stance phase
                result_eta = leg_model.move(current_theta[i], current_beta[i], {dS, 0});
            } else { // Swing phase
                double swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
                // Placeholder swing profile calculation
                std::array<double, 2>  temp = sp[i].getFootendPoint(swing_phase_ratio);
                double curve_point[2] = {temp[0]-hip[i][0], temp[1]-hip[i][1]};
                result_eta = leg_model.inverse(curve_point, "G");
            }//end if else
            next_theta[i] = result_eta[0];
            next_beta[i] = result_eta[1];

            duty[i] += incre_duty;
            if (duty[i] >= (1 - swing_time) && swing_phase[i] == 0) {
                swing_phase[i] = 1;
                foothold[i] = {hip[i][0] + ((1-swing_time)/2+swing_time)*step_length, 0};
                // Bezier curve for swing phase
                leg_model.forward(next_theta[i], next_beta[i]);
                std::array<double, 2> p_lo[2] = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};  // G position when leave ground
                // calculate contact rim when touch ground
                for (int j=0; j<3; j++) {   // G, L_l, U_l
                    double contact_point[2] = {step_length/2*(1-swing_time), -stand_height+contact_height_list[j]};
                    result_eta = leg_model.inverse(contact_point, touch_rim_list[j]);
                    leg_model.contact_map(result_eta[0], result_eta[1]);
                    if (leg_model.rim == touch_rim_idx[j]) {
                        current_rim = leg_model.rim;
                        break;
                    }//end if
                }//end for
                // G position when touch ground
                leg_model.forward(result_eta[0], result_eta[1]);
                std::array<double, 2> p_td;
                if (current_rim == 3) {   // G
                    p_td = {foothold[i][0], foothold[i][1] + leg_model.r};
                } else if (current_rim == 2) {  // L_l
                    p_td = {foothold[i][0] + leg_model.G[0]-leg_model.L_l[0], foothold[i][1] + leg_model.G[1]-leg_model.L_l[1] + leg_model.radius};
                } else if (current_rim == 1) {  // U_l
                    p_td = {foothold[i][0] + leg_model.G[0]-leg_model.U_l[0], foothold[i][1] + leg_model.G[1]-leg_model.U_l[1] + leg_model.radius};
                }//end if else
                sp[i] = SwingProfile(p_lo, p_td, step_height);
            } else if (duty[i] >= 1.0) {
                swing_phase[i] = 0;
                duty[i] -= 1.0;
            }//end if else

            hip[i][0] += dS;
        }//end for
        traveled_distance += dS;

        // Check and update theta, beta
        for (int i=0; i<4; i++) {
            if (next_theta[i] > M_PI*160.0/180.0) {
                std::cout << "Exceed upper bound." << std::endl;
            }//end if 
            if (next_theta[i] < M_PI*17.0/180.0) {
                std::cout << "Exceed lower bound." << std::endl;
            }//end if 
            motor_cmd_modules[i]->theta = next_theta[i];
            if (i==1 || i==2) {
                motor_cmd_modules[i]->beta  = next_beta[i];
            } else {
                motor_cmd_modules[i]->beta  = -next_beta[i];
            }
            current_theta[i] = next_theta[i];
            current_beta[i]  = next_beta[i];
            std::cout << "current_theta " << i << ": "<< current_theta[i]*180.0/M_PI << std::endl;
            std::cout << "current_beta " << i << ": "<< current_beta[i]*180.0/M_PI << std::endl;
        }//end for
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }//end while

    ros::shutdown();
    return 0;
}//end main
