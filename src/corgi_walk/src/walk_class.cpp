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

class WalkGait {
    public:
        WalkGait(double init_eta[8], bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2) : 
            leg_model(sim),
            CoM_bias(CoM_bias),
            rate(rate), 
            BL(BL),
            BW(BW),
            BH(BH)
        {
            double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
            double init_beta[4] = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};

            dS = velocity / rate;
            incre_duty = dS / step_length;
            initialize(init_theta, init_beta);
        }//end WalkGait

        void initialize(double init_theta[4], double init_beta[4]) {
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
            if (first_swing_leg == 0) {
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
            // Initial leg configuration
            for (int i=0; i<4; i++) {
                foothold[i] = {hip[i][0] + relative_foothold[i][0] + CoM_bias, hip[i][1] + relative_foothold[i][1]};
            }//end for
        }//end initialize

        std::array<std::array<double, 4>, 2> step() {
            for (int i=0; i<4; i++) {
                if (swing_phase[i] == 0) { // Stance phase
                    result_eta = leg_model.move(current_theta[i], current_beta[i], {dS, 0});
                } else { // Swing phase
                    swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
                    curve_point_temp = sp[i].getFootendPoint(swing_phase_ratio);
                    double curve_point[2] = {curve_point_temp[0] - hip[i][0], curve_point_temp[1] - hip[i][1]};
                    result_eta = leg_model.inverse(curve_point, "G");
                }//end if else
                next_theta[i] = result_eta[0];
                next_beta[i] = result_eta[1];

                duty[i] += incre_duty;
                if (duty[i] >= (1 - swing_time) && swing_phase[i] == 0) {
                    swing_phase[i] = 1;
                    foothold[i] = {hip[i][0] + ((1-swing_time)/2+swing_time)*step_length, 0};
                    // Bezier curve setup
                    leg_model.forward(next_theta[i], next_beta[i]);
                    p_lo = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
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
            return {next_theta, next_beta};
        }//end step

        // void set_velocity(double new_vel);
        // void set_stand_height(double new_h);
        // void set_step_length(double new_len);
        // void set_step_height(double new_h);

    private:
        LegModel leg_model;

        // constant value
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const double CoM_bias;
        const double swing_time = 0.2;

        // variable
        int rate;
        double dS;
        double incre_duty;
        double velocity     = 0.1;
        double stand_height = 0.2;
        double step_length  = 0.3;
        double step_height  = 0.05;

        // state
        std::array<double, 4> current_theta;
        std::array<double, 4> current_beta;
        std::array<double, 4> next_theta;
        std::array<double, 4> next_beta;
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> hip;

        std::array<SwingProfile, 4> sp;
        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};

        // Intermediate variables
        int current_rim;
        std::string touch_rim_list[3] = {"G", "L_l", "U_l"};
        int touch_rim_idx[3] = {3, 2, 1};
        double contact_height_list[5] = {leg_model.r, leg_model.radius, leg_model.radius, leg_model.radius, leg_model.radius};
        double swing_phase_ratio;
        std::array<double, 2> curve_point_temp;
        std::array<double, 2> result_eta;
        std::array<double, 2> p_lo;
        std::array<double, 2> p_td;
};//end class WalkGait

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
        motor_cmd_modules[i]->kp = 90;
        motor_cmd_modules[i]->ki = 0;
        motor_cmd_modules[i]->kd = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    double CoM_bias = 0.0;
    int sampling_rate = 1000;
    ros::Rate rate(sampling_rate);
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    WalkGait walk_gait(init_eta, true, CoM_bias, sampling_rate);

    std::array<std::array<double, 4>, 2> eta_list;
    while (ros::ok()) {
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

    ros::shutdown();
    return 0;
}//end main
