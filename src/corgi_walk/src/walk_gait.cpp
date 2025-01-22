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

WalkGait::WalkGait(double init_eta[8], bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
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
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i] = init_beta[i];
    }//end for
}//end WalkGait

void WalkGait::initialize(double init_theta[4], double init_beta[4]) {
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
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;
    // Initial leg configuration
    for (int i=0; i<4; i++) {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};
    }//end for
}//end initialize

std::array<std::array<double, 4>, 2> WalkGait::step() {
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS;
        if (swing_phase[i] == 0) { // Stance phase
            result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0]-hip[i][0], next_hip[i][1]-hip[i][1]});
        } else { // Swing phase
            if ( sp[i].getDirection()==1 ) {    // direction == 1
                swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
            } else {    // direction == -1
                swing_phase_ratio = (1.0 - duty[i]) / swing_time;
            }//end if else
            curve_point_temp = sp[i].getFootendPoint(swing_phase_ratio);
            double curve_point[2] = {curve_point_temp[0] - next_hip[i][0], curve_point_temp[1] - next_hip[i][1]};
            result_eta = leg_model.inverse(curve_point, "G");
        }//end if else
        theta[i] = result_eta[0];
        beta[i] = result_eta[1];

        duty[i] += incre_duty;
        if (duty[i] < 0){
            duty[i] += 1.0;
        }//end if
        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0) {
            swing_phase[i] = 1;
            foothold[i] = {next_hip[i][0] + direction*((1-swing_time)/2+swing_time)*step_length, 0};
            // Bezier curve setup
            leg_model.forward(theta[i], beta[i]);
            p_lo = {next_hip[i][0] + leg_model.G[0], next_hip[i][1] + leg_model.G[1]};
            // calculate contact rim when touch ground
            for (int j=0; j<5; j++) {   // G, L_l, U_l
                double contact_height = j==0? leg_model.r : leg_model.radius;
                double contact_point[2] = {direction*step_length/2*(1-swing_time), -stand_height+contact_height};
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
            } else if (current_rim == 4) {  // L_r
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.L_r[0], foothold[i][1] + leg_model.G[1]-leg_model.L_r[1] + leg_model.radius};
            } else if (current_rim == 1) {  // U_l
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.U_l[0], foothold[i][1] + leg_model.G[1]-leg_model.U_l[1] + leg_model.radius};
            } else if (current_rim == 5) {  // U_r
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.U_r[0], foothold[i][1] + leg_model.G[1]-leg_model.U_r[1] + leg_model.radius};
            }//end if else
            sp[i] = SwingProfile(p_lo, p_td, step_height, direction);
            if ( ((direction == 1) && (i==2 || i==3)) || ((direction == -1) && (i==0 || i==1)) ) {
                step_length = new_step_length;
            }//end if 
        } else if ( (direction == 1) && (duty[i] > 1.0)) {             // entering stance phase when velocirty > 0
            swing_phase[i] = 0;
            duty[i] -= 1.0;
        } else if ( (direction == -1) && (duty[i] < (1.0-swing_time))) {    // entering stance phase when velocirty < 0
            swing_phase[i] = 0;
        }//end if else

        hip[i] = next_hip[i];
    }//end for
    return {theta, beta};
}//end step

void WalkGait::set_velocity(double new_value){
    velocity = new_value;
    dS = velocity / rate;
    incre_duty = dS / step_length;
    direction = velocity>=0? 1 : -1;
}//end set_velocity

void WalkGait::set_stand_height(double new_value){
    stand_height = new_value;
    for (int i=0; i<4; i++) {
        next_hip[i][1] = stand_height;
    }//end for
}//end set_stand_height

void WalkGait::set_step_length(double new_value){
    new_step_length = new_value;
}//end set_step_length

void WalkGait::set_step_height(double new_value){
    step_height = new_value;
}//end set_step_height