#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <iomanip>
// #include "nlopt.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "bezier.hpp"
#include "wlw.hpp"

using namespace std;
using namespace Eigen;

bool sim = true;
std::array<double, 2> tb;
ros::Publisher motor_pub;
ros::Rate* rate_ptr;

// Variable
double body_velocity = 0.15; // m/s
double steplength = 0.3; // m
double total_distance = steplength*4/3; // m
double loop_time =(total_distance/body_velocity); // s
double Height = 0.159; // m
int total_steps =  static_cast<int> (1000 * loop_time);
double CoM_bias = 0.00;
int pub_rate = 1000; // 1KHz
double dS;
double incre_duty;
std::array<double, 4> duty;
const double swing_time = 0.2;

double current_eta[4][2];
double next_eta[4][2];   

double relative_foothold[4][2] = {};
std::array<std::array<double, 2>, 4> foothold;
std::array<std::array<double, 2>, 4> hip;
std::array<std::array<double, 2>, 4> next_hip;

LegModel leg_model(sim);
corgi_msgs::MotorCmdStamped motor_cmd;
std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
    &motor_cmd.module_a,
    &motor_cmd.module_b,
    &motor_cmd.module_c,
    &motor_cmd.module_d
};

void setCmd(std::array<double, 2> send, int index, bool dir);
void publish(int freq);
std::array<double, 2> find_pose(double height, float shift, float steplength);
void Send(double eta[4][2], int position_index, int freq);  // <-- forward declaration
void Initialize(double SL, int swing_index, double expected_height, double eta[4][2], int pub_time);
void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg);
void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, double eta[4][2], int swing_leg, int total_swing_step, int current_step);
void move_forward(double relative_eta[4][2], double eta[4][2], double move_distance, int swing_index, double Height, double SL);


void setCmd(std::array<double, 2> send, int index, bool dir) {
    if (dir==true){
        motor_cmd_modules[index]->beta  = -send[1];
    }
    else{
        motor_cmd_modules[index]->beta  = send[1];
    }
    motor_cmd_modules[index]->theta = send[0];
    motor_cmd_modules[index]->kp_r = 90;
    motor_cmd_modules[index]->ki_r = 0;
    motor_cmd_modules[index]->kd_r = 1.75;
    motor_cmd_modules[index]->kp_l = 90;
    motor_cmd_modules[index]->ki_l = 0;
    motor_cmd_modules[index]->kd_l = 1.75;
}

void publish(int freq) {
  for (int i=0; i<freq; i++) {
        motor_pub.publish(motor_cmd);
        rate_ptr->sleep();
    }
}

std::array<double, 2> find_pose(double height, float shift, float steplength) {
    std::array<double, 2> pose;
    double pos[2] = {0, -height + 0.0125};
    pose = leg_model.inverse(pos, "G");

    if (steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            pose = leg_model.move(pose[0], pose[1], {0.001, 0}, 0);
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = leg_model.move(pose[0], pose[1], {-0.001, 0}, 0);
        }
    }
    return pose;
}

void Send(double eta[4][2], int position_index, int freq){
    for(int i =0; i<4; i++){
        // std::cout << i << ": " <<eta[i][0]*180.0/M_PI << ", "<< eta[i][1]*180.0/M_PI << std::endl;
        std::array<double, 2> tmp = { eta[i][0], eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void Initialize(double SL, int swing_index, double expected_height, double eta[4][2], int pub_time) {
    // 1>3>0>2, index = who swings first
    switch (swing_index) {
        case 0:
        {
            duty= {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
            // LF: 0(-SL*3/6), 1(SL*1/6), 2(-SL*1/6), 3(SL*3/6)
            auto tmp0 = find_pose(expected_height, 0.00, (-SL*3/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(expected_height, 0.00, ( SL*1/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(expected_height, 0.00, (-SL*1/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(expected_height, 0.00, ( SL*3/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 1:
        {
            duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
            // RF: 0( SL*1/6), 1(-SL*3/6), 2( SL*3/6), 3(-SL*1/6)
            auto tmp0 = find_pose(expected_height, 0.00, ( SL*1/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(expected_height, 0.00, (-SL*3/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(expected_height, 0.00, ( SL*3/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(expected_height, 0.00, (-SL*1/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 2:
        {
            duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
            // RF: 0( SL*3/6), 1(-SL*1/6), 2(-SL*3/6), 3( SL*1/6)
            auto tmp0 = find_pose(expected_height, 0.00, ( SL*3/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(expected_height, 0.00, (-SL*1/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(expected_height, 0.00, (-SL*3/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(expected_height, 0.00, ( SL*1/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 3:
        {
            duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
            // RF: 0(-SL*1/6), 1( SL*3/6), 2( SL*1/6), 3(-SL*3/6)
            auto tmp0 = find_pose(expected_height, 0.00, (-SL*1/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(expected_height, 0.00, ( SL*3/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(expected_height, 0.00, ( SL*1/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(expected_height, 0.00, (-SL*3/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        default:
            eta[0][0] = 0.0; eta[0][1] = 0.0;
            eta[1][0] = 0.0; eta[1][1] = 0.0;
            eta[2][0] = 0.0; eta[2][1] = 0.0;
            eta[3][0] = 0.0; eta[3][1] = 0.0;
            break;
    }
    for (int i=0; i<4; i++) {
        leg_model.contact_map(eta[i][0], eta[i][1],0);
        leg_model.forward(eta[i][0], eta[i][1],true);
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -expected_height;
         // set current duty position
    }
    Send(eta, swing_index, pub_time);
}

void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];

    double endX   = target[0];
    double endY   = target[1];
    if(endY < startY) {
        endY += 2.0 * M_PI;
    } 

    variation[0] = endX - startX;
    variation[1] = endY - startY;

    target[0] = startX;
    target[1] = startY;
}

void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, double eta[4][2], int swing_leg, int total_swing_step, int current_step){
    double ratio = static_cast<double>(current_step) / static_cast<double>(total_swing_step);
    double currentX = target[0] + variation[0] * ratio;
    double currentY = target[1] + variation[1] * ratio;

    eta[swing_leg][0] = currentX;
    eta[swing_leg][1] = currentY;
}

void move_forward(double relative_eta[4][2], double eta[4][2], double move_distance, int swing_index, double Height, double SL){
    std::array<double, 2> swing_pose;
    std::array<double, 2> swing_variation;
    swing_pose = find_pose(Height, 0.00, (SL*3/6));  
    Swing(relative_eta, swing_pose, swing_variation, swing_index);

    for (double i = 0; i >= -move_distance; i-=0.0001) {
        for (int j=0; j<4; j++){
            std::array<double, 2> result_eta;
            if (j!= swing_index){
                result_eta = leg_model.move(relative_eta[j][0], relative_eta[j][1], {-0.0001, 0}, 0);
                eta[j][0] = result_eta[0];
                eta[j][1] = result_eta[1];
            }
            else{
                Swing_step(swing_pose, swing_variation, eta, swing_index, (int)(abs(move_distance)*10000), i*-10000);
            }
        }   
        Send(eta, i*-10000+1, 1);
        for (int leg = 0; leg < 4; ++leg) {
            for (int dim = 0; dim < 2; ++dim) {
                relative_eta[leg][dim] = eta[leg][dim];
            }
        }
    }
    
}

void next_step(){
    // for pub's next step(1khz)
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS ;
        duty[i] += incre_duty;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wlw_test");
    ros::NodeHandle nh;
    motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", 1);
    ros::Rate rate(1000);
    rate_ptr = &rate; 
    
    // Setting
    dS = body_velocity / pub_rate; 
    incre_duty = dS / steplength; 
    hip = {{{ 0.44/2, Height} ,
            { 0.44/2, Height} ,
            {-0.44/2, Height},
            {-0.44/2, Height}}};
    next_hip = hip;

    // single looop generation on a flat terrain
    Initialize(steplength, 1, Height, current_eta, 500); 
    sleep(3);
    // Initial leg configuration (Stability)
    for (int i=0; i<4; i++) {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};
    }

    
    move_forward(current_eta, next_eta, steplength*1/3, 0, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 2, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 1, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 3, Height, steplength);

    move_forward(current_eta, next_eta, steplength*1/3, 0, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 2, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 1, Height, steplength);
    move_forward(current_eta, next_eta, steplength*1/3, 3, Height, steplength);
    
    

    // stability
    /*
        TODO: 
    */ 
    
    
    // collision
    // slope
    // print (draw)
    // how to swing
    // multi-loop
    // variaty
    // add reading initial pose (find the leg that should be the front)
    // add transfer(like csv)
    // transform
    // turn
    return 0;
}
