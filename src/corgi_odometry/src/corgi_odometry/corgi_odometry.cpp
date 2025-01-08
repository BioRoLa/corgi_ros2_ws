#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "KLD_estimation/InformationFilter.hpp"
#include "KLD_estimation/csv_reader.hpp"
using namespace estimation_model;

// Constants
const int SAMPLE_RATE = 200; //Hz
const float THREHOLD = 0.08; //threshold of KLD
const int J = 10; //sample time (matrix size)
const float MOTOR_OFFSET[3] = {0.222, 0.193, 0};
const float WHEEL_RADIUS = 0.1;
const float WHEEL_WIDTH = 0.012;

// Variables
bool trigger = false;
float dt = 1 / (float)SAMPLE_RATE;

// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_odometry");

    ros::NodeHandle nh;

    // ROS Publishers
    /* TODO: add state publisher
    std::vector<std::string> cols = {
        "v_.x", "v_.y", "v_.z", 
        "p.x", "p.y", "p.z", 
        "zLF.x", "zLF.y", "zLF.z", 
        "zRF.x", "zRF.y", "zRF.z", 
        "zRH.x", "zRH.y", "zRH.z", 
        "zLH.x", "zLH.y", "zLH.z", 
        "zP.x", "zP.y", "zP.z", 
        "ba.x", "ba.y", "ba.z", 
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "lf.c","rf.c","rh.c","lh.c",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
    };*/
    // ROS Subscribers
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", SAMPLE_RATE, trigger_cb);
    // TODO: Add IMU(ax,ay,az),IMU(wx,wy,wz),IMU(quaternion),leg encoder(theta,beta,(beta_d,theta_d))

    Eigen::initParallel();
    ros::Rate rate(SAMPLE_RATE);

    /* Estimate model initialization */

    //IMU data (Observation)
    U u(J, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt);

    //Legs model
    Leg lf_leg(Eigen::Vector3f( MOTOR_OFFSET[0],  MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rf_leg(Eigen::Vector3f( MOTOR_OFFSET[0], -MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rh_leg(Eigen::Vector3f(-MOTOR_OFFSET[0], -MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg lh_leg(Eigen::Vector3f(-MOTOR_OFFSET[0],  MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    
    //Dynamic predictor
    
    DP lf(J + 1, lf_leg, encoder_lf, 0, &u);
    DP rf(J + 1, rf_leg, encoder_rf, 0, &u);
    DP rh(J + 1, rh_leg, encoder_rh, 0, &u);
    DP lh(J + 1, lh_leg, encoder_lh, 0, &u);

    while (ros::ok()){
        ros::spinOnce();
        if(trigger){

        }
    }

    ros::shutdown();
    
    return 0;
}
