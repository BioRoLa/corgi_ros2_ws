#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry>
#include <algorithm>
#include "ros/ros.h"

#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/ContactStateStamped.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float64.h>

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

bool sim = true;
LegModel legmodel(sim);

// Constants
#define SAMPLE_RATE 1000.0 //Hz

ros::Rate rate(SAMPLE_RATE);

// Variables
bool trigger = false;
corgi_msgs::MotorStateStamped motor_state;
sensor_msgs::Imu imu;
corgi_msgs::ContactStateStamped contact_state;
std_msgs::Float64 prev_z_COM;

// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::Imu msg){
    imu = msg;
}

void contact_cb(const corgi_msgs::ContactStateStamped msg){
    contact_state = msg;
}

double estimate_z(double theta, double beta) {
    legmodel.forward(theta, beta);
    
    return -legmodel.contact_p[1]; // Replace with actual calculation
}

// Function to compute Euler angles from a quaternion using ZYX order.
void quaternionToEuler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    // Normalize the quaternion (if not already normalized)
    Eigen::Quaterniond q_norm = q.normalized();

    // Calculate roll (x-axis rotation)
    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    // Calculate pitch (y-axis rotation)
    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    // Calculate yaw (z-axis rotation)
    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_z_positiony");

    ros::NodeHandle nh;

    // ROS Publishers
    ros::Publisher z_position_pub = nh.advertise<std_msgs::Float64>("odometry/z_position_hip", 10);
    // ROS Subscribers
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", SAMPLE_RATE, trigger_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", SAMPLE_RATE, motor_state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", SAMPLE_RATE, imu_cb);
    ros::Subscriber contact_sub = nh.subscribe<corgi_msgs::ContactStateStamped>("odometry/contact", SAMPLE_RATE, contact_cb);

    Eigen::Quaterniond q;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    prev_z_COM.data = 0.0;

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::ContactState*> contact_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    double z_leg[4];

    while (ros::ok()){

        ros::spinOnce();

        q = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        quaternionToEuler(q, roll, pitch, yaw);

        if(trigger){
            for (int i=0; i<4; i++){
    
                if (i == 1 || i == 2) {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta-pitch);
                }
                else {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta+pitch);
                }
            }

            std::vector<double> contact_heights;
            for (int i = 0; i < 4; i++) {
                if (contact_modules[i]->contact) {
                    contact_heights.push_back(z_leg[i]);
                }
            }

            std_msgs::Float64 z_COM;
            if (contact_heights.empty()) {
                z_COM.data = prev_z_COM.data;
            } 
            else {
                std::sort(contact_heights.begin(), contact_heights.end());
                
                double median;
                size_t n = contact_heights.size();
                if (n % 2 == 0) {
                    median = (contact_heights[n / 2 - 1] + contact_heights[n / 2]) / 2.0;
                } else {
                    median = contact_heights[n / 2];
                }
                z_COM.data = median;
            }
            prev_z_COM.data = z_COM.data;

            z_position_pub.publish(z_COM);
            ROS_INFO("z_COM: %f", z_COM);

            rate.sleep();
        }
    }

    ros::shutdown();
    
    return 0;
}
