#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/ForceStateStamped.h"

#include "leg_model.hpp"


corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::ForceStateStamped force_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}


int main(int argc, char **argv) {

    ROS_INFO("Impedance Command Publisher Starts\n");

    ros::init(argc, argv, "imp_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        for (int i=0; i<4; i++){
            double theta = 0;
            double force_cmd_x = 0;
            double force_cmd_y = 0;

            if (loop_count < 5000) {
                force_cmd_y = 0;
                theta = (55 + 5*cos(loop_count/500.0*M_PI))/180.0*M_PI;
                // theta = 60/180.0*M_PI;
            }
            else if (loop_count < 9000) {
                force_cmd_y = -50 + 10*sin((loop_count-5000)/100.0*M_PI);
                theta = 60/180.0*M_PI;
            }
            else {
                force_cmd_y = 0;
                theta = (55 + 5*cos((loop_count-9000)/500.0*M_PI))/180.0*M_PI;
                // theta = 60/180.0*M_PI;
            }

            imp_cmd_modules[i]->theta = theta;
            imp_cmd_modules[i]->beta = 0;
            imp_cmd_modules[i]->Fx = force_cmd_x;
            imp_cmd_modules[i]->Fy = force_cmd_y;
        }

        imp_cmd.header.seq = loop_count;

        imp_cmd_pub.publish(imp_cmd);
        
        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}