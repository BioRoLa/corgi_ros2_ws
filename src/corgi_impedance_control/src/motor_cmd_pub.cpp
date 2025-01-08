#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"


int main(int argc, char **argv) {

    ROS_INFO("Motor Command Publisher Starts\n");
    
    ros::init(argc, argv, "motor_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    
    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        for (auto& cmd : motor_cmd_modules){
            if (loop_count < 3000) cmd->theta = (loop_count/6000.0+17/180.0)*M_PI;
            else cmd->theta = (3000/6000.0+17/180.0)*M_PI;
            cmd->beta = 0;
        }

        motor_cmd.header.seq = loop_count;

        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}