#include <iostream>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_rt_control");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    ROS_INFO("Leg Transform Starts\n");
    
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        cmd->kd_r = 1.75;
        cmd->kd_l = 1.75;
    }

    for (int i=0; i<2000; i++){
        motor_cmd.header.seq = -1;

        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }

    ROS_INFO("Leg Transform Finished\n");

    while (ros::ok()){
        ros::spinOnce();

        if (trigger){
            ROS_INFO("Real Time Trajectory Starts\n");

            int seq = 0;
            double loop_count = 0.0;
            while (ros::ok()) {
                if (loop_count < 10000)
                for (auto& cmd : motor_cmd_modules) {
                    cmd->beta += 90/10000.0/180.0*M_PI;
                }
                else if (loop_count < 15000)
                for (auto& cmd : motor_cmd_modules) {
                
                }
                else if (loop_count < 35000)
                for (auto& cmd : motor_cmd_modules) {
                    cmd->beta -= 180/20000.0/180.0*M_PI;
                }
                else if (loop_count < 40000)
                for (auto& cmd : motor_cmd_modules) {
                
                }
                else if (loop_count < 50000)
                for (auto& cmd : motor_cmd_modules) {
                    cmd->beta += 90/10000.0/180.0*M_PI;
                }
                else {
                    break;
                }

                motor_cmd.header.seq = seq;

                motor_cmd_pub.publish(motor_cmd);

                seq++;
                loop_count++;

                rate.sleep();
            }
            break;
        }
    }

    ROS_INFO("Real Time Trajectory Finished\n");

    ros::shutdown();
    
    return 0;
}
