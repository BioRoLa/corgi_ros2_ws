#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {

    ROS_INFO("Impedance Command Publisher Starts\n");
    
    ros::init(argc, argv, "imp_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    double M = 0;
    double K = 1000;
    double B = 30;

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fx = 0;
        cmd->Fy = -55;
        cmd->Mx = M;
        cmd->My = M;
        cmd->Bx = B;
        cmd->By = B;
        cmd->Kx = K;
        cmd->Ky = K;
    }

    for (int i=0; i<2000; i++){
        imp_cmd_modules[0]->theta += 43/2000.0/180.0*M_PI;
        imp_cmd_modules[1]->theta += 43/2000.0/180.0*M_PI;
        imp_cmd_modules[2]->theta += 43/2000.0/180.0*M_PI;
        imp_cmd_modules[3]->theta += 43/2000.0/180.0*M_PI;
        // imp_cmd_modules[0]->beta += 90/1000.0/180.0*M_PI;
        // imp_cmd_modules[1]->beta += 90/1000.0/180.0*M_PI;
        // imp_cmd_modules[2]->beta += 90/1000.0/180.0*M_PI;
        // imp_cmd_modules[3]->beta += 90/1000.0/180.0*M_PI;

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            int loop_count = 0;
            while (ros::ok()) {
                if (loop_count < 1000) {
                    
                }

                else if (loop_count < 2000) {
                    // imp_cmd_modules[0]->theta += 43/1000.0/180.0*M_PI;
                    // imp_cmd_modules[1]->theta += 43/1000.0/180.0*M_PI;
                    // imp_cmd_modules[2]->theta += 43/1000.0/180.0*M_PI;
                    // imp_cmd_modules[3]->theta += 43/1000.0/180.0*M_PI;
                    // imp_cmd_modules[0]->beta += 90/1000.0/180.0*M_PI;
                    // imp_cmd_modules[1]->beta += 90/1000.0/180.0*M_PI;
                    // imp_cmd_modules[2]->beta += 90/1000.0/180.0*M_PI;
                    // imp_cmd_modules[3]->beta += 90/1000.0/180.0*M_PI;
                }

                else if (loop_count < 3000) {

                }

                else if (loop_count < 6000) {
                    imp_cmd_modules[0]->Fy = -35;
                    imp_cmd_modules[1]->Fy = -75;
                    imp_cmd_modules[2]->Fy = -35;
                    imp_cmd_modules[3]->Fy = -75;
                }

                else if (loop_count < 7000) {
                    imp_cmd_modules[0]->Fy = -55;
                    imp_cmd_modules[1]->Fy = -55;
                    imp_cmd_modules[2]->Fy = -55;
                    imp_cmd_modules[3]->Fy = -55;
                }

                else if (loop_count < 8000) {
                    imp_cmd_modules[0]->Fy = -75;
                    imp_cmd_modules[1]->Fy = -35;
                    imp_cmd_modules[2]->Fy = -75;
                    imp_cmd_modules[3]->Fy = -35;
                }

                else if (loop_count < 9000) {
                    imp_cmd_modules[0]->Fy = -55;
                    imp_cmd_modules[1]->Fy = -55;
                    imp_cmd_modules[2]->Fy = -55;
                    imp_cmd_modules[3]->Fy = -55;
                }

                else if (loop_count < 10000) {
                    imp_cmd_modules[0]->Fy = -35;
                    imp_cmd_modules[1]->Fy = -75;
                    imp_cmd_modules[2]->Fy = -35;
                    imp_cmd_modules[3]->Fy = -75;
                }

                else if (loop_count < 11000) {
                    imp_cmd_modules[0]->Fy = -55;
                    imp_cmd_modules[1]->Fy = -55;
                    imp_cmd_modules[2]->Fy = -55;
                    imp_cmd_modules[3]->Fy = -55;
                }

                else if (loop_count < 12000) {
                    imp_cmd_modules[0]->Fy = -75;
                    imp_cmd_modules[1]->Fy = -35;
                    imp_cmd_modules[2]->Fy = -75;
                    imp_cmd_modules[3]->Fy = -35;
                }

                else {
                    imp_cmd_modules[0]->Fy = -55;
                    imp_cmd_modules[1]->Fy = -55;
                    imp_cmd_modules[2]->Fy = -55;
                    imp_cmd_modules[3]->Fy = -55;
                }

                imp_cmd.header.seq = loop_count;

                imp_cmd_pub.publish(imp_cmd);

                loop_count++;

                rate.sleep();
            }

            ros::shutdown();
            
            return 0;
        }

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}