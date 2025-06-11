#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "force_estimation.hpp"

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

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fx = 0;
        cmd->Fy = 0;
        cmd->Mx = 0;
        cmd->My = 0;
        cmd->Bx = 80;
        cmd->By = 10;
        cmd->Kx = 2000;
        cmd->Ky = 100;
    }

    std::array<double, 2> eta;
    double ds = 0.0;
    double mg = 19.68*9.81;

    double s = 0.0;
    double h = 0.0;
    
    double theta = 17.0/180.0*M_PI;
    double beta = 0;

    double force_A = -mg/4.0;

    for (int i=0; i<2000; i++){
        theta += 73.0/180.0*M_PI;
        beta += 30.0/180.0*M_PI;

        imp_cmd_modules[0]->theta = theta;
        imp_cmd_modules[1]->theta = theta;
        imp_cmd_modules[2]->theta = theta;
        imp_cmd_modules[3]->theta = theta;

        imp_cmd_modules[0]->beta = -beta;
        imp_cmd_modules[1]->beta =  beta;
        imp_cmd_modules[2]->beta = -beta;
        imp_cmd_modules[3]->beta =  beta;
        
        imp_cmd_modules[0]->Fy = force_A;
        imp_cmd_modules[1]->Fy = -mg/2.0 - force_A;
        imp_cmd_modules[2]->Fy = force_A;
        imp_cmd_modules[3]->Fy = -mg/2.0 - force_A;

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            int loop_count = 0;
            while (ros::ok()) {
                if (loop_count < 10000) {
                    imp_cmd_modules[0]->Fy = force_A;
                    imp_cmd_modules[1]->Fy = -mg/2.0 - force_A;
                    imp_cmd_modules[2]->Fy = force_A;
                    imp_cmd_modules[3]->Fy = -mg/2.0 - force_A;
                }
                else {
                    break;
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