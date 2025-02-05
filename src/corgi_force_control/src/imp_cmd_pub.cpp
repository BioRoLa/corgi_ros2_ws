#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"


int main(int argc, char **argv) {

    ROS_INFO("Impedance Command Publisher Starts\n");
    
    ros::init(argc, argv, "imp_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };
    
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fx = 0;
        cmd->Fy = 0;

        cmd->Mx = 0;
        cmd->My = 0;
        cmd->Bx = 200;
        cmd->By = 200;
        cmd->Kx = 15000;
        cmd->Ky = 15000;
        
        cmd->adaptive_kp_x = 30;
        cmd->adaptive_kp_y = 30;
        cmd->adaptive_ki_x = 10;
        cmd->adaptive_ki_y = 10;
        cmd->adaptive_kd_x = 20;
        cmd->adaptive_kd_y = 20;
    }

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        if (loop_count < 1000) {

        }

        else if (loop_count < 2000) {
            imp_cmd_modules[0]->theta += 43/1000.0/180.0*M_PI;
            // imp_cmd_modules[1]->theta += 43/1000.0/180.0*M_PI;
            // imp_cmd_modules[2]->theta += 43/1000.0/180.0*M_PI;
            // imp_cmd_modules[3]->theta += 43/1000.0/180.0*M_PI;
        }

        else if (loop_count < 3000) {

        }

        else if (loop_count < 4000) {
            imp_cmd_modules[0]->theta = (60-20*sin((loop_count-3000)/200.0*M_PI))/180.0*M_PI;
        }

        else if (loop_count < 5000) {

        }

        else if (loop_count < 7000) {
            imp_cmd_modules[0]->Fy = -70 + 20*sin((loop_count-5000)/50.0*M_PI);
            // imp_cmd_modules[0]->Fy = -50;
            // imp_cmd_modules[1]->Fy = -50;
            // imp_cmd_modules[2]->Fy = -50;
            // imp_cmd_modules[3]->Fy = -50;

            for (int i=0; i<4; i++){
                imp_cmd_modules[i]->Kx = 500;
                imp_cmd_modules[i]->Ky = 500;
                imp_cmd_modules[i]->Bx = 10;
                imp_cmd_modules[i]->By = 10;
            }
        }

        else {
            
        }

        imp_cmd.header.seq = loop_count;

        imp_cmd_pub.publish(imp_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}