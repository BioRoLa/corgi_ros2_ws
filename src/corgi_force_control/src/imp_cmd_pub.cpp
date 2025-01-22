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
    
    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        int mod_idx = 0;
        for (auto& cmd : imp_cmd_modules){
            cmd->theta = 60/180.0*M_PI;
            cmd->beta = 0/180.0*M_PI;
            cmd->Fx = 0;
            cmd->Fy = 0;

            if (loop_count < 1000){
                cmd->theta = 17/180.0*M_PI;
            }
            else if (loop_count < 3000){
                cmd->theta = (17+43*(loop_count-1000)/2000.0)/180.0*M_PI;
            }
            else if (loop_count < 5000){
                cmd->theta = (60+10*sin((loop_count-1000)/400.0*M_PI))/180.0*M_PI;
            }
            else if (loop_count < 5500){
                cmd->theta = 60/180.0*M_PI;
            }
            else if (loop_count < 6500){
                cmd->Fy = -56 + 5*sin((loop_count-5500)/100.0*M_PI);
            }
            else {
                cmd->Fy = 0;
            }

            cmd->Mx = 0.652;
            cmd->My = 0.652;
            cmd->Kx = 200000;
            cmd->Ky = 200000;
            cmd->Dx = 400;
            cmd->Dy = 400;

            // cmd->Mx = 0;
            // cmd->My = 0;
            // cmd->Kx = 2000;
            // cmd->Ky = 2000;
            // cmd->Dx = 100;
            // cmd->Dy = 100;

            cmd->adaptive_kp_x = 30;
            cmd->adaptive_kp_y = 30;
            cmd->adaptive_ki_x = 10;
            cmd->adaptive_ki_y = 10;
            cmd->adaptive_kd_x = 20;
            cmd->adaptive_kd_y = 20;

            mod_idx++;
        }

        imp_cmd.header.seq = loop_count;

        imp_cmd_pub.publish(imp_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}