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
            cmd->beta = 30/180.0*M_PI;
            cmd->Fx = 0;
            cmd->Fy = 0;
            cmd->Mx = 0;
            cmd->My = 0;
            cmd->Bx = 100;
            cmd->By = 100;
            cmd->Kx = 10000;
            cmd->Ky = 10000;
            
            if (loop_count < 1000){
                cmd->theta = 60/180.0*M_PI;
            }
            else if (loop_count < 2000){
                // cmd->theta = 60/180.0*M_PI;
                cmd->theta = (60 + 10*sin((loop_count-1000)/200.0*M_PI))/180.0*M_PI;
                // cmd->beta = (70*(loop_count-1000)/1000.0)/180.0*M_PI;
            }
            else if (loop_count < 3000){
                cmd->theta = 60/180.0*M_PI;
            }
            else if (loop_count < 5000){
                cmd->theta = 60/180.0*M_PI;
                cmd->Fy = -50 + 20*sin((loop_count-3000)/200.0*M_PI);
                cmd->Bx = 30;
                cmd->By = 30;
                cmd->Kx = 3000;
                cmd->Ky = 3000;
            }
            else if (loop_count < 7000){
                cmd->theta = 60/180.0*M_PI;
                cmd->Fy = -70 + 20*sin((loop_count-5000)/200.0*M_PI);
                cmd->Bx = 30;
                cmd->By = 30;
                cmd->Kx = 3000;
                cmd->Ky = 3000;
            }
            else{
                cmd->theta = 60/180.0*M_PI;
            }

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