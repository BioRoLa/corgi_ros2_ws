#include <iostream>

#include "ros/ros.h"
#include "Eigen/Dense"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "leg_model.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

Eigen::Vector4d distribute_forces(double sa, double sb, double sc, double sd, double mg, double L, double t)
{
    double Fa = t;
    double Fd = mg / 2.0 - t;
    
    double denominator = L + sb - sc;
    
    double Fc = (t * (L - sa - sd) + (mg / 2.0) * (sb + sd)) / denominator;
    
    double Fb = mg / 2.0 - Fc;
    
    return Eigen::Vector4d(Fa, Fb, Fc, Fd);
}


int main(int argc, char **argv) {

    ROS_INFO("Impednace Experiment Starts\n");
    
    ros::init(argc, argv, "imp_exp");

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

    int exp_case = 2;  // 0: G, 1: L, 2: U
    double F_init = -20;

    double M = 0;
    double K = 1000;
    double B = 30;

    bool sim = true;
    LegModel legmodel(sim);

    Eigen::Vector4d forces;
    double sa = 0;
    double sb = 0;
    double sc = 0;
    double sd = 0;
    double mg = -22*9.81;

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Mx = M;
        cmd->My = M;
        cmd->Bx = B;
        cmd->By = B;
        cmd->Kx = K;
        cmd->Ky = K;
    }

    for (int i=0; i<2000; i++){
        if (exp_case == 0) {
            imp_cmd_modules[0]->theta += 83/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 83/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 83/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 83/2000.0/180.0*M_PI;
        }
        if (exp_case == 1) {
            imp_cmd_modules[0]->theta += 13/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 13/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 13/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 13/2000.0/180.0*M_PI;
            imp_cmd_modules[0]->beta += 20/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->beta -= 20/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->beta -= 20/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->beta += 20/2000.0/180.0*M_PI;
        }
        if (exp_case == 2) {
            imp_cmd_modules[0]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[0]->beta += 60/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->beta -= 60/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->beta -= 60/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->beta += 60/2000.0/180.0*M_PI;
        }

        legmodel.contact_map(imp_cmd_modules[0]->theta, imp_cmd_modules[0]->beta);
        sa = legmodel.contact_p[0];

        legmodel.contact_map(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta);
        sb = legmodel.contact_p[0];

        legmodel.contact_map(imp_cmd_modules[2]->theta, imp_cmd_modules[2]->beta);
        sc = legmodel.contact_p[0];

        legmodel.contact_map(imp_cmd_modules[3]->theta, imp_cmd_modules[3]->beta);
        sd = legmodel.contact_p[0];

        forces = distribute_forces(sa, sb, sc, sd, mg, 0.444, F_init);
        std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;

        for (int i=0; i<4; i++) {
            imp_cmd_modules[i]->Fy = forces[i];
        }

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            int loop_count = 0;
            while (ros::ok()) {
                // Stay
                if (loop_count < 1000) {
                }
                else if (loop_count < 2000) {
                }
                else if (loop_count < 3000) {
                }
                else if (loop_count < 4000) {
                }
                else if (loop_count < 5000) {
                    F_init -= 0.06;
                }
                else if (loop_count < 6000) {
                }
                else if (loop_count < 7000) {
                    F_init += 0.06;
                }
                else if (loop_count < 8000) {
                }
                else if (loop_count < 9000) {
                    F_init -= 0.04;
                }
                else if (loop_count < 10000) {
                }
                else if (loop_count < 11000) {
                    F_init += 0.04;
                }
                else if (loop_count < 12000) {
                }
                else if (loop_count < 13000) {
                    F_init -= 0.02;
                }
                else if (loop_count < 14000) {
                }
                else if (loop_count < 15000) {
                    F_init += 0.02;
                }
                else {
                    break;
                }

                legmodel.contact_map(imp_cmd_modules[0]->theta, imp_cmd_modules[0]->beta);
                sa = legmodel.contact_p[0];
        
                legmodel.contact_map(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta);
                sb = legmodel.contact_p[0];
        
                legmodel.contact_map(imp_cmd_modules[2]->theta, imp_cmd_modules[2]->beta);
                sc = legmodel.contact_p[0];
        
                legmodel.contact_map(imp_cmd_modules[3]->theta, imp_cmd_modules[3]->beta);
                sd = legmodel.contact_p[0];
        
                forces = distribute_forces(sa, sb, sc, sd, mg, 0.444, F_init);
                std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;
        
                for (int i=0; i<4; i++) {
                    imp_cmd_modules[i]->Fy = forces[i];
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