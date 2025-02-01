#include <iostream>

#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"

#include "wheel_to_leg.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_main");

    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);

    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp = 90;
        motor_cmd_modules[i]->ki = 0;
        motor_cmd_modules[i]->kd = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }

    double init_eta[8] = {17/180.0*M_PI, 0, 17/180.0*M_PI, 55/180.0*M_PI, 17/180.0*M_PI, 0, 17/180.0*M_PI, 0};
    WheelToLegTransformer WheelToLegTransformer(true);
    WheelToLegTransformer.initialize(init_eta);
    
    std::array<std::array<double, 4>, 2> eta_list;

    auto start = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        if (WheelToLegTransformer.transform_finished) { break; }
        
        eta_list = WheelToLegTransformer.step();

        for (int i=0; i<4; i++) {
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
            // std::cout << i << ": " << eta_list[0][i] << ", " << eta_list[1][i] << std::endl;
        }
        // std::cout << std::endl;

        motor_pub.publish(motor_cmd);
        rate.sleep();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    
    ros::shutdown();
    
    return 0;
}