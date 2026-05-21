#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/fitted_coefficient.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"

std::array<double, 2> eta;
corgi_msgs::msg::MotorStateStamped motor_state;

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("corgi_reset");
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 5);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 5, motor_state_cb);
    rclcpp::Rate rate(1000);
    rclcpp::spin_some(node);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    RCLCPP_INFO(node->get_logger(), "Reset Starts");

    double theta_err[4];
    double beta_err[4];

    for (int i=0; i<1000; i++) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->theta = motor_state_modules[i]->theta;
        motor_cmd_modules[i]->beta = motor_state_modules[i]->beta;
        motor_cmd_modules[i]->kp_r = 80;
        motor_cmd_modules[i]->kp_l = 80;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;

        theta_err[i] = (17.0/180.0*M_PI - motor_state_modules[i]->theta);
        beta_err[i] = (0 - motor_state_modules[i]->beta);
    }

    // Step 1: theta -> 17 deg (1.5 sec)
    for (int i=0; i<1500; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->theta += theta_err[j]/1500.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Step 2: beta -> 0 (3 sec)
    for (int i=0; i<3000; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->beta += beta_err[j]/3000.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Step 3: theta -> 120 deg (3 sec)
    double theta_err2[4];
    for (int i=0; i<4; i++) {
        theta_err2[i] = (120.0/180.0*M_PI - motor_cmd_modules[i]->theta);
    }

    for (int i=0; i<1500; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->theta += theta_err2[j]/1500.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Reset Completed");
    rclcpp::shutdown();

    return 0;
}
