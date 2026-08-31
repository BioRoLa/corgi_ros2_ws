#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/fitted_coefficient.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"

std::array<double, 2> eta;
corgi_msgs::msg::MotorStateStamped motor_state;

bool state_received = false;

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
    state_received = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("corgi_homing");
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 5);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 1, motor_state_cb);
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

    RCLCPP_INFO(node->get_logger(), "Homing Starts");

    double theta_err[4];
    double beta_err[4];
    double gamma_err[4];

    // Wait for a REAL /motor/state before reading theta.
    //
    // This used to be a blind fixed spin of 1000 iterations (~1 s), after
    // which the code read the zero-initialised global regardless. If DDS
    // discovery had not delivered the first /motor/state by then, theta read
    // 0.00, tripped the "< 17 deg" guard below and homing aborted -- and it
    // returned 0, so the panel reported success. That race is why homing
    // "needed several presses": a retry found discovery already warm. It gets
    // worse on a loaded machine, which is exactly when it matters.
    const int wait_ticks = 5000;   // 1 kHz rate -> 5 s
    int waited = 0;
    while (rclcpp::ok() && !state_received && waited < wait_ticks) {
        rclcpp::spin_some(node);
        rate.sleep();
        waited++;
    }
    if (!state_received) {
        RCLCPP_ERROR(node->get_logger(),
                     "No /motor/state after %d ms -- is the ROS bridge up? "
                     "Homing ABORTED, nothing was moved.", wait_ticks);
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(),
                "motor/state acquired after %d ms; settling", waited);
    for (int i=0; i<500; i++) {   // let a few more states land so theta is current
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->theta = motor_state_modules[i]->theta;
        motor_cmd_modules[i]->beta = motor_state_modules[i]->beta;
        motor_cmd_modules[i]->gamma = motor_state_modules[i]->gamma;
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->kp_h = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->ki_h = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->kd_h = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
        motor_cmd_modules[i]->torque_h = 0;

        theta_err[i] = (17/180.0*M_PI-motor_state_modules[i]->theta);
        beta_err[i] = (-motor_state_modules[i]->beta);
        gamma_err[i] = (-motor_state_modules[i]->gamma);

        if (motor_cmd_modules[i]->theta < 17/180.0*M_PI) {
            // Name the leg and the value: "too small" alone gave no way to
            // tell a genuinely low leg from a state that never arrived.
            // Non-zero exit so the caller can tell this from success -- both
            // paths used to return 0.
            RCLCPP_WARN(node->get_logger(),
                        "Leg %d theta is %.2f deg, below the 17 deg home "
                        "target -- homing ABORTED, nothing was moved. Raise "
                        "that leg above 17 deg and retry.",
                        i, motor_cmd_modules[i]->theta * 180.0 / M_PI);
            rclcpp::shutdown();
            return 1;
        }
    }

    for (int i=0; i<2000; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->theta += theta_err[j]/2000.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    for (int i=0; i<5000; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->beta += beta_err[j]/5000.0;
            motor_cmd_modules[j]->gamma += gamma_err[j]/5000.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Homing Completed");
    rclcpp::shutdown();
    
    return 0;
}
