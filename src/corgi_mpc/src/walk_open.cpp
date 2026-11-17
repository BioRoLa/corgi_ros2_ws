// Open-loop walking: corgi_walk's WalkGait sent straight to the motors as
// position commands (no MPC, no force control). The open-loop counterpart of
// mpc_walk, with the same start-up and stop:
//   unfold from wheel mode over 3 s -> hold -> wait for /trigger ->
//   1 s ramp-up -> walk for `duration` -> finish the step -> hold the pose.
//
// Motor command convention as walk_main: beta is negated on modules A and D.

#include <array>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

#include "corgi_walk/walk_gait.hpp"

namespace {
const double kWheelTheta = 17.0 / 180.0 * M_PI;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_walk_open");

    bool sim = false;
    node->get_parameter("use_sim_time", sim);
    const double stand_height = node->declare_parameter<double>("stand_height", 0.20);
    const double velocity = node->declare_parameter<double>("velocity", 0.10);
    const double step_length = node->declare_parameter<double>("step_length", 0.20);
    const double step_height = node->declare_parameter<double>("step_height", 0.04);
    const double duration = node->declare_parameter<double>("duration", 20.0);  // s, ramp-up included
    const double kp = node->declare_parameter<double>("kp", 90.0);
    const double kd = node->declare_parameter<double>("kd", 1.75);
    RCLCPP_INFO(node->get_logger(),
                "Open loop (%s): stand_height %.3f m, velocity %.3f m/s, step_length %.3f m, "
                "step_height %.3f m, duration %.1f s, kp %.1f, kd %.2f",
                sim ? "sim" : "real", stand_height, velocity, step_length, step_height, duration, kp, kd);

    bool trigger = false;
    auto motor_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto swing_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, [&](const corgi_msgs::msg::TriggerStamped::SharedPtr msg) { trigger = msg->enable; });

    if (sim) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock ...");
        while (rclcpp::ok() && node->now().seconds() <= 0.0) {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd *, 4> cmds = {
        &motor_cmd.module_a, &motor_cmd.module_b, &motor_cmd.module_c, &motor_cmd.module_d};
    for (auto *cmd : cmds) {
        cmd->theta = kWheelTheta;
        cmd->beta = 0.0;
        cmd->gamma = 0.0;  // ABAD held at zero
        cmd->kp_r = kp; cmd->kp_l = kp; cmd->kp_h = kp;
        cmd->ki_r = 0.0; cmd->ki_l = 0.0; cmd->ki_h = 0.0;
        cmd->kd_r = kd; cmd->kd_l = kd; cmd->kd_h = kd;
        cmd->torque_r = 0.0; cmd->torque_l = 0.0; cmd->torque_h = 0.0;
    }

    // ---- Gait ----
    const int freq = 1000;
    WalkGait walk_gait(sim, 0.0, freq);
    walk_gait.stand_height = stand_height;
    walk_gait.step_height = step_height;
    std::array<double, 8> init_eta = walk_gait.get_init_eta(step_length);
    walk_gait.initialize(init_eta.data(), step_length);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_height(step_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_velocity(0.0);

    rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / freq);
    rclcpp::Time next_time = node->now();
    auto sleep_one_period = [&]() {
        next_time += period;
        return node->get_clock()->sleep_until(next_time);
    };
    auto publish = [&]() {
        motor_cmd.header.stamp = node->now();
        motor_pub->publish(motor_cmd);
    };
    auto hold = [&](int steps) {
        for (int k = 0; k < steps && rclcpp::ok(); ++k) {
            rclcpp::spin_some(node);
            publish();
            sleep_one_period();
        }
    };

    if (!sim) {
        RCLCPP_INFO(node->get_logger(), "Wait ...");
        hold(3 * freq);
    }

    // ---- Unfold from wheel mode (init_eta is already in the motor convention) ----
    RCLCPP_INFO(node->get_logger(), "Transform Starts");
    const int transform_steps = 3 * freq;
    for (int k = 1; k <= transform_steps && rclcpp::ok(); ++k) {
        double ratio = static_cast<double>(k) / transform_steps;
        for (int i = 0; i < 4; ++i) {
            cmds[i]->theta = kWheelTheta + ratio * (init_eta[2 * i] - kWheelTheta);
            cmds[i]->beta = ratio * init_eta[2 * i + 1];
        }
        publish();
        sleep_one_period();
    }
    RCLCPP_INFO(node->get_logger(), "Transform Finished");
    hold(2 * freq);

    RCLCPP_INFO(node->get_logger(), "Waiting for trigger ...");
    while (rclcpp::ok() && !trigger) {
        rclcpp::spin_some(node);
        publish();
        sleep_one_period();
    }
    if (!sim) {
        RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");
        hold(3 * freq);
    }

    // ---- Walk ----
    // WalkGait advances the swing with the distance walked, so slowing down
    // would freeze a leg in the air. After `duration` keep cruise speed until
    // all four feet are down (3 s timeout), then hold that pose.
    RCLCPP_INFO(node->get_logger(), "Walking ...");
    std_msgs::msg::Int32MultiArray swing_msg;
    swing_msg.data.assign(4, 0);
    const int target_loop = static_cast<int>(duration * freq);
    const int settle_timeout = 3 * freq;
    for (int loop = 0; rclcpp::ok(); ++loop) {
        rclcpp::spin_some(node);
        walk_gait.set_velocity(velocity * std::min(1.0, (loop + 1) / static_cast<double>(freq)));
        auto eta = walk_gait.step();
        auto swing = walk_gait.get_swing_phase();
        bool all_stance = true;
        for (int i = 0; i < 4; ++i) {
            cmds[i]->theta = eta[0][i];
            cmds[i]->beta = (i == 1 || i == 2) ? eta[1][i] : -eta[1][i];
            swing_msg.data[i] = swing[i];
            all_stance = all_stance && swing[i] == 0;
        }
        publish();
        swing_pub->publish(swing_msg);
        if (loop >= target_loop && (all_stance || loop >= target_loop + settle_timeout)) {
            if (!all_stance) RCLCPP_WARN(node->get_logger(), "Stopped with a leg still in swing (timeout)");
            break;
        }
        if (!sleep_one_period()) break;
    }

    RCLCPP_INFO(node->get_logger(), "Walk finished, holding the pose (Ctrl+C to stop)");
    while (rclcpp::ok()) hold(1);
    rclcpp::shutdown();
    return 0;
}
