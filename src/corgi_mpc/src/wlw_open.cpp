/**
 * wlw_open.cpp
 *
 * Open-loop WLW (Walk-Leg-Walk) node using Hybrid gait + GaitSelector.
 * Port of the ROS1 wlw_open_loop.cpp to ROS2, with parameters
 * loaded from config/config.yaml (common.wlw section).
 *
 * Topics published:
 *   motor/command    (corgi_msgs/msg/MotorCmdStamped)  — joint commands @ 1 kHz
 *   walk/swing_phase (std_msgs/msg/Int32MultiArray)    — per-leg swing flag
 *
 * Topics subscribed:
 *   trigger          (corgi_msgs/msg/TriggerStamped)   — start signal
 *
 * Launch parameter:
 *   config_profile   string  "sim" or "real"  (default: "sim")
 */

#include "walk_utils.hpp"
#include "corgi_hybrid/hybrid_gen.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>

// ── Trigger callback ──────────────────────────────────────────────────────────
static bool trigger = false;

static void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
{
    trigger = msg->enable;
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_wlw_open");

    // ── Config profile ────────────────────────────────────────────────────────
    node->declare_parameter<std::string>("config_profile", "sim");
    std::string config_profile = node->get_parameter("config_profile").as_string();
    if (config_profile != "sim" && config_profile != "real") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid config_profile='%s', fallback to 'sim'",
                    config_profile.c_str());
        config_profile = "sim";
    }
    sim = (config_profile == "sim");

    RCLCPP_INFO(node->get_logger(), "WLW Open-Loop Starts — profile: %s", config_profile.c_str());

    // ── Wait for clock synchronization ────────────────────────────────────────
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced (%.2f s)", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // ── Load config.yaml ──────────────────────────────────────────────────────
    const char *home_path = std::getenv("HOME");
    if (!home_path) {
        throw std::runtime_error("HOME environment variable not set");
    }
    const std::string config_file =
        std::string(home_path) + "/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml";

    YAML::Node config = YAML::LoadFile(config_file);
    YAML::Node wlw_cfg = config["common"]["wlw"];
    if (!wlw_cfg) {
        throw std::runtime_error("Missing common.wlw section in " + config_file);
    }

    const double velocity     = wlw_cfg["velocity"].as<double>();
    const double stand_height = wlw_cfg["stand_height"].as<double>();
    const double step_length  = wlw_cfg["step_length"].as<double>();
    const int    swing_index  = wlw_cfg["swing_index"].as<int>();
    const int    target_loop  = wlw_cfg["target_loop"].as<int>();
    const int    ramp_loops   = wlw_cfg["ramp_loops"].as<int>();

    RCLCPP_INFO(node->get_logger(),
                "Params: velocity=%.2f  stand_height=%.2f  step_length=%.2f  "
                "swing_index=%d  target_loop=%d  ramp_loops=%d",
                velocity, stand_height, step_length, swing_index, target_loop, ramp_loops);

    // ── ROS2 interfaces ───────────────────────────────────────────────────────
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
        "motor/command", 10);
    auto swing_phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>(
        "walk/swing_phase", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, trigger_cb);

    rclcpp::Duration period(0, 1000000);  // 1 ms → 1 kHz
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std_msgs::msg::Int32MultiArray swing_phase_msg;
    swing_phase_msg.data.resize(4, 0);

    std::vector<corgi_msgs::msg::MotorCmd *> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // ── Gait planner setup ────────────────────────────────────────────────────
    // GaitSelector owns all shared state (static).  do_pub=0 disables its
    // internal motor_cmd_pub_ so only this node publishes commands.
    auto gait_selector = std::make_shared<GaitSelector>(node, sim, 0.0, 1000);
    gait_selector->do_pub = 0;

    // Set gait parameters *before* Initialize() which reads them directly.
    // change_Step_length() only sets new_step_length; step_length must be set
    // directly so that find_pose() inside Initialize() uses the correct value.
    gait_selector->step_length = step_length;
    gait_selector->current_step_length = {step_length, step_length, step_length, step_length};
    gait_selector->next_step_length    = {step_length, step_length, step_length, step_length};
    gait_selector->new_step_length     = step_length;

    Hybrid hybrid_gait(gait_selector);
    hybrid_gait.change_Height_all(stand_height);   // sets current_stand_height[i]
    hybrid_gait.change_Velocity(velocity);         // sets dS and incre_duty

    hybrid_gait.Initialize(swing_index, 1);

    // ── Sync state after Initialize() ────────────────────────────────────────
    // Initialize() only writes next_eta; Step() reads eta for stance calculation,
    // so copy next_eta → eta now (GaitSelector::Send() normally does this copy
    // but we bypass Send).
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            gait_selector->eta[i][j] = gait_selector->next_eta[i][j];
        }
    }
    // change_Height_all() only updates next_hip[i][2]; align hip so the first
    // Step() stance displacement (next_hip - hip) is zero in the Z axis.
    for (int i = 0; i < 4; i++) {
        gait_selector->hip[i][2] = gait_selector->next_hip[i][2];
    }
    gait_selector->body[2]      = stand_height;
    gait_selector->next_body[2] = stand_height;

    // ── Build init_eta in motor wire-format ───────────────────────────────────
    // Motor beta convention (matches GaitSelector::Send and walk_h20_v10_open):
    //   legs 0,3 (right): motor_beta = -next_eta[i][1]
    //   legs 1,2 (left) : motor_beta = +next_eta[i][1]
    double init_eta[8];
    for (int i = 0; i < 4; i++) {
        init_eta[2*i]   = gait_selector->next_eta[i][0];
        init_eta[2*i+1] = (i == 0 || i == 3) ? -gait_selector->next_eta[i][1]
                                               :  gait_selector->next_eta[i][1];
    }

    // ── Motor initial command (17°/0 standing pose) ───────────────────────────
    for (auto &cmd : motor_cmd_modules) {
        cmd->theta = 17.0 / 180.0 * M_PI;
        cmd->beta  = 0.0;
        cmd->kp_r  = 90;
        cmd->kp_l  = 90;
        cmd->ki_r  = 0;
        cmd->ki_l  = 0;
        cmd->kd_r  = 1.75;
        cmd->kd_l  = 1.75;
    }

    RCLCPP_INFO(node->get_logger(), "Wait ...");

    // Real robot: hold 3 s so the hardware can zero itself before we move.
    if (!sim) {
        for (int i = 0; i < 3000; i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    // ── Ramp joints from 17° → init_eta over 3 seconds ───────────────────────
    for (int i = 0; i < 3000; i++) {
        for (int j = 0; j < 4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j] - 17.0/180.0*M_PI) / 3000.0;
            motor_cmd_modules[j]->beta  += init_eta[2*j+1] / 3000.0;
        }
        motor_cmd.header.stamp = node->now();
        motor_cmd_pub->publish(motor_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // ── Hold 2 seconds ────────────────────────────────────────────────────────
    for (int i = 0; i < 2000; i++) {
        rclcpp::spin_some(node);
        motor_cmd.header.stamp = node->now();
        motor_cmd_pub->publish(motor_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    // ── Wait for trigger ──────────────────────────────────────────────────────
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (trigger) {
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

            // Real robot: publish static stance for 3 s while odometry warms up.
            if (!sim) {
                for (int i = 0; i < 3000; i++) {
                    rclcpp::spin_some(node);
                    motor_cmd.header.stamp = node->now();
                    motor_cmd_pub->publish(motor_cmd);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
            }

            RCLCPP_INFO(node->get_logger(), "Controller Starts ...");

            double target_vel_x = 0.0;
            double target_pos_x = 0.0;
            const double dt = 1.0 / 1000.0;  // 1 kHz control period (s)

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // Velocity ramp up
                if (loop_count < ramp_loops) {
                    target_vel_x += velocity / static_cast<double>(ramp_loops);
                    hybrid_gait.change_Velocity(target_vel_x);
                }

                // Velocity ramp down
                if (loop_count > target_loop * 10 - ramp_loops &&
                    loop_count < target_loop * 10)
                {
                    target_vel_x -= velocity / static_cast<double>(ramp_loops);
                    hybrid_gait.change_Velocity(target_vel_x);
                }

                target_pos_x += target_vel_x * dt;

                // Step gait (updates next_eta)
                hybrid_gait.Step();

                // Publish swing phase
                for (int i = 0; i < 4; i++) {
                    swing_phase_msg.data[i] = GaitSelector::swing_phase[i];
                }
                swing_phase_pub->publish(swing_phase_msg);

                // Joint angle bounds check and motor command
                for (int i = 0; i < 4; i++) {
                    const double theta = gait_selector->next_eta[i][0];
                    const double beta  = gait_selector->next_eta[i][1];

                    if (theta > M_PI * 160.0 / 180.0) {
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                             "Leg %d theta=%.3f exceeds upper bound (160 deg)", i, theta);
                    }
                    if (theta < M_PI * 17.0 / 180.0) {
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                             "Leg %d theta=%.3f exceeds lower bound (17 deg)", i, theta);
                    }

                    motor_cmd_modules[i]->theta = theta;
                    // Legs 0,3 (right): negate beta; legs 1,2 (left): keep sign.
                    motor_cmd_modules[i]->beta = (i == 0 || i == 3) ? -beta : beta;
                }

                motor_cmd.header.stamp = node->now();
                motor_cmd_pub->publish(motor_cmd);

                // Advance eta → next_eta (replaces GaitSelector::Send's copy)
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 2; j++) {
                        gait_selector->eta[i][j] = gait_selector->next_eta[i][j];
                    }
                }

                std::cout << std::fixed << std::setprecision(3)
                          << "Target Position X: " << target_pos_x << "\n"
                          << "Current Velocity X: " << target_vel_x << "\n"
                          << "= = = = = = = = = =" << "\n\n";

                loop_count++;
                if (loop_count >= target_loop * 10) break;

                next_time += period;
                if (!node->get_clock()->sleep_until(next_time)) {
                    RCLCPP_WARN(node->get_logger(), "Sleep until failed — timing overrun");
                    break;
                }
            }
            break;
        }

        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    rclcpp::shutdown();
    return 0;
}
