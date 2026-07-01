#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_walk/walk_gait.hpp"
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

#define INIT_THETA (M_PI * 17.0 / 180.0)
#define INIT_BETA (0.0)

corgi_msgs::msg::TriggerStamped trigger_msg;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
{
    trigger_msg = *msg;
} // end trigger_cb

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("walk_test");
    auto motor_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, trigger_cb);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std_msgs::msg::Int32MultiArray phase_msg;
    phase_msg.data.resize(4, 0);
    std::array<corgi_msgs::msg::MotorCmd *, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d};
    for (int i = 0; i < 4; i++)
    {
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    } // end for

    enum STATES
    {
        INIT,
        TRANSFORM,
        WAIT,
        WALK,
        END
    };
    const std::array<double, 2> CoM_bias = {-0.02, 0.0}; // real robot com bias
    const int sampling_rate = 1000;
    const int transform_count = 5 * sampling_rate; // 5s
    // const double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915}; // stand height 0.25
    // const double init_eta[8] = {1.8571554834938668,0.4790144528333341,2.0636290799909855,0.10633741753260191,2.0636290799909855,-0.10633741753260191,1.8571554834938668,-0.4790144528333341}; // left stand height 0.25, right stand 0.3
    const double init_eta[8] = {1.2744470401482761, 0.4161719979302237, 1.1222141023936798, 0.11005079310996896, 1.1222141023936798, -0.11005079310996896, 1.2744470401482761, -0.4161719979302237};  // stand height 0.2
    double velocity = 0.05;
    double stand_height = 0.2;
    double step_length = 0.2;
    double step_height = 0.08;
    std::array<double, 4> ground_offset = {0.0, 0.0, 0.0, 0.0}; // LF, RF, RH, LH
    double curvature = 0.0;
    int count = 0;
    std::array<int, 4> step_count;
    double max_cal_time = 0.0;

    /* Initial variable */
    WalkGait walk_gait(false, CoM_bias[0], sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA, INIT_BETA, INIT_BETA, INIT_BETA}}}; // init eta (wheel mode)

    /* Other variable */
    STATES state = INIT, last_state = INIT;
    double transform_ratio;
    bool trigger;
    int command_count;

    // Runtime-tunable leg ground offsets (LF, RF, RH, LH), useful for different leg-length tests.
    node->declare_parameter<std::vector<double>>("ground_offset", {ground_offset[0], ground_offset[1], ground_offset[2], ground_offset[3]});
    std::array<double, 4> applied_ground_offset = ground_offset;
    auto to_array4 = [](const std::vector<double> & values) {
        std::array<double, 4> out = {0.0, 0.0, 0.0, 0.0};
        std::copy_n(values.begin(), 4, out.begin());
        return out;
    };
    auto offsets_changed = [](const std::array<double, 4> & a, const std::array<double, 4> & b) {
        const double eps = 1e-9;
        for (int i = 0; i < 4; i++)
        {
            if (std::abs(a[i] - b[i]) > eps)
            {
                return true;
            }
        }
        return false;
    };

    /* Behavior loop */
    // --- Synchronization Setup ---
    // Define control period based on sampling_rate (seconds = 1 / sampling_rate)
    rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(sampling_rate));

    // Wait for the ROS 2 clock to start (important if simulation is paused)
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    while (rclcpp::ok())
    {
        // 1. Process callbacks to try to receive /clock messages
        rclcpp::spin_some(node);

        // 2. Check if the current time is greater than 0 (indicates that the clock has been received)
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break; // Successfully synchronized, exit waiting loop
        }

        // 3. Sleep briefly to avoid 100% CPU usage (Wall time is fine while waiting for the connection)
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto start_time = node->now();
    rclcpp::Time next_time = start_time;
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    walk_gait.set_ground_offset(ground_offset);
    while (rclcpp::ok())
    {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        rclcpp::spin_some(node);

        std::vector<double> ground_offset_param;
        if (node->get_parameter("ground_offset", ground_offset_param))
        {
            if (ground_offset_param.size() == 4)
            {
                std::array<double, 4> requested_ground_offset = to_array4(ground_offset_param);
                if (offsets_changed(requested_ground_offset, applied_ground_offset))
                {
                    try
                    {
                        walk_gait.set_ground_offset(requested_ground_offset);
                        applied_ground_offset = requested_ground_offset;
                        RCLCPP_INFO(node->get_logger(),
                                    "Updated ground_offset [LF, RF, RH, LH] = [%.4f, %.4f, %.4f, %.4f]",
                                    applied_ground_offset[0],
                                    applied_ground_offset[1],
                                    applied_ground_offset[2],
                                    applied_ground_offset[3]);
                    }
                    catch (const std::exception & e)
                    {
                        RCLCPP_WARN(node->get_logger(),
                                    "Reject ground_offset update: %s",
                                    e.what());
                        node->set_parameter(rclcpp::Parameter(
                            "ground_offset",
                            std::vector<double>{applied_ground_offset[0],
                                                applied_ground_offset[1],
                                                applied_ground_offset[2],
                                                applied_ground_offset[3]}));
                    }
                }
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(),
                                     *node->get_clock(),
                                     2000,
                                     "Parameter ground_offset must have exactly 4 values: [LF, RF, RH, LH]");
            }
        }

        if (state == END)
        {
            break;
        } // end if
        // state machine
        switch (state)
        {
        case INIT:
            transform_ratio = 0.0;
            trigger = false;
            command_count = 0;
            break;
        case TRANSFORM:
            transform_ratio += 1.0 / transform_count;
            for (int i = 0; i < 4; i++)
            {
                eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i * 2] - INIT_THETA);
                eta_list[1][i] = INIT_BETA + transform_ratio * (init_eta[i * 2 + 1] - INIT_BETA);
                eta_list[1][i] = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
            } // end for
            break;
        case WAIT:
            if (last_state != state)
            {
                double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                walk_gait.initialize(current_eta, step_length);
            } // end if
            break;
        case WALK:
            eta_list = walk_gait.step();
            command_count++;
            break;
        default:
            break;
        } // end switch
        last_state = state;

        // next state
        switch (state)
        {
        case INIT:
            state = TRANSFORM;
            break;
        case TRANSFORM:
            if (transform_ratio > 1.0)
            {
                state = WAIT;
            } // end if
            break;
        case WAIT:
            if (trigger_msg.enable)
            {
                state = WALK;
            } // end if
            break;
        case WALK:
            step_count = walk_gait.get_step_count();
            if (step_count[0] >= 5 && step_count[1] >= 5 && step_count[2] >= 5 && step_count[3] >= 5)
            { // all legs have stepped at least twice
                state = END;
            } // end if
            break;
        default:
            break;
        } // end switch

        /* Publish motor commands */
        for (int i = 0; i < 4; i++)
        {
            if (eta_list[0][i] > M_PI * 160.0 / 180.0)
            {
                std::cout << "Leg " << i << " exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI * 160.0 / 180.0;
            } // end if
            if (eta_list[0][i] < M_PI * 16.9 / 180.0)
            {
                std::cout << "Leg " << i << " exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI * 16.9 / 180.0;
            } // end if
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? (eta_list[1][i]) : -(eta_list[1][i]);
        } // end for
        motor_pub->publish(motor_cmd);
        auto swing_phase = walk_gait.get_swing_phase();
        for (int i = 0; i < 4; i++) { phase_msg.data[i] = swing_phase[i]; }
        phase_pub->publish(phase_msg);
        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time)
        {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        } // end if

        // --- Synchronized Sleep ---
        next_time += period;
        if (!node->get_clock()->sleep_until(next_time))
        {
            // If the clock jumps or we miss a cycle, warn but continue
            RCLCPP_WARN(node->get_logger(), "Missed control cycle or clock jump");
            next_time = node->now(); // Reset baseline
        }
    } // end while

    auto end_time = node->now();
    auto duration = end_time - start_time;

    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.seconds() << " seconds" << std::endl;
    std::cout << "total count: " << command_count << std::endl;

    rclcpp::shutdown();
    return 0;
} // end main