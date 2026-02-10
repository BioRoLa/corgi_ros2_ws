// ROS2 Leg Odometry Node — main entry point
#include <chrono>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "LegOdometryNode.hpp"
#include "Config.hpp"

// ============================================================
// Global node pointer (for signal handler)
// ============================================================
static rclcpp::Node::SharedPtr g_node = nullptr;

static void handle_signal(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt received, shutting down...");
    }
    rclcpp::shutdown();
    exit(signum);
}

/// Wait for simulation clock to become non-zero
static void wait_for_clock_sync(const rclcpp::Node::SharedPtr& node) {
    RCLCPP_INFO(node->get_logger(), "Waiting for simulation clock...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================
// LegOdometryNode — constructor
// ============================================================

LegOdometryNode::LegOdometryNode()
    : Node("leg_odometry"),
      processor_(corgi::Config::DT),
      observer_(
          corgi::Config::DT,
          corgi::Config::OBSERVER_CUTOFF_FREQ,
          corgi::Config::DOF,
          false,   // CSV logging
          ""       // No CSV filename
      )
{
    // --- Subscribers ---
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        corgi::Config::TOPIC_MOTOR_STATE, corgi::Config::QUEUE_SIZE_SUB,
        std::bind(&LegOdometryNode::cb_motor_state, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        corgi::Config::TOPIC_IMU, corgi::Config::QUEUE_SIZE_SUB,
        std::bind(&LegOdometryNode::cb_imu, this, std::placeholders::_1));

    position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        corgi::Config::TOPIC_ODOMETRY_POSITION, corgi::Config::QUEUE_SIZE_SUB,
        std::bind(&LegOdometryNode::cb_position, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        corgi::Config::TOPIC_ODOMETRY_VELOCITY, corgi::Config::QUEUE_SIZE_SUB,
        std::bind(&LegOdometryNode::cb_velocity, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        corgi::Config::TOPIC_TRIGGER, corgi::Config::QUEUE_SIZE_SUB,
        std::bind(&LegOdometryNode::cb_trigger, this, std::placeholders::_1));

    // --- Publishers ---
    contact_state_pub_ = this->create_publisher<corgi_msgs::msg::ContactStateStamped>(
        corgi::Config::TOPIC_CONTACT_STATE, corgi::Config::QUEUE_SIZE_PUB);
    // TODO: Create nav_msgs/Odometry publisher

    // TODO: Initialize ESEKF

    RCLCPP_INFO(this->get_logger(), "Leg Odometry Node Started");
    RCLCPP_INFO(this->get_logger(), "Loop rate: %.1f Hz", corgi::Config::ONLINE_LOOP_RATE);
}

// ============================================================
// process()  — called every tick from the spin loop
// ============================================================

void LegOdometryNode::process() {
    // Gate: wait until all topics have been received at least once
    if (!motor_state_received_ || !imu_received_ || !position_received_ || !velocity_received_) {
        if (iteration_count_ % static_cast<size_t>(corgi::Config::ONLINE_LOOP_RATE) == 0) {
            RCLCPP_WARN(this->get_logger(),
                "Waiting for data... Motor: %d, IMU: %d, Position: %d, Velocity: %d",
                motor_state_received_, imu_received_, position_received_, velocity_received_);
        }
        iteration_count_++;
        return;
    }

    // Gate: wait for trigger
    if (!is_triggered_) {
        if (iteration_count_ % static_cast<size_t>(corgi::Config::ONLINE_LOOP_RATE) == 0) {
            RCLCPP_INFO(this->get_logger(), "Waiting for trigger...");
        }
        iteration_count_++;
        observer_.reset();
        return;
    }

    // --- Contact estimation (disturbance observer) ---
    auto processed = processor_.process_realtime_data(position_, velocity_, imu_, motor_state_);

    auto disturbance = observer_.estimate_disturbance(
        processed.q, processed.q_dot, processed.tau, processed.I_c,
        iteration_count_, true);

    publish_contact_state(disturbance);

    // TODO: Extract IMU measurements (a_m, w_m) from imu_ msg
    // TODO: Extract encoder states (theta, theta_d, beta, beta_d) per leg from motor_state_
    // TODO: Build LegObservation per leg using ContactMap::lookup
    // TODO: esekf_.predict(a_m, w_m)
    // TODO: esekf_.update_all_legs(observations, w_m, exclude)
    // TODO: esekf_.inject_and_reset()
    // TODO: Publish odometry from esekf_.nominal()

    iteration_count_++;
}

// ============================================================
// ROS callbacks
// ============================================================

void LegOdometryNode::cb_motor_state(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
    motor_state_received_ = true;
}

void LegOdometryNode::cb_imu(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
    imu_received_ = true;
}

void LegOdometryNode::cb_position(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    position_ = *msg;
    position_received_ = true;
}

void LegOdometryNode::cb_velocity(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    velocity_ = *msg;
    velocity_received_ = true;
}

void LegOdometryNode::cb_trigger(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    is_triggered_ = msg->enable;
}

// ============================================================
// Contact state publisher
// ============================================================

void LegOdometryNode::publish_contact_state(const Eigen::VectorXd& disturbance) {
    corgi_msgs::msg::ContactStateStamped contact_msg;
    contact_msg.header.stamp = this->now();

    // Disturbance vector indices  (order: LF, RF, RH, LH)
    constexpr int rm_idx[4]   = {5, 7, 9, 11};
    constexpr int beta_idx[4] = {4, 6, 8, 10};

    // Schmitt-trigger contact detection
    for (int i = 0; i < 4; ++i) {
        double rm   = disturbance(rm_idx[i]);
        double beta = disturbance(beta_idx[i]);

        if (!leg_contact_state_[i]) {
            if (std::abs(rm) > corgi::Config::CONTACT_RM_THRESHOLD_HIGH ||
                std::abs(beta) > corgi::Config::CONTACT_BETA_THRESHOLD_HIGH) {
                leg_contact_state_[i] = true;
            }
        } else {
            if (std::abs(rm) < corgi::Config::CONTACT_RM_THRESHOLD_LOW &&
                std::abs(beta) < corgi::Config::CONTACT_BETA_THRESHOLD_LOW) {
                leg_contact_state_[i] = false;
            }
        }
    }

    // Fill per-module fields
    auto fill = [&](auto& module, int idx) {
        module.contact      = leg_contact_state_[idx];
        module.rm_force     = disturbance(rm_idx[idx]);
        module.beta_torque  = disturbance(beta_idx[idx]);
    };
    fill(contact_msg.module_a, 0);
    fill(contact_msg.module_b, 1);
    fill(contact_msg.module_c, 2);
    fill(contact_msg.module_d, 3);

    contact_state_pub_->publish(contact_msg);
}

// ============================================================
// Leg factory
// ============================================================

Leg LegOdometryNode::createLeg(double x_sign, double y_sign) {
    using corgi::Config;
    return Leg{
        Eigen::Vector3f(x_sign * Config::LEG_X_OFFSET,
                        y_sign * Config::LEG_Y_OFFSET,
                        Config::LEG_Z_OFFSET),
        Config::WHEEL_RADIUS,
        Config::TIRE_SKIN_RADIUS
    };
}

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, handle_signal);

    g_node = std::make_shared<LegOdometryNode>();

    // Sim-time clock sync
    bool use_sim_time = false;
    g_node->get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time) {
        wait_for_clock_sync(g_node);
    }

    const rclcpp::Duration period(0, static_cast<int>(1e9 / corgi::Config::ONLINE_LOOP_RATE));

    try {
        rclcpp::Time next_time = g_node->now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(g_node);
            std::dynamic_pointer_cast<LegOdometryNode>(g_node)->process();

            next_time += period;
            if (!g_node->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(g_node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
