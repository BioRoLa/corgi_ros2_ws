// ROS2 Leg Odometry Node — main entry point
#include <chrono>
#include <algorithm>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "node/LegOdometryNode.hpp"
#include "common/Config.hpp"
#include "common/ParamsIO.hpp"

// ============================================================
// Global node pointer (for signal handler)
// ============================================================
static rclcpp::Node::SharedPtr g_node = nullptr;

static void handle_signal(int signum)
{
    if (g_node)
    {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt received, shutting down...");
    }
    rclcpp::shutdown();
    exit(signum);
}

/// Wait for simulation clock to become non-zero
static void wait_for_clock_sync(const rclcpp::Node::SharedPtr &node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for simulation clock...");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0)
        {
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
      params_([]() -> corgi::Params
              {
          try {
              const std::string pkg =
                  ament_index_cpp::get_package_share_directory("corgi_odometry");
              return corgi::load_params(pkg + "/config/config_online.yaml");
          } catch (const std::exception& e) {
              RCLCPP_WARN(rclcpp::get_logger("leg_odometry"),
                          "Config load failed (%s), using defaults", e.what());
              return corgi::Params{};
          } }()),
      processor_(corgi::Config::DT, params_.encoder_cutoff_freq),
      observer_(
          corgi::Config::DT,
          params_.observer_cutoff_freq,
          corgi::Config::DOF,
          false, // CSV logging
          ""     // No CSV filename
          ),
      esekf_()
{
    contact_rm_threshold_high_ = params_.contact_rm_threshold_high;
    contact_rm_threshold_low_ = params_.contact_rm_threshold_low;
    contact_beta_threshold_high_ = params_.contact_beta_threshold_high;
    contact_beta_threshold_low_ = params_.contact_beta_threshold_low;
    contact_gamma_threshold_high_ = params_.contact_gamma_threshold_high;
    contact_gamma_threshold_low_ = params_.contact_gamma_threshold_low;
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
    ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf", corgi::Config::QUEUE_SIZE_PUB);

    // --- Debug Publishers ---
    debug_leg_obs_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "debug/leg_obs", corgi::Config::QUEUE_SIZE_PUB);
    debug_zleg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "debug/z_leg", corgi::Config::QUEUE_SIZE_PUB);

    RCLCPP_INFO(this->get_logger(), "Leg Odometry Node Started");
    RCLCPP_INFO(this->get_logger(), "Loop rate: %.1f Hz", corgi::Config::ONLINE_LOOP_RATE);
}

// ============================================================
// process()  — called every tick from the spin loop
// ============================================================

void LegOdometryNode::process()
{
    // Gate: wait until all topics have been received at least once
    if (!motor_state_received_ || !imu_received_ || !position_received_ || !velocity_received_)
    {
        if (iteration_count_ % static_cast<size_t>(corgi::Config::ONLINE_LOOP_RATE) == 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for data... Motor: %d, IMU: %d, Position: %d, Velocity: %d",
                        motor_state_received_, imu_received_, position_received_, velocity_received_);
        }
        iteration_count_++;
        return;
    }

    // Gate: wait for trigger
    if (!is_triggered_)
    {
        if (iteration_count_ % static_cast<size_t>(corgi::Config::ONLINE_LOOP_RATE) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for trigger...");
        }
        iteration_count_++;
        observer_.reset();
        esekf_initialized_ = false;
        last_esekf_imu_time_valid_ = false;
        prev_imu_valid_ = false;
        esekf_tick_ = 0;
        return;
    }

    // ==========================================================
    // GMO pipeline (runs every tick @ 1000 Hz)
    // ==========================================================
    auto processed = processor_.process_realtime_data(position_, velocity_, imu_, motor_state_);

    auto disturbance = observer_.estimate_disturbance(
        processed.q, processed.q_dot, processed.tau, processed.I_c,
        iteration_count_, false);

    publish_contact_state(disturbance);

    // ==========================================================
    // ES-EKF pipeline (runs every ESEKF_DECIMATION ticks @ 500 Hz)
    // ==========================================================
    esekf_tick_++;
    if (esekf_tick_ >= static_cast<size_t>(corgi::Config::ESEKF_DECIMATION))
    {
        esekf_tick_ = 0;

        // --- Compute dynamic dt from IMU timestamp ---
        rclcpp::Time current_imu_time(imu_.header.stamp);
        float esekf_dt = static_cast<float>(corgi::Config::ESEKF_DT); // nominal fallback
        if (last_esekf_imu_time_valid_)
        {
            double dt_sec = (current_imu_time - last_esekf_imu_time_).seconds();
            // Sanity check: clamp to [0.5×nominal, 2×nominal] to reject outliers
            constexpr double dt_min = corgi::Config::ESEKF_DT * 0.5;
            constexpr double dt_max = corgi::Config::ESEKF_DT * 2.0;
            if (dt_sec > dt_min && dt_sec < dt_max)
            {
                esekf_dt = static_cast<float>(dt_sec);
            }
        }
        last_esekf_imu_time_ = current_imu_time;
        last_esekf_imu_time_valid_ = true;

        // --- Initialize ESEKF on first triggered tick ---
        if (!esekf_initialized_)
        {
            estimation_model::NominalState x0;
            x0.q = Eigen::Quaternionf(
                       static_cast<float>(imu_.orientation.w),
                       static_cast<float>(imu_.orientation.x),
                       static_cast<float>(imu_.orientation.y),
                       static_cast<float>(imu_.orientation.z))
                       .normalized();
            esekf_.init(x0);
            esekf_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "ES-EKF initialized (%.0f Hz, nominal dt=%.4f s)",
                        corgi::Config::ESEKF_RATE, corgi::Config::ESEKF_DT);
        }

        // --- 1. Extract IMU measurements (raw, in sensor frame) ---
        Eigen::Vector3f a_m(
            static_cast<float>(imu_.linear_acceleration.x),
            static_cast<float>(imu_.linear_acceleration.y),
            static_cast<float>(imu_.linear_acceleration.z));
        Eigen::Vector3f w_m(
            static_cast<float>(imu_.angular_velocity.x),
            static_cast<float>(imu_.angular_velocity.y),
            static_cast<float>(imu_.angular_velocity.z));

        // Trapezoidal average with the intermediate tick's IMU sample.
        // predict uses average over the 2ms interval; update uses the
        // instantaneous measurement at the current tick.
        Eigen::Vector3f a_pred = prev_imu_valid_ ? 0.5f * (prev_imu_a_ + a_m) : a_m;
        Eigen::Vector3f w_pred = prev_imu_valid_ ? 0.5f * (prev_imu_w_ + w_m) : w_m;

        // --- 2. Predict (IMU propagation with dynamic dt) ---
        esekf_.predict(a_pred, w_pred, esekf_dt);

        // --- 3. Build per-leg observations ---
        const float w_y = static_cast<float>(imu_.angular_velocity.y) - static_cast<float>(esekf_.nominal().bw.y());

        const corgi_msgs::msg::MotorState *modules[4] = {
            &motor_state_.module_a,
            &motor_state_.module_b,
            &motor_state_.module_c,
            &motor_state_.module_d};

        std::vector<estimation_model::LegObservation> observations;
        observations.reserve(4);
        std::array<bool, 4> exclude_flags{};

        for (int i = 0; i < 4; ++i)
        {
            auto obs = build_leg_observation(*modules[i], legs_[i], i, w_y);
            exclude_flags[i] = !obs.in_contact;
            observations.push_back(obs);
        }

        // --- Debug: publish per-leg observation inputs and z_leg ---
        publish_debug(observations, w_m);

        // --- 4. Update (sequential per-leg velocity constraint) ---
        esekf_.update_all_legs(observations, w_m, exclude_flags);

        // --- 5. Inject error state into nominal + reset ---
        esekf_.inject_and_reset();

        // --- 6. Publish ESEKF state ---
        {
            const auto &st = esekf_.nominal();
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = st.p.x();
            odom_msg.pose.pose.position.y = st.p.y();
            odom_msg.pose.pose.position.z = st.p.z();

            odom_msg.pose.pose.orientation.w = st.q.w();
            odom_msg.pose.pose.orientation.x = st.q.x();
            odom_msg.pose.pose.orientation.y = st.q.y();
            odom_msg.pose.pose.orientation.z = st.q.z();

            odom_msg.twist.twist.linear.x = st.v.x();
            odom_msg.twist.twist.linear.y = st.v.y();
            odom_msg.twist.twist.linear.z = st.v.z();

            Eigen::Vector3f w_corrected = w_m - st.bw;
            odom_msg.twist.twist.angular.x = w_corrected.x();
            odom_msg.twist.twist.angular.y = w_corrected.y();
            odom_msg.twist.twist.angular.z = w_corrected.z();

            ekf_pub_->publish(odom_msg);
        }
    }

    // Store current IMU for trapezoidal averaging at next ESEKF tick
    prev_imu_a_ << static_cast<float>(imu_.linear_acceleration.x),
        static_cast<float>(imu_.linear_acceleration.y),
        static_cast<float>(imu_.linear_acceleration.z);
    prev_imu_w_ << static_cast<float>(imu_.angular_velocity.x),
        static_cast<float>(imu_.angular_velocity.y),
        static_cast<float>(imu_.angular_velocity.z);
    prev_imu_valid_ = true;

    iteration_count_++;
}

// ============================================================
// ROS callbacks
// ============================================================

void LegOdometryNode::cb_motor_state(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg)
{
    motor_state_ = *msg;
    motor_state_received_ = true;
}

void LegOdometryNode::cb_imu(const corgi_msgs::msg::ImuStamped::SharedPtr msg)
{
    imu_ = *msg;
    imu_received_ = true;
}

void LegOdometryNode::cb_position(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    position_ = *msg;
    position_received_ = true;
}

void LegOdometryNode::cb_velocity(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    velocity_ = *msg;
    velocity_received_ = true;
}

void LegOdometryNode::cb_trigger(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
{
    is_triggered_ = msg->enable;
}

// ============================================================
// Build per-leg observation for ES-EKF update
// ============================================================

estimation_model::LegObservation LegOdometryNode::build_leg_observation(
    const corgi_msgs::msg::MotorState &module,
    Leg *leg, int leg_idx, float w_y)
{
    (void)w_y;

    // --- Extract theta (opening angle) ---
    float theta = static_cast<float>(module.theta);

    // --- Extract beta (rotation angle) with sign convention ---
    // Right-side legs (RF=1, RH=2) have negated beta
    bool is_right_side = (leg_idx == 1 || leg_idx == 2);
    float beta = is_right_side ? -static_cast<float>(module.beta)
                               : static_cast<float>(module.beta);

    // --- Motor velocities → joint velocities ---
    // theta_d = (-velocity_r + velocity_l) / 2
    // beta_d  = ( velocity_r + velocity_l) / 2   (negated for right-side legs)
    float theta_d = static_cast<float>((-module.velocity_r + module.velocity_l) / 2.0);
    float beta_d = -static_cast<float>((module.velocity_r + module.velocity_l) / 2.0);
    float gamma = static_cast<float>(module.gamma);
    float gamma_d = static_cast<float>(module.velocity_h);
    if (is_right_side)
        beta_d = -beta_d;

    // --- Accumulate contact_beta ---
    // Deprecated: Accumulating contact_beta causes divergence starting from arbitrary joint angles.
    // Use pitch compensated beta for rim lookup.
    // Get pitch from ESEKF nominal quaternion
    float w = esekf_.nominal().q.w();
    float x = esekf_.nominal().q.x();
    float y = esekf_.nominal().q.y();
    float z = esekf_.nominal().q.z();
    float sinp = 2.0f * (w * y - z * x);
    float pitch = std::abs(sinp) >= 1.0f ? std::copysign(static_cast<float>(M_PI) / 2.0f, sinp) : std::asin(sinp);

    // Apply contact_beta compensation: right side -(beta), left side +(beta)
    // Need to subtract pitch for right side, add pitch for left side
    float compensated_beta = is_right_side ? (beta - pitch) : (beta + pitch);

    // --- Determine contact rim via ContactMap ---
    RIM rim = contact_map_.lookup(theta, compensated_beta);

    // --- Contact angle relative to body frame ---
    // Alpha is now 0.0f
    float alpha = 0.0f;

    // --- Contact flag from Schmitt trigger ---
    bool in_contact = leg_contact_state_[leg_idx] && (rim != NO_CONTACT);

    return estimation_model::LegObservation{
        leg, theta, theta_d, beta, beta_d, gamma, gamma_d, rim, alpha, in_contact};
}

// ============================================================
// Debug publisher
// ============================================================

void LegOdometryNode::publish_debug(
    std::vector<estimation_model::LegObservation> &observations,
    const Eigen::Vector3f &w_m)
{
    // --- debug/leg_obs ---
    // Layout: 4 legs × 9 values = 36 floats
    //   [theta, theta_d, beta, beta_d, contact_beta(0), alpha, rim, in_contact, contact_flag]
    {
        std_msgs::msg::Float64MultiArray dbg;
        for (int i = 0; i < 4; ++i)
        {
            const auto &o = observations[i];
            dbg.data.push_back(o.theta);
            dbg.data.push_back(o.theta_d);
            dbg.data.push_back(o.beta);
            dbg.data.push_back(o.beta_d);
            dbg.data.push_back(0.0); // contact_beta deprecated, placeholder
            dbg.data.push_back(o.alpha);
            dbg.data.push_back(static_cast<double>(o.rim));
            dbg.data.push_back(o.in_contact ? 1.0 : 0.0);
            dbg.data.push_back(leg_contact_state_[i] ? 1.0 : 0.0);
        }
        debug_leg_obs_pub_->publish(dbg);
    }

    // --- debug/z_leg ---
    // z_leg = observed body velocity from encoder kinematics
    // Layout: 4 legs × 6 values = 24 floats
    //   [z_leg.x, z_leg.y, z_leg.z, contact_pt.x, contact_pt.y, contact_pt.z]
    {
        std_msgs::msg::Float64MultiArray dbg;
        Eigen::Vector3f w_corrected = w_m - esekf_.nominal().bw;

        for (int i = 0; i < 4; ++i)
        {
            auto &o = observations[i];
            // Recompute FK + contact for debug (same as ESEKF::update_leg)
            o.leg->Calculate(o.theta, o.theta_d, 0,
                             o.beta, o.beta_d, 0,
                             o.gamma, o.gamma_d, 0);
            o.leg->PointContact(o.rim, o.alpha);
            Eigen::Vector3f v_zero = Eigen::Vector3f::Zero();
            o.leg->PointVelocity(v_zero, w_corrected, o.rim, o.alpha, true);
            Eigen::Vector3f z_leg = -o.leg->contact_velocity;

            dbg.data.push_back(z_leg.x());
            dbg.data.push_back(z_leg.y());
            dbg.data.push_back(z_leg.z());
            dbg.data.push_back(o.leg->contact_point.x());
            dbg.data.push_back(o.leg->contact_point.y());
            dbg.data.push_back(o.leg->contact_point.z());
        }
        debug_zleg_pub_->publish(dbg);
    }
}

// ============================================================
// Contact state publisher
// ============================================================

void LegOdometryNode::publish_contact_state(const Eigen::VectorXd &disturbance)
{
    corgi_msgs::msg::ContactStateStamped contact_msg;
    contact_msg.header.stamp = this->now();

    // Disturbance vector indices  (order: LF, RF, RH, LH)
    constexpr int rm_idx[4] = {5, 7, 9, 11};
    constexpr int beta_idx[4] = {4, 6, 8, 10};
    constexpr int gamma_idx[4] = {12, 13, 14, 15};

    // Schmitt-trigger contact detection
    for (int i = 0; i < 4; ++i)
    {
        double rm = disturbance(rm_idx[i]);
        double beta = disturbance(beta_idx[i]);
        double gamma = (disturbance.size() > gamma_idx[i]) ? disturbance(gamma_idx[i]) : 0.0;

        if (!leg_contact_state_[i])
        {
            if (std::abs(rm) > contact_rm_threshold_high_ ||
                std::abs(beta) > contact_beta_threshold_high_ ||
                std::abs(gamma) > contact_gamma_threshold_high_)
            {
                leg_contact_state_[i] = true;
            }
        }
        else
        {
            if (std::abs(rm) < contact_rm_threshold_low_ &&
                std::abs(beta) < contact_beta_threshold_low_ &&
                std::abs(gamma) < contact_gamma_threshold_low_)
            {
                leg_contact_state_[i] = false;
            }
        }
    }

    // Fill per-module fields
    auto fill = [&](auto &module, int idx)
    {
        const double rm = disturbance(rm_idx[idx]);
        const double beta = disturbance(beta_idx[idx]);
        const double gamma = (disturbance.size() > gamma_idx[idx]) ? disturbance(gamma_idx[idx]) : 0.0;
        module.contact = leg_contact_state_[idx];
        module.score = std::max(std::max(std::abs(rm), std::abs(beta)), std::abs(gamma));
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

Leg LegOdometryNode::createLeg(double x_sign, double y_sign)
{
    using corgi::Config;
    return Leg{
        Eigen::Vector3f(x_sign * Config::LEG_X_OFFSET,
                        y_sign * Config::LEG_Y_OFFSET,
                        Config::LEG_Z_OFFSET),
        Config::WHEEL_RADIUS,
        Config::TIRE_SKIN_RADIUS};
}

// ============================================================
// main
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, handle_signal);

    g_node = std::make_shared<LegOdometryNode>();

    // Sim-time clock sync
    bool use_sim_time = false;
    g_node->get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time)
    {
        wait_for_clock_sync(g_node);
    }

    const rclcpp::Duration period(0, static_cast<int>(1e9 / corgi::Config::ONLINE_LOOP_RATE));

    try
    {
        rclcpp::Time next_time = g_node->now();
        while (rclcpp::ok())
        {
            rclcpp::spin_some(g_node);
            std::dynamic_pointer_cast<LegOdometryNode>(g_node)->process();

            next_time += period;
            if (!g_node->get_clock()->sleep_until(next_time))
            {
                RCLCPP_WARN(g_node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
