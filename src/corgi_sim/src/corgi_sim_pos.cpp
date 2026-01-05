// ROS2 version - works with standard Webots ROS2 topics
#include <iostream>
#include <chrono>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>

using namespace std::chrono_literals;

// Global message objects
corgi_msgs::msg::MotorCmdStamped motor_cmd;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::TriggerStamped trigger;
sensor_msgs::msg::Imu imu;
sensor_msgs::msg::Imu imu_filtered;

// Motor positions and velocities
double AR_phi = 0.0, AL_phi = 0.0, BR_phi = 0.0, BL_phi = 0.0;
double CR_phi = 0.0, CL_phi = 0.0, DR_phi = 0.0, DL_phi = 0.0;
double AR_phi_dot = 0.0, AL_phi_dot = 0.0, BR_phi_dot = 0.0, BL_phi_dot = 0.0;
double CR_phi_dot = 0.0, CL_phi_dot = 0.0, DR_phi_dot = 0.0, DL_phi_dot = 0.0;

rclcpp::Node::SharedPtr g_node = nullptr;

// Callbacks for motor encoders
void motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped::SharedPtr cmd) { motor_cmd = *cmd; }

void AR_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    AR_phi_dot = (msg->data - AR_phi) * 1000.0;
    AR_phi = msg->data;
}
void AL_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    AL_phi_dot = (msg->data - AL_phi) * 1000.0;
    AL_phi = msg->data;
}
void BR_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    BR_phi_dot = (msg->data - BR_phi) * 1000.0;
    BR_phi = msg->data;
}
void BL_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    BL_phi_dot = (msg->data - BL_phi) * 1000.0;
    BL_phi = msg->data;
}
void CR_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    CR_phi_dot = (msg->data - CR_phi) * 1000.0;
    CR_phi = msg->data;
}
void CL_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    CL_phi_dot = (msg->data - CL_phi) * 1000.0;
    CL_phi = msg->data;
}
void DR_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    DR_phi_dot = (msg->data - DR_phi) * 1000.0;
    DR_phi = msg->data;
}
void DL_encoder_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    DL_phi_dot = (msg->data - DL_phi) * 1000.0;
    DL_phi = msg->data;
}

void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) { imu = *msg; }

// Helper functions
double find_closest_phi(double phi_ref, double phi_fb)
{
    double diff = fmod(phi_ref - phi_fb + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    return phi_fb + diff - M_PI;
}

void phi2tb(double phi_r, double phi_l, double &theta, double &beta)
{
    theta = (phi_l - phi_r) / 2.0 + 17.0 / 180.0 * M_PI;
    beta = (phi_l + phi_r) / 2.0;
}

void tb2phi(corgi_msgs::msg::MotorCmd motor_cmd, double &phi_r, double &phi_l,
            double phi_r_fb, double phi_l_fb)
{
    double theta_0 = 17.0 / 180.0 * M_PI;
    // Sanitize inputs: replace NaNs with zeros
    if (!std::isfinite(motor_cmd.theta))
        motor_cmd.theta = theta_0;
    if (!std::isfinite(motor_cmd.beta))
        motor_cmd.beta = 0.0;
    // Enforce minimum theta
    if (motor_cmd.theta < theta_0)
        motor_cmd.theta = theta_0;
    double delta_theta = motor_cmd.theta - theta_0;
    // Compute desired motor angles
    double desired_r = motor_cmd.beta - delta_theta;
    double desired_l = motor_cmd.beta + delta_theta;

    if (!std::isfinite(desired_r))
        desired_r = phi_r_fb; // fallback
    if (!std::isfinite(desired_l))
        desired_l = phi_l_fb; // fallback

    phi_r = desired_r;
    phi_l = desired_l;
    // phi_r = find_closest_phi(desired_r, phi_r_fb);
    // phi_l = find_closest_phi(desired_l, phi_l_fb);
}

// Removed blocking stdin prompt; output filename now comes from a ROS parameter.

void signal_handler(int signum)
{
    RCLCPP_INFO(g_node->get_logger(), "Interrupt received.");
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("corgi_sim");
    RCLCPP_INFO(g_node->get_logger(), "Corgi Simulation POS Starts (ROS2 Webots)\n");
    rclcpp::Time now = g_node->now();
    
    // Publishers for motor position commands (standard Webots ROS2 interface)
    auto AR_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("lf_left_motor/set_position", 10);
    auto AL_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("lf_right_motor/set_position", 10);
    auto BR_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("rf_left_motor/set_position", 10);
    auto BL_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("rf_right_motor/set_position", 10);
    auto CR_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("rh_left_motor/set_position", 10);
    auto CL_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("rh_right_motor/set_position", 10);
    auto DR_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("lh_left_motor/set_position", 10);
    auto DL_motor_pub = g_node->create_publisher<std_msgs::msg::Float64>("lh_right_motor/set_position", 10);

    // Subscribers for encoder feedback (standard Webots ROS2 interface)
    auto AR_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("lf_left_motor_sensor/value", 10, AR_encoder_cb);
    auto AL_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("lf_right_motor_sensor/value", 10, AL_encoder_cb);
    auto BR_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("rf_left_motor_sensor/value", 10, BR_encoder_cb);
    auto BL_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("rf_right_motor_sensor/value", 10, BL_encoder_cb);
    auto CR_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("rh_left_motor_sensor/value", 10, CR_encoder_cb);
    auto CL_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("rh_right_motor_sensor/value", 10, CL_encoder_cb);
    auto DR_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("lh_left_motor_sensor/value", 10, DR_encoder_cb);
    auto DL_encoder_sub = g_node->create_subscription<std_msgs::msg::Float64>("lh_right_motor_sensor/value", 10, DL_encoder_cb);

    // Subscribe to IMU
    auto imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imu_cb);

    // Subscribe to motor commands from controller
    auto motor_cmd_sub = g_node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10, motor_cmd_cb);

    // Publishers for feedback
    auto motor_state_pub = g_node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000);
    auto imu_pub = g_node->create_publisher<sensor_msgs::msg::Imu>("imu/filtered", 1000);
    auto trigger_pub = g_node->create_publisher<corgi_msgs::msg::TriggerStamped>("trigger", 1000);

    // rclcpp::WallRate rate(1000ms); // 1kHz control loop
    // rclcpp::Rate rate(1000.0, g_node->get_clock());
    // use_sim_time setting
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = g_node->now();
    signal(SIGINT, signal_handler);

    trigger.enable = true;

    // Declare and read parameter (default empty) rather than blocking on stdin.
    g_node->declare_parameter<std::string>("output_filename", "");
    trigger.output_filename = g_node->get_parameter("output_filename").as_string();

    RCLCPP_INFO(g_node->get_logger(), "Starting simulation loop...");

    int loop_counter = 0;
    std_msgs::msg::Float64 motor_cmd_msg;

    while (rclcpp::ok())
    {
        // Process callbacks
        rclcpp::spin_some(g_node);

        // Convert theta-beta commands to motor positions
        double ar_phi, al_phi, br_phi, bl_phi, cr_phi, cl_phi, dr_phi, dl_phi;

        tb2phi(motor_cmd.module_a, ar_phi, al_phi, AR_phi, AL_phi);
        tb2phi(motor_cmd.module_b, br_phi, bl_phi, BR_phi, BL_phi);
        tb2phi(motor_cmd.module_c, cr_phi, cl_phi, CR_phi, CL_phi);
        tb2phi(motor_cmd.module_d, dr_phi, dl_phi, DR_phi, DL_phi);

        // Publish motor position commands (skip NaNs)
        auto publish_safe = [&](auto &pub, double value)
        {
            if (std::isfinite(value))
            {
                motor_cmd_msg.data = value;
                pub->publish(motor_cmd_msg);
            }
        };
        publish_safe(AR_motor_pub, ar_phi);
        publish_safe(AL_motor_pub, al_phi);
        publish_safe(BR_motor_pub, br_phi);
        publish_safe(BL_motor_pub, bl_phi);
        publish_safe(CR_motor_pub, cr_phi);
        publish_safe(CL_motor_pub, cl_phi);
        publish_safe(DR_motor_pub, dr_phi);
        publish_safe(DL_motor_pub, dl_phi);

        // Update motor_state feedback
        phi2tb(AR_phi, AL_phi, motor_state.module_a.theta, motor_state.module_a.beta);
        phi2tb(BR_phi, BL_phi, motor_state.module_b.theta, motor_state.module_b.beta);
        phi2tb(CR_phi, CL_phi, motor_state.module_c.theta, motor_state.module_c.beta);
        phi2tb(DR_phi, DL_phi, motor_state.module_d.theta, motor_state.module_d.beta);

        motor_state.header.stamp = g_node->get_clock()->now();
        motor_state.module_a.velocity_r = AR_phi_dot;
        motor_state.module_a.velocity_l = AL_phi_dot;
        motor_state.module_b.velocity_r = BR_phi_dot;
        motor_state.module_b.velocity_l = BL_phi_dot;
        motor_state.module_c.velocity_r = CR_phi_dot;
        motor_state.module_c.velocity_l = CL_phi_dot;
        motor_state.module_d.velocity_r = DR_phi_dot;
        motor_state.module_d.velocity_l = DL_phi_dot;

        // Filter IMU (remove gravity)
        Eigen::Quaterniond orientation(imu.orientation.w, imu.orientation.x,
                                       imu.orientation.y, imu.orientation.z);
        Eigen::Vector3d linear_acceleration(imu.linear_acceleration.x,
                                            imu.linear_acceleration.y,
                                            imu.linear_acceleration.z);
        Eigen::Vector3d gravity_global(0, 0, 9.81);
        Eigen::Vector3d gravity_body = orientation.inverse() * gravity_global;
        linear_acceleration -= gravity_body;

        imu_filtered.header.stamp = g_node->get_clock()->now();
        imu_filtered.orientation = imu.orientation;
        imu_filtered.angular_velocity = imu.angular_velocity;
        imu_filtered.linear_acceleration.x = linear_acceleration(0);
        imu_filtered.linear_acceleration.y = linear_acceleration(1);
        imu_filtered.linear_acceleration.z = linear_acceleration(2);

        // Publish feedback
        motor_state_pub->publish(motor_state);
        trigger_pub->publish(trigger);
        imu_pub->publish(imu_filtered);

        loop_counter++;
        // Maintain loop rate
        next_time += period;
        if (!g_node->get_clock()->sleep_until(next_time))
        {
            RCLCPP_WARN(g_node->get_logger(), "Sleep until failed!");
            break;
        }
        // rate.sleep();
    }

    trigger.enable = false;
    trigger_pub->publish(trigger);
    rclcpp::shutdown();
    return 0;
}
