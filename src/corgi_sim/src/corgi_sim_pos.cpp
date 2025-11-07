#include <iostream>
#include <chrono>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "corgi_sim/srv/set_int.hpp"
#include <corgi_sim/srv/set_float.hpp>
#include <corgi_sim/srv/get_uint64.hpp>
#include <corgi_sim/srv/motor_set_control_pid.hpp>
#include <corgi_sim/srv/node_get_position.hpp>
#include <corgi_sim/srv/node_get_orientation.hpp>
#include <corgi_sim/msg/float64_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/sim_data_stamped.hpp>

using namespace std::chrono_literals;

corgi_msgs::msg::MotorCmdStamped motor_cmd;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::TriggerStamped trigger;
corgi_msgs::msg::SimDataStamped sim_data;
sensor_msgs::msg::Imu imu;
sensor_msgs::msg::Imu imu_filtered;

double AR_phi = 0.0;
double AL_phi = 0.0;
double BR_phi = 0.0;
double BL_phi = 0.0;
double CR_phi = 0.0;
double CL_phi = 0.0;
double DR_phi = 0.0;
double DL_phi = 0.0;

corgi_sim::srv::SetFloat AR_motor_pos_srv;
corgi_sim::srv::SetFloat AL_motor_pos_srv;
corgi_sim::srv::SetFloat BR_motor_pos_srv;
corgi_sim::srv::SetFloat BL_motor_pos_srv;
corgi_sim::srv::SetFloat CR_motor_pos_srv;
corgi_sim::srv::SetFloat CL_motor_pos_srv;
corgi_sim::srv::SetFloat DR_motor_pos_srv;
corgi_sim::srv::SetFloat DL_motor_pos_srv;

corgi_sim::srv::MotorSetControlPid AR_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid AL_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid BR_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid BL_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid CR_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid CL_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid DR_motor_pid_srv;
corgi_sim::srv::MotorSetControlPid DL_motor_pid_srv;

corgi_sim::srv::SetInt time_step_srv;

// convert all service objects into seperate request objects for ROS2 compatibility (removed node_value_srv, etc.)

rosgraph_msgs::msg::Clock simulationClock;

void motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd) { motor_cmd = cmd; }

void AR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { AR_phi = phi.data; }
void AL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { AL_phi = phi.data; }
void BR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { BR_phi = phi.data; }
void BL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { BL_phi = phi.data; }
void CR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { CR_phi = phi.data; }
void CL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { CL_phi = phi.data; }
void DR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { DR_phi = phi.data; }
void DL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { DL_phi = phi.data; }

void AR_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_a.torque_r = trq.data; }
void AL_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_a.torque_l = trq.data; }
void BR_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_b.torque_r = trq.data; }
void BL_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_b.torque_l = trq.data; }
void CR_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_c.torque_r = trq.data; }
void CL_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_c.torque_l = trq.data; }
void DR_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_d.torque_r = trq.data; }
void DL_torque_cb(corgi_sim::msg::Float64Stamped trq) { motor_state.module_d.torque_l = trq.data; }

void gyro_cb(sensor_msgs::msg::Imu values) { imu.orientation = values.orientation; }
void ang_vel_cb(sensor_msgs::msg::Imu values) { imu.angular_velocity = values.angular_velocity; }
void imu_cb(sensor_msgs::msg::Imu values) { imu.linear_acceleration = values.linear_acceleration; }

// --- CHANGED ---
// This function is refactored. Instead of modifying global service objects,
// it now creates and returns a new PID Request *pointer*.
auto create_pid_request(double kp, double ki, double kd)
{
    auto request = std::make_shared<corgi_sim::srv::MotorSetControlPid::Request>();
    request->controlp = kp;
    request->controli = ki;
    request->controld = kd;
    return request;
}

double find_closest_phi(double phi_ref, double phi_fb)
{
    double diff = fmod(phi_ref - phi_fb + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    return phi_fb + diff - M_PI;
}

void phi2tb(double phi_r, double phi_l, double &theta, double &beta)
{
    theta = (phi_l - phi_r) / 2.0 + 17 / 180.0 * M_PI;
    beta = (phi_l + phi_r) / 2.0;
}

void tb2phi(double theta, double beta, double &phi_r, double &phi_l, double phi_r_fb, double phi_l_fb)
{
    double theta_0 = 17 / 180.0 * M_PI;
    if (theta < theta_0)
    {
        theta = theta_0;
    }
    phi_r = find_closest_phi(beta - theta + theta_0, phi_r_fb);
    phi_l = find_closest_phi(beta + theta - theta_0, phi_l_fb);
}

std::string get_lastest_input()
{
    std::string input;
    // --- CHANGED ---
    // This function will block forever. std::getline blocks.
    // I'll leave the logic, but this is probably not what you want in a ROS node.
    // For this example, it's fine.
    std::getline(std::cin, input);
    return input;
}

// We need the node pointer to log and shutdown, so we make it global.
rclcpp::Node::SharedPtr g_node = nullptr;

void signal_handler(int signum)
{
    RCLCPP_INFO(g_node->get_logger(), "Interrupt received.");
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // --- CHANGED ---
    // Assign to the global pointer so the signal handler can use it.
    g_node = rclcpp::Node::make_shared("corgi_sim");
    RCLCPP_INFO(g_node->get_logger(), "Corgi Simulation Starts\n");

    // --- CHANGED ---
    // You MUST create the client *before* you can wait for its service.
    auto time_step_client = g_node->create_client<corgi_sim::srv::SetInt>("robot/time_step");
    auto clock_pub = g_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    // --- CHANGED ---
    // This is the ROS2 way to wait for a service.
    while (!time_step_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(g_node->get_logger(), "Interrupted while waiting for service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(g_node->get_logger(), "service not available, waiting again...");
    }

    // --- CHANGED ---
    // This was spinning on a non-existent 'node' variable.
    // We'll spin inside the loop instead.
    // rclcpp::spin_some(node); // <--- DELETED

    // --- CHANGED ---
    // Create a client for the /supervisor/get_self service
    auto get_self_client = g_node->create_client<corgi_sim::srv::GetUint64>("/supervisor/get_self");

    auto node_pos_client = g_node->create_client<corgi_sim::srv::NodeGetPosition>("supervisor/node/get_position");
    auto node_orient_client = g_node->create_client<corgi_sim::srv::NodeGetOrientation>("supervisor/node/get_orientation");

    auto AR_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("lf_left_motor/set_position");
    auto AL_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("lf_right_motor/set_position");
    auto BR_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("rf_left_motor/set_position");
    auto BL_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("rf_right_motor/set_position");
    auto CR_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("rh_left_motor/set_position");
    auto CL_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("rh_right_motor/set_position");
    auto DR_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("lh_left_motor/set_position");
    auto DL_motor_pos_client = g_node->create_client<corgi_sim::srv::SetFloat>("lh_right_motor/set_position");

    auto AR_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("lf_left_motor/set_control_pid");
    auto AL_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("lf_right_motor/set_control_pid");
    auto BR_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("rf_left_motor/set_control_pid");
    auto BL_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("rf_right_motor/set_control_pid");
    auto CR_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("rh_left_motor/set_control_pid");
    auto CL_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("rh_right_motor/set_control_pid");
    auto DR_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("lh_left_motor/set_control_pid");
    auto DL_motor_pid_client = g_node->create_client<corgi_sim::srv::MotorSetControlPid>("lh_right_motor/set_control_pid");

    auto AR_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_left_motor_sensor/value", 1, AR_encoder_cb);
    auto AL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_right_motor_sensor/value", 1, AL_encoder_cb);
    auto BR_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rf_left_motor_sensor/value", 1, BR_encoder_cb);
    auto BL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rf_right_motor_sensor/value", 1, BL_encoder_cb);
    auto CR_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rh_left_motor_sensor/value", 1, CR_encoder_cb);
    auto CL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rh_right_motor_sensor/value", 1, CL_encoder_cb);
    auto DR_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lh_left_motor_sensor/value", 1, DR_encoder_cb);
    auto DL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lh_right_motor_sensor/value", 1, DL_encoder_cb);

    auto AR_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_left_motor/torque_feedback", 1, AR_torque_cb);
    auto AL_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_right_motor/torque_feedback", 1, AL_torque_cb);
    auto BR_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rf_left_motor/torque_feedback", 1, BR_torque_cb);
    auto BL_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rf_right_motor/torque_feedback", 1, BL_torque_cb);
    auto CR_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rh_left_motor/torque_feedback", 1, CR_torque_cb);
    auto CL_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("rh_right_motor/torque_feedback", 1, CL_torque_cb);
    auto DR_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lh_left_motor/torque_feedback", 1, DR_torque_cb);
    auto DL_torque_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lh_right_motor/torque_feedback", 1, DL_torque_cb);

    auto gyro_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("gyro/quaternion", 1, gyro_cb);
    auto ang_vel_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("ang_vel/values", 1, ang_vel_cb);
    auto imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("imu/values", 1, imu_cb);

    auto motor_cmd_sub = g_node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1, motor_cmd_cb);
    auto motor_state_pub = g_node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000);
    auto imu_pub = g_node->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
    auto trigger_pub = g_node->create_publisher<corgi_msgs::msg::TriggerStamped>("trigger", 1000);
    auto sim_data_pub = g_node->create_publisher<corgi_msgs::msg::SimDataStamped>("sim/data", 1000);

    rclcpp::WallRate rate(1000);

    // --- CHANGED ---
    // This is the new, asynchronous way to call a service and wait for it.
    uint64_t node_id;
    { // Use a scope for temporary request/future
        auto request = std::make_shared<corgi_sim::srv::GetUint64::Request>();
        request->ask = true;

        auto future = get_self_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to call service /supervisor/get_self");
            rclcpp::shutdown();
            return 1;
        }
        auto response = future.get();
        node_id = response->value;
    }
    RCLCPP_INFO(g_node->get_logger(), "Got node ID: %ld", node_id);

    signal(SIGINT, signal_handler);

    trigger.enable = true;

    // --- CHANGED ---
    // Create the request *before* the loop, since it's constant.
    auto time_step_request = std::make_shared<corgi_sim::srv::SetInt::Request>();
    time_step_request->value = 1;

    std::cout << "\nInput the output filename and press Enter to start the simulation: ";
    trigger.output_filename = get_lastest_input();

    // --- CHANGED ---
    // Use the new `create_pid_request` function
    auto pid_request = create_pid_request(30, 0, 0.1);

    // Call and wait for each PID service
    rclcpp::spin_until_future_complete(g_node, AR_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, AL_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, BR_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, BL_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, CR_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, CL_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, DR_motor_pid_client->async_send_request(pid_request));
    rclcpp::spin_until_future_complete(g_node, DL_motor_pid_client->async_send_request(pid_request));

    // --- CHANGED ---
    // Create request objects *before* the loop. We will re-use them
    // to avoid creating new objects every millisecond.
    auto node_pos_request = std::make_shared<corgi_sim::srv::NodeGetPosition::Request>();
    node_pos_request->node = node_id;
    auto node_orien_request = std::make_shared<corgi_sim::srv::NodeGetOrientation::Request>();
    node_orien_request->node = node_id;

    auto ar_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto al_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto br_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto bl_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto cr_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto cl_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto dr_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto dl_pos_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();

    int loop_counter = 0;

    // --- CHANGED ---
    // Refactored the main loop logic
    while (rclcpp::ok())
    {
        // --- 1. Call time_step and check response ---
        auto time_step_future = time_step_client->async_send_request(time_step_request);
        if (rclcpp::spin_until_future_complete(g_node, time_step_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node->get_logger(), "Time step service call failed. Exiting.");
            break;
        }
        // You can check the response if the service provides one:
        // auto time_step_response = time_step_future.get();
        // if (!time_step_response->success) { break; }

        // --- 2. Spin for callbacks ---
        rclcpp::spin_some(g_node); // --- CHANGED --- Was 'node'

        // --- 3. Get Node Position/Orientation ---
        // Send requests
        auto pos_future = node_pos_client->async_send_request(node_pos_request);
        auto orien_future = node_orient_client->async_send_request(node_orien_request);

        // Wait for responses
        if (rclcpp::spin_until_future_complete(g_node, pos_future) != rclcpp::FutureReturnCode::SUCCESS ||
            rclcpp::spin_until_future_complete(g_node, orien_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to get node pose. Exiting.");
            break;
        }

        auto pos_response = pos_future.get();
        auto orien_response = orien_future.get();

        // --- 4. Update sim_data ---
        // sim_data.header.seq = loop_counter; // --- CHANGED --- 'seq' does not exist
        sim_data.header.stamp = g_node->get_clock()->now(); // Use current time
        sim_data.position = pos_response->position;
        sim_data.orientation = orien_response->orientation;

        // --- 5. Update motor position requests ---
        tb2phi(motor_cmd.module_a.theta, motor_cmd.module_a.beta, ar_pos_req->value, al_pos_req->value, AR_phi, AL_phi);
        tb2phi(motor_cmd.module_b.theta, motor_cmd.module_b.beta, br_pos_req->value, bl_pos_req->value, BR_phi, BL_phi);
        tb2phi(motor_cmd.module_c.theta, motor_cmd.module_c.beta, cr_pos_req->value, cl_pos_req->value, CR_phi, CL_phi);
        tb2phi(motor_cmd.module_d.theta, motor_cmd.module_d.beta, dr_pos_req->value, dl_pos_req->value, DR_phi, DL_phi);

        // --- 6. Send motor position requests ---
        // These can be fire-and-forget, so we don't wait for the future.
        AR_motor_pos_client->async_send_request(ar_pos_req);
        AL_motor_pos_client->async_send_request(al_pos_req);
        BR_motor_pos_client->async_send_request(br_pos_req);
        BL_motor_pos_client->async_send_request(bl_pos_req);
        CR_motor_pos_client->async_send_request(cr_pos_req);
        CL_motor_pos_client->async_send_request(cl_pos_req);
        DR_motor_pos_client->async_send_request(dr_pos_req);
        DL_motor_pos_client->async_send_request(dl_pos_req);

        // --- 7. Update motor_state ---
        phi2tb(AR_phi, AL_phi, motor_state.module_a.theta, motor_state.module_a.beta);
        phi2tb(BR_phi, BL_phi, motor_state.module_b.theta, motor_state.module_b.beta);
        phi2tb(CR_phi, CL_phi, motor_state.module_c.theta, motor_state.module_c.beta);
        phi2tb(DR_phi, DL_phi, motor_state.module_d.theta, motor_state.module_d.beta);

        // motor_state.header.seq = loop_counter; // --- CHANGED --- 'seq' does not exist
        motor_state.header.stamp = g_node->get_clock()->now(); // Use current time

        // --- 8. IMU Filtering ---
        Eigen::Quaterniond orientation(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
        Eigen::Vector3d linear_acceleration(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        Eigen::Vector3d gravity_global(0, 0, 9.81);
        Eigen::Vector3d gravity_body = orientation.inverse() * gravity_global;

        linear_acceleration -= gravity_body;

        imu_filtered.orientation = imu.orientation;
        imu_filtered.angular_velocity = imu.angular_velocity;
        imu_filtered.linear_acceleration.x = linear_acceleration(0);
        imu_filtered.linear_acceleration.y = linear_acceleration(1);
        imu_filtered.linear_acceleration.z = linear_acceleration(2);

        // --- 9. Publish messages ---
        motor_state_pub->publish(motor_state);
        trigger_pub->publish(trigger);
        imu_pub->publish(imu_filtered);
        sim_data_pub->publish(sim_data);

        double clock = loop_counter * 0.001;
        simulationClock.clock.sec = (int)clock;
        simulationClock.clock.nanosec = round(1000 * (clock - simulationClock.clock.sec)) * 1.0e+6;
        clock_pub->publish(simulationClock);

        loop_counter++;

        rate.sleep();
    }

    trigger.enable = false;
    trigger_pub->publish(trigger);

    rclcpp::shutdown();
    return 0;
}