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
#include <corgi_sim/srv/set_int.hpp>
#include <corgi_sim/srv/set_float.hpp>
#include <corgi_sim/srv/field_get_string.hpp>
#include <corgi_sim/srv/get_uint64.hpp>
#include <corgi_sim/srv/motor_set_control_pid.hpp>
#include <corgi_sim/srv/node_enable_contact_points_tracking.hpp>
#include <corgi_sim/srv/node_get_contact_points.hpp>
#include <corgi_sim/srv/node_get_def.hpp>
#include <corgi_sim/srv/node_get_field.hpp>
#include <corgi_sim/srv/node_get_position.hpp>
#include <corgi_sim/srv/node_get_orientation.hpp>
#include <corgi_sim/srv/supervisor_get_from_id.hpp>
#include <corgi_sim/msg/float64_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/sim_contact_point.hpp>
#include <corgi_msgs/msg/sim_data_stamped.hpp>

using namespace std::chrono_literals;

// Global message objects
corgi_msgs::msg::MotorCmdStamped motor_cmd;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::TriggerStamped trigger;
corgi_msgs::msg::SimDataStamped sim_data;
sensor_msgs::msg::Imu imu;
sensor_msgs::msg::Imu imu_filtered;

// Global state variables
double AR_phi = 0.0, AL_phi = 0.0, BR_phi = 0.0, BL_phi = 0.0;
double CR_phi = 0.0, CL_phi = 0.0, DR_phi = 0.0, DL_phi = 0.0;
double AR_phi_dot = 0.0, AL_phi_dot = 0.0, BR_phi_dot = 0.0, BL_phi_dot = 0.0;
double CR_phi_dot = 0.0, CL_phi_dot = 0.0, DR_phi_dot = 0.0, DL_phi_dot = 0.0;

// --- CHANGED ---
// All global cgi_sim::srv::... objects have been removed.
// They were the source of the ROS1-style errors.

rosgraph_msgs::msg::Clock simulationClock;

// --- CHANGED ---
// Global node pointer for signal handler
rclcpp::Node::SharedPtr g_node = nullptr;

void motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd) { motor_cmd = cmd; }

// Callbacks are fine
void AR_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    AR_phi_dot = (phi.data - AR_phi) * 1000;
    AR_phi = phi.data;
}
void AL_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    AL_phi_dot = (phi.data - AL_phi) * 1000;
    AL_phi = phi.data;
}
void BR_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    BR_phi_dot = (phi.data - BR_phi) * 1000;
    BR_phi = phi.data;
}
void BL_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    BL_phi_dot = (phi.data - BL_phi) * 1000;
    BL_phi = phi.data;
}
void CR_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    CR_phi_dot = (phi.data - CR_phi) * 1000;
    CR_phi = phi.data;
}
void CL_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    CL_phi_dot = (phi.data - CL_phi) * 1000;
    CL_phi = phi.data;
}
void DR_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    DR_phi_dot = (phi.data - DR_phi) * 1000;
    DR_phi = phi.data;
}
void DL_encoder_cb(corgi_sim::msg::Float64Stamped phi)
{
    DL_phi_dot = (phi.data - DL_phi) * 1000;
    DL_phi = phi.data;
}

void gyro_cb(sensor_msgs::msg::Imu values) { imu.orientation = values.orientation; }
void ang_vel_cb(sensor_msgs::msg::Imu values) { imu.angular_velocity = values.angular_velocity; }
void imu_cb(sensor_msgs::msg::Imu values) { imu.linear_acceleration = values.linear_acceleration; }

// Helper functions (no ROS code) are unchanged
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

void tb2phi(corgi_msgs::msg::MotorCmd motor_cmd, double &trq_r, double &trq_l, double phi_r_fb, double phi_l_fb, double phi_r_dot_fb, double phi_l_dot_fb)
{
    if (motor_cmd.theta == 0)
    {
        motor_cmd.kp_r = 90;
        motor_cmd.kp_l = 90;
        motor_cmd.kd_r = 1.75;
        motor_cmd.kd_l = 1.75;
    }
    double theta_0 = 17 / 180.0 * M_PI;
    if (motor_cmd.theta < theta_0)
    {
        motor_cmd.theta = theta_0;
    }
    double phi_r = find_closest_phi(motor_cmd.beta - motor_cmd.theta + theta_0, phi_r_fb);
    double phi_l = find_closest_phi(motor_cmd.beta + motor_cmd.theta - theta_0, phi_l_fb);
    trq_r = motor_cmd.kp_r * (phi_r - phi_r_fb) + motor_cmd.kd_r * (-phi_r_dot_fb) + motor_cmd.torque_r;
    trq_l = motor_cmd.kp_l * (phi_l - phi_l_fb) + motor_cmd.kd_l * (-phi_l_dot_fb) + motor_cmd.torque_l;
}

std::string get_lastest_input()
{
    std::string input;
    // This is still a blocking call, which is unusual for ROS
    // but we'll leave the logic as-is.
    std::getline(std::cin, input);
    return input;
}

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
    // Assign to global node pointer
    g_node = rclcpp::Node::make_shared("corgi_sim");
    RCLCPP_INFO(g_node->get_logger(), "Corgi Simulation Starts\n");

    // --- CHANGED ---
    // All 'nh->' changed to 'g_node->'
    auto clock_pub = g_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    // Create clients
    auto time_step_client = g_node->create_client<corgi_sim::srv::SetInt>("robot/time_step");
    auto get_self_client = g_node->create_client<corgi_sim::srv::GetUint64>("/supervisor/get_self");
    auto node_pos_client = g_node->create_client<corgi_sim::srv::NodeGetPosition>("supervisor/node/get_position");
    auto node_orient_client = g_node->create_client<corgi_sim::srv::NodeGetOrientation>("supervisor/node/get_orientation");
    auto AR_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("lf_left_motor/set_torque");
    auto AL_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("lf_right_motor/set_torque");
    auto BR_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("rf_left_motor/set_torque");
    auto BL_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("rf_right_motor/set_torque");
    auto CR_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("rh_left_motor/set_torque");
    auto CL_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("rh_right_motor/set_torque");
    auto DR_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("lh_left_motor/set_torque");
    auto DL_motor_trq_client = g_node->create_client<corgi_sim::srv::SetFloat>("lh_right_motor/set_torque");
    auto cp_enable_client = g_node->create_client<corgi_sim::srv::NodeEnableContactPointsTracking>("supervisor/node/enable_contact_points_tracking");
    auto cp_get_client = g_node->create_client<corgi_sim::srv::NodeGetContactPoints>("supervisor/node/get_contact_points");

    // --- CHANGED ---
    // Clients for the contact point lookup loop
    auto get_from_id_client = g_node->create_client<corgi_sim::srv::SupervisorGetFromId>("supervisor/get_from_id");
    auto get_field_client = g_node->create_client<corgi_sim::srv::NodeGetField>("supervisor/node/get_field");
    auto get_string_client = g_node->create_client<corgi_sim::srv::FieldGetString>("supervisor/field/get_string");
    auto get_def_client = g_node->create_client<corgi_sim::srv::NodeGetDef>("supervisor/node/get_def");

    // Wait for a critical service
    while (!time_step_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(g_node->get_logger(), "Interrupted while waiting for service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(g_node->get_logger(), "service not available, waiting again...");
    }

    // Subscriptions
    auto AR_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_left_motor_sensor/value", 1, AR_encoder_cb);
    auto AL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lf_right_motor_sensor/value", 1, AL_encoder_cb);
    // ... (rest of subscriptions are correct) ...
    auto DL_encoder_sub = g_node->create_subscription<corgi_sim::msg::Float64Stamped>("lh_right_motor_sensor/value", 1, DL_encoder_cb);
    auto gyro_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("gyro/quaternion", 1, gyro_cb);
    auto ang_vel_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("ang_vel/values", 1, ang_vel_cb);
    auto imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>("imu/values", 1, imu_cb);
    auto motor_cmd_sub = g_node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1, motor_cmd_cb);

    // Publishers
    auto motor_state_pub = g_node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000);
    auto imu_pub = g_node->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
    auto trigger_pub = g_node->create_publisher<corgi_msgs::msg::TriggerStamped>("trigger", 1000);
    auto sim_data_pub = g_node->create_publisher<corgi_msgs::msg::SimDataStamped>("sim/data", 1000);

    rclcpp::WallRate rate(1000ms);

    // --- CHANGED ---
    // This section is now ROS2-compliant
    uint64_t node_id;
    {
        auto request = std::make_shared<corgi_sim::srv::GetUint64::Request>();
        request->ask = true;
        auto future = get_self_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to call service /supervisor/get_self. Exiting.");
            return 1;
        }
        node_id = future.future.get()->value;
    }

    // Prepare requests that will be re-used in the loop
    auto node_pos_request = std::make_shared<corgi_sim::srv::NodeGetPosition::Request>();
    node_pos_request->node = node_id;
    auto node_orien_request = std::make_shared<corgi_sim::srv::NodeGetOrientation::Request>();
    node_orien_request->node = node_id;

    auto cp_enable_request = std::make_shared<corgi_sim::srv::NodeEnableContactPointsTracking::Request>();
    cp_enable_request->node = node_id;
    cp_enable_request->include_descendants = true;

    auto cp_get_request = std::make_shared<corgi_sim::srv::NodeGetContactPoints::Request>();
    cp_get_request->node = node_id;
    cp_get_request->include_descendants = true;

    // Enable contact points *once*
    auto cp_enable_future = cp_enable_client->async_send_request(cp_enable_request);
    if (rclcpp::spin_until_future_complete(g_node, cp_enable_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to enable contact points. Exiting.");
        return 1;
    }

    signal(SIGINT, signal_handler);

    trigger.enable = true;
    auto time_step_request = std::make_shared<corgi_sim::srv::SetInt::Request>();
    time_step_request->value = 1;

    std::cout << "\nInput the output filename and press Enter to start the simulation: ";
    trigger.output_filename = get_lastest_input();

    // Pre-allocate motor torque requests
    auto ar_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto al_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto br_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto bl_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto cr_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto cl_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto dr_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();
    auto dl_trq_req = std::make_shared<corgi_sim::srv::SetFloat::Request>();

    int loop_counter = 0;

    // --- CHANGED ---
    // Main loop converted to ROS2
    while (rclcpp::ok())
    {

        // 1. Call time_step to pace the loop
        auto time_step_future = time_step_client->async_send_request(time_step_request);
        if (rclcpp::spin_until_future_complete(g_node, time_step_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(g_node->get_logger(), "Time step service call failed. Simulation may have ended.");
            break;
        }

        // 2. Spin for callbacks
        rclcpp::spin_some(g_node);

        // 3. Get node pose
        auto pos_future = node_pos_client->async_send_request(node_pos_request);
        auto orien_future = node_orient_client->async_send_request(node_orien_request);

        // Wait for pose
        if (rclcpp::spin_until_future_complete(g_node, pos_future) != rclcpp::FutureReturnCode::SUCCESS ||
            rclcpp::spin_until_future_complete(g_node, orien_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to get node pose. Exiting.");
            break;
        }
        auto pos_response = pos_future.future.get();
        auto orien_response = orien_future.future.get();

        sim_data.header.stamp = g_node->get_clock()->now();
        sim_data.position = pos_response->position;
        sim_data.orientation = orien_response->orientation;

        // 4. Calculate and set motor torques
        double ar_trq_r, al_trq_l, br_trq_r, bl_trq_l, cr_trq_r, cl_trq_l, dr_trq_r, dl_trq_l;

        tb2phi(motor_cmd.module_a, ar_trq_r, al_trq_l, AR_phi, AL_phi, AR_phi_dot, AL_phi_dot);
        tb2phi(motor_cmd.module_b, br_trq_r, bl_trq_l, BR_phi, BL_phi, BR_phi_dot, BL_phi_dot);
        tb2phi(motor_cmd.module_c, cr_trq_r, cl_trq_l, CR_phi, CL_phi, CR_phi_dot, CL_phi_dot);
        tb2phi(motor_cmd.module_d, dr_trq_r, dl_trq_l, DR_phi, DL_phi, DR_phi_dot, DL_phi_dot);

        ar_trq_req->value = ar_trq_r;
        AL_motor_trq_client->async_send_request(al_trq_req);
        al_trq_req->value = al_trq_l;
        AR_motor_trq_client->async_send_request(ar_trq_req);
        br_trq_req->value = br_trq_r;
        BR_motor_trq_client->async_send_request(br_trq_req);
        bl_trq_req->value = bl_trq_l;
        BL_motor_trq_client->async_send_request(bl_trq_req);
        cr_trq_req->value = cr_trq_r;
        CR_motor_trq_client->async_send_request(cr_trq_req);
        cl_trq_req->value = cl_trq_l;
        CL_motor_trq_client->async_send_request(cl_trq_req);
        dr_trq_req->value = dr_trq_r;
        DR_motor_trq_client->async_send_request(dr_trq_req);
        dl_trq_req->value = dl_trq_l;
        DL_motor_trq_client->async_send_request(dl_trq_req);

        // 5. Update motor_state message
        phi2tb(AR_phi, AL_phi, motor_state.module_a.theta, motor_state.module_a.beta);
        phi2tb(BR_phi, BL_phi, motor_state.module_b.theta, motor_state.module_b.beta);
        phi2tb(CR_phi, CL_phi, motor_state.module_c.theta, motor_state.module_c.beta);
        phi2tb(DR_phi, DL_phi, motor_state.module_d.theta, motor_state.module_d.beta);

        motor_state.header.stamp = g_node->get_clock()->now();
        motor_state.module_a.torque_r = ar_trq_r;
        motor_state.module_a.torque_l = al_trq_l;
        motor_state.module_b.torque_r = br_trq_r;
        motor_state.module_b.torque_l = bl_trq_l;
        motor_state.module_c.torque_r = cr_trq_r;
        motor_state.module_c.torque_l = cl_trq_l;
        motor_state.module_d.torque_r = dr_trq_r;
        motor_state.module_d.torque_l = dl_trq_l;

        motor_state.module_a.velocity_r = AR_phi_dot;
        motor_state.module_a.velocity_l = AL_phi_dot;
        motor_state.module_b.velocity_r = BR_phi_dot;
        motor_state.module_b.velocity_l = BL_phi_dot;
        motor_state.module_c.velocity_r = CR_phi_dot;
        motor_state.module_c.velocity_l = CL_phi_dot;
        motor_state.module_d.velocity_r = DR_phi_dot;
        motor_state.module_d.velocity_l = DL_phi_dot;

        // 6. Get Contact Points (The complex, nested part)
        auto cp_get_future = cp_get_client->async_send_request(cp_get_request);
        if (rclcpp::spin_until_future_complete(g_node, cp_get_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto cp_response = cp_get_future.future.get();
            size_t n = cp_response->contact_points.size();
            sim_data.contact.clear();
            if (n > 0)
            {
                // Note: This is a lot of synchronous service calls inside a 1ms loop.
                // This is very slow, but it matches the original ROS1 logic.
                for (size_t m = 0; m < n; m++)
                {
                    const auto &p = cp_response->contact_points[m];

                    // 1. Get Node from ID
                    auto node_id_req = std::make_shared<corgi_sim::srv::SupervisorGetFromId::Request>();
                    node_id_req->id = p.node_id;
                    auto node_id_future = get_from_id_client->async_send_request(node_id_req);
                    if (rclcpp::spin_until_future_complete(g_node, node_id_future) != rclcpp::FutureReturnCode::SUCCESS)
                        continue;
                    auto node_id_resp = node_id_future.future.get();
                    uint64_t contact_node_handle = node_id_resp->node;

                    // 2. Get Field from Node
                    auto field_get_req = std::make_shared<corgi_sim::srv::NodeGetField::Request>();
                    field_get_req->node = contact_node_handle;
                    field_get_req->field_name = "name";
                    auto field_get_future = get_field_client->async_send_request(field_get_req);
                    if (rclcpp::spin_until_future_complete(g_node, field_get_future) != rclcpp::FutureReturnCode::SUCCESS)
                        continue;
                    auto field_get_resp = field_get_future.future.get();
                    uint64_t field_handle = field_get_resp->field;

                    // 3. Get String from Field
                    auto name_get_req = std::make_shared<corgi_sim::srv::FieldGetString::Request>();
                    name_get_req->field = field_handle;
                    auto name_get_future = get_string_client->async_send_request(name_get_req);
                    if (rclcpp::spin_until_future_complete(g_node, name_get_future) != rclcpp::FutureReturnCode::SUCCESS)
                        continue;
                    auto name_get_resp = name_get_future.future.get();

                    // 4. Get Def from Node
                    auto def_get_req = std::make_shared<corgi_sim::srv::NodeGetDef::Request>();
                    def_get_req->node = contact_node_handle;
                    auto def_get_future = get_def_client->async_send_request(def_get_req);
                    if (rclcpp::spin_until_future_complete(g_node, def_get_future) != rclcpp::FutureReturnCode::SUCCESS)
                        continue;
                    auto def_get_resp = def_get_future.future.get();

                    // 5. Populate message
                    corgi_msgs::msg::SimContactPoint contact;
                    contact.def_name = def_get_resp->name;
                    contact.name = name_get_resp->value;
                    contact.point = p.point;
                    sim_data.contact.push_back(contact);
                }
            }
        }

        // 7. IMU Filtering
        Eigen::Quaterniond orientation(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
        Eigen::Vector3d linear_acceleration(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        Eigen::Vector3d gravity_global(0, 0, 9.81);
        Eigen::Vector3d gravity_body = orientation.inverse() * gravity_global;

        linear_acceleration -= gravity_body;

        imu_filtered.orientation = imu.orientation; // --- CHANGED --- Removed duplicate .w assignment
        imu_filtered.angular_velocity = imu.angular_velocity;
        imu_filtered.linear_acceleration.x = linear_acceleration(0);
        imu_filtered.linear_acceleration.y = linear_acceleration(1);
        imu_filtered.linear_acceleration.z = linear_acceleration(2);

        // 8. Publish all messages
        // --- CHANGED ---
        // All publish calls now use the -> (arrow) operator
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
    trigger_pub->publish(trigger); // --- CHANGED --- Use ->

    rclcpp::shutdown();
    return 0;
}