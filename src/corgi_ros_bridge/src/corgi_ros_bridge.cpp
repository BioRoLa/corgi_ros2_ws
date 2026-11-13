#include <iostream>
#include <mutex>
#include <array>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "corgi_utils/module_directions.hpp"
#include "corgi_utils/module_frame.hpp"

#include "NodeHandler.h"
#include "Config.pb.h"
#include "Log.pb.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Robot.pb.h"
#include "Steering.pb.h"

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <corgi_msgs/msg/config_stamped.hpp>
#include <corgi_msgs/msg/log_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/power_state_stamped.hpp>
#include <corgi_msgs/msg/robot_cmd_stamped.hpp>
#include <corgi_msgs/msg/robot_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>

std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_robot_cmd;
std::mutex mutex_grpc_steer_cmd;
std::mutex mutex_grpc_config_cmd;

std::mutex mutex_ros_motor_state;
std::mutex mutex_ros_power_state;
std::mutex mutex_ros_robot_state;
std::mutex mutex_ros_steer_state;
std::mutex mutex_ros_config_state;

std::mutex mutex_ros_log;

corgi_msgs::msg::MotorCmdStamped         ros_motor_cmd;
corgi_msgs::msg::RobotCmdStamped         ros_robot_cmd;
corgi_msgs::msg::SteeringCmdStamped      ros_steer_cmd;
corgi_msgs::msg::ConfigStamped           ros_config_cmd;
corgi_msgs::msg::MotorStateStamped       ros_motor_state;
corgi_msgs::msg::PowerStateStamped       ros_power_state;
corgi_msgs::msg::RobotStateStamped       ros_robot_state;
corgi_msgs::msg::SteeringStateStamped    ros_steer_state;
corgi_msgs::msg::ConfigStamped           ros_config_state;
corgi_msgs::msg::LogStamped              ros_log;

motor_msg::MotorCmdStamped          grpc_motor_cmd;
robot_msg::RobotCmdStamped          grpc_robot_cmd;
steering_msg::SteeringCmdStamped    grpc_steer_cmd;
config_msg::ConfigStamped           grpc_config_cmd;
motor_msg::MotorStateStamped        grpc_motor_state;
power_msg::PowerStateStamped        grpc_power_state;
robot_msg::RobotStateStamped        grpc_robot_state;
steering_msg::SteeringStateStamped  grpc_steer_state;
config_msg::ConfigStamped           grpc_config_state;
log_msg::LogEntry                   grpc_log;

rclcpp::Publisher<corgi_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::PowerStateStamped>::SharedPtr ros_power_state_pub;
rclcpp::Publisher<corgi_msgs::msg::RobotStateStamped>::SharedPtr ros_robot_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::SteeringStateStamped>::SharedPtr ros_steer_state_pub;
rclcpp::Publisher<corgi_msgs::msg::ConfigStamped>::SharedPtr ros_config_state_pub;
rclcpp::Publisher<corgi_msgs::msg::LogStamped>::SharedPtr ros_log_pub;

core::Publisher<motor_msg::MotorCmdStamped>         *grpc_motor_cmd_pub;
core::Publisher<robot_msg::RobotCmdStamped>         *grpc_robot_cmd_pub;
core::Publisher<steering_msg::SteeringCmdStamped>   *grpc_steer_cmd_pub;
core::Publisher<config_msg::ConfigStamped>          *grpc_config_cmd_pub;

// The ROS <-> leg-module boundary (CONVENTIONS.md). Positions, velocities,
// torques and gains are converted together through one ModuleFrame per leg.
// Until 2026-11-13 only positions were converted (joint_dir) while torques and
// velocities took a separate motor_dir (+1 everywhere) and gains passed
// straight through, so on B/C (beta) and B/D (gamma) the angles and the torques
// were in different frames -- the force-plate sessions' sign surprises (log
// 344) and the vault's "Torque Frame Mismatch" note. legacy_torque_frame:=true
// restores that behaviour for A/B comparison and rollback only.
std::array<corgi::ModuleFrame, 4> module_frames;
bool legacy_torque_frame = false;

// The one direction table (CONVENTIONS.md section 5), shared with the sim
// driver: share/corgi_utils/config/module_directions.yaml, or
// CORGI_MODULE_DIRECTIONS if set. The module_directions_path parameter
// overrides both.
std::string default_module_directions_path()
{
    const char* env = std::getenv("CORGI_MODULE_DIRECTIONS");
    if (env != nullptr && *env != '\0') {
        return env;
    }
    return ament_index_cpp::get_package_share_directory("corgi_utils") + "/config/module_directions.yaml";
}

// Throws on anything it cannot use (corgi::load_module_directions): a bridge
// running with the wrong directions drives mirrored legs the wrong way, so it
// must not start (it used to fall back to all +1 with a warning).
void load_module_frames(const std::string& path, const rclcpp::Logger& logger)
{
    module_frames = corgi::load_module_directions(path);
    for (size_t i = 0; i < module_frames.size(); ++i) {
        const auto& d = module_frames[i].directions();
        RCLCPP_INFO(logger, "leg %s: theta %+.0f beta %+.0f gamma %+.0f",
                    corgi::module_leg_names()[i].c_str(), d.theta, d.beta, d.gamma);
    }
    RCLCPP_INFO(logger, "Loaded leg module directions: %s", path.c_str());
}

// Gains must be finite and non-negative. There is deliberately no upper
// bound here: the CAN field range applies AFTER fpga_driver divides each
// gain by its motor's KT, which only the driver knows, so range saturation
// (with its own warning) lives in fpga_driver.
double sanitize_gain(double value, const char* name)
{
    double clean = (std::isfinite(value) && value > 0.0) ? value : 0.0;
    if (clean != value) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("corgi_ros_bridge"), steady_clock, 1000,
            "%s=%g is negative or not finite, sent as 0", name, value);
    }
    return clean;
}

void ros_motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd){
    std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);

    ros_motor_cmd = cmd;

    std::vector<motor_msg::MotorCmd*> grpc_motor_modules = {
        grpc_motor_cmd.mutable_module_a(),
        grpc_motor_cmd.mutable_module_b(),
        grpc_motor_cmd.mutable_module_c(),
        grpc_motor_cmd.mutable_module_d()
    };

    std::vector<corgi_msgs::msg::MotorCmd> ros_motor_modules = {
        ros_motor_cmd.module_a,
        ros_motor_cmd.module_b,
        ros_motor_cmd.module_c,
        ros_motor_cmd.module_d
    };

    for (int i = 0; i < 4; i++) {
        const corgi::ModuleFrame& f = module_frames[i];
        const auto& m = ros_motor_modules[i];
        grpc_motor_modules[i]->set_theta(std::min(std::max(f.theta(m.theta), 17/180.0*M_PI), 160/180.0*M_PI));
        grpc_motor_modules[i]->set_beta(f.beta(m.beta));
        grpc_motor_modules[i]->set_gamma(f.gamma(m.gamma));

        // r/l gains swap and torques swap-and-negate on a mirrored-beta leg;
        // the ABAD torque follows gamma (CONVENTIONS.md section 5)
        const double kp_r = sanitize_gain(m.kp_r, "kp_r"), kp_l = sanitize_gain(m.kp_l, "kp_l");
        const double ki_r = sanitize_gain(m.ki_r, "ki_r"), ki_l = sanitize_gain(m.ki_l, "ki_l");
        const double kd_r = sanitize_gain(m.kd_r, "kd_r"), kd_l = sanitize_gain(m.kd_l, "kd_l");
        const auto kp = legacy_torque_frame ? std::make_pair(kp_r, kp_l) : f.gain_rl(kp_r, kp_l);
        const auto ki = legacy_torque_frame ? std::make_pair(ki_r, ki_l) : f.gain_rl(ki_r, ki_l);
        const auto kd = legacy_torque_frame ? std::make_pair(kd_r, kd_l) : f.gain_rl(kd_r, kd_l);
        const auto tq = legacy_torque_frame ? std::make_pair(m.torque_r, m.torque_l) : f.torque_rl(m.torque_r, m.torque_l);
        grpc_motor_modules[i]->set_kp_r(kp.first);
        grpc_motor_modules[i]->set_kp_l(kp.second);
        grpc_motor_modules[i]->set_kp_h(sanitize_gain(m.kp_h, "kp_h"));
        grpc_motor_modules[i]->set_ki_r(ki.first);
        grpc_motor_modules[i]->set_ki_l(ki.second);
        grpc_motor_modules[i]->set_ki_h(sanitize_gain(m.ki_h, "ki_h"));
        grpc_motor_modules[i]->set_kd_r(kd.first);
        grpc_motor_modules[i]->set_kd_l(kd.second);
        grpc_motor_modules[i]->set_kd_h(sanitize_gain(m.kd_h, "kd_h"));
        grpc_motor_modules[i]->set_torque_r(tq.first);
        grpc_motor_modules[i]->set_torque_l(tq.second);
        grpc_motor_modules[i]->set_torque_h(legacy_torque_frame ? m.torque_h : f.torque_h(m.torque_h));
    }

    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.stamp.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.stamp.nanosec / 1000);

    grpc_motor_cmd_pub->publish(grpc_motor_cmd);
}

void ros_steer_cmd_cb(const corgi_msgs::msg::SteeringCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_steer_cmd);

    ros_steer_cmd = cmd;

    grpc_steer_cmd.set_voltage(ros_steer_cmd.voltage);
    grpc_steer_cmd.set_angle(ros_steer_cmd.angle);

    grpc_steer_cmd.mutable_header()->set_seq(ros_steer_cmd.header.seq);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_sec(ros_steer_cmd.header.stamp.sec);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_usec(ros_steer_cmd.header.stamp.nanosec / 1000);

    grpc_steer_cmd_pub->publish(grpc_steer_cmd);
}

void ros_robot_cmd_cb(const corgi_msgs::msg::RobotCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_robot_cmd);

    ros_robot_cmd = cmd;

    grpc_robot_cmd.set_request_robot_mode(static_cast<robot_msg::ROBOTMODE>(ros_robot_cmd.request_robot_mode));

    grpc_robot_cmd.mutable_header()->set_seq(ros_robot_cmd.header.seq);
    grpc_robot_cmd.mutable_header()->mutable_stamp()->set_sec(ros_robot_cmd.header.stamp.sec);
    grpc_robot_cmd.mutable_header()->mutable_stamp()->set_usec(ros_robot_cmd.header.stamp.nanosec / 1000);

    grpc_robot_cmd_pub->publish(grpc_robot_cmd);
}

void ros_config_cmd_cb(const corgi_msgs::msg::ConfigStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_config_cmd);

    ros_config_cmd = cmd;

    // Map ROS message fields to gRPC proto fields according to Config.proto
    // grpc_config_cmd.set_transmit(ros_config_cmd.transmit);
    grpc_config_cmd.set_module(static_cast<config_msg::Module>(ros_config_cmd.module));
    grpc_config_cmd.set_motor(static_cast<config_msg::Motor>(ros_config_cmd.motor));
    grpc_config_cmd.set_mode(static_cast<config_msg::ConfigMode>(ros_config_cmd.mode));
    grpc_config_cmd.set_type(static_cast<config_msg::ConfigType>(ros_config_cmd.type));
    grpc_config_cmd.set_address(ros_config_cmd.address);
    grpc_config_cmd.set_value_f(ros_config_cmd.value_f);
    grpc_config_cmd.set_value_i(ros_config_cmd.value_i);
    grpc_config_cmd.set_error_code(ros_config_cmd.error_code);

    grpc_config_cmd.mutable_header()->set_seq(ros_config_cmd.header.seq);
    grpc_config_cmd.mutable_header()->mutable_stamp()->set_sec(ros_config_cmd.header.stamp.sec);
    grpc_config_cmd.mutable_header()->mutable_stamp()->set_usec(ros_config_cmd.header.stamp.nanosec / 1000);

    grpc_config_cmd_pub->publish(grpc_config_cmd);
}

void grpc_motor_state_cb(const motor_msg::MotorStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_motor_state);

    grpc_motor_state = state;

    std::vector<const motor_msg::MotorState*> grpc_motor_modules = {
        &grpc_motor_state.module_a(),
        &grpc_motor_state.module_b(),
        &grpc_motor_state.module_c(),
        &grpc_motor_state.module_d()
    };

    std::vector<corgi_msgs::msg::MotorState*> ros_motor_modules = {
        &ros_motor_state.module_a,
        &ros_motor_state.module_b,
        &ros_motor_state.module_c,
        &ros_motor_state.module_d
    };

    for (int i = 0; i < 4; i++) {
        const corgi::ModuleFrame& f = module_frames[i];
        const motor_msg::MotorState& g = *grpc_motor_modules[i];
        ros_motor_modules[i]->theta = f.theta(g.theta());
        ros_motor_modules[i]->beta = f.beta(g.beta());
        ros_motor_modules[i]->gamma = f.gamma(g.gamma());
        // the maps are self-inverse, so module -> ROS is the same call
        const auto v = legacy_torque_frame ? std::make_pair(g.velocity_r(), g.velocity_l())
                                           : f.velocity_rl(g.velocity_r(), g.velocity_l());
        const auto tq = legacy_torque_frame ? std::make_pair(g.torque_r(), g.torque_l())
                                            : f.torque_rl(g.torque_r(), g.torque_l());
        ros_motor_modules[i]->velocity_r = v.first;
        ros_motor_modules[i]->velocity_l = v.second;
        ros_motor_modules[i]->velocity_h = legacy_torque_frame ? g.velocity_h() : f.velocity_h(g.velocity_h());
        ros_motor_modules[i]->torque_r = tq.first;
        ros_motor_modules[i]->torque_l = tq.second;
        ros_motor_modules[i]->torque_h = legacy_torque_frame ? g.torque_h() : f.torque_h(g.torque_h());
    }

    ros_motor_state.motor_mode = static_cast<int32_t>(grpc_motor_state.motor_mode());
    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.stamp.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.stamp.nanosec = grpc_motor_state.header().stamp().usec() * 1000;

    ros_motor_state_pub->publish(ros_motor_state);
}

void grpc_power_state_cb(const power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    grpc_power_state = state;

    ros_power_state.pb1_digital = grpc_power_state.pb1_digital();
    ros_power_state.pb1_signal = grpc_power_state.pb1_signal();
    ros_power_state.pb1_power = grpc_power_state.pb1_power();
    ros_power_state.pb2_digital = grpc_power_state.pb2_digital();
    ros_power_state.pb2_signal = grpc_power_state.pb2_signal();
    ros_power_state.pb2_power = grpc_power_state.pb2_power();
    ros_power_state.clean = grpc_power_state.clean();

    ros_power_state.pb1_v_0 = grpc_power_state.pb1_v_0();
    ros_power_state.pb1_i_0 = grpc_power_state.pb1_i_0();
    ros_power_state.pb1_v_1 = grpc_power_state.pb1_v_1();
    ros_power_state.pb1_i_1 = grpc_power_state.pb1_i_1();
    ros_power_state.pb1_v_2 = grpc_power_state.pb1_v_2();
    ros_power_state.pb1_i_2 = grpc_power_state.pb1_i_2();
    ros_power_state.pb1_v_3 = grpc_power_state.pb1_v_3();
    ros_power_state.pb1_i_3 = grpc_power_state.pb1_i_3();
    ros_power_state.pb1_v_4 = grpc_power_state.pb1_v_4();
    ros_power_state.pb1_i_4 = grpc_power_state.pb1_i_4();
    ros_power_state.pb1_v_5 = grpc_power_state.pb1_v_5();
    ros_power_state.pb1_i_5 = grpc_power_state.pb1_i_5();
    ros_power_state.pb1_v_6 = grpc_power_state.pb1_v_6();
    ros_power_state.pb1_i_6 = grpc_power_state.pb1_i_6();
    ros_power_state.pb1_v_7 = grpc_power_state.pb1_v_7();
    ros_power_state.pb1_i_7 = grpc_power_state.pb1_i_7();

    ros_power_state.pb2_v_0 = grpc_power_state.pb2_v_0();
    ros_power_state.pb2_i_0 = grpc_power_state.pb2_i_0();
    ros_power_state.pb2_v_1 = grpc_power_state.pb2_v_1();
    ros_power_state.pb2_i_1 = grpc_power_state.pb2_i_1();
    ros_power_state.pb2_v_2 = grpc_power_state.pb2_v_2();
    ros_power_state.pb2_i_2 = grpc_power_state.pb2_i_2();
    ros_power_state.pb2_v_3 = grpc_power_state.pb2_v_3();
    ros_power_state.pb2_i_3 = grpc_power_state.pb2_i_3();
    ros_power_state.pb2_v_4 = grpc_power_state.pb2_v_4();
    ros_power_state.pb2_i_4 = grpc_power_state.pb2_i_4();
    ros_power_state.pb2_v_5 = grpc_power_state.pb2_v_5();
    ros_power_state.pb2_i_5 = grpc_power_state.pb2_i_5();
    ros_power_state.pb2_v_6 = grpc_power_state.pb2_v_6();
    ros_power_state.pb2_i_6 = grpc_power_state.pb2_i_6();
    ros_power_state.pb2_v_7 = grpc_power_state.pb2_v_7();
    ros_power_state.pb2_i_7 = grpc_power_state.pb2_i_7();

    ros_power_state.header.seq = grpc_power_state.header().seq();
    ros_power_state.header.stamp.sec = grpc_power_state.header().stamp().sec();
    ros_power_state.header.stamp.nanosec = grpc_power_state.header().stamp().usec() * 1000;

    ros_power_state_pub->publish(ros_power_state);
}

void grpc_steer_state_cb(const steering_msg::SteeringStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_steer_state);

    grpc_steer_state = state;

    ros_steer_state.current_angle = grpc_steer_state.current_angle();
    ros_steer_state.current_state = grpc_steer_state.current_state();
    ros_steer_state.cmd_finish = grpc_steer_state.cmd_finish();
    ros_steer_state.steering_cali = grpc_steer_state.steering_cali();

    ros_steer_state.header.seq = grpc_steer_state.header().seq();
    ros_steer_state.header.stamp.sec = grpc_steer_state.header().stamp().sec();
    ros_steer_state.header.stamp.nanosec = grpc_steer_state.header().stamp().usec() * 1000;

    ros_steer_state_pub->publish(ros_steer_state);
}

void grpc_robot_state_cb(const robot_msg::RobotStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_robot_state);

    grpc_robot_state = state;

    ros_robot_state.robot_mode = grpc_robot_state.robot_mode();

    ros_robot_state.header.seq = grpc_robot_state.header().seq();
    ros_robot_state.header.stamp.sec = grpc_robot_state.header().stamp().sec();
    ros_robot_state.header.stamp.nanosec = grpc_robot_state.header().stamp().usec() * 1000;

    ros_robot_state_pub->publish(ros_robot_state);
}

void grpc_config_state_cb(const config_msg::ConfigStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_config_state);

    grpc_config_state = state;

    // Map gRPC proto fields to ROS message fields according to Config.proto
    // ros_config_state.transmit = grpc_config_state.transmit();
    ros_config_state.module = static_cast<int32_t>(grpc_config_state.module());
    ros_config_state.motor = static_cast<int32_t>(grpc_config_state.motor());
    ros_config_state.mode = static_cast<int32_t>(grpc_config_state.mode());
    ros_config_state.type = static_cast<int32_t>(grpc_config_state.type());
    ros_config_state.address = grpc_config_state.address();
    ros_config_state.value_f = grpc_config_state.value_f();
    ros_config_state.value_i = grpc_config_state.value_i();
    ros_config_state.error_code = grpc_config_state.error_code();

    ros_config_state.header.seq = grpc_config_state.header().seq();
    ros_config_state.header.stamp.sec = grpc_config_state.header().stamp().sec();
    ros_config_state.header.stamp.nanosec = grpc_config_state.header().stamp().usec() * 1000;

    ros_config_state_pub->publish(ros_config_state);
}

void grpc_log_cb(const log_msg::LogEntry log) {
    std::lock_guard<std::mutex> lock(mutex_ros_log);

    grpc_log = log;

    ros_log.level = grpc_log.level();
    ros_log.node_name = grpc_log.node_name();
    ros_log.message = grpc_log.message();

    ros_log.header.seq = grpc_log.header().seq();
    ros_log.header.stamp.sec = grpc_log.header().stamp().sec();
    ros_log.header.stamp.nanosec = grpc_log.header().stamp().usec() * 1000;

    ros_log_pub->publish(ros_log);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_ros_bridge");
    RCLCPP_INFO(node->get_logger(), "Corgi ROS Bridge Starts");
    try {
        const auto path = node->declare_parameter<std::string>("module_directions_path", "");
        load_module_frames(path.empty() ? default_module_directions_path() : path, node->get_logger());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Leg module directions unusable: %s. Not starting.", e.what());
        rclcpp::shutdown();
        return 1;
    }
    legacy_torque_frame = node->declare_parameter<bool>("legacy_torque_frame", false);
    if (legacy_torque_frame) {
        RCLCPP_WARN(node->get_logger(),
            "LEGACY TORQUE FRAME: positions converted, torques/velocities/gains NOT -- "
            "B/C (beta) and B/D (gamma) torques are in the module frame. A/B and rollback only "
            "(CONVENTIONS.md).");
    } else {
        RCLCPP_INFO(node->get_logger(),
            "MODULE FRAME: positions, velocities, torques and gains all converted per leg (CONVENTIONS.md).");
    }

    bool debug_mode = false;
    if (argc >= 2 && argv[1] != nullptr) {
        if (strcmp(argv[1], "log") == 0) {
            debug_mode = true;
        }
    }

    auto ros_motor_cmd_sub = node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1, ros_motor_cmd_cb);
    auto ros_steer_cmd_sub = node->create_subscription<corgi_msgs::msg::SteeringCmdStamped>("steer/command", 1, ros_steer_cmd_cb);
    auto ros_robot_cmd_sub = node->create_subscription<corgi_msgs::msg::RobotCmdStamped>("robot/command", 1, ros_robot_cmd_cb);
    auto ros_config_cmd_sub = node->create_subscription<corgi_msgs::msg::ConfigStamped>("config/command", 1, ros_config_cmd_cb);
    ros_motor_state_pub = node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = node->create_publisher<corgi_msgs::msg::PowerStateStamped>("power/state", 1);
    ros_steer_state_pub = node->create_publisher<corgi_msgs::msg::SteeringStateStamped>("steer/state", 1);
    ros_robot_state_pub = node->create_publisher<corgi_msgs::msg::RobotStateStamped>("robot/state", 1);
    ros_config_state_pub = node->create_publisher<corgi_msgs::msg::ConfigStamped>("config/state", 1);
    ros_log_pub = node->create_publisher<corgi_msgs::msg::LogStamped>("log", 1);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    core::Subscriber<steering_msg::SteeringStateStamped> &grpc_steer_state_sub = nh_.subscribe<steering_msg::SteeringStateStamped>("steer/state", 1000, grpc_steer_state_cb);
    core::Subscriber<robot_msg::RobotStateStamped> &grpc_robot_state_sub = nh_.subscribe<robot_msg::RobotStateStamped>("robot/state", 1000, grpc_robot_state_cb);
    core::Subscriber<config_msg::ConfigStamped> &grpc_config_state_sub = nh_.subscribe<config_msg::ConfigStamped>("motor/config/reply", 1000, grpc_config_state_cb, 100);
    core::Subscriber<log_msg::LogEntry> &grpc_log_sub = nh_.subscribe<log_msg::LogEntry>("/log", 100, grpc_log_cb, 100);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_steer_cmd_pub = &(nh_.advertise<steering_msg::SteeringCmdStamped>("steer/command"));
    grpc_robot_cmd_pub = &(nh_.advertise<robot_msg::RobotCmdStamped>("robot/command"));
    grpc_config_cmd_pub = &(nh_.advertise<config_msg::ConfigStamped>("motor/config/request"));

    core::Rate rate(1000);

    int loop_counter = 0;
    while (rclcpp::ok()) {
        if (debug_mode) RCLCPP_INFO(node->get_logger(), "Loop Count: %d", loop_counter);

        rclcpp::spin_some(node);
        core::spinOnce();

        if (debug_mode) RCLCPP_INFO(node->get_logger()," ");

        loop_counter++;
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiRosBridge"), "Corgi ROS Bridge is killed");

    rclcpp::shutdown();
    
    return 0;
}
