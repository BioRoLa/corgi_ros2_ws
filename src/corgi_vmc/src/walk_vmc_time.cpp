#include "walk_utils.hpp"
#include "vmc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <array>
#include <cstdlib>
#include <nav_msgs/msg/odometry.hpp>
#include <corgi_msgs/msg/gmo_contact_state_stamped.hpp>

namespace {

constexpr const char* kVariantName = "walk_vmc_time";

struct WalkConfig {
    std::string contact_source;
    double stand_height = 0.2;
    double cruise_velocity = 0.05;
    double step_length = 0.2;
    double step_height = 0.08;
    int ramp_loops = 500;
    int target_loop = 11000;
    double stop_x = 2.0;
    double decel_margin = 1.2;
    std::array<double, 8> init_eta{};
};

bool trigger = false;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::ImuStamped imu;

geometry_msgs::msg::Vector3 odom_pos{};
geometry_msgs::msg::Vector3 odom_vel{};
double odom_z = 0.0;

geometry_msgs::msg::Vector3 sim_body_vel{};
geometry_msgs::msg::Vector3 sim_body_pos{};
bool has_sim_body_vel = false;
bool has_sim_body_pos = false;

nav_msgs::msg::Odometry ekf_odom{};
bool has_ekf_odom = false;

corgi_msgs::msg::ImuStamped imu_raw{};
bool has_imu_raw = false;

corgi_msgs::msg::GMOContactStateStamped gmo_contact{};
bool has_gmo_contact = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger = msg->enable;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
    force_state = *msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state = *msg;
}

void odom_pos_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_vel = *msg;
}

void odom_z_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    odom_z = msg->data;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu = *msg;
}

void ekf_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ekf_odom = *msg;
    has_ekf_odom = true;
}

void imu_raw_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_raw = *msg;
    has_imu_raw = true;
}

void gmo_contact_cb(const corgi_msgs::msg::GMOContactStateStamped::SharedPtr msg) {
    gmo_contact = *msg;
    has_gmo_contact = true;
}

void sim_body_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    sim_body_vel = *msg;
    has_sim_body_vel = true;
}

void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& t : msg->transforms) {
        if (t.header.frame_id == "odom" && t.child_frame_id == "base_link") {
            sim_body_pos.x = t.transform.translation.x;
            sim_body_pos.y = t.transform.translation.y;
            sim_body_pos.z = t.transform.translation.z;
            has_sim_body_pos = true;
            break;
        }
    }
}

void update_robot_state(
    VirtualModelController& vmc,
    const std::string& state_source,
    const rclcpp::Logger& logger,
    const rclcpp::Clock::SharedPtr& clock) {
    static rclcpp::Time last_warn_time(0, 0, RCL_ROS_TIME);

    if (state_source == "esekf") {
        if (has_ekf_odom) {
            vmc.robot_pos[0] = ekf_odom.pose.pose.position.x;
            vmc.robot_pos[1] = ekf_odom.pose.pose.position.y;
            vmc.robot_pos[2] = ekf_odom.pose.pose.position.z;
            vmc.robot_vel[0] = ekf_odom.twist.twist.linear.x;
            vmc.robot_vel[1] = ekf_odom.twist.twist.linear.y;
            vmc.robot_vel[2] = ekf_odom.twist.twist.linear.z;
            vmc.robot_ang = Eigen::Quaterniond(
                ekf_odom.pose.pose.orientation.w,
                ekf_odom.pose.pose.orientation.x,
                ekf_odom.pose.pose.orientation.y,
                ekf_odom.pose.pose.orientation.z);
            vmc.robot_ang_vel[0] = ekf_odom.twist.twist.angular.x;
            vmc.robot_ang_vel[1] = ekf_odom.twist.twist.angular.y;
            vmc.robot_ang_vel[2] = ekf_odom.twist.twist.angular.z;
            return;
        }
        const auto now = clock->now();
        if ((now - last_warn_time).seconds() > 2.0) {
            RCLCPP_WARN(logger, "state_source=esekf but /ekf not ready, fallback to odom_legacy");
            last_warn_time = now;
        }
    }

    if (state_source == "sim_driver") {
        if (has_sim_body_pos && has_sim_body_vel) {
            vmc.robot_vel[0] = sim_body_vel.x;
            vmc.robot_vel[1] = sim_body_vel.y;
            vmc.robot_vel[2] = sim_body_vel.z;
            vmc.robot_pos[0] = sim_body_pos.x;
            vmc.robot_pos[1] = sim_body_pos.y;
            vmc.robot_pos[2] = sim_body_pos.z;
            return;
        }
        const auto now = clock->now();
        if ((now - last_warn_time).seconds() > 2.0) {
            RCLCPP_WARN(logger,
                        "state_source=sim_driver but /tf(odom->base_link) or /sim/body/velocity not ready, fallback to odom_legacy");
            last_warn_time = now;
        }
    }

    vmc.robot_vel[0] = odom_vel.x;
    vmc.robot_vel[1] = odom_vel.y;
    vmc.robot_vel[2] = odom_vel.z;
    vmc.robot_pos[0] = odom_pos.x;
    vmc.robot_pos[1] = odom_pos.y;
    vmc.robot_pos[2] = odom_z;
}

void update_robot_orientation(VirtualModelController& vmc, const std::string& state_source) {
    if (state_source == "esekf" && has_ekf_odom) {
        return;
    }

    if (state_source == "esekf") {
        if (has_imu_raw) {
            vmc.robot_ang_vel[0] = imu_raw.angular_velocity.x;
            vmc.robot_ang_vel[1] = imu_raw.angular_velocity.y;
            vmc.robot_ang_vel[2] = imu_raw.angular_velocity.z;
        }
        return;
    }

    vmc.robot_ang.x() = imu.orientation.x;
    vmc.robot_ang.y() = imu.orientation.y;
    vmc.robot_ang.z() = imu.orientation.z;
    vmc.robot_ang.w() = imu.orientation.w;
    vmc.robot_ang_vel[0] = imu.angular_velocity.x;
    vmc.robot_ang_vel[1] = imu.angular_velocity.y;
    vmc.robot_ang_vel[2] = imu.angular_velocity.z;
}

void convert_force_to_local(double *f_global, const Eigen::Matrix3d& R_T) {
    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
    Eigen::Vector3d f_local = R_T * f_global_vec;
    f_global[0] = f_local(0);
    f_global[1] = f_local(1);
    f_global[2] = f_local(2);
}

WalkConfig load_walk_config(const std::string& profile) {
    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        throw std::runtime_error("HOME environment variable not set");
    }

    const std::string config_path =
        std::string(home_path) + "/corgi_ws/corgi_ros2_ws/src/corgi_vmc/config/config.yaml";
    YAML::Node cfg = YAML::LoadFile(config_path);
    YAML::Node common_cfg = cfg["common"];
    YAML::Node walk_cfg = common_cfg ? common_cfg["walk"] : YAML::Node{};
    YAML::Node profile_cfg = cfg[profile];

    if (!profile_cfg) {
        throw std::runtime_error("Missing required profile: " + profile);
    }

    auto read_node = [&](const std::string& key) -> YAML::Node {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key];
        if (walk_cfg && walk_cfg[key]) return walk_cfg[key];
        if (common_cfg && common_cfg[key]) return common_cfg[key];
        throw std::runtime_error("walk_vmc: missing required config key: " + key);
    };

    WalkConfig cfg_out;
    cfg_out.contact_source = read_node("contact_source").as<std::string>();
    cfg_out.stand_height = read_node("stand_height").as<double>();
    cfg_out.cruise_velocity = read_node("cruise_velocity").as<double>();
    cfg_out.step_length = read_node("step_length").as<double>();
    cfg_out.step_height = read_node("step_height").as<double>();
    cfg_out.ramp_loops = read_node("ramp_loops").as<int>();
    cfg_out.target_loop = read_node("target_loop").as<int>();
    cfg_out.stop_x = read_node("stop_x").as<double>();
    cfg_out.decel_margin = read_node("decel_margin").as<double>();

    const std::vector<double> init_eta_vec = read_node("init_eta").as<std::vector<double>>();
    if (init_eta_vec.size() != cfg_out.init_eta.size()) {
        throw std::runtime_error("init_eta must have exactly 8 values");
    }
    for (size_t i = 0; i < cfg_out.init_eta.size(); ++i) {
        cfg_out.init_eta[i] = init_eta_vec[i];
    }

    if (cfg_out.ramp_loops <= 0) {
        throw std::runtime_error("ramp_loops must be > 0");
    }
    if (cfg_out.cruise_velocity <= 0.0) {
        throw std::runtime_error("cruise_velocity must be > 0");
    }

    return cfg_out;
}

void apply_gait_contact(
    const std::array<int, 4>& swing_phase,
    const std::array<double, 4>& duty,
    bool* touched,
    bool* selection_matrix,
    std::vector<corgi_msgs::msg::ImpedanceCmd*>& imp_cmd_modules,
    std::vector<corgi_msgs::msg::ContactState*>& contact_state_modules,
    std_msgs::msg::Int32MultiArray& swing_phase_msg,
    const VirtualModelController& vmc) {
    for (int i = 0; i < 4; i++) {
        if (swing_phase[i] == 1 && touched[i]) {
            selection_matrix[i] = false;
            touched[i] = false;
            imp_cmd_modules[i]->by = vmc.By_swing;
            imp_cmd_modules[i]->ky = vmc.Ky_swing;
        } else if (swing_phase[i] == 0 && !touched[i]) {
            selection_matrix[i] = true;
            touched[i] = true;
            imp_cmd_modules[i]->by = vmc.By_stance;
            imp_cmd_modules[i]->ky = vmc.Ky_stance;
        }

        swing_phase_msg.data[i] = swing_phase[i];
        contact_state_modules[i]->contact = (duty[i] < 0.75 && duty[i] > 0.05);
    }
}

void apply_gmo_contact(
    bool* touched,
    bool* selection_matrix,
    std::vector<corgi_msgs::msg::ImpedanceCmd*>& imp_cmd_modules,
    std::vector<corgi_msgs::msg::ContactState*>& contact_state_modules,
    std_msgs::msg::Int32MultiArray& swing_phase_msg,
    const VirtualModelController& vmc) {
    const std::array<bool, 4> in_contact = {
        gmo_contact.module_a.contact,
        gmo_contact.module_b.contact,
        gmo_contact.module_c.contact,
        gmo_contact.module_d.contact
    };

    for (int i = 0; i < 4; i++) {
        if (!in_contact[i] && touched[i]) {
            selection_matrix[i] = false;
            touched[i] = false;
            imp_cmd_modules[i]->by = vmc.By_swing;
            imp_cmd_modules[i]->ky = vmc.Ky_swing;
        } else if (in_contact[i] && !touched[i]) {
            selection_matrix[i] = true;
            touched[i] = true;
            imp_cmd_modules[i]->by = vmc.By_stance;
            imp_cmd_modules[i]->ky = vmc.Ky_stance;
        }

        swing_phase_msg.data[i] = in_contact[i] ? 0 : 1;
        contact_state_modules[i]->contact = in_contact[i];
    }
}

}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_vmc");

    node->declare_parameter<std::string>("config_profile", "sim");
    node->declare_parameter<std::string>("state_source", "odom_legacy");
    std::string config_profile = node->get_parameter("config_profile").as_string();
    std::string state_source = node->get_parameter("state_source").as_string();

    if (config_profile != "sim" && config_profile != "real") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid config_profile='%s', fallback to 'sim'",
                    config_profile.c_str());
        config_profile = "sim";
    }
    if (state_source != "odom_legacy" && state_source != "sim_driver" && state_source != "esekf") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid state_source='%s' (valid: odom_legacy|sim_driver|esekf), fallback to 'odom_legacy'",
                    state_source.c_str());
        state_source = "odom_legacy";
    }
    sim = (config_profile == "sim");

    RCLCPP_INFO(node->get_logger(), "Corgi VMC Starts (%s)", kVariantName);
    RCLCPP_INFO(node->get_logger(), "Config profile: %s", config_profile.c_str());
    RCLCPP_INFO(node->get_logger(), "State source: %s", state_source.c_str());

    WalkConfig walk_cfg;
    VirtualModelController vmc;
    try {
        walk_cfg = load_walk_config(config_profile);
        vmc.load_config(config_profile);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to load VMC config: %s", e.what());
        return 1;
    }

    if (walk_cfg.contact_source != "gait" && walk_cfg.contact_source != "gmo") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid contact_source='%s' (valid: gait|gmo), fallback to 'gait'",
                    walk_cfg.contact_source.c_str());
        walk_cfg.contact_source = "gait";
    }
    vmc.target_loop = walk_cfg.target_loop;
    vmc.target_pos_z = walk_cfg.stand_height;

    RCLCPP_INFO(node->get_logger(), "Contact source: %s", walk_cfg.contact_source.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Gait: stand_height=%.2f cruise_vel=%.2f step_length=%.2f step_height=%.3f ramp_loops=%d target_loop=%d",
                walk_cfg.stand_height, walk_cfg.cruise_velocity, walk_cfg.step_length,
                walk_cfg.step_height, walk_cfg.ramp_loops, walk_cfg.target_loop);

    RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);
    auto contact_pub = node->create_publisher<corgi_msgs::msg::ContactStateStamped>("odometry/legacy/contact", 10);
    auto swing_phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);
    auto force_state_sub = node->create_subscription<corgi_msgs::msg::ForceStateStamped>("force/state", 10, force_state_cb);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 10, motor_state_cb);
    auto odom_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/position", 10, odom_pos_cb);
    auto odom_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/velocity", 10, odom_vel_cb);
    auto odom_z_sub = node->create_subscription<std_msgs::msg::Float64>("odometry/legacy/z_position_hip", 10, odom_z_cb);
    auto sim_body_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>("sim/body/velocity", 10, sim_body_vel_cb);
    auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, tf_cb);
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>("imu", 10, imu_cb);
    auto ekf_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/ekf", 10, ekf_odom_cb);
    auto imu_raw_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>("/imu_raw", 10, imu_raw_cb);
    auto gmo_contact_sub = node->create_subscription<corgi_msgs::msg::GMOContactStateStamped>(
        "gmo/contact_state", 10, gmo_contact_cb);

    rclcpp::Duration period(0, 1000000000.0 / vmc.freq);
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    corgi_msgs::msg::ContactStateStamped contact_state;
    std_msgs::msg::Int32MultiArray swing_phase_msg;
    swing_phase_msg.data.resize(4, 0);

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a, &imp_cmd.module_b, &imp_cmd.module_c, &imp_cmd.module_d
    };
    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a, &contact_state.module_b, &contact_state.module_c, &contact_state.module_d
    };
    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a, &force_state.module_b, &force_state.module_c, &force_state.module_d
    };
    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a, &motor_state.module_b, &motor_state.module_c, &motor_state.module_d
    };

    WalkGait walk_gait(sim, 0, vmc.freq);
    walk_gait.stand_height = walk_cfg.stand_height;
    walk_gait.velocity = walk_cfg.cruise_velocity;
    walk_gait.step_length = walk_cfg.step_length;
    walk_gait.step_height = walk_cfg.step_height;
    walk_gait.initialize(walk_cfg.init_eta.data(), walk_gait.step_length);
    walk_gait.set_velocity(vmc.target_vel_x);

    bool touched[4] = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};

    for (auto& cmd : imp_cmd_modules) {
        cmd->theta = 17 / 180.0 * M_PI;
        cmd->beta = 0 / 180.0 * M_PI;
        cmd->fy = -vmc.m * vmc.gravity / 4.0;
        cmd->mx = vmc.Mx;
        cmd->my = vmc.My;
        cmd->bx = vmc.Bx_swing;
        cmd->by = vmc.By_swing;
        cmd->kx = vmc.Kx_swing;
        cmd->ky = vmc.Ky_swing;
    }

    RCLCPP_INFO(node->get_logger(), "Wait For Force Control Node ...");
    if (!sim) {
        for (int i = 0; i < int(3 * vmc.freq); i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");
    for (int i = 0; i < int(3 * vmc.freq); i++) {
        for (int j = 0; j < 4; j++) {
            imp_cmd_modules[j]->theta += (walk_cfg.init_eta[2 * j] - 17 / 180.0 * M_PI) / (3 * vmc.freq);
            imp_cmd_modules[j]->beta += walk_cfg.init_eta[2 * j + 1] / (3 * vmc.freq);
        }
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }
    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    for (int i = 0; i < int(2 * vmc.freq); i++) {
        rclcpp::spin_some(node);
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger) {
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

            const int contact_wait_loops = sim ? 0 : int(3 * vmc.freq);
            for (int i = 0; i < contact_wait_loops; i++) {
                for (auto& state : contact_state_modules) {
                    state->contact = true;
                }
                contact_state.header.stamp = node->now();
                contact_pub->publish(contact_state);
                next_time += period;
                node->get_clock()->sleep_until(next_time);
            }

            for (auto& cmd : imp_cmd_modules) {
                cmd->bx = vmc.Bx_stance;
                cmd->by = vmc.By_stance;
                cmd->kx = vmc.Kx_stance;
                cmd->ky = vmc.Ky_stance;
            }

            RCLCPP_INFO(node->get_logger(), "VMC Controller Starts ...");

            int loop_count = 0;
            rclcpp::Time last_contact_warn_time(0, 0, RCL_ROS_TIME);

            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                update_robot_state(vmc, state_source, node->get_logger(), node->get_clock());
                update_robot_orientation(vmc, state_source);

                if (loop_count < walk_cfg.ramp_loops) {
                    vmc.target_vel_x += walk_cfg.cruise_velocity / walk_cfg.ramp_loops;
                    walk_gait.set_velocity(vmc.target_vel_x);
                } else if (loop_count >= vmc.target_loop - walk_cfg.ramp_loops &&
                           loop_count < vmc.target_loop) {
                    vmc.target_vel_x -= walk_cfg.cruise_velocity / walk_cfg.ramp_loops;
                    walk_gait.set_velocity(vmc.target_vel_x);
                }

                vmc.target_pos_x += vmc.target_vel_x * vmc.dt;

                vmc.eta_list = walk_gait.step();
                const auto swing_phase = walk_gait.get_swing_phase();

                for (int i = 0; i < 4; i++) {
                    imp_cmd_modules[i]->theta = vmc.eta_list[0][i];
                    imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? vmc.eta_list[1][i] : -vmc.eta_list[1][i];
                }

                if (walk_cfg.contact_source == "gmo" && has_gmo_contact) {
                    apply_gmo_contact(touched, selection_matrix, imp_cmd_modules,
                                      contact_state_modules, swing_phase_msg, vmc);
                } else {
                    if (walk_cfg.contact_source == "gmo") {
                        const auto now = node->get_clock()->now();
                        if ((now - last_contact_warn_time).seconds() > 2.0) {
                            RCLCPP_WARN(node->get_logger(),
                                        "contact_source=gmo but gmo/contact_state not ready, fallback to gait contact");
                            last_contact_warn_time = now;
                        }
                    }
                    apply_gait_contact(swing_phase, walk_gait.get_duty(), touched, selection_matrix,
                                       imp_cmd_modules, contact_state_modules, swing_phase_msg, vmc);
                }

                quaternion_to_euler(vmc.robot_ang, vmc.roll, vmc.pitch, vmc.yaw);

                Eigen::VectorXd x(13);
                x << vmc.roll,             vmc.pitch,            vmc.yaw,
                     vmc.robot_pos[0],     vmc.robot_pos[1],     vmc.robot_pos[2],
                     vmc.robot_ang_vel[0], vmc.robot_ang_vel[1], vmc.robot_ang_vel[2],
                     vmc.robot_vel[0],     vmc.robot_vel[1],     vmc.robot_vel[2],
                     -vmc.gravity;

                Eigen::VectorXd x_ref(13);
                x_ref << 0,                0,                0,
                         vmc.target_pos_x, 0, vmc.target_pos_z,
                         0,                0,                0,
                         vmc.target_vel_x, 0,                0,
                         -vmc.gravity;

                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0] + 0.222, 0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = { legmodel.contact_p[0] + 0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = { legmodel.contact_p[0] - 0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0] - 0.222, 0.2, legmodel.contact_p[1]};

                Eigen::VectorXd force = vmc.step(x, x_ref, selection_matrix, ra, rb, rc, rd);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R_T = vmc.robot_ang.toRotationMatrix().transpose();
                if (!sim) {
                    R_T(1, 2) *= -1;
                    R_T(2, 1) *= -1;
                }

                convert_force_to_local(force_A, R_T);
                convert_force_to_local(force_B, R_T);
                convert_force_to_local(force_C, R_T);
                convert_force_to_local(force_D, R_T);

                imp_cmd_modules[0]->fx = -force_A[0];
                imp_cmd_modules[0]->fy = -force_A[2];
                imp_cmd_modules[1]->fx =  force_B[0];
                imp_cmd_modules[1]->fy = -force_B[2];
                imp_cmd_modules[2]->fx =  force_C[0];
                imp_cmd_modules[2]->fy = -force_C[2];
                imp_cmd_modules[3]->fx = -force_D[0];
                imp_cmd_modules[3]->fy = -force_D[2];

                imp_cmd.header.stamp = node->now();
                imp_cmd_pub->publish(imp_cmd);
                swing_phase_pub->publish(swing_phase_msg);
                contact_state.header.stamp = node->now();
                contact_pub->publish(contact_state);

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Ref Pos = [" << x_ref[3] << ", " << x_ref[4] << ", " << x_ref[5] << "]" << std::endl << std::endl;
                std::cout << "Odom Pos = [" << vmc.robot_pos[0] << ", " << vmc.robot_pos[1] << ", " << vmc.robot_pos[2] << "]" << std::endl;
                std::cout << "Odom Vel = [" << vmc.robot_vel[0] << ", " << vmc.robot_vel[1] << ", " << vmc.robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang (deg) = [" << vmc.roll / M_PI * 180 << ", " << vmc.pitch / M_PI * 180 << ", " << vmc.yaw / M_PI * 180 << "]" << std::endl << std::endl;
                std::cout << "Force A: [" << force_A[0] << ", " << force_A[2] << "]" << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[2] << "]" << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[2] << "]" << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[2] << "]" << std::endl;
                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                ++loop_count;
                if (loop_count >= vmc.target_loop) {
                    std::cout << "Finished" << std::endl;
                    break;
                }

                next_time += period;
                if (!node->get_clock()->sleep_until(next_time)) {
                    RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                    break;
                }
            }
            break;
        }
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }
    return 0;
}
