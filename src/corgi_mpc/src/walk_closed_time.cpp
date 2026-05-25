// walk_closed_time.cpp
// Time-driven closed-loop walk controller.
// The robot walks for exactly (target_loop × dt) seconds from trigger onset,
// then stops — regardless of how far it has actually traveled.
// For position-based stopping, use walk_closed_dist.cpp.

#include "walk_utils.hpp"
#include "mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <corgi_msgs/msg/gmo_contact_state_stamped.hpp>

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

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg){
    force_state = *msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
}

void odom_pos_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
    odom_vel = *msg;
}

void odom_z_cb(const std_msgs::msg::Float64::SharedPtr msg){
    odom_z = msg->data;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg){
    imu = *msg;
}

void ekf_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
    ekf_odom = *msg;
    has_ekf_odom = true;
}

void imu_raw_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg){
    imu_raw = *msg;
    has_imu_raw = true;
}

corgi_msgs::msg::GMOContactStateStamped gmo_contact{};
bool has_gmo_contact = false;

void gmo_contact_cb(const corgi_msgs::msg::GMOContactStateStamped::SharedPtr msg) {
    gmo_contact = *msg;
    has_gmo_contact = true;
}

void sim_body_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
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
    ModelPredictiveController& mpc,
    const std::string& state_source,
    const rclcpp::Logger& logger,
    const rclcpp::Clock::SharedPtr& clock) {
    static rclcpp::Time last_warn_time(0, 0, RCL_ROS_TIME);

    // ── ESEKF: all state from /ekf (pose + twist, bias-corrected) ──────────
    if (state_source == "esekf") {
        if (has_ekf_odom) {
            mpc.robot_pos[0] = ekf_odom.pose.pose.position.x;
            mpc.robot_pos[1] = ekf_odom.pose.pose.position.y;
            mpc.robot_pos[2] = ekf_odom.pose.pose.position.z;
            mpc.robot_vel[0] = ekf_odom.twist.twist.linear.x;
            mpc.robot_vel[1] = ekf_odom.twist.twist.linear.y;
            mpc.robot_vel[2] = ekf_odom.twist.twist.linear.z;
            mpc.robot_ang = Eigen::Quaterniond(
                ekf_odom.pose.pose.orientation.w,
                ekf_odom.pose.pose.orientation.x,
                ekf_odom.pose.pose.orientation.y,
                ekf_odom.pose.pose.orientation.z);
            mpc.robot_ang_vel[0] = ekf_odom.twist.twist.angular.x;
            mpc.robot_ang_vel[1] = ekf_odom.twist.twist.angular.y;
            mpc.robot_ang_vel[2] = ekf_odom.twist.twist.angular.z;
            return;
        }
        const auto now = clock->now();
        if ((now - last_warn_time).seconds() > 2.0) {
            RCLCPP_WARN(logger,
                        "state_source=esekf but /ekf not ready, fallback to odom_legacy");
            last_warn_time = now;
        }
    }

    // ── sim_driver: position+velocity from TF and sim/body/velocity ────────
    if (state_source == "sim_driver") {
        if (has_sim_body_pos && has_sim_body_vel) {
            mpc.robot_vel[0] = sim_body_vel.x;
            mpc.robot_vel[1] = sim_body_vel.y;
            mpc.robot_vel[2] = sim_body_vel.z;
            mpc.robot_pos[0] = sim_body_pos.x;
            mpc.robot_pos[1] = sim_body_pos.y;
            mpc.robot_pos[2] = sim_body_pos.z;
            return;
        }
        const auto now = clock->now();
        if ((now - last_warn_time).seconds() > 2.0) {
            RCLCPP_WARN(logger,
                        "state_source=sim_driver but /tf(odom->base_link) or /sim/body/velocity not ready, fallback to odom_legacy");
            last_warn_time = now;
        }
    }

    // ── odom_legacy (default / fallback) ────────────────────────────────────
    mpc.robot_vel[0] = odom_vel.x;
    mpc.robot_vel[1] = odom_vel.y;
    mpc.robot_vel[2] = odom_vel.z;
    mpc.robot_pos[0] = odom_pos.x;
    mpc.robot_pos[1] = odom_pos.y;
    mpc.robot_pos[2] = odom_pos.z;
}

void convert_force_to_local(double *f_global, const Eigen::Matrix3d& R_T) {
    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
    Eigen::Vector3d f_local = R_T * f_global_vec;
    f_global[0] = f_local(0);
    f_global[1] = f_local(1);
    f_global[2] = f_local(2);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_mpc");

    node->declare_parameter<std::string>("config_profile", "sim");
    node->declare_parameter<std::string>("state_source", "odom_legacy");
    std::string config_profile = node->get_parameter("config_profile").as_string();
    std::string state_source = node->get_parameter("state_source").as_string();
    if (config_profile != "sim" && config_profile != "real") {
        RCLCPP_WARN(
            node->get_logger(),
            "Invalid config_profile='%s', fallback to 'sim'",
            config_profile.c_str());
        config_profile = "sim";
    }
    if (state_source != "odom_legacy" && state_source != "sim_driver" && state_source != "esekf") {
        RCLCPP_WARN(
            node->get_logger(),
            "Invalid state_source='%s' (valid: odom_legacy|sim_driver|esekf), fallback to 'odom_legacy'",
            state_source.c_str());
        state_source = "odom_legacy";
    }
    sim = (config_profile == "sim");

    RCLCPP_INFO(node->get_logger(), "Corgi MPC Starts (walk_closed_time)");
    RCLCPP_INFO(node->get_logger(), "Config profile: %s", config_profile.c_str());
    RCLCPP_INFO(node->get_logger(), "State source: %s", state_source.c_str());

    node->declare_parameter<std::string>("contact_source", "gait");
    std::string contact_source = node->get_parameter("contact_source").as_string();
    if (contact_source != "gait" && contact_source != "gmo") {
        RCLCPP_WARN(node->get_logger(),
            "Invalid contact_source='%s' (valid: gait|gmo), fallback to 'gait'",
            contact_source.c_str());
        contact_source = "gait";
    }
    RCLCPP_INFO(node->get_logger(), "Contact source: %s", contact_source.c_str());

    // ── Load gait & walk parameters from config.yaml ──────────────────────
    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        RCLCPP_FATAL(node->get_logger(), "HOME environment variable not set");
        return 1;
    }
    const std::string config_path = std::string(home_path) +
        "/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml";
    YAML::Node cfg         = YAML::LoadFile(config_path);
    YAML::Node common_cfg  = cfg["common"];
    YAML::Node profile_cfg = cfg[config_profile];

    // Lookup order: profile-specific section first, then common section.
    auto gait_read_double = [&](const std::string& key) -> double {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<double>();
        if (common_cfg   && common_cfg[key])  return common_cfg[key].as<double>();
        throw std::runtime_error("walk_closed_time: missing required config key: " + key);
    };
    auto gait_read_int = [&](const std::string& key) -> int {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<int>();
        if (common_cfg   && common_cfg[key])  return common_cfg[key].as<int>();
        throw std::runtime_error("walk_closed_time: missing required config key: " + key);
    };
    auto gait_read_vec = [&](const std::string& key) -> std::vector<double> {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<std::vector<double>>();
        if (common_cfg   && common_cfg[key])  return common_cfg[key].as<std::vector<double>>();
        throw std::runtime_error("walk_closed_time: missing required config key: " + key);
    };

    // ── Shared gait parameters (walk_closed_time and walk_closed_dist) ─────
    const double stand_height    = gait_read_double("stand_height");
    const double cruise_velocity = gait_read_double("cruise_velocity");
    const double step_length     = gait_read_double("step_length");
    const double step_height_cfg = gait_read_double("step_height");
    // ramp_loops: velocity ramp-up/down duration in control cycles (= 1 s at 100 Hz).
    const int    ramp_loops      = gait_read_int("ramp_loops");

    // ── Initial joint angles (profile-specific: different for sim and real) ─
    const std::vector<double> init_eta_vec = gait_read_vec("init_eta");
    if (static_cast<int>(init_eta_vec.size()) != 8) {
        RCLCPP_FATAL(node->get_logger(),
            "init_eta must have exactly 8 values, got %zu", init_eta_vec.size());
        return 1;
    }
    double init_eta[8];
    for (int i = 0; i < 8; ++i) init_eta[i] = init_eta_vec[i];

    // ── Time-driven stop parameter ─────────────────────────────────────────
    // The robot walks for exactly (target_loop × dt) seconds regardless of
    // how far it has traveled. For position-based stopping use walk_closed_dist.
    const int target_loop = gait_read_int("target_loop");

    RCLCPP_INFO(node->get_logger(),
        "Gait: stand_height=%.2f  cruise_vel=%.2f  step_length=%.2f  "
        "step_height=%.3f  ramp_loops=%d  target_loop=%d",
        stand_height, cruise_velocity, step_length,
        step_height_cfg, ramp_loops, target_loop);

    // Wait for clock synchronization
    RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ModelPredictiveController mpc;
    mpc.load_config(config_profile);
    mpc.target_loop = target_loop;

    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);
    auto swing_phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);
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

    rclcpp::Duration period(0, 1000000000.0 / mpc.freq); // Convert Hz to nanoseconds
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    std_msgs::msg::Int32MultiArray swing_phase_msg;
    swing_phase_msg.data.resize(4, 0);

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    mpc.target_pos_z = stand_height;

    WalkGait walk_gait(sim, 0, mpc.freq);
    double velocity = cruise_velocity; // runtime velocity command; ramps 0 → cruise → 0

    walk_gait.stand_height = stand_height;
    walk_gait.velocity     = velocity;
    walk_gait.step_length  = step_length;
    walk_gait.step_height  = step_height_cfg;

    walk_gait.initialize(init_eta, walk_gait.step_length);
    walk_gait.set_velocity(mpc.target_vel_x);

    bool touched[4] = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};

    // Initialize impedance command
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->fy = -mpc.m*mpc.gravity/4.0;

        cmd->mx = mpc.Mx;
        cmd->my = mpc.My;
        cmd->bx = mpc.Bx_swing;
        cmd->by = mpc.By_swing;
        cmd->kx = mpc.Kx_swing;
        cmd->ky = mpc.Ky_swing;
    }

    RCLCPP_INFO(node->get_logger(), "Wait For Force Control Node ...");

    if (!sim) {
        for (int i=0; i<int(3*mpc.freq); i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    for (int i=0; i<int(3*mpc.freq); i++) {
        for (int j=0; j<4; j++) {
            imp_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/(3*mpc.freq);
            imp_cmd_modules[j]->beta += init_eta[2*j+1]/(3*mpc.freq);
        }
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // stay
    for (int i=0; i<int(2*mpc.freq); i++) {
        rclcpp::spin_some(node);
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

            if (!sim) {
                for (int i = 0; i < int(3 * mpc.freq); i++) {
                    rclcpp::spin_some(node);
                    imp_cmd.header.stamp = node->now();
                    imp_cmd_pub->publish(imp_cmd);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
            }

            for (auto& cmd : imp_cmd_modules){
                cmd->bx = mpc.Bx_stance;
                cmd->by = mpc.By_stance;
                cmd->kx = mpc.Kx_stance;
                cmd->ky = mpc.Ky_stance;
            }

            RCLCPP_INFO(node->get_logger(), "MPC Controller Starts ...");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // ── Time-driven velocity profile ───────────────────────────
                // Ramp up over ramp_loops cycles, cruise at velocity, then ramp
                // down over ramp_loops cycles before target_loop.
                if (loop_count < ramp_loops) {
                    mpc.target_vel_x += velocity / ramp_loops;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }
                else if (loop_count > mpc.target_loop - ramp_loops && loop_count < mpc.target_loop) {
                    mpc.target_vel_x -= velocity / ramp_loops;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }

                // NOTE: X/Y position weights in Q are 0, so target_pos_x does NOT
                // influence the MPC force solution. It is accumulated here solely for
                // logging and for the Z-axis / velocity reference channels.
                mpc.target_pos_x += mpc.target_vel_x * mpc.dt;

                // get next eta
                mpc.eta_list = walk_gait.step();
                const auto swing_phase = walk_gait.get_swing_phase();

                for (int i = 0; i < 4; i++) {
                    imp_cmd_modules[i]->theta = mpc.eta_list[0][i];
                    imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];
                }

                // contact state: gait (time-based scheduler) or gmo (sensor-based, requires state_source:=esekf)
                // gmo convention: contact=true → stance (swing_phase=0), contact=false → swing (swing_phase=1)
                if (contact_source == "gmo" && has_gmo_contact) {
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
                            imp_cmd_modules[i]->by = mpc.By_swing;
                            imp_cmd_modules[i]->ky = mpc.Ky_swing;
                        } else if (in_contact[i] && !touched[i]) {
                            selection_matrix[i] = true;
                            touched[i] = true;
                            imp_cmd_modules[i]->by = mpc.By_stance;
                            imp_cmd_modules[i]->ky = mpc.Ky_stance;
                        }
                        swing_phase_msg.data[i] = in_contact[i] ? 0 : 1;
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        if (swing_phase[i] == 1 && touched[i]) {
                            selection_matrix[i] = false;
                            touched[i] = false;
                            imp_cmd_modules[i]->by = mpc.By_swing;
                            imp_cmd_modules[i]->ky = mpc.Ky_swing;
                        }
                        else if (swing_phase[i] == 0 && !touched[i]) {
                            selection_matrix[i] = true;
                            touched[i] = true;
                            imp_cmd_modules[i]->by = mpc.By_stance;
                            imp_cmd_modules[i]->ky = mpc.Ky_stance;
                        }
                        swing_phase_msg.data[i] = swing_phase[i];
                    }
                }

                // update state (odom_legacy / sim_driver / esekf)
                update_robot_state(mpc, state_source, node->get_logger(), node->get_clock());

                // Orientation and angular velocity:
                //   esekf (valid):      already set inside update_robot_state() from /ekf
                //   esekf (not ready):  /imu_raw gyro for ang_vel; orientation stays at
                //                       identity until /ekf provides the first estimate.
                //                       imu_raw_node does NOT run an AHRS filter so its
                //                       orientation field is always identity — do not use it.
                //   odom_legacy / sim:  imu_node (CX5_AHRS) is running; use its gravity-
                //                       compensated acceleration + AHRS orientation directly.
                if (state_source == "esekf" && !has_ekf_odom) {
                    if (has_imu_raw) {
                        mpc.robot_ang_vel[0] = imu_raw.angular_velocity.x;
                        mpc.robot_ang_vel[1] = imu_raw.angular_velocity.y;
                        mpc.robot_ang_vel[2] = imu_raw.angular_velocity.z;
                    }
                } else if (state_source != "esekf") {
                    mpc.robot_ang.x() = imu.orientation.x;
                    mpc.robot_ang.y() = imu.orientation.y;
                    mpc.robot_ang.z() = imu.orientation.z;
                    mpc.robot_ang.w() = imu.orientation.w;

                    mpc.robot_ang_vel[0] = imu.angular_velocity.x;
                    mpc.robot_ang_vel[1] = imu.angular_velocity.y;
                    mpc.robot_ang_vel[2] = imu.angular_velocity.z;
                }

                quaternion_to_euler(mpc.robot_ang, mpc.roll, mpc.pitch, mpc.yaw);

                Eigen::VectorXd x(mpc.n_x);
                x << mpc.roll,             mpc.pitch,            mpc.yaw,
                     mpc.robot_pos[0],     mpc.robot_pos[1],     mpc.robot_pos[2],
                     mpc.robot_ang_vel[0], mpc.robot_ang_vel[1], mpc.robot_ang_vel[2],
                     mpc.robot_vel[0],     mpc.robot_vel[1],     mpc.robot_vel[2],
                     -mpc.gravity;

                Eigen::VectorXd x_ref = Eigen::VectorXd::Zero((mpc.N-1) * mpc.n_x);
                for (int i = 0; i < mpc.N-1; ++i) {
                    x_ref.segment(i * mpc.n_x, mpc.n_x) << 0,                0,                0,
                                                           mpc.target_pos_x, 0, mpc.target_pos_z,
                                                           0,                0,                0,
                                                           mpc.target_vel_x, 0,                0,
                                                           -mpc.gravity;
                }

                // model predictive control
                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0]+0.222, 0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = {legmodel.contact_p[0]+0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = {legmodel.contact_p[0]-0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0]-0.222, 0.2, legmodel.contact_p[1]};

                mpc.init_matrices(ra, rb, rc, rd);

                Eigen::VectorXd force = mpc.step(x, x_ref, selection_matrix, force_state_modules);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R_T = mpc.robot_ang.toRotationMatrix().transpose();

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

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Ref Pos = [" << x_ref[3] << ", " << x_ref[4] << ", " << x_ref[5] << "]" << std::endl << std::endl;
                std::cout << "Odom Pos = [" << mpc.robot_pos[0] << ", " << mpc.robot_pos[1] << ", " << mpc.robot_pos[2] << "]" << std::endl;
                std::cout << "Odom Vel = [" << mpc.robot_vel[0] << ", " << mpc.robot_vel[1] << ", " << mpc.robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang (deg) = [" << mpc.roll/M_PI*180 << ", " << mpc.pitch/M_PI*180 << ", " << mpc.yaw/M_PI*180 << "]" << std::endl << std::endl;

                std::cout << "Force A: [" << force_A[0] << ", " << force_A[2] << "]" << std::endl;
                std::cout << "State A: [" << force_state_modules[0]->fx << ", " << force_state_modules[0]->fy << "]" << std::endl << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[2] << "]" << std::endl;
                std::cout << "State B: [" << force_state_modules[1]->fx << ", " << force_state_modules[1]->fy << "]" << std::endl << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[2] << "]" << std::endl;
                std::cout << "State C: [" << force_state_modules[2]->fx << ", " << force_state_modules[2]->fy << "]" << std::endl << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[2] << "]" << std::endl;
                std::cout << "State D: [" << force_state_modules[3]->fx << ", " << force_state_modules[3]->fy << "]" << std::endl << std::endl;

                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= mpc.target_loop) {
                    std::cout << "Finished" << std::endl;
                    break;
                }

                next_time += period;
                if(!node->get_clock()->sleep_until(next_time)){
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
