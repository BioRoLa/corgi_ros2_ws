#include "walk_utils.hpp"
#include "mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

bool trigger = false;
corgi_msgs::msg::ForceStateStamped force_state;

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg){
    force_state = *msg;
}

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_trot");

    node->declare_parameter<std::string>("config_profile", "sim");
    std::string config_profile = node->get_parameter("config_profile").as_string();
    if (config_profile != "sim" && config_profile != "real") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid config_profile='%s', fallback to 'sim'",
                    config_profile.c_str());
        config_profile = "sim";
    }
    sim = (config_profile == "sim");

    RCLCPP_INFO(node->get_logger(), "Corgi Trot Starts");
    RCLCPP_INFO(node->get_logger(), "Config profile: %s", config_profile.c_str());

    // ── Load trot parameters from config.yaml common.trot section ─────────
    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        RCLCPP_FATAL(node->get_logger(), "HOME environment variable not set");
        return 1;
    }
    const std::string config_path = std::string(home_path) +
        "/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml";
    YAML::Node cfg        = YAML::LoadFile(config_path);
    YAML::Node common_cfg = cfg["common"];
    YAML::Node trot_cfg   = common_cfg ? common_cfg["trot"] : YAML::Node{};
    YAML::Node profile_cfg = cfg[config_profile];

    // Lookup order: profile-specific section first, then common.trot section.
    auto trot_read_double = [&](const std::string& key) -> double {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<double>();
        if (trot_cfg    && trot_cfg[key])    return trot_cfg[key].as<double>();
        throw std::runtime_error("trot_open: missing required config key: " + key);
    };
    auto trot_read_int = [&](const std::string& key) -> int {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<int>();
        if (trot_cfg    && trot_cfg[key])    return trot_cfg[key].as<int>();
        throw std::runtime_error("trot_open: missing required config key: " + key);
    };
    auto trot_read_vec = [&](const std::string& key) -> std::vector<double> {
        if (profile_cfg && profile_cfg[key]) return profile_cfg[key].as<std::vector<double>>();
        // init_eta is in the profile section (sim/real), not in trot sub-block
        throw std::runtime_error("trot_open: missing required config key: " + key);
    };

    const double target_pos_z  = trot_read_double("target_pos_z");
    const double velocity      = trot_read_double("velocity");
    const double stand_height  = trot_read_double("stand_height");
    const double step_length   = trot_read_double("step_length");
    const double step_height   = trot_read_double("step_height");
    const int    target_loop   = trot_read_int("target_loop");

    // ── Initial joint angles (profile-specific: different for sim and real) ─
    const std::vector<double> init_eta_vec = trot_read_vec("init_eta");
    if (static_cast<int>(init_eta_vec.size()) != 8) {
        RCLCPP_FATAL(node->get_logger(),
            "init_eta must have exactly 8 values, got %zu", init_eta_vec.size());
        return 1;
    }
    double init_eta[8];
    for (int i = 0; i < 8; ++i) init_eta[i] = init_eta_vec[i];

    RCLCPP_INFO(node->get_logger(),
        "Trot: target_pos_z=%.2f  velocity=%.2f  stand_height=%.2f  "
        "step_length=%.2f  step_height=%.3f  target_loop=%d",
        target_pos_z, velocity, stand_height, step_length, step_height, target_loop);

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
    mpc.target_loop  = target_loop;
    mpc.target_pos_z = target_pos_z;

    auto motor_cmd_pub  = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto swing_phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto trigger_sub    = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);

    rclcpp::Duration period(0, 1000000); // 1 ms
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std_msgs::msg::Int32MultiArray swing_phase_msg;
    swing_phase_msg.data.resize(4, 0);

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    TrotGait walk_gait(sim, 0, mpc.freq);
    walk_gait.initialize(init_eta);

    walk_gait.stand_height = stand_height;
    walk_gait.velocity     = velocity;
    walk_gait.step_length  = step_length;
    walk_gait.step_height  = step_height;

    walk_gait.set_velocity(mpc.target_vel_x);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);

    bool touched[4]          = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};

    // Initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta  = 0/180.0*M_PI;
        cmd->kp_r  = 90;
        cmd->kp_l  = 90;
        cmd->ki_r  = 0;
        cmd->ki_l  = 0;
        cmd->kd_r  = 1.75;
        cmd->kd_l  = 1.75;
    }

    RCLCPP_INFO(node->get_logger(), "Wait ...");

    if (!sim) {
        for (int i = 0; i < 3000; i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    // Transform: smoothly move joints from default pose to init_eta
    for (int i = 0; i < 3000; i++) {
        for (int j = 0; j < 4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j]   - 17/180.0*M_PI) / 3000.0;
            motor_cmd_modules[j]->beta  += init_eta[2*j+1] / 3000.0;
        }
        motor_cmd.header.stamp = node->now();
        motor_cmd_pub->publish(motor_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // Stay at init pose
    for (int i = 0; i < 2000; i++) {
        rclcpp::spin_some(node);
        motor_cmd.header.stamp = node->now();
        motor_cmd_pub->publish(motor_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger) {
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

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

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // Velocity ramp-up and ramp-down (1000 cycles each)
                if (loop_count < 1000) {
                    mpc.target_vel_x += velocity / 1000.0;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }
                if (loop_count > mpc.target_loop*10 - 1000 && loop_count < mpc.target_loop*10) {
                    mpc.target_vel_x -= velocity / 1000.0;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }

                mpc.target_pos_x += mpc.target_vel_x * mpc.dt / 10.0;

                // Get next joint angles from trot gait
                mpc.eta_list = walk_gait.step();
                const auto swing_phase = walk_gait.get_swing_phase();
                for (int i = 0; i < 4; i++) {
                    swing_phase_msg.data[i] = swing_phase[i];
                }

                for (int i = 0; i < 4; i++) {
                    if (mpc.eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (mpc.eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = mpc.eta_list[0][i];
                    motor_cmd_modules[i]->beta  = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];
                }

                motor_cmd.header.stamp = node->now();
                motor_cmd_pub->publish(motor_cmd);
                swing_phase_pub->publish(swing_phase_msg);

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Target Position X: "  << mpc.target_pos_x  << std::endl << std::endl;
                std::cout << "Current Velocity X: " << mpc.target_vel_x  << std::endl << std::endl;
                std::cout << "'=' '=' '=' '=' '='" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= mpc.target_loop*10) break;

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
