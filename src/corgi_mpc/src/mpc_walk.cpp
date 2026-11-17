// Walking with the convex MPC on the ABAD Corgi.
//
// WalkGait supplies the leg trajectory (theta, beta; gamma = 0) and the swing
// phases. The MPC turns the body state into ground reaction forces for the
// stance legs, which go out as feed-forward forces on impedance/command.
//
// Frames: forces and the body state are in the world frame (z up), the same
// frame force_control uses for the impedance command.

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/contact_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/robot_params.hpp"
#include "corgi_force_estimation/frame_transform.hpp"
#include "corgi_walk/walk_gait.hpp"

#include "mpc.hpp"

namespace {

const double kWheelTheta = 17.0 / 180.0 * M_PI;

struct Gains {
    double kx, ky, kz, bx, by, bz;
};

void set_gains(corgi_msgs::msg::ImpedanceCmd *cmd, const Gains &g) {
    cmd->mx = 0.0; cmd->my = 0.0; cmd->mz = 0.0;
    cmd->kx = g.kx; cmd->ky = g.ky; cmd->kz = g.kz;
    cmd->bx = g.bx; cmd->by = g.by; cmd->bz = g.bz;
}

// WalkGait keeps all legs in one sagittal frame with +x forward. The leg
// frames of module B and C face backwards, so their model beta is mirrored.
Eigen::Vector3d gait_to_motor(int module, const corgi_force::LegFrame &frame, double theta, double beta) {
    double model_beta = (module == 0 || module == 3) ? beta : -beta;
    return frame.motor_angles(Eigen::Vector3d(theta, model_beta, 0.0));
}

} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_mpc");
    RCLCPP_INFO(node->get_logger(), "Corgi MPC Starts");

    bool sim = false;
    node->get_parameter("use_sim_time", sim);

    // Gait
    const double stand_height = node->declare_parameter<double>("stand_height", 0.25); // hip height above the ground
    const double velocity     = node->declare_parameter<double>("velocity", 0.1);
    const double step_length  = node->declare_parameter<double>("step_length", 0.3);
    const double step_height  = node->declare_parameter<double>("step_height", 0.04);
    const double duration     = node->declare_parameter<double>("duration", 20.0);     // s of walking, ramp-up included
    // Impedance about the gait trajectory, world x / y / z
    Gains stance, swing;
    stance.kx = node->declare_parameter<double>("stance.kx", 2000.0);
    stance.ky = node->declare_parameter<double>("stance.ky", 2000.0);
    stance.kz = node->declare_parameter<double>("stance.kz", 2000.0);
    stance.bx = node->declare_parameter<double>("stance.bx", 200.0);
    stance.by = node->declare_parameter<double>("stance.by", 200.0);
    stance.bz = node->declare_parameter<double>("stance.bz", 200.0);
    swing.kx = node->declare_parameter<double>("swing.kx", 2000.0);
    swing.ky = node->declare_parameter<double>("swing.ky", 2000.0);
    swing.kz = node->declare_parameter<double>("swing.kz", 2000.0);
    swing.bx = node->declare_parameter<double>("swing.bx", 200.0);
    swing.by = node->declare_parameter<double>("swing.by", 200.0);
    swing.bz = node->declare_parameter<double>("swing.bz", 200.0);
    // Body state sources
    const std::string position_topic = node->declare_parameter<std::string>("position_topic", "odometry/legacy/position");
    const std::string velocity_topic = node->declare_parameter<std::string>("velocity_topic", "odometry/legacy/velocity");
    const std::string height_topic   = node->declare_parameter<std::string>("height_topic", "odometry/legacy/z_position_hip");
    const std::string contact_topic  = node->declare_parameter<std::string>("contact_topic", "odometry/legacy/contact");
    const std::string config_path    = node->declare_parameter<std::string>("config_path", "");
    // odom_legacy: the topics above. sim_driver (simulation only): Webots ground
    // truth, /tf odom -> base_link and sim/body/velocity from corgi_sim.
    const std::string state_source   = node->declare_parameter<std::string>("state_source", "odom_legacy");
    if (state_source != "odom_legacy" && state_source != "sim_driver") {
        RCLCPP_ERROR(node->get_logger(), "state_source must be odom_legacy or sim_driver");
        return 1;
    }

    if (state_source == "sim_driver" && !sim) {
        RCLCPP_ERROR(node->get_logger(), "state_source:=sim_driver needs use_sim_time:=true");
        return 1;
    }

    if (sim) {
        RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (node->now().seconds() > 0.0) {
                RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    ModelPredictiveController mpc;
    mpc.load_config(config_path);

    // ---- Communication ----
    bool trigger = false;
    bool directions_received = false;
    bool imu_received = false;
    bool motor_state_received = false;
    corgi_msgs::msg::MotorStateStamped motor_state;
    corgi_msgs::msg::ImuStamped imu;
    geometry_msgs::msg::Vector3 odom_pos, odom_vel;
    double odom_z = stand_height;
    const bool use_sim_state = state_source == "sim_driver";
    bool sim_pos_received = false, sim_vel_received = false;
    geometry_msgs::msg::Vector3 sim_pos, sim_vel;

    std::array<corgi_force::LegFrame, 4> leg_frames;
    for (int i = 0; i < 4; ++i) leg_frames[i] = corgi_force::leg_frame(i, sim);

    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);
    auto contact_pub = node->create_publisher<corgi_msgs::msg::ContactStateStamped>(contact_topic, 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, [&](const corgi_msgs::msg::TriggerStamped::SharedPtr msg) { trigger = msg->enable; });
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10, [&](const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
            motor_state = *msg;
            motor_state_received = true;
        });
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 10, [&](const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
            imu = *msg;
            imu_received = true;
        });
    auto odom_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
        position_topic, 10, [&](const geometry_msgs::msg::Vector3::SharedPtr msg) { odom_pos = *msg; });
    auto odom_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
        velocity_topic, 10, [&](const geometry_msgs::msg::Vector3::SharedPtr msg) { odom_vel = *msg; });
    auto odom_z_sub = node->create_subscription<std_msgs::msg::Float64>(
        height_topic, 10, [&](const std_msgs::msg::Float64::SharedPtr msg) { odom_z = msg->data; });
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sim_tf_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sim_vel_sub;
    if (use_sim_state) {
        sim_tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto &tf : msg->transforms) {
                    if (tf.child_frame_id != "base_link") continue;
                    sim_pos.x = tf.transform.translation.x;
                    sim_pos.y = tf.transform.translation.y;
                    sim_pos.z = tf.transform.translation.z;
                    sim_pos_received = true;
                }
            });
        sim_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
            "sim/body/velocity", 10, [&](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                sim_vel = *msg;
                sim_vel_received = true;
            });
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr directions_sub;
    if (sim) {
        // Joint and motor signs of the simulated robot, needed by the leg frames.
        directions_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "motor/directions", rclcpp::QoS(1).transient_local(),
            [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                directions_received = corgi_force::update_directions(msg->data, leg_frames);
                if (!directions_received) RCLCPP_ERROR(node->get_logger(), "Invalid motor/directions: expected 20 signs (+1 or -1)");
            });
        RCLCPP_INFO(node->get_logger(), "Waiting for motor/directions ...");
        while (rclcpp::ok() && !directions_received) {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    corgi_msgs::msg::ContactStateStamped contact_state;
    std::array<corgi_msgs::msg::ImpedanceCmd *, 4> imp_cmd_modules = {
        &imp_cmd.module_a, &imp_cmd.module_b, &imp_cmd.module_c, &imp_cmd.module_d};
    std::array<corgi_msgs::msg::ContactState *, 4> contact_state_modules = {
        &contact_state.module_a, &contact_state.module_b, &contact_state.module_c, &contact_state.module_d};
    std::array<corgi_msgs::msg::MotorState *, 4> motor_state_modules = {
        &motor_state.module_a, &motor_state.module_b, &motor_state.module_c, &motor_state.module_d};

    rclcpp::Duration period = rclcpp::Duration::from_seconds(mpc.dt);
    rclcpp::Time next_time = node->now();
    auto sleep_one_period = [&]() {
        next_time += period;
        return node->get_clock()->sleep_until(next_time);
    };
    auto publish_cmd = [&]() {
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
    };

    // ---- Gait ----
    LegModel legmodel(sim);
    WalkGait walk_gait(sim, 0.0, mpc.freq);
    walk_gait.stand_height = stand_height;
    walk_gait.step_height = step_height;
    std::array<double, 8> init_eta = walk_gait.get_init_eta(step_length);
    walk_gait.initialize(init_eta.data(), step_length);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_height(step_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_velocity(0.0);

    // Standing pose in motor coordinates (init_eta stores beta negated for A and D)
    std::array<Eigen::Vector3d, 4> stand_pose;
    for (int i = 0; i < 4; ++i) {
        double gait_beta = (i == 1 || i == 2) ? init_eta[2 * i + 1] : -init_eta[2 * i + 1];
        stand_pose[i] = gait_to_motor(i, leg_frames[i], init_eta[2 * i], gait_beta);
    }

    // ---- Stand up from wheel mode ----
    const double weight_per_leg = mpc.m * mpc.gravity / 4.0;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d wheel_pose = leg_frames[i].motor_angles(Eigen::Vector3d(kWheelTheta, 0.0, 0.0));
        imp_cmd_modules[i]->theta = wheel_pose.x();
        imp_cmd_modules[i]->beta = wheel_pose.y();
        imp_cmd_modules[i]->gamma = wheel_pose.z();
        // The command is the force the foot applies to the ground.
        imp_cmd_modules[i]->fx = 0.0;
        imp_cmd_modules[i]->fy = 0.0;
        imp_cmd_modules[i]->fz = -weight_per_leg;
        set_gains(imp_cmd_modules[i], swing);
    }

    if (!sim) {
        RCLCPP_INFO(node->get_logger(), "Wait For Force Control Node ...");
        for (int i = 0; i < 3 * mpc.freq && rclcpp::ok(); i++) sleep_one_period();
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");
    const int transform_steps = 3 * mpc.freq;
    std::array<Eigen::Vector3d, 4> start_pose;
    for (int i = 0; i < 4; ++i) {
        start_pose[i] << imp_cmd_modules[i]->theta, imp_cmd_modules[i]->beta, imp_cmd_modules[i]->gamma;
    }
    for (int k = 1; k <= transform_steps && rclcpp::ok(); k++) {
        double ratio = static_cast<double>(k) / transform_steps;
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d pose = start_pose[i] + ratio * (stand_pose[i] - start_pose[i]);
            imp_cmd_modules[i]->theta = pose.x();
            imp_cmd_modules[i]->beta = pose.y();
            imp_cmd_modules[i]->gamma = pose.z();
        }
        publish_cmd();
        sleep_one_period();
    }
    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    for (int k = 0; k < 2 * mpc.freq && rclcpp::ok(); k++) {
        rclcpp::spin_some(node);
        publish_cmd();
        sleep_one_period();
    }

    // ---- Wait for the trigger ----
    RCLCPP_INFO(node->get_logger(), "Waiting for trigger ...");
    while (rclcpp::ok() && !trigger) {
        rclcpp::spin_some(node);
        publish_cmd();
        sleep_one_period();
    }

    RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");
    for (auto &state : contact_state_modules) state->contact = true;
    for (int k = 0; k < (sim ? 1 : 3) * mpc.freq && rclcpp::ok(); k++) {
        rclcpp::spin_some(node);
        contact_state.header.stamp = node->now();
        contact_pub->publish(contact_state);
        publish_cmd();
        sleep_one_period();
    }

    for (auto &cmd : imp_cmd_modules) set_gains(cmd, stance);

    // ---- MPC loop ----
    RCLCPP_INFO(node->get_logger(), "MPC Controller Starts ...");
    const Eigen::Matrix3d imu_axes = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(); // hardware IMU convention, see body_rotation()
    const int target_loop = static_cast<int>(duration * mpc.freq);
    const int ramp_loop = mpc.freq; // 1 s
    std::array<bool, 4> in_stance = {true, true, true, true};
    double target_vel = 0.0;
    bool reference_set = false;
    bool ground_located = false;
    double ground_z = 0.0;
    double yaw_ref = 0.0;
    Eigen::Vector2d pos_ref = Eigen::Vector2d::Zero();
    Eigen::Vector2d heading(1.0, 0.0);

    // Stopping: WalkGait advances the swing with the distance walked, so
    // slowing down keeps a leg in the air longer and at zero speed freezes it
    // there. After the walking time the robot keeps cruise speed until all
    // four feet are down (or settle_timeout passes), then stands.
    const int settle_timeout = 3 * mpc.freq;
    bool standing = false;

    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (standing) {
            // Hold the last stance pose with an even share of the weight, so
            // force control is not left holding the last MPC forces.
            publish_cmd();
            if (!sleep_one_period()) break;
            continue;
        }

        // Velocity profile: ramp up, then cruise until the stop.
        if (loop_count < ramp_loop) {
            target_vel += velocity / ramp_loop;
        }
        walk_gait.set_velocity(target_vel);

        // Leg trajectory and stance / swing switching
        std::array<std::array<double, 4>, 2> eta = walk_gait.step();
        std::array<int, 4> swing_phase = walk_gait.get_swing_phase();
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d pose = gait_to_motor(i, leg_frames[i], eta[0][i], eta[1][i]);
            imp_cmd_modules[i]->theta = pose.x();
            imp_cmd_modules[i]->beta = pose.y();
            imp_cmd_modules[i]->gamma = pose.z();
            if (swing_phase[i] == 1 && in_stance[i]) {
                in_stance[i] = false;
                set_gains(imp_cmd_modules[i], swing);
            } else if (swing_phase[i] == 0 && !in_stance[i]) {
                in_stance[i] = true;
                set_gains(imp_cmd_modules[i], stance);
            }
        }

        // Body state in the world frame
        Eigen::Matrix3d world_from_body = Eigen::Matrix3d::Identity();
        Eigen::Vector3d ang_vel_body = Eigen::Vector3d::Zero();
        if (imu_received) {
            Eigen::Quaterniond q(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
            if (!corgi_force::body_rotation(q, sim, world_from_body)) world_from_body.setIdentity();
            ang_vel_body << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;
            if (!sim) ang_vel_body = imu_axes * ang_vel_body;
        }
        double roll = std::atan2(world_from_body(2, 1), world_from_body(2, 2));
        double pitch = -std::asin(std::max(-1.0, std::min(1.0, world_from_body(2, 0))));
        double yaw = std::atan2(world_from_body(1, 0), world_from_body(0, 0));
        Eigen::Vector3d ang_vel = world_from_body * ang_vel_body;

        // Foot positions relative to the CoM, from the measured joint angles
        std::array<Eigen::Vector3d, 4> r;
        for (int i = 0; i < 4; ++i) {
            const corgi_force::LegFrame &frame = leg_frames[i];
            Eigen::Vector3d model(eta[0][i], (i == 0 || i == 3) ? eta[1][i] : -eta[1][i], 0.0);
            if (motor_state_received) {
                model = frame.model_angles(motor_state_modules[i]->theta, motor_state_modules[i]->beta, motor_state_modules[i]->gamma);
            }
            Eigen::Matrix3d world_from_leg = world_from_body * frame.body_from_leg;
            Eigen::Vector3d normal = world_from_leg.transpose() * Eigen::Vector3d::UnitZ();
            legmodel.contact_map_3d(model.x(), model.y(), model.z(),
                                    std::array<double, 3>{normal.x(), normal.y(), normal.z()});
            std::array<double, 3> hip = corgi_robot::abad_origin(i);
            Eigen::Vector3d foot_body = Eigen::Vector3d(hip[0], hip[1], hip[2])
                                        + frame.body_from_leg * Eigen::Map<Eigen::Vector3d>(legmodel.contact_p_3d.data());
            r[i] = world_from_body * (foot_body - mpc.com_offset);
        }

        // Body position and velocity (world frame). The MPC height is the CoM
        // height above the ground.
        Eigen::Vector3d pos(odom_pos.x, odom_pos.y, odom_z);
        Eigen::Vector3d vel(odom_vel.x, odom_vel.y, odom_vel.z);
        if (use_sim_state) {
            if (sim_pos_received && sim_vel_received) {
                const double com_z = sim_pos.z + (world_from_body * mpc.com_offset).z();
                if (!ground_located) {
                    // The Webots floor is not at z = 0: locate it once, from the
                    // leg kinematics while all four feet are down.
                    double foot_z = 0.0;
                    for (const auto &ri : r) foot_z += ri.z() / 4.0;
                    ground_z = com_z + foot_z;
                    ground_located = true;
                }
                pos << sim_pos.x, sim_pos.y, com_z - ground_z;
                vel << sim_vel.x, sim_vel.y, sim_vel.z;
            } else {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                    "state_source=sim_driver: no /tf base_link or sim/body/velocity yet, using legacy odometry");
            }
        }

        if (!reference_set) {
            // Walk along the heading the robot has when the controller starts.
            yaw_ref = yaw;
            heading << std::cos(yaw), std::sin(yaw);
            pos_ref << pos.x(), pos.y();
            reference_set = true;
        }
        pos_ref += target_vel * mpc.dt * heading;

        Eigen::VectorXd x(mpc.n_x);
        x << roll, pitch, yaw,
             pos,
             ang_vel,
             vel,
             -mpc.gravity;
        // Keep the yaw error continuous across +-pi.
        x(2) = yaw_ref + std::remainder(yaw - yaw_ref, 2.0 * M_PI);

        Eigen::VectorXd x_ref((mpc.N - 1) * mpc.n_x);
        for (int k = 0; k < mpc.N - 1; ++k) {
            Eigen::Vector2d pos_k = pos_ref + (k + 1) * mpc.dt * target_vel * heading;
            x_ref.segment(k * mpc.n_x, mpc.n_x) << 0.0, 0.0, yaw_ref,
                                                   pos_k.x(), pos_k.y(), stand_height,
                                                   0.0, 0.0, 0.0,
                                                   target_vel * heading.x(), target_vel * heading.y(), 0.0,
                                                   -mpc.gravity;
        }

        mpc.update_model(yaw, r);
        Eigen::VectorXd force = mpc.step(x, x_ref, in_stance);
        if (!mpc.solved) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "MPC did not converge, holding the previous forces");
        }

        // The MPC gives the ground reaction on the robot; the command is the
        // opposite force, applied by the foot to the ground.
        for (int i = 0; i < 4; ++i) {
            imp_cmd_modules[i]->fx = -force(3 * i);
            imp_cmd_modules[i]->fy = -force(3 * i + 1);
            imp_cmd_modules[i]->fz = -force(3 * i + 2);
        }
        publish_cmd();

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "v_ref %.2f | pos [%.3f %.3f %.3f] vel [%.3f %.3f %.3f] rpy(deg) [%.1f %.1f %.1f] | fz [%.0f %.0f %.0f %.0f] fy [%.0f %.0f %.0f %.0f]",
            target_vel, x(3), x(4), x(5), x(9), x(10), x(11),
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
            force(2), force(5), force(8), force(11), force(1), force(4), force(7), force(10));

        loop_count++;
        if (loop_count >= target_loop) {
            const bool all_stance = in_stance[0] && in_stance[1] && in_stance[2] && in_stance[3];
            if (all_stance || loop_count >= target_loop + settle_timeout) {
                if (!all_stance) {
                    RCLCPP_WARN(node->get_logger(), "Not all feet down %d cycles after the walk, stopping anyway", settle_timeout);
                }
                for (auto &cmd : imp_cmd_modules) {
                    set_gains(cmd, stance);
                    cmd->fx = 0.0;
                    cmd->fy = 0.0;
                    cmd->fz = -weight_per_leg;
                }
                standing = true;
                RCLCPP_INFO(node->get_logger(), "Finished");
            }
        }

        if (!sleep_one_period()) {
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
