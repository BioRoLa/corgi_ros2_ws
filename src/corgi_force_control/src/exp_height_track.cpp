#include <iostream>
#include <array>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_force_control/force_control.hpp"

class HeightTrackNode : public rclcpp::Node {
public:
    HeightTrackNode();
    void run();

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);
    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void initialize_impedance_command();
    void execute_initialization_phase();
    void execute_control_phase();

    static void quaternion_to_euler(const Eigen::Quaterniond& q,
                                    double& roll, double& pitch, double& yaw);

    // ROS2 interfaces
    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;

    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;

    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules_;
    corgi_msgs::msg::ImuStamped imu_;
    corgi_msgs::msg::MotorStateStamped motor_state_;

    // State
    bool sim_;
    bool trigger_;

    // Physical parameters
    double mg_;

    // Height tracking parameters
    double h_target_;   // target body height above ground [m]
    double Kp_h_;       // height P gain [N/m]
    double Kp_pitch_;   // pitch correction proportional gain [N·m/rad]
    double Kd_pitch_;   // pitch correction derivative gain  [N·m·s/rad]
    double Kp_roll_;    // roll correction proportional gain [N·m/rad]
    double Kd_roll_;    // roll correction derivative gain   [N·m·s/rad]
};

// Hip positions in body frame (x=forward, y=lateral-left, z=up, z_hip=0 for all)
// Module order: A (left-front), B (right-front), C (right-rear), D (left-rear)
static constexpr double HIP_X[4] = { 0.212,  0.212, -0.212, -0.212};
static constexpr double HIP_Y[4] = { 0.1995, -0.1995, -0.1995,  0.1995};

HeightTrackNode::HeightTrackNode()
    : Node("height_track_node"),
      sim_(false),
      trigger_(false),
      mg_(23.68 * 9.81),
      h_target_(0.24),
      Kp_h_(50.0),
      Kp_pitch_(100.0),
      Kd_pitch_(10.0),
      Kp_roll_(0.0),
      Kd_roll_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Height Track Node Starts");

    this->get_parameter_or("use_sim_time", sim_, false);
    kinematics_ = std::make_unique<KinematicsHelper>(sim_);

    if (sim_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (this->now().seconds() > 0.0) {
                RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Real hardware mode: using system wall clock.");
    }

    imp_cmd_pub_ = this->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>(
        "impedance/command", 1000);

    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 1000,
        std::bind(&HeightTrackNode::trigger_cb, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 10,
        std::bind(&HeightTrackNode::imu_cb, this, std::placeholders::_1));

    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10,
        std::bind(&HeightTrackNode::motor_state_cb, this, std::placeholders::_1));

    imp_cmd_modules_ = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };

    initialize_impedance_command();
}

void HeightTrackNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger_ = msg->enable;
}

void HeightTrackNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void HeightTrackNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

void HeightTrackNode::quaternion_to_euler(const Eigen::Quaterniond& q,
                                          double& roll, double& pitch, double& yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void HeightTrackNode::initialize_impedance_command() {
    for (auto& cmd : imp_cmd_modules_) {
        cmd->theta = 17.0 / 180.0 * M_PI;
        cmd->beta  = 0.0 / 180.0 * M_PI;
        cmd->fx    = 0;
        cmd->fy    = 0;
        cmd->mx    = 0;
        cmd->my    = 0;
        if (sim_) {
            cmd->bx = 200;
            cmd->by = 100;
            cmd->kx = 2000;
            cmd->ky = 500;
        } else {
            cmd->bx = 80;
            cmd->by = 5;
            cmd->kx = 2000;
            cmd->ky = 50;
        }
    }
}

void HeightTrackNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 2> eta;

    rclcpp::Duration period(0, 1000000);  // 1 ms
    rclcpp::Time next_time = this->now();

    for (int i = 0; i < 2000; i++) {
        eta = legmodel.move(imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta,
                            {0.0, 0.12 / 2000.0});

        imp_cmd_modules_[0]->theta = eta[0];
        imp_cmd_modules_[1]->theta = eta[0];
        imp_cmd_modules_[2]->theta = eta[0];
        imp_cmd_modules_[3]->theta = eta[0];

        imp_cmd_modules_[0]->beta = -eta[1];
        imp_cmd_modules_[1]->beta =  eta[1];
        imp_cmd_modules_[2]->beta =  eta[1];
        imp_cmd_modules_[3]->beta = -eta[1];

        imp_cmd_modules_[0]->fy = -mg_ / 4.0;
        imp_cmd_modules_[1]->fy = -mg_ / 4.0;
        imp_cmd_modules_[2]->fy = -mg_ / 4.0;
        imp_cmd_modules_[3]->fy = -mg_ / 4.0;

        imp_cmd_.header.seq   = -1;
        imp_cmd_.header.stamp = this->now();
        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

void HeightTrackNode::execute_control_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();

    // Pointers into the latest motor_state_ (updated by motor_state_cb via spin_some)
    std::vector<corgi_msgs::msg::MotorState*> motor_modules = {
        &motor_state_.module_a,
        &motor_state_.module_b,
        &motor_state_.module_c,
        &motor_state_.module_d
    };

    rclcpp::Duration period(0, 1000000);  // 1 ms
    rclcpp::Time next_time = this->now();

    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        // --- A. IMU: body orientation and angular velocity ---
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        const double norm2 = imu_.orientation.x * imu_.orientation.x
                           + imu_.orientation.y * imu_.orientation.y
                           + imu_.orientation.z * imu_.orientation.z
                           + imu_.orientation.w * imu_.orientation.w;
        if (norm2 > 0.5) {
            Eigen::Quaterniond q(
                imu_.orientation.w,
                imu_.orientation.x,
                imu_.orientation.y,
                imu_.orientation.z);
            quaternion_to_euler(q, roll, pitch, yaw);
        }
        // Pitch and roll rates from IMU (body frame, used for derivative damping)
        const double omega_y = imu_.angular_velocity.y;  // pitch rate [rad/s]
        const double omega_x = imu_.angular_velocity.x;  // roll  rate [rad/s]

        // --- B. Pitch-corrected height estimation (z_position_legacy approach) ---
        // B(1), C(2) are right-side legs: beta_corr = beta - pitch
        // A(0), D(3) are left-side legs:  beta_corr = beta + pitch
        double z_leg[4];
        for (int i = 0; i < 4; i++) {
            const double beta_corr = (i == 1 || i == 2)
                ? motor_modules[i]->beta - pitch
                : motor_modules[i]->beta + pitch;
            legmodel.contact_map(motor_modules[i]->theta, beta_corr);
            z_leg[i] = -legmodel.contact_p[1];
        }
        const double h_est = (z_leg[0] + z_leg[1] + z_leg[2] + z_leg[3]) / 4.0;

        // --- C. Analytical VMC force distribution ---
        // Decomposes into 3 independent objectives distributed to 4 legs:
        //   Fz:  total vertical support force
        //   Ty:  pitch correction moment  (pitch > 0 = nose down → front legs get more support)
        //   Tx:  roll  correction moment  (Kp_roll_ = 0 until sign is verified by experiment)
        constexpr double SUM_X2 = 4.0 * 0.212  * 0.212;   // Σhip_x² [m²]
        constexpr double SUM_Y2 = 4.0 * 0.1995 * 0.1995;  // Σhip_y² [m²]

        const double Fz = -mg_ - Kp_h_    * (h_target_ - h_est);
        const double Ty = Kp_pitch_ * pitch + Kd_pitch_ * omega_y;
        const double Tx = Kp_roll_  * roll  + Kd_roll_  * omega_x;

        for (int i = 0; i < 4; i++) {
            imp_cmd_modules_[i]->fy = Fz / 4.0
                                    - Ty * HIP_X[i] / SUM_X2
                                    - Tx * HIP_Y[i] / SUM_Y2;
        }

        if (loop_count % 100 == 0) {
            printf("[HT] h_est=%.4f pitch=%.4f(%.3f) roll=%.4f(%.3f) "
                   "fy=[%.1f, %.1f, %.1f, %.1f]\n",
                   h_est, pitch, omega_y, roll, omega_x,
                   imp_cmd_modules_[0]->fy,
                   imp_cmd_modules_[1]->fy,
                   imp_cmd_modules_[2]->fy,
                   imp_cmd_modules_[3]->fy);
        }

        imp_cmd_.header.seq   = loop_count;
        imp_cmd_.header.stamp = this->now();
        imp_cmd_pub_->publish(imp_cmd_);

        loop_count++;

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

void HeightTrackNode::run() {
    execute_initialization_phase();

    rclcpp::Duration period(0, 1000000);  // 1 ms
    rclcpp::Time next_time = this->now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (trigger_) {
            execute_control_phase();
            break;
        }

        next_time += period;
        this->get_clock()->sleep_until(next_time);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightTrackNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
