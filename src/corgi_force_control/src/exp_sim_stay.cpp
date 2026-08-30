#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "corgi_force_estimation/force_estimation.hpp"

class ExpSimStayNode : public rclcpp::Node {
public:
    ExpSimStayNode();
    void run();

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);
    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);

    void initialize_impedance_command();
    void execute_initialization_phase();
    void execute_control_phase();

    static Eigen::Vector4d distribute_forces(double sa, double sb, double sc, double sd, double mg, double L, double t);
    static Eigen::Vector4d distribute_forces_(double sa, double sd, double mg, double L, double t);

    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;

    std::unique_ptr<KinematicsHelper> kinematics_;

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ImuStamped imu_;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules_;
    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules_;

    bool trigger_;
    bool sim_;
    int exp_case_;
    double mg_;
    double f_init_;
};

ExpSimStayNode::ExpSimStayNode()
    : Node("imp_sim_stay"),
      trigger_(false),
      sim_(false),
      exp_case_(2),
      mg_(-19.5 * 9.81),
      f_init_(mg_ / 4.0)
{
    RCLCPP_INFO(this->get_logger(), "Simulation Stay Experiment Starts");

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

    imp_cmd_pub_ = this->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000);
    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 1000,
        std::bind(&ExpSimStayNode::trigger_cb, this, std::placeholders::_1));
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 1000,
        std::bind(&ExpSimStayNode::motor_state_cb, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 1000,
        std::bind(&ExpSimStayNode::imu_cb, this, std::placeholders::_1));

    imp_cmd_modules_ = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };

    motor_state_modules_ = {
        &motor_state_.module_a,
        &motor_state_.module_b,
        &motor_state_.module_c,
        &motor_state_.module_d
    };

    initialize_impedance_command();
}

void ExpSimStayNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger_ = msg->enable;
}

void ExpSimStayNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void ExpSimStayNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

Eigen::Vector4d ExpSimStayNode::distribute_forces(double sa, double sb, double sc, double sd, double mg, double L, double t)
{
    double Fa = t;
    double Fd = mg / 2.0 - t;

    double denominator = L + sb - sc;

    double Fc = (t * (L - sa - sd) + (mg / 2.0) * (sb + sd)) / denominator;

    double Fb = mg / 2.0 - Fc;

    return Eigen::Vector4d(Fa, Fb, Fc, Fd);
}

Eigen::Vector4d ExpSimStayNode::distribute_forces_(double sa, double sd, double mg, double L, double t)
{
    double Fa = t;
    double Fd = mg / 2.0 - t;
    double Fb = (mg / 2.0) * (L - sa - sd) / L - Fa;
    double Fc = mg / 2.0 - Fb;

    return Eigen::Vector4d(Fa, Fb, Fc, Fd);
}

void ExpSimStayNode::initialize_impedance_command() {
    for (auto& cmd : imp_cmd_modules_) {
        cmd->theta = 17 / 180.0 * M_PI;
        cmd->beta = 0 / 180.0 * M_PI;
        cmd->gamma = 0.0;
        cmd->fx = 0.0;
        cmd->fy = 0.0;
        cmd->fz = 0.0;
        cmd->mx = 0.0;
        cmd->my = 0.0;
        cmd->mz = 0.0;
        cmd->bx = 100.0;
        cmd->by = 100.0;
        cmd->bz = 0.0;
        cmd->kx = 2000.0;
        cmd->ky = 2000.0;
        cmd->kz = 0.0;
    }
}

void ExpSimStayNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();

    Eigen::Vector4d forces;
    double sa = 0.0;
    double sd = 0.0;

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int i = 0; i < 2000 && rclcpp::ok(); i++) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (exp_case_ == 0) {
            imp_cmd_modules_[0]->theta += 63 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[1]->theta += 63 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[2]->theta += 63 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[3]->theta += 63 / 2000.0 / 180.0 * M_PI;
        }
        if (exp_case_ == 1) {
            imp_cmd_modules_[0]->theta += 23 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[1]->theta += 23 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[2]->theta += 23 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[3]->theta += 23 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[0]->beta += 25 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[1]->beta -= 25 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[2]->beta -= 25 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[3]->beta += 25 / 2000.0 / 180.0 * M_PI;
        }
        if (exp_case_ == 2) {
            imp_cmd_modules_[0]->theta += 43 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[1]->theta += 43 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[2]->theta += 43 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[3]->theta += 43 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[0]->beta += 80 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[1]->beta -= 80 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[2]->beta -= 80 / 2000.0 / 180.0 * M_PI;
            imp_cmd_modules_[3]->beta += 80 / 2000.0 / 180.0 * M_PI;
        }

        legmodel.contact_map(motor_state_modules_[0]->theta, motor_state_modules_[0]->beta);
        sa = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules_[3]->theta, motor_state_modules_[3]->beta);
        sd = legmodel.contact_p[0];

        forces = distribute_forces_(-sa / 2.0, -sd / 2.0, mg_, 0.222, f_init_);

        std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;

        for (int module_index = 0; module_index < 4; module_index++) {
            imp_cmd_modules_[module_index]->fy = forces[module_index];
        }

        imp_cmd_.header.seq = -1;
        imp_cmd_.header.stamp = this->now();
        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

void ExpSimStayNode::execute_control_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();

    Eigen::Vector4d forces;
    double sa = 0.0;
    double sb = 0.0;
    double sc = 0.0;
    double sd = 0.0;

    Eigen::Quaterniond robot_ang = Eigen::Quaterniond::Identity();
    double pitch = 0.0;

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (loop_count < 2000) {
        }
        else if (loop_count < 3000) {
            f_init_ -= 0.03;
        }
        else if (loop_count < 4000) {
        }
        else if (loop_count < 5000) {
            f_init_ += 0.03;
        }
        else if (loop_count < 6000) {
        }
        else if (loop_count < 7000) {
            f_init_ -= 0.02;
        }
        else if (loop_count < 8000) {
        }
        else if (loop_count < 9000) {
            f_init_ += 0.02;
        }
        else if (loop_count < 10000) {
        }
        else if (loop_count < 11000) {
            f_init_ -= 0.01;
        }
        else if (loop_count < 12000) {
        }
        else if (loop_count < 13000) {
            f_init_ += 0.01;
        }
        else if (loop_count < 14000) {
        }
        else {
            break;
        }

        robot_ang.x() = imu_.orientation.x;
        robot_ang.y() = imu_.orientation.y;
        robot_ang.z() = imu_.orientation.z;
        robot_ang.w() = imu_.orientation.w;

        Eigen::Vector3d euler = robot_ang.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
        pitch = euler.y();

        legmodel.contact_map(motor_state_modules_[0]->theta, motor_state_modules_[0]->beta + pitch);
        sa = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules_[1]->theta, motor_state_modules_[1]->beta - pitch);
        sb = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules_[2]->theta, motor_state_modules_[2]->beta - pitch);
        sc = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules_[3]->theta, motor_state_modules_[3]->beta + pitch);
        sd = legmodel.contact_p[0];

        std::cout << sa << ", " << sb << ", " << sc << ", " << sd << std::endl << std::endl;

        forces = distribute_forces_(-sa / 2.0, -sd / 2.0, mg_, 0.222, f_init_);

        std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;

        for (int module_index = 0; module_index < 4; module_index++) {
            imp_cmd_modules_[module_index]->fy = forces[module_index];
        }

        if (loop_count > 2000 && loop_count < 13000) {
            imp_cmd_modules_[0]->fx = 30 * sin(loop_count / 1000.0 * M_PI);
            imp_cmd_modules_[1]->fx = 30 * sin(loop_count / 1000.0 * M_PI);
            imp_cmd_modules_[2]->fx = -30 * sin(loop_count / 1000.0 * M_PI);
            imp_cmd_modules_[3]->fx = -30 * sin(loop_count / 1000.0 * M_PI);
        }

        imp_cmd_.header.seq = loop_count;
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

void ExpSimStayNode::run() {
    execute_initialization_phase();

    rclcpp::Duration period(0, 1000000); // 1ms
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExpSimStayNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}