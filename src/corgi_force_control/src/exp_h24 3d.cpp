#include <iostream>
#include <array>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_force_control/force_control.hpp"

class ImpedanceCmdPublisherNode : public rclcpp::Node {
public:
    ImpedanceCmdPublisherNode();
    void run();

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);
    void initialize_impedance_command();
    void execute_initialization_phase();
    void execute_control_phase();
    
    // ROS2 interfaces
    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    
    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;
    
    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules_;

    // Physical parameters
    bool sim_;
    
    // State variables
    bool trigger_;
    double mg_;
    double s_;
    double h_;
    int test_module_idx_;
    double gamma_amp_rad_;
    double gamma_freq_hz_;
    
};

ImpedanceCmdPublisherNode::ImpedanceCmdPublisherNode()
    : Node("imp_cmd_pub"),
    sim_(false),
    trigger_(false),
      mg_(19.68 * 9.81),
      s_(0.0),
    h_(0.0),
    test_module_idx_(1),
    gamma_amp_rad_(8.0/180.0*M_PI),
    gamma_freq_hz_(0.5)
{
    RCLCPP_INFO(this->get_logger(), "Impedance Command Publisher Starts");

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
        std::bind(&ImpedanceCmdPublisherNode::trigger_cb, this, std::placeholders::_1));
    
    // Initialize module pointers
    imp_cmd_modules_ = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };
    
    initialize_impedance_command();
}

void ImpedanceCmdPublisherNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger_ = msg->enable;
}

void ImpedanceCmdPublisherNode::initialize_impedance_command() {
    for (auto& cmd : imp_cmd_modules_){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->gamma = 0.0;
        cmd->fx = 0;
        cmd->fy = 0;
        cmd->fz = 0;
        cmd->mx = 0;
        cmd->my = 0;
        cmd->mz = 0;
        if (sim_) {
            cmd->bx = 0;
            cmd->by = 0;
            cmd->bz = 6;
            cmd->kx = 0;
            cmd->ky = 0;
            cmd->kz = 20;
        }
        else {
            cmd->bx = 0;
            cmd->by = 0;
            cmd->bz = 2;
            cmd->kx = 0;
            cmd->ky = 0;
            cmd->kz = 8;
        }
    }
}

void ImpedanceCmdPublisherNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 3> eta;

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int i=0; i<2000; i++){
        s_ = 0.12;
        h_ = 0.12;

        eta = legmodel.move_3d(
            imp_cmd_modules_[1]->theta,
            imp_cmd_modules_[1]->beta,
            imp_cmd_modules_[1]->gamma,
            {-s_ / 2000.0, 0.0, h_ / 2000.0}
        );

        imp_cmd_modules_[0]->theta = eta[0];
        imp_cmd_modules_[1]->theta = eta[0];
        imp_cmd_modules_[2]->theta = eta[0];
        imp_cmd_modules_[3]->theta = eta[0];

        imp_cmd_modules_[0]->beta = -eta[1];
        imp_cmd_modules_[1]->beta = eta[1];
        imp_cmd_modules_[2]->beta = eta[1];
        imp_cmd_modules_[3]->beta = -eta[1];

        imp_cmd_modules_[0]->gamma = eta[2];
        imp_cmd_modules_[1]->gamma = eta[2];
        imp_cmd_modules_[2]->gamma = eta[2];
        imp_cmd_modules_[3]->gamma = eta[2];

        legmodel.contact_map_3d(eta[0], eta[1], eta[2]);
        double s_front = 0.222 + legmodel.contact_p_3d[0];
        double f_hind = -mg_/2.0*(s_front/0.444);
        imp_cmd_modules_[0]->fy = -mg_/2.0 - f_hind;
        imp_cmd_modules_[1]->fy = -mg_/2.0 - f_hind;
        imp_cmd_modules_[2]->fy = f_hind;
        imp_cmd_modules_[3]->fy = f_hind;

        imp_cmd_.header.seq = -1;
        imp_cmd_.header.stamp = this->now();

        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

void ImpedanceCmdPublisherNode::execute_control_phase() {
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    int loop_count = 0;
    const int total_count = 10000;
    while (rclcpp::ok()) {
        if (loop_count >= total_count) {
            break;
        }

        for (auto& cmd : imp_cmd_modules_) {
            cmd->theta = 17/180.0*M_PI;
            cmd->beta = 0.0;
            cmd->gamma = 0.0;
            cmd->fx = 0.0;
            cmd->fy = 0.0;
            cmd->fz = 0.0;
            cmd->mx = 0.0;
            cmd->my = 0.0;
            cmd->mz = 0.0;
            cmd->bx = 0.0;
            cmd->by = 0.0;
            cmd->bz = 0.0;
            cmd->kx = 0.0;
            cmd->ky = 0.0;
            cmd->kz = 0.0;
        }

        const double t = static_cast<double>(loop_count) / 1000.0;
        const double gamma_cmd = gamma_amp_rad_ * std::sin(2.0 * M_PI * gamma_freq_hz_ * t);

        auto* test_cmd = imp_cmd_modules_[test_module_idx_];
        test_cmd->gamma = gamma_cmd;

        if (sim_) {
            test_cmd->bz = 6.0;
            test_cmd->kz = 20.0;
        } else {
            test_cmd->bz = 2.0;
            test_cmd->kz = 8.0;
        }

        imp_cmd_.header.seq = loop_count;
        imp_cmd_.header.stamp = this->now();

        imp_cmd_pub_->publish(imp_cmd_);

        loop_count++;

        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

void ImpedanceCmdPublisherNode::run() {
    execute_initialization_phase();
    
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        if (trigger_){
            execute_control_phase();
            break;
        }

        next_time += period;
        this->get_clock()->sleep_until(next_time);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImpedanceCmdPublisherNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}