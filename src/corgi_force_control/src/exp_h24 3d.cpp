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
    double b_value_;
    double k_value_;
    
};

ImpedanceCmdPublisherNode::ImpedanceCmdPublisherNode()
    : Node("imp_cmd_pub"),
    sim_(false),
    trigger_(false),
    mg_(23.05 * 9.81),
    s_(0.0),
    h_(0.0),
    b_value_(0.0),
    k_value_(0.0)
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

    const double default_b = sim_ ? 200.0 : 300.0;
    const double default_k = sim_ ? 2000.0 : 4000.0;
    b_value_ = this->declare_parameter<double>("b", default_b);
    k_value_ = this->declare_parameter<double>("k", default_k);
    RCLCPP_INFO(this->get_logger(), "Using impedance params: b=%.2f, k=%.2f", b_value_, k_value_);

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
    if (msg->enable && !trigger_) {
        RCLCPP_INFO(this->get_logger(), "Trigger received: enable=true");
    }
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
        cmd->bx = b_value_;
        cmd->by = b_value_;
        cmd->bz = b_value_;
        cmd->kx = k_value_;
        cmd->ky = k_value_;
        cmd->kz = k_value_;
    }
}

void ImpedanceCmdPublisherNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 3> eta;
    bool init_completed = true;

    RCLCPP_INFO(this->get_logger(), "Initialization phase started");

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
        double s_front = 0.255 + legmodel.contact_p_3d[0];
        double f_hind = -mg_/2.0*(s_front/0.510);
        imp_cmd_modules_[0]->fz = -mg_/2.0 - f_hind;
        imp_cmd_modules_[1]->fz = -mg_/2.0 - f_hind;
        imp_cmd_modules_[2]->fz = f_hind;
        imp_cmd_modules_[3]->fz = f_hind;

        imp_cmd_.header.seq = -1;
        imp_cmd_.header.stamp = this->now();

        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            init_completed = false;
            break;
        }
    }

    if (init_completed) {
        RCLCPP_INFO(this->get_logger(), "Initialization phase completed");
    } else {
        RCLCPP_WARN(this->get_logger(), "Initialization phase ended early");
    }
}

void ImpedanceCmdPublisherNode::execute_control_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 3> eta;
    double ds = 0.0;
    
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    int loop_count = 0;
    while (rclcpp::ok()) {
        if (loop_count < 2000) {
            ds = 0.0;
        }
        else if (loop_count < 10000) {
            if (loop_count < 2200) { ds += 2*s_/2000.0/200.0; }
            else if (loop_count < 3800) { ds =   2*s_/2000.0; }
            else if (loop_count < 4000) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 4200) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 5800) { ds =  -2*s_/2000.0; }
            else if (loop_count < 6000) { ds +=  2*s_/2000.0/200.0; }
            else if (loop_count < 6200) { ds +=  2*s_/2000.0/200.0; }
            else if (loop_count < 7800) { ds =   2*s_/2000.0; }
            else if (loop_count < 8000) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 8200) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 9800) { ds =  -2*s_/2000.0; }
            else if (loop_count < 10000) { ds += 2*s_/2000.0/200.0; }

            eta = legmodel.move_3d(
                imp_cmd_modules_[1]->theta,
                imp_cmd_modules_[1]->beta,
                imp_cmd_modules_[1]->gamma,
                {ds, 0.0, 0.0}
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
            double s_front = 0.255 + legmodel.contact_p_3d[0];
            double f_hind = -mg_/2.0*(s_front/0.510);
            imp_cmd_modules_[0]->fz = -mg_/2.0 - f_hind;
            imp_cmd_modules_[1]->fz = -mg_/2.0 - f_hind;
            imp_cmd_modules_[2]->fz = f_hind;
            imp_cmd_modules_[3]->fz = f_hind;

            if (loop_count > 6000 && loop_count < 10000) {
                imp_cmd_modules_[0]->fz += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[1]->fz -= 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[2]->fz += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[3]->fz -= 10 * sin((loop_count-2000)/500.0*M_PI);
            }
        }
        else {
            break;
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
            RCLCPP_INFO(this->get_logger(), "Trigger latched, entering control phase");
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