#include <iostream>
#include <array>
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
    KinematicsHelper kinematics_;
    
    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules_;
    
    // State variables
    bool trigger_;
    double mg_;
    double s_;
    double h_;
    
    // Physical parameters
    const bool sim_ = true;
};

ImpedanceCmdPublisherNode::ImpedanceCmdPublisherNode()
    : Node("imp_cmd_pub"),
      kinematics_(sim_),
      trigger_(false),
      mg_(19.68 * 9.81),
      s_(0.0),
      h_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Impedance Command Publisher Starts");
    
    // Wait for clock synchronization
    RCLCPP_INFO(this->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (this->now().seconds() > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
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
    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules_){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->fx = 0;
        cmd->fy = 0;
        cmd->mx = 0;
        cmd->my = 0;
        if (sim_) {
            cmd->bx = 80; //200
            cmd->by = 10; //200
            cmd->kx = 2000; //2000
            cmd->ky = 100; //2000
        }
        else {
            cmd->bx = 80;
            cmd->by = 10;
            cmd->kx = 2000;
            cmd->ky = 100;
        }
    }
}

void ImpedanceCmdPublisherNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_.get_leg_model();
    std::array<double, 2> eta;
    
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    for (int i=0; i<2000; i++){
        s_ = 0.12;
        h_ = 0.12;

        eta = legmodel.move(imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta, {-s_/2000.0, h_/2000.0});
        
        imp_cmd_modules_[0]->theta = eta[0];
        imp_cmd_modules_[1]->theta = eta[0];
        imp_cmd_modules_[2]->theta = eta[0];
        imp_cmd_modules_[3]->theta = eta[0];

        imp_cmd_modules_[0]->beta = -eta[1];
        imp_cmd_modules_[1]->beta = eta[1];
        imp_cmd_modules_[2]->beta = eta[1];
        imp_cmd_modules_[3]->beta = -eta[1];
        
        legmodel.contact_map(eta[0], eta[1]);
        double s_front = 0.222+legmodel.contact_p[0];
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
    LegModel& legmodel = kinematics_.get_leg_model();
    std::array<double, 2> eta;
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

            eta = legmodel.move(imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta, {ds, 0.0});

            imp_cmd_modules_[0]->theta = eta[0];
            imp_cmd_modules_[1]->theta = eta[0];
            imp_cmd_modules_[2]->theta = eta[0];
            imp_cmd_modules_[3]->theta = eta[0];

            imp_cmd_modules_[0]->beta = -eta[1];
            imp_cmd_modules_[1]->beta = eta[1];
            imp_cmd_modules_[2]->beta = eta[1];
            imp_cmd_modules_[3]->beta = -eta[1];

            legmodel.contact_map(eta[0], eta[1]);
            double s_front = 0.222+legmodel.contact_p[0];
            double f_hind = -mg_/2.0*(s_front/0.444);
            imp_cmd_modules_[0]->fy = -mg_/2.0 - f_hind;
            imp_cmd_modules_[1]->fy = -mg_/2.0 - f_hind;
            imp_cmd_modules_[2]->fy = f_hind;
            imp_cmd_modules_[3]->fy = f_hind;

            if (loop_count > 6000 && loop_count < 10000) {
                imp_cmd_modules_[0]->fy += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[1]->fy -= 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[2]->fy += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[3]->fy -= 10 * sin((loop_count-2000)/500.0*M_PI);
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