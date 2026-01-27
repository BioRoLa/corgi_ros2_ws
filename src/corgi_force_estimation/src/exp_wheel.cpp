#include "corgi_force_estimation/force_estimation.hpp"

class CorgiWheelNode : public rclcpp::Node {
public:
    CorgiWheelNode();
    
    void run();

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);
    void initialize_motor_command();
    void execute_transform_phase();
    void execute_stay_phase();
    void execute_control_phase();

    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    
    // Message cache
    corgi_msgs::msg::MotorCmdStamped motor_cmd_;
    
    // Motor command module pointers
    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules_;
    
    // State variables
    bool trigger_;
    double velocity_;
    
    // Parameters
    const bool sim_ = false;
    const int target_loop_ = 8000;
    const double init_eta_[8] = {
        0.29670597283903605, -0.0, 
        0.29670597283903605, 0.0, 
        0.29670597283903605, 0.0, 
        0.29670597283903605, 0.0
    };
    // Alternative: {18.0/180.0*M_PI, 0.0, 18.0/180.0*M_PI, 0.0, 18.0/180.0*M_PI, 0.0, 18.0/180.0*M_PI, 0.0}
    
    // Kinematics helper for leg model access
    KinematicsHelper kinematics_;
};

CorgiWheelNode::CorgiWheelNode()
    : Node("corgi_wheel"),
      trigger_(false),
      velocity_(0.0),
      kinematics_(sim_)
{
    RCLCPP_INFO(this->get_logger(), "Corgi Wheel Starts");
    
    // Create publisher and subscriber
    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 1000, 
        std::bind(&CorgiWheelNode::trigger_cb, this, std::placeholders::_1));
    
    // Initialize motor command module pointers
    motor_cmd_modules_ = {
        &motor_cmd_.module_a,
        &motor_cmd_.module_b,
        &motor_cmd_.module_c,
        &motor_cmd_.module_d
    };
    
    // Initialize motor command
    initialize_motor_command();
}

void CorgiWheelNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger_ = msg->enable;
}

void CorgiWheelNode::initialize_motor_command() {
    for (auto& cmd : motor_cmd_modules_) {
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim_) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }
}

void CorgiWheelNode::execute_transform_phase() {
    RCLCPP_INFO(this->get_logger(), "Transform Starts");
    
    for (int i=0; i<3000; i++) {
        for (int j=0; j<4; j++) {
            motor_cmd_modules_[j]->theta += (init_eta_[2*j] - 17/180.0*M_PI) / 3000.0;
            motor_cmd_modules_[j]->beta += init_eta_[2*j+1] / 3000.0;
        }
        motor_cmd_.header.seq = -1;
        motor_cmd_pub_->publish(motor_cmd_);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    
    RCLCPP_INFO(this->get_logger(), "Transform Finished");
}

void CorgiWheelNode::execute_stay_phase() {
    for (int i=0; i<2000; i++) {
        motor_cmd_.header.seq = -1;
        motor_cmd_pub_->publish(motor_cmd_);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
}

void CorgiWheelNode::execute_control_phase() {
    RCLCPP_INFO(this->get_logger(), "Controller Starts ...");
    
    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (loop_count < 500) {
            velocity_ += 0.5 / 500.0;
        }
        else if (loop_count > target_loop_ - 500 && loop_count < target_loop_) {
            velocity_ -= 0.5 / 500.0;
        }

        double radius = kinematics_.get_leg_model().radius;
        motor_cmd_modules_[0]->beta += velocity_ / 1000.0 / radius / 2.0;
        motor_cmd_modules_[1]->beta -= velocity_ / 1000.0 / radius / 2.0;
        motor_cmd_modules_[2]->beta -= velocity_ / 1000.0 / radius / 2.0;
        motor_cmd_modules_[3]->beta += velocity_ / 1000.0 / radius / 2.0;

        motor_cmd_.header.seq = loop_count;
        motor_cmd_pub_->publish(motor_cmd_);

        loop_count++;
        if (loop_count >= target_loop_) break;

        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
}

void CorgiWheelNode::run() {
    execute_transform_phase();
    execute_stay_phase();
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (trigger_) {
            execute_control_phase();
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorgiWheelNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}