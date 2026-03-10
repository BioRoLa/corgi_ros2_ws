#include "corgi_force_estimation/force_estimation.hpp"
#include "corgi_walk/walk_gait.hpp"

#include <chrono>

class CorgiWalkNode : public rclcpp::Node {
public:
    CorgiWalkNode();

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

    // State
    bool trigger_;

    // Gait parameters
    double velocity_;
    double stand_height_;
    double step_length_;
    double step_height_;

    // Parameters
    const bool sim_ = true;
    const int target_loop_ = 20000;

    // sim, h25, sl0.3
    double init_eta_[8] = {1.9107879909396832, 0.4678492649476779,
                           1.6644526642960358, 0.1256503306098462,
                           1.6644526642960358, -0.1256503306098462,
                           1.9107879909396832, -0.4678492649476779};

    // sim, h20, sl0.3
    // double init_eta_[8] = {1.5145026111157143, 0.573900181729176,
    //                        1.1975094246645916, 0.1586552621864014,
    //                        1.1975094246645916, -0.1586552621864014,
    //                        1.5145026111157143, -0.573900181729176};

    // real, h25, sl0.3
    // double init_eta_[8] = {1.857467698281913, 0.4791102940603916,
    //                        1.6046663223045279, 0.12914729012802004,
    //                        1.6046663223045279, -0.12914729012802004,
    //                        1.857467698281913, -0.4791102940603916};

    // real, h20, sl0.3
    // double init_eta_[8] = {1.4863321792421085, 0.6075431293162905,
    //                        1.1354779956465793, 0.16425262030677687,
    //                        1.1354779956465793, -0.16425262030677687,
    //                        1.4863321792421085, -0.6075431293162905};

    WalkGait walk_gait_;
};

CorgiWalkNode::CorgiWalkNode()
    : Node("corgi_walk"),
      trigger_(false),
      velocity_(0.1),
      stand_height_(0.25),
      step_length_(0.3),
      step_height_(0.04),
      walk_gait_(sim_, 0, 1000)
{
    RCLCPP_INFO(this->get_logger(), "Corgi Walk Starts");

    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10,
        std::bind(&CorgiWalkNode::trigger_cb, this, std::placeholders::_1));

    motor_cmd_modules_ = {
        &motor_cmd_.module_a,
        &motor_cmd_.module_b,
        &motor_cmd_.module_c,
        &motor_cmd_.module_d
    };

    initialize_motor_command();

    walk_gait_.initialize(init_eta_);
    walk_gait_.set_velocity(velocity_);
    walk_gait_.set_stand_height(stand_height_);
    walk_gait_.set_step_length(step_length_);
    walk_gait_.set_step_height(step_height_);

    // Wait for simulation clock synchronization
    RCLCPP_INFO(this->get_logger(), "Waiting for clock...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (this->now().seconds() > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Clock synced! Time: %.2f", this->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void CorgiWalkNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger_ = msg->enable;
}

void CorgiWalkNode::initialize_motor_command() {
    for (auto& cmd : motor_cmd_modules_) {
        cmd->theta = 17/180.0*M_PI;
        cmd->beta  = 0/180.0*M_PI;
        cmd->kp_r  = 90;
        cmd->kp_l  = 90;
        cmd->ki_r  = 0;
        cmd->ki_l  = 0;
        if (sim_) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        } else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }
}

void CorgiWalkNode::execute_transform_phase() {
    RCLCPP_INFO(this->get_logger(), "Transform Starts");

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int i = 0; i < 3000; i++) {
        rclcpp::spin_some(this->get_node_base_interface());

        for (int j = 0; j < 4; j++) {
            motor_cmd_modules_[j]->theta += (init_eta_[2*j] - 17/180.0*M_PI) / 3000.0;
            motor_cmd_modules_[j]->beta  += init_eta_[2*j+1] / 3000.0;
        }

        motor_cmd_.header.stamp = this->now();
        motor_cmd_.header.seq = -1;
        motor_cmd_pub_->publish(motor_cmd_);

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            break;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Transform Finished");
}

void CorgiWalkNode::execute_stay_phase() {
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int i = 0; i < 2000; i++) {
        rclcpp::spin_some(this->get_node_base_interface());

        motor_cmd_.header.stamp = this->now();
        motor_cmd_.header.seq = -1;
        motor_cmd_pub_->publish(motor_cmd_);

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            break;
        }
    }
}

void CorgiWalkNode::execute_control_phase() {
    RCLCPP_INFO(this->get_logger(), "Controller Starts ...");

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    std::array<std::array<double, 4>, 2> eta_list;
    int loop_count = 0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        // if (loop_count > target_loop_ - 3000 && loop_count < target_loop_) {
        //     velocity_ -= 0.1 / 3000.0;
        //     walk_gait_.set_velocity(velocity_);
        // }

        // get next eta
        eta_list = walk_gait_.step();

        for (int i = 0; i < 4; i++) {
            if (eta_list[0][i] > M_PI * 160.0 / 180.0) {
                RCLCPP_WARN(this->get_logger(), "Module %d: Exceed upper bound.", i);
            }
            if (eta_list[0][i] < M_PI * 17.0 / 180.0) {
                RCLCPP_WARN(this->get_logger(), "Module %d: Exceed lower bound.", i);
            }
            motor_cmd_modules_[i]->theta = eta_list[0][i];
            motor_cmd_modules_[i]->beta  = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
        }

        motor_cmd_.header.stamp = this->now();
        motor_cmd_.header.seq = loop_count;
        motor_cmd_pub_->publish(motor_cmd_);

        loop_count++;
        if (loop_count >= target_loop_) break;

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            break;
        }
    }
}

void CorgiWalkNode::run() {
    execute_transform_phase();
    execute_stay_phase();

    // Wait for trigger
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (trigger_) {
            execute_control_phase();
            break;
        }

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            break;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorgiWalkNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}