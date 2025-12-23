#include "corgi_force_estimation/force_estimation.hpp"
#include "corgi_hybrid/hybrid_gen.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_wlw");

    RCLCPP_INFO(node->get_logger(), "Corgi WLW Starts");

    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    
    auto rate = std::chrono::milliseconds(1);  // 1000 Hz

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // initialize gait
    double CoM_bias = 0.0;

    auto gait_selector = std::make_shared<GaitSelector>(node, sim, CoM_bias, 1000);
    gait_selector->do_pub = 0;
    Hybrid hybrid_gait(gait_selector); 

    std::cout << "hybrid" << std::endl;
    double velocity = 0.1;  // 0.1, 0.15
    hybrid_gait.Initialize(1, 1);
    hybrid_gait.change_Velocity(velocity);
    // Hybrid API expects either change_Height(value, leg_index) or change_Height_all(value)
    hybrid_gait.change_Height_all(0.16);  // 0.16, 0.18
    hybrid_gait.change_Step_length(0.3);
    
    double init_eta[8];
    for (int i=0; i<4; i++) {
        init_eta[2*i] = gait_selector->eta[i][0];
        init_eta[2*i+1] = gait_selector->eta[i][1];
    }

    init_eta[3] *= -1;
    init_eta[5] *= -1;
    
    std::array<std::array<double, 4>, 2> eta_list = {{{17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI}, {0, 0, 0, 0}}};
    int target_loop = 20000;

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    // transform
    for (int i=0; i<3000; i++) {
        for (int j=0; j<4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/3000.0;
            motor_cmd_modules[j]->beta += init_eta[2*j+1]/3000.0;
        }
        motor_cmd.header.seq = -1;
        motor_cmd_pub->publish(motor_cmd);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // stay
    for (int i=0; i<2000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub->publish(motor_cmd);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(node->get_logger(), "Controller Starts ...");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // if (loop_count > target_loop-3000 && loop_count < target_loop) {
                //     velocity -= 0.1/3000.0;
                //     hybrid_gait.change_Velocity(velocity);
                // }

                // get next eta
                hybrid_gait.Step();

                for (int i=0; i<4; i++) {
                    eta_list[0][i] = gait_selector->eta[i][0];
                    eta_list[1][i] = gait_selector->eta[i][1];
                }


                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 0 || i == 3) ? eta_list[1][i] : -eta_list[1][i];
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub->publish(motor_cmd);

                loop_count++;
                if (loop_count >= target_loop) break;

                rclcpp::sleep_for(std::chrono::milliseconds(1));
            }
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    
    rclcpp::shutdown();
    return 0;
}