#include "corgi_wheeled/wheeled_gen.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_wheeled_test");
    RCLCPP_INFO(node->get_logger(), "Wheeled mode test");

    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    auto gait_selector = std::make_shared<GaitSelector>(node, sim, CoM_bias, pub_rate);
    auto wheeled = std::make_shared<Wheeled>(node, "joystick");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
