#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include "Simple_fsm.hpp"

namespace
{
    Gait parse_gait(const std::string &name)
    {
        if (name == "wheeled")
            return Gait::WHEELED;
        if (name == "legged")
            return Gait::LEGGED;
        if (name == "hybrid")
            return Gait::HYBRID;
        if (name == "transform")
            return Gait::TRANSFORM;
        return Gait::WHEELED; // default fallback
    }

    std::string gait_to_string(Gait gait)
    {
        switch (gait)
        {
        case Gait::WHEELED:
            return "wheeled";
        case Gait::LEGGED:
            return "legged";
        case Gait::HYBRID:
            return "hybrid";
        case Gait::TRANSFORM:
            return "transform";
        default:
            return "unknown";
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_algo");

    // Parameters with sane defaults
    std::string default_gait = "wheeled";
    bool sim = true;
    double com_bias = 0.0;
    int pub_rate = 1000;
    double BL = 0.444;
    double BW = 0.4;
    double BH = 0.2;

    node->declare_parameter("default_gait", default_gait);
    node->declare_parameter("sim", sim);
    node->declare_parameter("com_bias", com_bias);
    node->declare_parameter("pub_rate", pub_rate);
    node->declare_parameter("body_length", BL);
    node->declare_parameter("body_width", BW);
    node->declare_parameter("body_height", BH);

    node->get_parameter("default_gait", default_gait);
    node->get_parameter("sim", sim);
    node->get_parameter("com_bias", com_bias);
    node->get_parameter("pub_rate", pub_rate);
    node->get_parameter("body_length", BL);
    node->get_parameter("body_width", BW);
    node->get_parameter("body_height", BH);

    const Gait gait = parse_gait(default_gait);

    // Construct gait selector (handles pub/sub internally)
    GaitSelector gait_selector(node, sim, com_bias, pub_rate, BL, BW, BH);
    gait_selector.currentGait = gait;
    gait_selector.newGait = gait;

    RCLCPP_INFO(node->get_logger(),
                "corgi_algo initialized: gait=%s sim=%s pub_rate=%d BL=%.3f BW=%.3f BH=%.3f",
                gait_to_string(gait).c_str(), sim ? "true" : "false", pub_rate, BL, BW, BH);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
