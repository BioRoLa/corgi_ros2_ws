#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "corgi_transform/transform_gen.hpp"
#include "corgi_hybrid/hybrid_gen.hpp"
#include "corgi_wheeled/wheeled_gen.hpp"
#include "corgi_legged/legged_gen.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_transform_test");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&exec]()
                            { exec.spin(); });

    /*     Gait Selector Setting    */
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    auto gaitSelector = std::make_shared<GaitSelector>(node, sim, CoM_bias, pub_rate);
    RCLCPP_INFO(node->get_logger(), "gaitSelector initialized");

    /*    Initialize of each mode   */
    auto hybrid = std::make_shared<Hybrid>(gaitSelector);
    auto legged = std::make_shared<Legged>(gaitSelector);
    Transform transformer(hybrid, legged);
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            gaitSelector->next_eta[j][0] = 17 * PI / 180;
            if (j == 0)
            {
                gaitSelector->next_eta[j][1] = 100 * PI / 180;
            }
            else if (j == 1)
            {
                gaitSelector->next_eta[j][1] = 100 * PI / 180;
            }
            else
            {
                gaitSelector->next_eta[j][1] = 50 * PI / 180;
            }
        }
        gaitSelector->Transfer(gaitSelector->transfer_sec, gaitSelector->wait_sec);
    }
    gaitSelector->Send();
    while (rclcpp::ok())
    {
        if (static_cast<int>(gaitSelector->trigger_msg.enable))
        {
            transformer.GaitTransform(Gait::WHEELED, Gait::HYBRID);
            transformer.GaitTransform(Gait::HYBRID, Gait::LEGGED);
            break;
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for trigger...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_INFO(node->get_logger(), "End of testing!");
    rclcpp::shutdown();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    return 0;
}
