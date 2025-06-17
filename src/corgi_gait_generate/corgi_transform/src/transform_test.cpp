#include "transform_gen.hpp"
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Transform mode test\n");
    ros::init(argc, argv, "corgi_transform_test");
    ros::NodeHandle nh;
    
    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    // GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    auto gaitSelector = std::make_shared<GaitSelector>(nh, sim, CoM_bias, pub_rate);
    std::cout << "gaitSelector initialized" << std::endl;

    /*    Initialize of each mode   */ 
    auto hybrid = std::make_shared<Hybrid>(gaitSelector); 
    Transform transformer(hybrid);
    // transformer.GaitTransform(Gait::WHEELED, Gait::HYBRID);
    transformer.GaitTransform(Gait::HYBRID, Gait::LEGGED);
    
    std::cout << "End of testing!" << std::endl;
    return 0;
}
