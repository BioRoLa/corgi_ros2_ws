#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "hybrid_gen.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stable_triangle_node");
    // ros::NodeHandle nh;

    // bool sim = true;
    // double CoM_bias = 0.0;
    // int pub_rate = 1000;
    // LegModel leg_model(sim);

    // GaitSelector gs(nh, sim, CoM_bias, pub_rate);

    // ros::Rate loop_rate(pub_rate);  
    // while (ros::ok()) {
    //     gs.process_and_visualize();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}
