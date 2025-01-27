#include "fsm_control.hpp"


int main(int argc, char **argv) {

    ROS_INFO("FSM Command Publisher Starts\n");
    
    ros::init(argc, argv, "fsm_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher fsm_cmd_pub = nh.advertise<corgi_msgs::FsmCmdStamped>("fsm/command", 1000);
    ros::Rate rate(1000);

    corgi_msgs::FsmCmdStamped fsm_cmd;

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        fsm_cmd.next_mode = REST_MODE;
        fsm_cmd.pause = false;
        fsm_cmd.stop = false;


        fsm_cmd.header.seq = loop_count;

        fsm_cmd_pub.publish(fsm_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}