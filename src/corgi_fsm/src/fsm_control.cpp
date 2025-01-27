#include "fsm_control.hpp"

#include "walk_gait.hpp"


corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::FsmCmdStamped fsm_cmd;
corgi_msgs::FsmStateStamped fsm_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void fsm_cmd_cb(const corgi_msgs::FsmCmdStamped cmd){
    fsm_cmd = cmd;
}


int main(int argc, char **argv) {

    ROS_INFO("FSM Starts\n");

    ros::init(argc, argv, "corgi_fsm");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber fsm_cmd_sub = nh.subscribe<corgi_msgs::FsmCmdStamped>("fsm/command", 1000, fsm_cmd_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Publisher fsm_state_pub = nh.advertise<corgi_msgs::FsmStateStamped>("fsm/state", 1000);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };


    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();






        

        std::cout << "= = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();

        fsm_state.header.seq = loop_count;
        fsm_state.header.stamp = ros::Time::now();

        motor_cmd_pub.publish(motor_cmd);
        fsm_state_pub.publish(fsm_state);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}