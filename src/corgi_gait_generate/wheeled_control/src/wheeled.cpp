#include "wheeled.hpp"

Wheeled::Wheeled()
{
    // Publishers
    motor_cmd_pub_ = wnh_.advertise<corgi_msgs::MotorCmdStamped>("/motor_cmd", 1);

    // Subscribers
    wheel_cmd_sub_ = wnh_.subscribe("wheel_cmd", 1, &Wheeled::wheelCmdCallback, this);
    motor_state_sub_ = wnh_.subscribe("/motor/state", 1, &Wheeled::motorsStateCallback, this);
}

void Wheeled::wheelCmdCallback(const corgi_msgs::WheelCmd::ConstPtr& msg)
{
    std::cout << "Received wheel cmd: " << std::endl;
    // do something useful with msg here
}

void Wheeled::motorsStateCallback(const corgi_msgs::MotorStateStamped::ConstPtr& msg)
{
    std::cout << "Received motor state: " << std::endl;
    // show the motor's state first and then compare with the wheel cmd to calculate the motor cmd's value
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_control");
    Wheeled wheel_node;
    ros::spin();
    return 0;
}
