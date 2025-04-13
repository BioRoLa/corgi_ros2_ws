#ifndef WHEELED_GEN_HPP
#define WHEELED_GEN_HPP


#include "Simple_fsm.hpp"
#include "wheeled_cmd.hpp"
#include <corgi_msgs/WheelCmd.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/SteeringStateStamped.h>

class Wheeled : public GaitSelector, public WheeledCmd
{
public:
    Wheeled(ros::NodeHandle& nh);
    ~Wheeled() = default;
    void Roll(int pub_time, int do_pub, bool dir, bool ground_rotate, int voltage, float angle);
private:
    void wheelCmdCallback(const corgi_msgs::WheelCmd::ConstPtr& msg);
    void steerCmdCallback(const corgi_msgs::SteeringCmdStamped::ConstPtr& msg);

    ros::Subscriber wheel_cmd_sub_;
    ros::Subscriber steer_cmd_sub_;

    corgi_msgs::WheelCmd current_wheel_cmd_;
    corgi_msgs::SteeringCmdStamped current_steer_cmd_;
    
    float beta_adjustment;
    float beta_adjustment_r;
    float beta_adjustment_l;

    void MoveLinear(bool dir);
    void MoveAngular(bool dir);

};

#endif
