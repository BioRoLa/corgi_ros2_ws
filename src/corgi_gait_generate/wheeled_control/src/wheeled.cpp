#include "wheeled_control/wheeled.hpp"
// show the motor's state first and then compare with the wheel cmd to calculate the motor cmd's value

Wheeled::Wheeled()
    : Node("wheel_control")
{
    // Publishers
    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("/motor/command", 1000);

    // Subscribers
    wheel_cmd_sub_ = this->create_subscription<corgi_msgs::msg::WheelCmd>(
        "/wheel_cmd", 1000,
        std::bind(&Wheeled::wheelCmdCallback, this, std::placeholders::_1));
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "/motor/state", 1000,
        std::bind(&Wheeled::motorsStateCallback, this, std::placeholders::_1));
    steer_cmd_sub_ = this->create_subscription<corgi_msgs::msg::SteeringCmdStamped>(
        "/steer/command", 1000,
        std::bind(&Wheeled::steerStateCallback, this, std::placeholders::_1));
}

void Wheeled::wheelCmdCallback(const corgi_msgs::msg::WheelCmd::SharedPtr msg)
{
    // std::cout << "Received" << std::endl;
    current_wheel_cmd_ = *msg;
    if (current_steer_cmd_.angle == 0.0)
    {
        float beta_adjustment = (current_wheel_cmd_.velocity / 0.119) * (M_PI / 180.0); // Convert velocity to radians based on wheel radius
        if (current_wheel_cmd_.direction == false)
        {
            beta_adjustment = -beta_adjustment; // Reverse adjustment if direction is 0
        }
        if (current_wheel_cmd_.stop == true)
        {
            beta_adjustment = 0;
        }
        current_motor_cmd_.header.stamp = this->now();
        for (int i = 0; i < 4; ++i)
        {
            motor_cmds[i]->theta = 17 * (M_PI / 180.0);
            if (i == 1 || i == 2)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta - beta_adjustment;
            }
            else if (i == 0 || i == 3)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta + beta_adjustment;
            }
            motor_cmds[i]->kp_r = 90;
            motor_cmds[i]->ki_r = 0;
            motor_cmds[i]->kd_r = 1.75;
            motor_cmds[i]->kp_l = 90;
            motor_cmds[i]->ki_l = 0;
            motor_cmds[i]->kd_l = 1.75;
        }
    }
    else if ((current_steer_cmd_.angle > 0.0))
    {
        // right turn
        float beta_adjustment_l = (current_wheel_cmd_.velocity * 1.5 / 0.119) * (M_PI / 180.0);
        float beta_adjustment_r = (current_wheel_cmd_.velocity * 0.5 / 0.119) * (M_PI / 180.0);
        if (current_wheel_cmd_.stop == true)
        {
            beta_adjustment_l = 0;
            beta_adjustment_r = 0;
        }
        if (current_wheel_cmd_.direction == false)
        {
            beta_adjustment_l = -beta_adjustment_l; // Reverse adjustment if direction is 0
            beta_adjustment_r = -beta_adjustment_r; // Reverse adjustment if direction is 0
        }
        current_motor_cmd_.header.stamp = this->now();
        for (size_t i = 0; i < 4; ++i)
        {
            motor_cmds[i]->theta = 17 * (M_PI / 180.0);
            if (i == 1 || i == 2)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta - beta_adjustment_r;
            }
            else if (i == 0 || i == 3)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta + beta_adjustment_l;
            }
            motor_cmds[i]->kp_r = 90;
            motor_cmds[i]->ki_r = 0;
            motor_cmds[i]->kd_r = 1.75;
            motor_cmds[i]->kp_l = 90;
            motor_cmds[i]->ki_l = 0;
            motor_cmds[i]->kd_l = 1.75;
        }
    }
    else
    {
        // left turn
        float beta_adjustment_l = (current_wheel_cmd_.velocity * 0.5 / 0.119) * (M_PI / 180.0);
        float beta_adjustment_r = (current_wheel_cmd_.velocity * 1.5 / 0.119) * (M_PI / 180.0);
        if (current_wheel_cmd_.stop == true)
        {
            beta_adjustment_l = 0;
            beta_adjustment_r = 0;
        }
        if (current_wheel_cmd_.direction == false)
        {
            beta_adjustment_l = -beta_adjustment_l; // Reverse adjustment if direction is 0
            beta_adjustment_r = -beta_adjustment_r; // Reverse adjustment if direction is 0
        }
        current_motor_cmd_.header.stamp = this->now();
        for (size_t i = 0; i < 4; ++i)
        {
            motor_cmds[i]->theta = 17 * (M_PI / 180.0);
            if (i == 1 || i == 2)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta - beta_adjustment_r;
            }
            else if (i == 0 || i == 3)
            {
                motor_cmds[i]->beta = motor_state_modules[i]->beta + beta_adjustment_l;
            }
            motor_cmds[i]->kp_r = 90;
            motor_cmds[i]->ki_r = 0;
            motor_cmds[i]->kd_r = 1.75;
            motor_cmds[i]->kp_l = 90;
            motor_cmds[i]->ki_l = 0;
            motor_cmds[i]->kd_l = 1.75;
        }
    }

    motor_cmd_pub_->publish(current_motor_cmd_);
}

void Wheeled::motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg)
{
    current_motor_state_ = *msg;
}

void Wheeled::steerStateCallback(const corgi_msgs::msg::SteeringCmdStamped::SharedPtr msg)
{
    current_steer_cmd_ = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto joystick_node = std::make_shared<JoystickControl>();
    auto wheel_node = std::make_shared<Wheeled>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(joystick_node);
    executor.add_node(wheel_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
