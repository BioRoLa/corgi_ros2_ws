#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ZedOdomSubscriber : public rclcpp::Node
{
public:
    ZedOdomSubscriber() : Node("zed_odom_listener")
    {

        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/zed/zed_node/odom",
            qos_profile,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                // 1. HARDWARE TIMESTAMP
                // The ZED wrapper automatically populates this with the hardware capture time.
                auto timestamp_sec = msg->header.stamp.sec;
                auto timestamp_nanosec = msg->header.stamp.nanosec;

                // 2. POSITION (x, y, z)
                double x = msg->pose.pose.position.x;
                double y = msg->pose.pose.position.y;
                double z = msg->pose.pose.position.z;

                // 3. QUATERNION (x, y, z, w)
                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;

                // 4. TWIST / VELOCITY
                double linear_vx = msg->twist.twist.linear.x;
                double angular_vz = msg->twist.twist.angular.z;

                RCLCPP_INFO(this->get_logger(), "Got ZED Pose -> X: %.2f, Y: %.2f", x, y);
            });
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedOdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}