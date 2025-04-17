#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

// Odometry callback
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  const auto& p = msg->pose.pose.position;
  ROS_INFO("Odom → x: %.3f, y: %.3f, z: %.3f", p.x, p.y, p.z);
}

// RGB 訂閱 callback
void rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("ZED RGB", img);
    cv::waitKey(1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge RGB exception: %s", e.what());
  }
}

// Depth 訂閱 callback
void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // 假設 depth 圖是 32FC1
    cv::Mat depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::imshow("ZED Depth", depth);
    cv::waitKey(1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge Depth exception: %s", e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "zed_listener");
  ros::NodeHandle nh;

  // 訂閱 odometry
  ros::Subscriber odom_sub =
    nh.subscribe("/zedx/odom", 10, odomCallback);

  // 影像訂閱
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub =
    it.subscribe("/zedx/left/image_rect_color", 1, rgbCallback);
  image_transport::Subscriber depth_sub =
    it.subscribe("/zedx/depth/depth_registered", 1, depthCallback);

  ROS_INFO("ZED listener started…");
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}
