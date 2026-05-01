/// fake_lidar_odom.cpp
/// Subscribes to TF (odom → base_link), adds Gaussian noise + simulated
/// latency, and publishes nav_msgs/Odometry to /Odometry to mock Fast-LIO.
///
/// ROS 2 parameters:
///   use_sim_time  : bool   (default false)
///   publish_rate  : double [Hz]  (default 10.0)
///   sigma_p       : double [m]   (default 0.02)
///   sigma_q       : double [rad] (default 0.005)
///   latency_ms    : double [ms]  (default 80.0)
///   parent_frame  : string (default "odom")
///   child_frame   : string (default "base_link")
///   output_topic  : string (default "/Odometry")
///   gt_pos_topic  : string (default "")
///     When non-empty, subscribe to this geometry_msgs/Vector3 topic for
///     ground-truth position instead of reading from TF.  Orientation is
///     still taken from TF (ESEKF orientation error is negligible for
///     short straight-line walks).  Use "/sim/position" in simulation to
///     break the circular dependency where TF = ESEKF estimate.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>
#include <random>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// ----------------------------------------------------------------
struct StampedPose {
    rclcpp::Time stamp;
    Eigen::Vector3f    position;
    Eigen::Quaternionf orientation;
};

// ----------------------------------------------------------------
class FakeLidarOdomNode : public rclcpp::Node {
public:
    FakeLidarOdomNode()
        : Node("fake_lidar_odom"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          rng_(std::random_device{}()),
          dist_(0.0, 1.0)
    {
        this->declare_parameter("publish_rate",  10.0);
        this->declare_parameter("sigma_p",       0.02);
        this->declare_parameter("sigma_q",       0.005);
        this->declare_parameter("latency_ms",    80.0);
        this->declare_parameter("parent_frame",  std::string("odom"));
        this->declare_parameter("child_frame",   std::string("base_link"));
        this->declare_parameter("output_topic",  std::string("/Odometry"));
        this->declare_parameter("gt_pos_topic",  std::string(""));

        publish_rate_  = this->get_parameter("publish_rate").as_double();
        sigma_p_       = static_cast<float>(this->get_parameter("sigma_p").as_double());
        sigma_q_       = static_cast<float>(this->get_parameter("sigma_q").as_double());
        latency_ms_    = this->get_parameter("latency_ms").as_double();
        parent_frame_  = this->get_parameter("parent_frame").as_string();
        child_frame_   = this->get_parameter("child_frame").as_string();
        output_topic_  = this->get_parameter("output_topic").as_string();
        gt_pos_topic_  = this->get_parameter("gt_pos_topic").as_string();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            output_topic_, rclcpp::QoS(10));

        // Subscribe to GT position topic if requested
        if (!gt_pos_topic_.empty()) {
            sim_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                gt_pos_topic_, rclcpp::QoS(100),
                std::bind(&FakeLidarOdomNode::cb_sim_pos, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                "GT position source: topic '%s' (breaks ESEKF circular dependency)",
                gt_pos_topic_.c_str());
        }

        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&FakeLidarOdomNode::timer_cb, this));

        RCLCPP_INFO(this->get_logger(),
            "FakeLidarOdom started: rate=%.1f Hz, sigma_p=%.4f m, "
            "sigma_q=%.4f rad, latency=%.1f ms, topic=%s",
            publish_rate_, sigma_p_, sigma_q_, latency_ms_,
            output_topic_.c_str());
    }

private:
    // ── GT position subscriber (geometry_msgs/Vector3) ───────
    void cb_sim_pos(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        const rclcpp::Time now = this->now();

        // Detect time jump backward (e.g. bag replay restarting with sim_time)
        if (!sim_pos_buffer_.empty() &&
            (sim_pos_buffer_.back().stamp - now).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "FakeLidar: sim_pos time jumped back — clearing buffer");
            sim_pos_buffer_.clear();
        }

        StampedPose p;
        p.stamp     = now;
        p.position  = Eigen::Vector3f(
            static_cast<float>(msg->x),
            static_cast<float>(msg->y),
            static_cast<float>(msg->z));
        p.orientation = Eigen::Quaternionf::Identity(); // filled from TF in timer_cb
        sim_pos_buffer_.push_back(p);
        // Trim entries older than 2 s
        while (!sim_pos_buffer_.empty() &&
               (now - sim_pos_buffer_.front().stamp).seconds() > 2.0)
            sim_pos_buffer_.pop_front();
    }

    void timer_cb() {
        StampedPose delayed;
        bool found = false;

        if (!gt_pos_topic_.empty()) {
            // ── GT-position path: use /sim/position buffer ──────
            if (sim_pos_buffer_.empty()) return;
            const double latency_s = latency_ms_ * 1e-3;
            const rclcpp::Time now = this->now();

            // Clear if time jumped back
            if (!sim_pos_buffer_.empty() &&
                (sim_pos_buffer_.back().stamp - now).seconds() > 0.5) {
                sim_pos_buffer_.clear();
                return;
            }
            double best_diff = std::numeric_limits<double>::max();
            for (const auto& p : sim_pos_buffer_) {
                double age  = (now - p.stamp).seconds();
                double diff = std::abs(age - latency_s);
                if (diff < best_diff) { best_diff = diff; delayed = p; found = true; }
            }
            if (!found) return;
            // Accept if within half-period tolerance
            if (best_diff > 1.0 / publish_rate_) return;

            // Get orientation from TF (ESEKF; orientation error << position error)
            geometry_msgs::msg::TransformStamped tf_msg;
            try {
                tf_msg = tf_buffer_.lookupTransform(
                    parent_frame_, child_frame_, tf2::TimePointZero);
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "FakeLidar: TF lookup failed: %s", e.what());
                return;
            }
            delayed.orientation = Eigen::Quaternionf(
                static_cast<float>(tf_msg.transform.rotation.w),
                static_cast<float>(tf_msg.transform.rotation.x),
                static_cast<float>(tf_msg.transform.rotation.y),
                static_cast<float>(tf_msg.transform.rotation.z)).normalized();
        } else {
            // ── Default TF path ──────────────────────────────────
            geometry_msgs::msg::TransformStamped tf_msg;
            try {
                tf_msg = tf_buffer_.lookupTransform(
                    parent_frame_, child_frame_, tf2::TimePointZero);
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "FakeLidar: TF lookup failed: %s", e.what());
                return;
            }

            StampedPose pose;
            pose.stamp = rclcpp::Time(tf_msg.header.stamp);
            pose.position = Eigen::Vector3f(
                static_cast<float>(tf_msg.transform.translation.x),
                static_cast<float>(tf_msg.transform.translation.y),
                static_cast<float>(tf_msg.transform.translation.z));
            pose.orientation = Eigen::Quaternionf(
                static_cast<float>(tf_msg.transform.rotation.w),
                static_cast<float>(tf_msg.transform.rotation.x),
                static_cast<float>(tf_msg.transform.rotation.y),
                static_cast<float>(tf_msg.transform.rotation.z)).normalized();

            pose_buffer_.push_back(pose);
            const rclcpp::Time now = this->now();

            // Clear if time jumped back
            if (!pose_buffer_.empty() &&
                (pose_buffer_.back().stamp - now).seconds() > 0.5) {
                pose_buffer_.clear();
                return;
            }

            while (!pose_buffer_.empty()) {
                double age = (now - pose_buffer_.front().stamp).seconds();
                if (age > 2.0) pose_buffer_.pop_front(); else break;
            }

            const double latency_s = latency_ms_ * 1e-3;
            double best_diff = std::numeric_limits<double>::max();
            for (const auto& p : pose_buffer_) {
                double age  = (now - p.stamp).seconds();
                double diff = std::abs(age - latency_s);
                if (diff < best_diff) { best_diff = diff; delayed = p; found = true; }
            }
            if (!found) return;
            if (best_diff > 1.0 / publish_rate_) return;
        }

        // Add Gaussian noise to position
        Eigen::Vector3f p_noisy = delayed.position + Eigen::Vector3f(
            sigma_p_ * static_cast<float>(dist_(rng_)),
            sigma_p_ * static_cast<float>(dist_(rng_)),
            sigma_p_ * static_cast<float>(dist_(rng_)));

        // Add small angle noise to orientation (axis-angle perturbation)
        Eigen::Vector3f noise_axis = Eigen::Vector3f(
            static_cast<float>(dist_(rng_)),
            static_cast<float>(dist_(rng_)),
            static_cast<float>(dist_(rng_))).normalized();
        float noise_angle = sigma_q_ * static_cast<float>(dist_(rng_));
        Eigen::Quaternionf dq(Eigen::AngleAxisf(noise_angle, noise_axis));
        Eigen::Quaternionf q_noisy = (delayed.orientation * dq).normalized();

        // Publish
        nav_msgs::msg::Odometry msg;
        // Stamp the message at the delayed time (to allow ring-buffer matching)
        msg.header.stamp    = delayed.stamp;
        msg.header.frame_id = parent_frame_;  // "odom" = map equivalent in sim
        msg.child_frame_id  = child_frame_;
        msg.pose.pose.position.x    = static_cast<double>(p_noisy.x());
        msg.pose.pose.position.y    = static_cast<double>(p_noisy.y());
        msg.pose.pose.position.z    = static_cast<double>(p_noisy.z());
        msg.pose.pose.orientation.w = static_cast<double>(q_noisy.w());
        msg.pose.pose.orientation.x = static_cast<double>(q_noisy.x());
        msg.pose.pose.orientation.y = static_cast<double>(q_noisy.y());
        msg.pose.pose.orientation.z = static_cast<double>(q_noisy.z());

        // Diagonal covariance = sigma_p^2 for positions, sigma_q^2 for rotations
        const double var_p = static_cast<double>(sigma_p_ * sigma_p_);
        const double var_q = static_cast<double>(sigma_q_ * sigma_q_);
        msg.pose.covariance[0]  = var_p;
        msg.pose.covariance[7]  = var_p;
        msg.pose.covariance[14] = var_p;
        msg.pose.covariance[21] = var_q;
        msg.pose.covariance[28] = var_q;
        msg.pose.covariance[35] = var_q;

        odom_pub_->publish(msg);
    }

    // ── Members ──────────────────────────────────────────────
    tf2_ros::Buffer          tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sim_pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<StampedPose> pose_buffer_;     // TF-based buffer
    std::deque<StampedPose> sim_pos_buffer_;  // GT-position buffer

    // Parameters
    double      publish_rate_;
    float       sigma_p_;
    float       sigma_q_;
    double      latency_ms_;
    std::string parent_frame_;
    std::string child_frame_;
    std::string output_topic_;
    std::string gt_pos_topic_;

    // RNG
    std::mt19937                     rng_;
    std::normal_distribution<double> dist_;
};

// ----------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLidarOdomNode>());
    rclcpp::shutdown();
    return 0;
}
