#include "fusion/FusionNode.hpp"
#include "fusion/FusionParamsIO.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace fusion {

// ----------------------------------------------------------------
FusionNode::FusionNode(const rclcpp::NodeOptions& opts)
    : Node("corgi_fusion_node", opts)
{
    // ── Load noise params from YAML (config/fusion/config_fusion.yaml) ─
    FusionConfig cfg;
    try {
        const std::string pkg =
            ament_index_cpp::get_package_share_directory("corgi_odometry");
        cfg = load_fusion_config(pkg + "/config/fusion/config_fusion.yaml");
        RCLCPP_INFO(this->get_logger(), "FusionNode: loaded config/fusion/config_fusion.yaml");
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
            "FusionNode: YAML load failed (%s), using defaults", e.what());
    }

    // ── ROS parameter overrides (optional — set in launch to override YAML) ─
    this->declare_parameter("q_p",        static_cast<double>(cfg.noise.q_p));
    this->declare_parameter("q_th",       static_cast<double>(cfg.noise.q_th));
    this->declare_parameter("q_bv",       static_cast<double>(cfg.noise.q_bv));
    this->declare_parameter("r_p",        static_cast<double>(cfg.noise.r_p));
    this->declare_parameter("r_th",       static_cast<double>(cfg.noise.r_th));
    this->declare_parameter("map_frame",  cfg.map_frame);
    this->declare_parameter("odom_frame", cfg.odom_frame);

    noise_params_.q_p  = static_cast<float>(this->get_parameter("q_p").as_double());
    noise_params_.q_th = static_cast<float>(this->get_parameter("q_th").as_double());
    noise_params_.q_bv = static_cast<float>(this->get_parameter("q_bv").as_double());
    noise_params_.r_p  = static_cast<float>(this->get_parameter("r_p").as_double());
    noise_params_.r_th = static_cast<float>(this->get_parameter("r_th").as_double());
    map_frame_  = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();

    ekf_.set_noise(noise_params_);

    // ── Subscribers ────────────────────────────────────────────
    ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        corgi::Config::TOPIC_EKF_ODOM, rclcpp::QoS(corgi::Config::QUEUE_SIZE_PUB),
        std::bind(&FusionNode::cb_ekf, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        corgi::Config::TOPIC_LIDAR_ODOM, rclcpp::QoS(corgi::Config::QUEUE_SIZE_PUB),
        std::bind(&FusionNode::cb_lidar, this, std::placeholders::_1));

    // ── Publishers ─────────────────────────────────────────────
    odom_mapping_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        corgi::Config::TOPIC_ODOM_MAPPING, rclcpp::QoS(corgi::Config::QUEUE_SIZE_PUB));

    bv_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        corgi::Config::TOPIC_FUSION_BV, rclcpp::QoS(corgi::Config::QUEUE_SIZE_PUB));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(),
        "FusionNode started. map_frame=%s, odom_frame=%s",
        map_frame_.c_str(), odom_frame_.c_str());
}

// ----------------------------------------------------------------
// cb_ekf: inner ESEKF at 500 Hz → ring buffer + EKF predict
// ----------------------------------------------------------------
void FusionNode::cb_ekf(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const rclcpp::Time stamp(msg->header.stamp);

    // Buffer this state
    BufferedState bs;
    bs.stamp   = stamp;
    bs.p_odom  = Eigen::Vector3f(
        static_cast<float>(msg->pose.pose.position.x),
        static_cast<float>(msg->pose.pose.position.y),
        static_cast<float>(msg->pose.pose.position.z));
    bs.q_odom  = Eigen::Quaternionf(
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z)).normalized();

    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
        state_buffer_.push_back(bs);
        while (state_buffer_.size() > BUFFER_MAX)
            state_buffer_.pop_front();
    }

    // Predict step
    if (!ekf_.initialized()) return;

    if (last_ekf_stamp_valid_) {
        const double dt = (stamp - last_ekf_stamp_).seconds();
        if (dt > 0.0 && dt < 1.0) {
            ekf_.predict(static_cast<float>(dt));
        }
    }
    last_ekf_stamp_       = stamp;
    last_ekf_stamp_valid_ = true;
}

// ----------------------------------------------------------------
// cb_lidar: LiDAR odometry → EKF update → publish
// ----------------------------------------------------------------
void FusionNode::cb_lidar(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const rclcpp::Time stamp(msg->header.stamp);

    const Eigen::Vector3f p_lidar(
        static_cast<float>(msg->pose.pose.position.x),
        static_cast<float>(msg->pose.pose.position.y),
        static_cast<float>(msg->pose.pose.position.z));
    const Eigen::Quaternionf q_lidar(
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z));

    // Find the buffered inner-ESEKF state nearest to this LiDAR stamp
    BufferedState bs;
    if (!find_buffered_state(stamp, bs)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "FusionNode: no buffered ESEKF state near t=%.3f", stamp.seconds());
        // Initialise EKF with LiDAR pose directly (odom=origin)
        if (!ekf_.initialized()) {
            ekf_.init(p_lidar, q_lidar);
            RCLCPP_INFO(this->get_logger(), "OuterEKF initialised from first LiDAR.");
        }
        return;
    }

    // Initialise on first valid LiDAR+buffer match
    if (!ekf_.initialized()) {
        // T_map^odom = p_lidar - R_lidar * p_odom  (approx for small rotation)
        Eigen::Vector3f p_mo = p_lidar - q_lidar.toRotationMatrix() * bs.p_odom;
        Eigen::Quaternionf q_mo = (q_lidar * bs.q_odom.inverse()).normalized();
        ekf_.init(p_mo, q_mo);
        RCLCPP_INFO(this->get_logger(),
            "OuterEKF initialised: p_mo=[%.3f %.3f %.3f]",
            p_mo.x(), p_mo.y(), p_mo.z());
        last_lidar_stamp_       = stamp;
        last_lidar_stamp_valid_ = true;
        return;
    }

    // dt since last LiDAR (for bv H-matrix coupling)
    float dt_lidar = DT_LIDAR_DEFAULT_S;  // default 10 Hz
    if (last_lidar_stamp_valid_) {
        double dt = (stamp - last_lidar_stamp_).seconds();
        if (dt > 0.0 && dt < DT_LIDAR_MAX_S)
            dt_lidar = static_cast<float>(dt);
    }

    OuterEKF::UpdateDiag diag;
    ekf_.update_lidar(p_lidar, q_lidar, bs.p_odom, bs.q_odom, dt_lidar, &diag);

    last_lidar_stamp_       = stamp;
    last_lidar_stamp_valid_ = true;

    // Use latest inner-ESEKF state for the published fused pose
    BufferedState latest;
    {
        std::lock_guard<std::mutex> lk(buffer_mutex_);
        if (state_buffer_.empty()) return;
        latest = state_buffer_.back();
    }
    const rclcpp::Time now_stamp = latest.stamp;

    // ── Publish /odom_mapping ───────────────────────────────────
    Eigen::Vector3f    p_map = ekf_.map_position(latest.p_odom);
    Eigen::Quaternionf q_map = ekf_.map_orientation(latest.q_odom);
    publish_odom_mapping(now_stamp, p_map, q_map);

    // ── Publish /fusion/bv ─────────────────────────────────────
    geometry_msgs::msg::Vector3Stamped bv_msg;
    bv_msg.header.stamp    = now_stamp;
    bv_msg.header.frame_id = map_frame_;
    bv_msg.vector.x = static_cast<double>(ekf_.bv().x());
    bv_msg.vector.y = static_cast<double>(ekf_.bv().y());
    bv_msg.vector.z = static_cast<double>(ekf_.bv().z());
    bv_pub_->publish(bv_msg);

    // ── Broadcast TF map → odom ────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = now_stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id  = odom_frame_;
    tf_msg.transform.translation.x = static_cast<double>(ekf_.p_mo().x());
    tf_msg.transform.translation.y = static_cast<double>(ekf_.p_mo().y());
    tf_msg.transform.translation.z = static_cast<double>(ekf_.p_mo().z());
    tf_msg.transform.rotation.w = static_cast<double>(ekf_.q_mo().w());
    tf_msg.transform.rotation.x = static_cast<double>(ekf_.q_mo().x());
    tf_msg.transform.rotation.y = static_cast<double>(ekf_.q_mo().y());
    tf_msg.transform.rotation.z = static_cast<double>(ekf_.q_mo().z());
    tf_broadcaster_->sendTransform(tf_msg);
}

// ----------------------------------------------------------------
// find_buffered_state
// ----------------------------------------------------------------
bool FusionNode::find_buffered_state(const rclcpp::Time& stamp,
                                      BufferedState& out) const {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    if (state_buffer_.empty()) return false;

    double best_dt = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < state_buffer_.size(); ++i) {
        double dt = std::abs((stamp - state_buffer_[i].stamp).seconds());
        if (dt < best_dt) {
            best_dt  = dt;
            best_idx = i;
        }
    }
    // Accept if within ACCEPT_WINDOW_S
    if (best_dt > ACCEPT_WINDOW_S) return false;
    out = state_buffer_[best_idx];
    return true;
}

// ----------------------------------------------------------------
// publish_odom_mapping
// ----------------------------------------------------------------
void FusionNode::publish_odom_mapping(const rclcpp::Time& stamp,
                                       const Eigen::Vector3f& p_map,
                                       const Eigen::Quaternionf& q_map) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = map_frame_;
    msg.child_frame_id  = corgi::Config::FRAME_BASE_LINK;
    msg.pose.pose.position.x    = static_cast<double>(p_map.x());
    msg.pose.pose.position.y    = static_cast<double>(p_map.y());
    msg.pose.pose.position.z    = static_cast<double>(p_map.z());
    msg.pose.pose.orientation.w = static_cast<double>(q_map.w());
    msg.pose.pose.orientation.x = static_cast<double>(q_map.x());
    msg.pose.pose.orientation.y = static_cast<double>(q_map.y());
    msg.pose.pose.orientation.z = static_cast<double>(q_map.z());
    // Covariance: diagonal from outer EKF P (first 3 pos + 3 rot)
    for (int i = 0; i < 3; ++i) {
        msg.pose.covariance[i*7]   = static_cast<double>(ekf_.covariance()(i, i));
        msg.pose.covariance[(i+3)*7] = static_cast<double>(ekf_.covariance()(i+3, i+3));
    }
    odom_mapping_pub_->publish(msg);
}

}  // namespace fusion
