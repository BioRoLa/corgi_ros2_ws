#include <array>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/sim_leg_contact_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "corgi_foothold_estimator/foothold_detector.hpp"

class FootholdDetectionNode : public rclcpp::Node
{
public:
  FootholdDetectionNode()
  : Node("foothold_detection_node"),
    is_active_(false)
  {
    swing_phase_.fill(0);
    prev_swing_phase_.fill(0);
    leg_contact_.fill(false);

    const double delay_threshold = this->declare_parameter<double>("delay_threshold", 0.005);

    for (int i = 0; i < 4; ++i) {
      detector_[i] = std::make_unique<FootholdDetector>(delay_threshold);
    }

    // Seed per-leg landing timers; will be re-seeded after clock sync.
    last_swing_end_time_.fill(this->now());

    // --- Subscribers ---
    swing_phase_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/walk/swing_phase", 10,
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) { return; }
        const rclcpp::Time now = this->now();
        for (std::size_t i = 0; i < 4; ++i) {
          const int32_t current = msg->data[i];
          // Set active once any leg first enters swing (trigger has fired).
          if (current == 1) { is_active_ = true; }
          // Rising edge: stance (0) → swing (1): arm kinematic detection for this leg.
          if (prev_swing_phase_[i] == 0 && current == 1) {
            detector_[i]->notify_swing_started();
          }
          // Falling edge: swing (1) → stance (0): record landing time.
          if (prev_swing_phase_[i] == 1 && current == 0) {
            last_swing_end_time_[i] = now;
          }
          prev_swing_phase_[i] = swing_phase_[i];
          swing_phase_[i]      = current;
        }
      });

    contact_sub_ = this->create_subscription<corgi_msgs::msg::SimLegContactStamped>(
      "/sim/leg_contact", 10,
      [this](const corgi_msgs::msg::SimLegContactStamped::SharedPtr msg) {
        leg_contact_[0] = msg->module_a.contact;
        leg_contact_[1] = msg->module_b.contact;
        leg_contact_[2] = msg->module_c.contact;
        leg_contact_[3] = msg->module_d.contact;
      });

    // --- Publisher ---
    flags_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(
      "/foothold_detection_flags", 10);

    // 500 Hz timer (2 ms period).
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(2000),
      std::bind(&FootholdDetectionNode::timer_callback, this));
  }

  void reseed_timestamps()
  {
    last_swing_end_time_.fill(this->now());
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Int8MultiArray flags_msg;
    flags_msg.data.resize(4, 0);

    // Hold all flags at 0 until walking starts.
    if (!is_active_) {
      flags_pub_->publish(flags_msg);
      return;
    }

    const rclcpp::Time now = this->now();

    for (int i = 0; i < 4; ++i) {
      double tse = (now - last_swing_end_time_[i]).seconds();
      // While the leg is still in swing the foot has not yet landed.
      if (swing_phase_[i] == 1) { tse = 0.0; }

      const bool contact = leg_contact_[i];

      // data[0..3] — kinematic flag
      flags_msg.data[i] = static_cast<int8_t>(
        detector_[i]->evaluate_kinematic(tse, contact));
    }

    flags_pub_->publish(flags_msg);
  }

  // --- Subscriptions & publisher ---
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr          swing_phase_sub_;
  rclcpp::Subscription<corgi_msgs::msg::SimLegContactStamped>::SharedPtr   contact_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr              flags_pub_;
  rclcpp::TimerBase::SharedPtr                                             timer_;

  // --- Per-leg state (index: 0=A/LF, 1=B/RF, 2=C/RH, 3=D/LH) ---
  std::array<int32_t, 4>      swing_phase_;
  std::array<int32_t, 4>      prev_swing_phase_;
  std::array<bool, 4>         leg_contact_;
  std::array<rclcpp::Time, 4> last_swing_end_time_;

  // --- Activity gate: set true once any leg first enters swing ---
  bool is_active_;

  // --- Detector instance ---
  // detector_[i]: kinematic detection for leg i (contact-confirmed latch)
  std::array<std::unique_ptr<FootholdDetector>, 4> detector_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FootholdDetectionNode>();

  // Synchronise with the simulation clock when use_sim_time is enabled.
  bool use_sim_time = false;
  node->get_parameter_or("use_sim_time", use_sim_time, false);

  if (use_sim_time) {
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (node->now().seconds() > 0.0) {
        RCLCPP_INFO(node->get_logger(),
                    "Clock synced! Sim Time: %.2f", node->now().seconds());
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Real hardware mode: using system wall clock.");
  }

  // Re-seed landing timestamps now that the clock is valid.
  node->reseed_timestamps();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


