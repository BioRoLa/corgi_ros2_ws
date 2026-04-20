#include "corgi_foothold_estimator/foothold_detector.hpp"

FootholdDetector::FootholdDetector(double delay_threshold)
: delay_threshold_(delay_threshold),
  contact_confirmed_(false)
{
}

void FootholdDetector::notify_swing_started()
{
  contact_confirmed_ = false;
}

bool FootholdDetector::evaluate_kinematic(double time_since_swing_end,
                                                bool is_contact)
{
  // Once the foot has landed, detection is inactive until next swing.
  if (contact_confirmed_) { return false; }
  // While still airborne (tse forced to 0), skip latch update entirely.
  if (time_since_swing_end <= 0.0) { return false; }
  if (is_contact) { contact_confirmed_ = true; return false; }
  return time_since_swing_end > delay_threshold_;
}

