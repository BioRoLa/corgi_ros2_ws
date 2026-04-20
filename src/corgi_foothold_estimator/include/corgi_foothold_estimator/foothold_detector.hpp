#ifndef FOOTHOLD_DETECTOR_HPP
#define FOOTHOLD_DETECTOR_HPP

class FootholdDetector
{
public:
  FootholdDetector(double delay_threshold);
  void notify_swing_started();
  bool evaluate_kinematic(double time_since_swing_end, bool is_contact);

private:
  double delay_threshold_;
  bool contact_confirmed_;
};

#endif  // FOOTHOLD_DETECTOR_HPP


