#ifndef BLIMP_NAVIGATION_CPP__PID_HPP_
#define BLIMP_NAVIGATION_CPP__PID_HPP_

#include <chrono>

namespace blimp_navigation_cpp
{

class PID
{
public:
  PID(double kp, double ki, double kd, double setpoint, double out_min, double out_max);
  double update(double current_value, const std::chrono::steady_clock::time_point & current_time);
  void set_setpoint(double new_setpoint);

private:
  double kp_, ki_, kd_;
  double setpoint_;
  double out_min_, out_max_;
  std::chrono::steady_clock::time_point last_time_;
  double last_error_;
  double integral_term_;
};

}  // namespace blimp_navigation_cpp

#endif  // BLIMP_NAVIGATION_CPP__PID_HPP_