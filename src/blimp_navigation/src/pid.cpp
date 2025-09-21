#include "blimp_navigation/pid.hpp"
#include <algorithm>

namespace blimp_navigation_cpp
{

PID::PID(double kp, double ki, double kd, double setpoint, double out_min, double out_max)
: kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), out_min_(out_min), out_max_(out_max),
  last_time_(std::chrono::steady_clock::now()), last_error_(0.0), integral_term_(0.0)
{
}

void PID::set_setpoint(double new_setpoint)
{
  setpoint_ = new_setpoint;
  integral_term_ = 0.0;
  last_error_ = 0.0;
  last_time_ = std::chrono::steady_clock::now();
}

double PID::update(double current_value, const std::chrono::steady_clock::time_point & current_time)
{
  auto delta_time_ns = current_time - last_time_;
  double delta_time_s = std::chrono::duration_cast<std::chrono::nanoseconds>(delta_time_ns).count() / 1e9;
  
  if (delta_time_s <= 0) {
    delta_time_s = 0.001; // Prevent division by zero
  }

  double error = setpoint_ - current_value;
  
  // Proportional term
  double p_term = kp_ * error;

  // Integral term (with clamping)
  integral_term_ += error * delta_time_s;
  integral_term_ = std::clamp(integral_term_, out_min_, out_max_);
  double i_term = ki_ * integral_term_;

  // Derivative term
  double delta_error = error - last_error_;
  double d_term = kd_ * (delta_error / delta_time_s);

  // Combine terms and clamp output
  double output = std::clamp(p_term + i_term + d_term, out_min_, out_max_);

  last_error_ = error;
  last_time_ = current_time;

  return output;
}

}  // namespace blimp_navigation_cpp