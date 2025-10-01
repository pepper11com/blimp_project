#include "blimp_navigation/pid.hpp"

#include <cmath>

namespace blimp_navigation
{

PID::PID(const Gains &gains)
{
  setGains(gains);
}

void PID::setGains(const Gains &gains)
{
  gains_ = gains;
  reset();
}

void PID::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  has_previous_ = false;
}

double PID::update(double error, double dt)
{
  if (dt <= 0.0) {
    return gains_.kp * error;
  }

  integral_ += error * dt;
  if (gains_.integral_limit > 0.0) {
    integral_ = std::clamp(integral_, -gains_.integral_limit, gains_.integral_limit);
  }

  double derivative = 0.0;
  if (has_previous_) {
    derivative = (error - previous_error_) / dt;
  }

  previous_error_ = error;
  has_previous_ = true;

  return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
}

}  // namespace blimp_navigation
