#ifndef BLIMP_NAVIGATION__PID_HPP_
#define BLIMP_NAVIGATION__PID_HPP_

#include <algorithm>

namespace blimp_navigation
{

class PID
{
public:
  struct Gains
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double integral_limit{0.0};
  };

  PID() = default;
  explicit PID(const Gains &gains);

  void setGains(const Gains &gains);

  void reset();

  double update(double error, double dt);

private:
  Gains gains_{};
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_{false};
};

}  // namespace blimp_navigation

#endif  // BLIMP_NAVIGATION__PID_HPP_
