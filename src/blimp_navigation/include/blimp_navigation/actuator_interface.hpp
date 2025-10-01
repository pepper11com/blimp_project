#ifndef BLIMP_NAVIGATION__ACTUATOR_INTERFACE_HPP_
#define BLIMP_NAVIGATION__ACTUATOR_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace blimp_navigation
{

struct ActuatorConfig
{
  std::string serial_device{"/dev/ttyAMA0"};
  int serial_baud{115200};
  bool enable_actuators{false};

  double motor_deadband{0.02};
  double max_forward_magnitude{0.20};
  double max_reverse_magnitude{0.20};
  double motor_rate_limit{0.25};  // norm units per second
  double servo_rate_limit{120.0};  // microseconds per second

  double forward_activation_pwm{1550.0};
  double forward_max_pwm{1650.0};
  double reverse_activation_pwm{1375.0};
  double reverse_min_pwm{1250.0};

  double reverse_release_hold_time{0.12};
  double reverse_release_neutral_time{0.05};

  std::size_t motor_count{2};
  std::size_t servo_count{4};
  std::size_t left_servo_index{2};
  std::size_t right_servo_index{3};

  double servo_neutral_us{1500.0};
  double servo_min_us{1000.0};
  double servo_max_us{2000.0};
  double servo_gain{280.0};
  double servo_trim_left{0.0};
  double servo_trim_right{0.0};
  bool invert_left_servo{false};
  bool invert_right_servo{false};
};

struct ActuatorSetpoint
{
  double left_motor_norm{0.0};
  double right_motor_norm{0.0};
  double left_servo_norm{0.0};
  double right_servo_norm{0.0};
};

struct ActuatorDebug
{
  double left_motor_norm{0.0};
  double right_motor_norm{0.0};
  double left_servo_us{1500.0};
  double right_servo_us{1500.0};
};

class ActuatorInterface
{
public:
  explicit ActuatorInterface(const ActuatorConfig &config);
  ~ActuatorInterface();

  ActuatorInterface(const ActuatorInterface &) = delete;
  ActuatorInterface &operator=(const ActuatorInterface &) = delete;

  void updateConfig(const ActuatorConfig &config);

  void sendCommand(const ActuatorSetpoint &setpoint, double dt);

  void setActuatorEnabled(bool enabled);

  ActuatorDebug debug() const;

private:
  enum class ReleasePhase
  {
    NONE = 0,
    HOLD_REVERSE,
    HOLD_NEUTRAL
  };

  struct MotorState
  {
    double last_norm{0.0};
    ReleasePhase phase{ReleasePhase::NONE};
    double phase_time_remaining{0.0};
    double pending_norm{0.0};
  };

  void openSerial();
  void closeSerial();

  void writeMotorFrame();
  void writeServoFrame();

  static uint8_t crc8DvbS2(const std::vector<uint8_t> &data);
  static std::vector<uint8_t> buildMspv2Frame(uint16_t command, const std::vector<uint8_t> &payload);

  double applyMotorLimits(double desired, MotorState &state, double dt);
  uint16_t motorNormToPwm(double norm) const;
  double applyServoLimits(double desired_norm, double last_us, double trim, double dt);

  void ensureBufferSizes();

  ActuatorConfig config_;
  mutable std::mutex mutex_;

  std::vector<uint16_t> motor_pwm_;
  std::vector<uint16_t> servo_pwm_;
  std::vector<MotorState> motor_state_;

  bool actuators_enabled_{false};
  int serial_fd_{-1};

  ActuatorDebug debug_;
};

}  // namespace blimp_navigation

#endif  // BLIMP_NAVIGATION__ACTUATOR_INTERFACE_HPP_
