#ifndef BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_
#define BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_

#include <string>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <asio.hpp>

namespace blimp_navigation_cpp
{

class BlimpController
{
public:
  static constexpr std::size_t SERVO_COUNT = 4;
  static constexpr std::size_t MOTOR_COUNT = 2;
  static constexpr uint16_t SERVO_NEUTRAL_US = 1500;
  static constexpr uint16_t MOTOR_NEUTRAL_US = 1500;

  BlimpController(const std::string & device, int baud_rate);
  ~BlimpController();

  void set_servo_us(size_t idx, int us);
  void set_motor_pct(size_t motor_idx, double pct);

private:
  static constexpr double kForwardMinUs = 1550.0;
  static constexpr double kForwardMaxUs = 1950.0;
  static constexpr double kReverseLightUs = 1050.0;
  static constexpr double kReverseStrongUs = 1450.0;
  static constexpr int kMotorRampStepUs = 40;

  void sender_loop();
  void send_servos();
  void send_motors();
  uint16_t map_pct_to_pwm(double pct) const;
  void step_motor_outputs_locked();

  asio::io_context io_ctx_;
  asio::serial_port serial_port_;
  
  std::mutex mtx_;
  std::array<uint16_t, SERVO_COUNT> servo_values_;
  std::array<uint16_t, MOTOR_COUNT> motor_values_;
  std::array<uint16_t, MOTOR_COUNT> target_motor_values_;
  
  std::atomic<bool> run_thread_;
  std::thread sender_thread_;
};

}  // namespace blimp_navigation_cpp

#endif  // BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_