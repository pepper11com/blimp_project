#include "blimp_navigation/blimp_controller.hpp"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <vector>

namespace blimp_navigation_cpp
{

// MSPv2 helper functions
namespace msp
{
  uint8_t crc8_dvb_s2(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (uint8_t b : data) {
      crc ^= b;
      for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
      }
    }
    return crc;
  }

  std::vector<uint8_t> build_mspv2_native(uint16_t cmd, const std::vector<uint8_t>& payload, uint8_t flags = 0) {
    std::vector<uint8_t> frame;
    frame.push_back('$');
    frame.push_back('X');
    frame.push_back('<');
    frame.push_back(flags);
    frame.push_back(static_cast<uint8_t>(cmd & 0xFF));
    frame.push_back(static_cast<uint8_t>((cmd >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
    frame.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
    
    std::vector<uint8_t> checksum_data;
    checksum_data.insert(checksum_data.end(), frame.begin() + 3, frame.end());
    checksum_data.insert(checksum_data.end(), payload.begin(), payload.end());
    
    uint8_t crc = crc8_dvb_s2(checksum_data);
    
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(crc);
    
    return frame;
  }
} // namespace msp

BlimpController::BlimpController(const std::string & device, int baud_rate)
: serial_port_(io_ctx_),
  servo_values_{},
  motor_values_{},
  target_motor_values_{},
  run_thread_(true)
{
  servo_values_.fill(SERVO_NEUTRAL_US);
  motor_values_.fill(MOTOR_NEUTRAL_US);
  target_motor_values_.fill(MOTOR_NEUTRAL_US);

  try {
    serial_port_.open(device);
    serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
    sender_thread_ = std::thread(&BlimpController::sender_loop, this);
  } catch (const std::exception& e) {
    std::cerr << "FATAL: Failed to open serial port " << device << ": " << e.what() << std::endl;
    throw;
  }
}

BlimpController::~BlimpController()
{
  run_thread_ = false;
  if (sender_thread_.joinable()) {
    sender_thread_.join();
  }
  // Reset motors and servos to safe state
  {
    std::lock_guard<std::mutex> lock(mtx_);
    target_motor_values_.fill(MOTOR_NEUTRAL_US);
    motor_values_.fill(MOTOR_NEUTRAL_US);
    servo_values_.fill(SERVO_NEUTRAL_US);
  }
  send_motors();
  send_servos();
  if (serial_port_.is_open()) {
    serial_port_.close();
  }
}

void BlimpController::set_servo_us(size_t idx, int us)
{
  us = std::clamp(us, 500, 2500);
  uint16_t value_to_set = us;
  if (idx == 2) { // Invert servo 2
    value_to_set = 3000 - us;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  if (idx < servo_values_.size()) {
    servo_values_[idx] = value_to_set;
  }
}

void BlimpController::set_motor_pct(size_t motor_idx, double pct)
{
  const uint16_t us = map_pct_to_pwm(pct);

  std::lock_guard<std::mutex> lock(mtx_);
  if (motor_idx < MOTOR_COUNT) {
    target_motor_values_[motor_idx] = us;
  }
}

uint16_t BlimpController::map_pct_to_pwm(double pct) const
{
  const double pct_clamped = std::clamp(pct, -100.0, 100.0);
  double us = MOTOR_NEUTRAL_US;

  if (pct_clamped > 0.0) {
    const double span = kForwardMaxUs - kForwardMinUs;
    us = kForwardMinUs + (pct_clamped / 100.0) * span;
  } else if (pct_clamped < 0.0) {
    const double span = kReverseStrongUs - kReverseLightUs;
    const double magnitude = std::abs(pct_clamped) / 100.0;
    us = kReverseLightUs + magnitude * span;
  }

  us = std::clamp(us, kReverseLightUs, kForwardMaxUs);
  return static_cast<uint16_t>(std::lround(us));
}

void BlimpController::step_motor_outputs_locked()
{
  for (std::size_t i = 0; i < MOTOR_COUNT; ++i) {
    const int current = static_cast<int>(motor_values_[i]);
    const int target = static_cast<int>(target_motor_values_[i]);

    if (current == target) {
      continue;
    }

    const int diff = target - current;
    const int step = std::min(std::abs(diff), kMotorRampStepUs);
    motor_values_[i] = static_cast<uint16_t>(current + (diff > 0 ? step : -step));
  }
}

void BlimpController::send_servos()
{
  constexpr uint16_t MSP2_INAV_SET_SERVO_OVERRIDE = 0x20F6;
  std::vector<uint8_t> payload(servo_values_.size() * 2);
  for (size_t i = 0; i < servo_values_.size(); ++i) {
    payload[i*2] = servo_values_[i] & 0xFF;
    payload[i*2 + 1] = (servo_values_[i] >> 8) & 0xFF;
  }
  auto frame = msp::build_mspv2_native(MSP2_INAV_SET_SERVO_OVERRIDE, payload);
  asio::write(serial_port_, asio::buffer(frame));
}

void BlimpController::send_motors()
{
  constexpr uint16_t MSP2_INAV_SET_MOTOR_OVERRIDE = 0x20F5;
  std::vector<uint8_t> payload(motor_values_.size() * 2);
  for (size_t i = 0; i < motor_values_.size(); ++i) {
    payload[i*2] = motor_values_[i] & 0xFF;
    payload[i*2 + 1] = (motor_values_[i] >> 8) & 0xFF;
  }
  auto frame = msp::build_mspv2_native(MSP2_INAV_SET_MOTOR_OVERRIDE, payload);
  asio::write(serial_port_, asio::buffer(frame));
}

void BlimpController::sender_loop()
{
  const int UPDATE_HZ = 20;
  auto delay = std::chrono::milliseconds(1000 / UPDATE_HZ);
  while (run_thread_) {
    {
      std::lock_guard<std::mutex> lock(mtx_);

      step_motor_outputs_locked();

      try {
        send_servos();
        send_motors();
      } catch (const std::exception& e) {
        std::cerr << "BlimpController send error: " << e.what() << std::endl;
      }
    }
    std::this_thread::sleep_for(delay);
  }
}

}  // namespace blimp_navigation_cpp