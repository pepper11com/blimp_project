#include "blimp_navigation/actuator_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
	speed_t baudToTermios(int baud)
	{
		switch (baud)
		{
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 921600:
			return B921600;
		default:
			return B115200;
		}
	}
}

namespace blimp_navigation
{

ActuatorInterface::ActuatorInterface(const ActuatorConfig &config)
  : config_(config)
{
  ensureBufferSizes();
  setActuatorEnabled(config_.enable_actuators);
}

ActuatorInterface::~ActuatorInterface()
{
  closeSerial();
}

void ActuatorInterface::updateConfig(const ActuatorConfig &config)
{
  std::lock_guard<std::mutex> lock(mutex_);

  bool reopen = config.serial_device != config_.serial_device ||
                config.serial_baud != config_.serial_baud;

  config_ = config;
  ensureBufferSizes();

  if (actuators_enabled_) {
    if (!config_.enable_actuators) {
      setActuatorEnabled(false);
    } else if (reopen) {
      closeSerial();
      openSerial();
    }
  } else if (config_.enable_actuators) {
    setActuatorEnabled(true);
  }
}

void ActuatorInterface::setActuatorEnabled(bool enabled)
{
  if (enabled == actuators_enabled_) {
    return;
  }

  if (enabled) {
    actuators_enabled_ = true;
    openSerial();
    if (serial_fd_ < 0) {
      actuators_enabled_ = false;
    }
  } else {
    actuators_enabled_ = false;
    closeSerial();
  }
}

void ActuatorInterface::sendCommand(const ActuatorSetpoint &setpoint, double dt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ensureBufferSizes();

  if (dt <= 0.0) {
    dt = 0.05;  // fallback to 20 Hz
  }

  double left_command = applyMotorLimits(setpoint.left_motor_norm, motor_state_[0], dt);
  double right_command = applyMotorLimits(setpoint.right_motor_norm, motor_state_[1], dt);

  motor_pwm_[0] = motorNormToPwm(left_command);
  motor_pwm_[1] = motorNormToPwm(right_command);

  debug_.left_motor_norm = left_command;
  debug_.right_motor_norm = right_command;

  const auto left_servo_last = static_cast<double>(servo_pwm_[config_.left_servo_index]);
  const auto right_servo_last = static_cast<double>(servo_pwm_[config_.right_servo_index]);

  // Apply servo inversion if configured
  double left_servo_cmd = config_.invert_left_servo ? -setpoint.left_servo_norm : setpoint.left_servo_norm;
  double right_servo_cmd = config_.invert_right_servo ? -setpoint.right_servo_norm : setpoint.right_servo_norm;

  double left_servo = applyServoLimits(left_servo_cmd, left_servo_last, config_.servo_trim_left, dt);
  double right_servo = applyServoLimits(right_servo_cmd, right_servo_last, config_.servo_trim_right, dt);

  // Apply exponential moving average (EMA) filter to smooth servo jitter
  // Alpha = 0.3 means new value has 30% weight, smoothed has 70% weight
  // This gives good balance between responsiveness and smoothness
  const double servo_filter_alpha = 0.3;
  
  if (!servo_filter_initialized_) {
    // Initialize filter with first values
    left_servo_smoothed_ = left_servo;
    right_servo_smoothed_ = right_servo;
    servo_filter_initialized_ = true;
  } else {
    // Apply EMA filter: smoothed = alpha * new + (1-alpha) * smoothed
    left_servo_smoothed_ = servo_filter_alpha * left_servo + (1.0 - servo_filter_alpha) * left_servo_smoothed_;
    right_servo_smoothed_ = servo_filter_alpha * right_servo + (1.0 - servo_filter_alpha) * right_servo_smoothed_;
  }

  // Use smoothed values for output
  left_servo = left_servo_smoothed_;
  right_servo = right_servo_smoothed_;

  servo_pwm_[config_.left_servo_index] = static_cast<uint16_t>(std::lround(left_servo));
  servo_pwm_[config_.right_servo_index] = static_cast<uint16_t>(std::lround(right_servo));

  debug_.left_servo_us = left_servo;
  debug_.right_servo_us = right_servo;

  if (!actuators_enabled_ || serial_fd_ < 0) {
    return;
  }

  writeMotorFrame();
  writeServoFrame();
}

ActuatorDebug ActuatorInterface::debug() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return debug_;
}

void ActuatorInterface::openSerial()
{
  if (serial_fd_ >= 0) {
    return;
  }

  serial_fd_ = ::open(config_.serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    std::perror("Failed to open MSP serial device");
    return;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    std::perror("tcgetattr failed");
    closeSerial();
    return;
  }

  cfmakeraw(&tty);

  speed_t baud = baudToTermios(config_.serial_baud);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    std::perror("tcsetattr failed");
    closeSerial();
    return;
  }

  tcflush(serial_fd_, TCIOFLUSH);
}

void ActuatorInterface::closeSerial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void ActuatorInterface::writeMotorFrame()
{
  std::vector<uint8_t> payload(config_.motor_count * sizeof(uint16_t), 0U);
  for (std::size_t i = 0; i < config_.motor_count; ++i) {
    const uint16_t pwm = motor_pwm_[i];
    payload[i * 2] = static_cast<uint8_t>(pwm & 0xFF);
    payload[i * 2 + 1] = static_cast<uint8_t>((pwm >> 8) & 0xFF);
  }

  auto frame = buildMspv2Frame(0x20F5, payload);
  if (frame.empty()) {
    return;
  }

  ssize_t total_written = 0;
  const ssize_t frame_size = static_cast<ssize_t>(frame.size());
  while (total_written < frame_size) {
    const ssize_t written = ::write(serial_fd_, frame.data() + total_written, frame_size - total_written);
    if (written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      std::perror("Failed to write motor frame");
      break;
    }
    total_written += written;
  }

  tcdrain(serial_fd_);
}

void ActuatorInterface::writeServoFrame()
{
  std::vector<uint8_t> payload(config_.servo_count * sizeof(uint16_t), 0U);
  for (std::size_t i = 0; i < config_.servo_count; ++i) {
    const uint16_t pwm = servo_pwm_[i];
    payload[i * 2] = static_cast<uint8_t>(pwm & 0xFF);
    payload[i * 2 + 1] = static_cast<uint8_t>((pwm >> 8) & 0xFF);
  }

  auto frame = buildMspv2Frame(0x20F6, payload);
  if (frame.empty()) {
    return;
  }

  ssize_t total_written = 0;
  const ssize_t frame_size = static_cast<ssize_t>(frame.size());
  while (total_written < frame_size) {
    const ssize_t written = ::write(serial_fd_, frame.data() + total_written, frame_size - total_written);
    if (written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      std::perror("Failed to write servo frame");
      break;
    }
    total_written += written;
  }

  tcdrain(serial_fd_);
}

uint8_t ActuatorInterface::crc8DvbS2(const std::vector<uint8_t> &data)
{
  uint8_t crc = 0;
  for (auto byte : data) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x80U) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5U);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> ActuatorInterface::buildMspv2Frame(uint16_t command, const std::vector<uint8_t> &payload)
{
  std::vector<uint8_t> frame;
  frame.reserve(3 + 1 + 2 + 2 + payload.size() + 1);

  frame.push_back('$');
  frame.push_back('X');
  frame.push_back('<');
  frame.push_back(0x00);  // flags
  frame.push_back(static_cast<uint8_t>(command & 0xFF));
  frame.push_back(static_cast<uint8_t>((command >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
  frame.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));

  frame.insert(frame.end(), payload.begin(), payload.end());

  std::vector<uint8_t> crc_input(frame.begin() + 3, frame.end());
  const uint8_t crc = crc8DvbS2(crc_input);
  frame.push_back(crc);

  return frame;
}

double ActuatorInterface::applyMotorLimits(double desired, MotorState &state, double dt)
{
  desired = std::clamp(desired, -config_.max_reverse_magnitude, config_.max_forward_magnitude);
  if (std::fabs(desired) < config_.motor_deadband) {
    desired = 0.0;
  }

  if (state.phase != ReleasePhase::NONE) {
    state.phase_time_remaining -= dt;
    if (state.phase_time_remaining <= 0.0) {
      if (state.phase == ReleasePhase::HOLD_REVERSE) {
        state.phase = ReleasePhase::HOLD_NEUTRAL;
        state.phase_time_remaining = config_.reverse_release_neutral_time;
      } else {
        state.phase = ReleasePhase::NONE;
      }
    }
  }

  // Disabled reverse release sequence - causes unwanted "kick" when reversing
  // if (state.phase == ReleasePhase::NONE) {
  //   if (state.pending_norm != 0.0) {
  //     desired = state.pending_norm;
  //     state.pending_norm = 0.0;
  //   }

  //   if (state.last_norm < -config_.motor_deadband && desired > config_.motor_deadband) {
  //     state.phase = ReleasePhase::HOLD_REVERSE;
  //     state.phase_time_remaining = config_.reverse_release_hold_time;
  //     state.pending_norm = desired;
  //   }
  // }

  double command = desired;

  // Disabled hold phases
  // if (state.phase == ReleasePhase::HOLD_REVERSE) {
  //   command = -config_.max_reverse_magnitude;
  // } else if (state.phase == ReleasePhase::HOLD_NEUTRAL) {
  //   command = 0.0;
  // } else {
  {
    // Direct command with rate limiting only
    const double max_delta = config_.motor_rate_limit * dt;
    if (max_delta > 0.0) {
      const double lower = state.last_norm - max_delta;
      const double upper = state.last_norm + max_delta;
      command = std::clamp(command, lower, upper);
    }
  }  // end direct command block

  state.last_norm = command;
  return command;
}

uint16_t ActuatorInterface::motorNormToPwm(double norm) const
{
  norm = std::clamp(norm, -config_.max_reverse_magnitude, config_.max_forward_magnitude);
  if (std::fabs(norm) <= config_.motor_deadband) {
    return static_cast<uint16_t>(config_.servo_neutral_us);
  }

  if (norm > 0.0) {
    const double max_forward = std::max(config_.max_forward_magnitude, 1e-3);
    const double pwm_span = std::max(config_.forward_max_pwm - config_.forward_activation_pwm, 1.0);
    const double scaled = std::clamp(norm / max_forward, 0.0, 1.0);
    const double pwm = config_.forward_activation_pwm + scaled * pwm_span;
    return static_cast<uint16_t>(std::lround(pwm));
  }

  const double max_reverse = std::max(config_.max_reverse_magnitude, 1e-3);
  const double pwm_span = std::max(config_.reverse_activation_pwm - config_.reverse_min_pwm, 1.0);
  const double scaled = std::clamp((-norm) / max_reverse, 0.0, 1.0);
  const double pwm = config_.reverse_activation_pwm - scaled * pwm_span;
  return static_cast<uint16_t>(std::lround(pwm));
}

double ActuatorInterface::applyServoLimits(double desired_norm, double last_us, double trim, double dt)
{
  desired_norm = std::clamp(desired_norm, -1.0, 1.0);
  double target = config_.servo_neutral_us + config_.servo_gain * desired_norm + trim;
  target = std::clamp(target, config_.servo_min_us, config_.servo_max_us);

  // Servo rate limiting disabled - let servos move at their natural speed
  // const double max_delta = config_.servo_rate_limit * dt;
  // if (max_delta > 0.0) {
  //   const double lower = last_us - max_delta;
  //   const double upper = last_us + max_delta;
  //   target = std::clamp(target, lower, upper);
  // }

  return target;
}

void ActuatorInterface::ensureBufferSizes()
{
  if (config_.motor_count == 0) {
    config_.motor_count = 2;
  }

  if (motor_pwm_.size() != config_.motor_count) {
    motor_pwm_.assign(config_.motor_count, static_cast<uint16_t>(config_.servo_neutral_us));
  }

  if (motor_state_.size() != config_.motor_count) {
    motor_state_.assign(config_.motor_count, MotorState{});
  }

  const std::size_t required_servo_channels = std::max({config_.servo_count,
                                                        config_.left_servo_index + 1,
                                                        config_.right_servo_index + 1,
                                                        static_cast<std::size_t>(2)});
  if (servo_pwm_.size() != required_servo_channels) {
    servo_pwm_.assign(required_servo_channels, static_cast<uint16_t>(config_.servo_neutral_us));
  }
  config_.servo_count = required_servo_channels;

  if (config_.left_servo_index >= config_.servo_count) {
    config_.left_servo_index = 0;
  }
  if (config_.right_servo_index >= config_.servo_count) {
    config_.right_servo_index = std::min<std::size_t>(1, config_.servo_count - 1);
  }
}

}  // namespace blimp_navigation
