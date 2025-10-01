#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "blimp_navigation/actuator_interface.hpp"

namespace blimp_navigation
{

class ActuatorExerciserNode : public rclcpp::Node
{
public:
  ActuatorExerciserNode()
  : rclcpp::Node("actuator_exerciser")
  {
    declareParameters();
    configureInterface();
    runSequence();
  }

private:
  void declareParameters()
  {
    serial_device_ = this->declare_parameter<std::string>("serial", "/dev/ttyAMA0");
    serial_baud_ = this->declare_parameter<int>("baud", 115200);
    forward_duration_ = this->declare_parameter<double>("forward_duration", 3.0);
    reverse_duration_ = this->declare_parameter<double>("reverse_duration", 3.0);
    neutral_duration_ = this->declare_parameter<double>("neutral_duration", 1.0);
    servo_hold_duration_ = this->declare_parameter<double>("servo_hold_duration", 1.0);
    forward_norm_ = this->declare_parameter<double>("forward_norm", 0.10);
    reverse_norm_ = this->declare_parameter<double>("reverse_norm", -0.10);
    servo_high_norm_ = this->declare_parameter<double>("servo_high_norm", 0.6);
    servo_low_norm_ = this->declare_parameter<double>("servo_low_norm", -0.6);
    dry_run_ = this->declare_parameter<bool>("dry_run", false);
  }

  void configureInterface()
  {
    ActuatorConfig config;
    config.serial_device = serial_device_;
    config.serial_baud = serial_baud_;
    config.enable_actuators = !dry_run_;
    config.max_forward_magnitude = std::max(std::abs(forward_norm_), 0.20);
    config.max_reverse_magnitude = std::max(std::abs(reverse_norm_), 0.20);

    actuator_interface_ = std::make_unique<ActuatorInterface>(config);
  }

  void runSequence()
  {
    RCLCPP_INFO(get_logger(), "Starting actuator exercise (dry_run=%s)", dry_run_ ? "true" : "false");

    auto sleep_for = [](double seconds) {
      if (seconds <= 0.0) {
        return;
      }
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds)));
    };

    ActuatorSetpoint setpoint;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(neutral_duration_);

    if (forward_duration_ > 0.0 && forward_norm_ > 0.0) {
      setpoint.left_motor_norm = forward_norm_;
      setpoint.right_motor_norm = forward_norm_;
      actuator_interface_->sendCommand(setpoint, 0.1);
      sleep_for(forward_duration_);
    }

    setpoint.left_motor_norm = 0.0;
    setpoint.right_motor_norm = 0.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(neutral_duration_);

    if (reverse_duration_ > 0.0 && reverse_norm_ < 0.0) {
      setpoint.left_motor_norm = reverse_norm_;
      setpoint.right_motor_norm = reverse_norm_;
      actuator_interface_->sendCommand(setpoint, 0.1);
      sleep_for(reverse_duration_);
    }

    setpoint.left_motor_norm = 0.0;
    setpoint.right_motor_norm = 0.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(neutral_duration_);

    setpoint.left_servo_norm = servo_high_norm_;
    setpoint.right_servo_norm = servo_high_norm_;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(servo_hold_duration_);

    setpoint.left_servo_norm = servo_low_norm_;
    setpoint.right_servo_norm = servo_low_norm_;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(servo_hold_duration_);

    setpoint = ActuatorSetpoint{};
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(neutral_duration_);

    RCLCPP_INFO(get_logger(), "Actuator exercise complete");
  }

  std::string serial_device_;
  int serial_baud_{115200};
  double forward_duration_{3.0};
  double reverse_duration_{3.0};
  double neutral_duration_{1.0};
  double servo_hold_duration_{1.0};
  double forward_norm_{0.10};
  double reverse_norm_{-0.10};
  double servo_high_norm_{0.6};
  double servo_low_norm_{-0.6};
  bool dry_run_{false};

  std::unique_ptr<ActuatorInterface> actuator_interface_;
};

}  // namespace blimp_navigation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blimp_navigation::ActuatorExerciserNode>();
  rclcpp::shutdown();
  return 0;
}
