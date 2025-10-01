#include <gtest/gtest.h>

#include "blimp_navigation/actuator_interface.hpp"

TEST(ActuatorInterfaceTest, ServoRateLimiting)
{
  blimp_navigation::ActuatorConfig config;
  config.enable_actuators = false;
  config.servo_rate_limit = 100.0;  // microseconds per second
  config.servo_gain = 300.0;

  blimp_navigation::ActuatorInterface interface(config);

  blimp_navigation::ActuatorSetpoint setpoint;
  setpoint.left_servo_norm = 1.0;
  setpoint.right_servo_norm = 1.0;

  interface.sendCommand(setpoint, 0.05);  // 50 ms
  auto debug = interface.debug();
  EXPECT_NEAR(debug.left_servo_us, 1505.0, 1.0);

  interface.sendCommand(setpoint, 0.05);
  debug = interface.debug();
  EXPECT_NEAR(debug.left_servo_us, 1510.0, 1.5);
}
