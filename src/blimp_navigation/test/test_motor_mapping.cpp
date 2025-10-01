#include <gtest/gtest.h>

#include "blimp_navigation/actuator_interface.hpp"

TEST(ActuatorInterfaceTest, AppliesDeadbandAndLimits)
{
  blimp_navigation::ActuatorConfig config;
  config.enable_actuators = false;
  config.motor_deadband = 0.05;
  config.max_forward_magnitude = 0.20;
  config.max_reverse_magnitude = 0.20;
  config.motor_rate_limit = 10.0;

  blimp_navigation::ActuatorInterface interface(config);

  blimp_navigation::ActuatorSetpoint setpoint;
  setpoint.left_motor_norm = 0.02;
  setpoint.right_motor_norm = -0.02;
  interface.sendCommand(setpoint, 0.1);

  auto debug = interface.debug();
  EXPECT_DOUBLE_EQ(debug.left_motor_norm, 0.0);
  EXPECT_DOUBLE_EQ(debug.right_motor_norm, 0.0);

  setpoint.left_motor_norm = 0.5;  // Above max, should clamp to max_forward_magnitude
  setpoint.right_motor_norm = -0.5;
  interface.sendCommand(setpoint, 0.1);
  debug = interface.debug();
  EXPECT_NEAR(debug.left_motor_norm, config.max_forward_magnitude, 1e-6);
  EXPECT_NEAR(debug.right_motor_norm, -config.max_reverse_magnitude, 1e-6);
}

TEST(ActuatorInterfaceTest, ReverseTransitionEnforcesNeutral)
{
  blimp_navigation::ActuatorConfig config;
  config.enable_actuators = false;
  config.motor_deadband = 0.01;
  config.max_forward_magnitude = 0.2;
  config.max_reverse_magnitude = 0.2;
  config.motor_rate_limit = 10.0;
  config.reverse_release_hold_time = 0.1;
  config.reverse_release_neutral_time = 0.05;

  blimp_navigation::ActuatorInterface interface(config);

  blimp_navigation::ActuatorSetpoint setpoint;
  setpoint.left_motor_norm = -0.1;
  setpoint.right_motor_norm = -0.1;
  interface.sendCommand(setpoint, 0.05);
  auto debug = interface.debug();
  EXPECT_LT(debug.left_motor_norm, 0.0);

  setpoint.left_motor_norm = 0.1;
  setpoint.right_motor_norm = 0.1;
  interface.sendCommand(setpoint, 0.05);
  debug = interface.debug();
  EXPECT_LT(debug.left_motor_norm, 0.0);

  interface.sendCommand(setpoint, 0.05);
  debug = interface.debug();
  EXPECT_LT(debug.left_motor_norm, 0.0);

  interface.sendCommand(setpoint, 0.05);
  debug = interface.debug();
  EXPECT_NEAR(debug.left_motor_norm, 0.0, 1e-6);

  interface.sendCommand(setpoint, 0.05);
  debug = interface.debug();
  EXPECT_GT(debug.left_motor_norm, 0.0);
}
