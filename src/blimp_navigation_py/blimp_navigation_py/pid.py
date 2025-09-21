#!/usr/bin/env python3
import time

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-100, 100), sample_time=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.sample_time = sample_time

        self.last_time = time.time()
        self.last_error = 0.0
        self.integral_term = 0.0

    def update(self, current_value):
        """
        Calculates the PID output for a given measurement.
        This should be called in a loop.
        """
        current_time = time.time()
        delta_time = current_time - self.last_time

        # Always update, don't skip based on sample time for navigation control
        if delta_time <= 0:
            delta_time = 0.001  # Prevent division by zero

        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.Kp * error

        # Integral term (with clamping to prevent "windup")
        self.integral_term += error * delta_time
        self.integral_term = max(self.output_limits[0], min(self.output_limits[1], self.integral_term))
        i_term = self.Ki * self.integral_term

        # Derivative term
        delta_error = error - self.last_error
        d_term = 0.0
        if delta_time > 0:
            d_term = self.Kd * (delta_error / delta_time)

        # Combine terms
        output = p_term + i_term + d_term

        # Clamp output to defined limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Store state for next iteration
        self.last_error = error
        self.last_time = current_time

        return output

    def set_setpoint(self, new_setpoint):
        """Updates the target value and resets the controller's integral and derivative states."""
        self.setpoint = new_setpoint
        self.integral_term = 0.0
        self.last_error = 0.0
        self.last_time = time.time()