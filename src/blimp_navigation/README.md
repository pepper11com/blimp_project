# blimp_navigation

High-level navigation control stack for the autonomous blimp. This package consumes 3D paths produced by `blimp_ompl_planner`, applies sluggish-friendly PID control to keep the airship on track, and drives the dual-thruster vectored actuation system via MSP overrides.

## Features

- MSPv2 actuator interface with slew-rate limiting, neutral enforcement, and forward/reverse safe zones
- Slow-response PID controllers for yaw alignment and altitude trimming tailored for a 3 m blimp
- Pure-pursuit style path follower with lookahead, cross-track compensation, and braking envelope
- Conservative throttle scheduler that attenuates thrust when yaw or cross-track error is high
- RViz visualisation topics for the complete path and the remaining leg still to be flown
- Optional actuator debug topic exposing the normalised commands, servo microseconds, and key error metrics

## Nodes

### `blimp_navigator_node`

| Parameter | Description | Default |
|-----------|-------------|---------|
| `path_topic` | Topic providing `nav_msgs/Path` inputs (from planner) | `/blimp/planned_path` |
| `robot_frame` | TF frame for the blimp base | `camera_link` |
| `global_frame` | TF frame for navigation | `map` |
| `control_rate_hz` | Control loop frequency | `20.0` |
| `enable_actuators` | Connect to MSP hardware if `true` | `false` |
| `serial_device` | MSP serial device path | `/dev/ttyAMA0` |
| `serial_baud` | MSP baudrate | `115200` |
| `motor_rate_limit` | Max command slew (norm / s) | `0.25` |
| `servo_rate_limit` | Max servo slew (µs / s) | `120.0` |
| `motor_deadband` | Neutral deadband around zero | `0.02` |
| `max_forward_magnitude` | Forward thrust limit (norm) | `0.20` |
| `max_reverse_magnitude` | Reverse thrust limit (norm) | `0.20` |
| `reverse_release_hold_time` | Ramp duration down to 1050 µs before neutral (s) | `0.12` |
| `reverse_release_neutral_time` | Neutral hold after reverse dwell (s) | `0.05` |
| `lookahead_distance` | Lookahead distance for target point | `1.8` |
| `waypoint_tolerance` | Radius for waypoint completion | `0.6` |
| `goal_tolerance` | Terminal completion radius | `0.8` |
| `max_speed_command` | Hard cap on forward command | `0.12` |
| `cruise_speed_command` | Typical forward command when aligned | `0.10` |
| `min_speed_command` | Minimum forward command when moving | `0.03` |
| `brake_distance` | Range to start ramping down speed | `1.5` |
| `brake_min_scale` | Minimum fraction of cruise speed near goal | `0.05` |
| `yaw_limit` | Maximum yaw correction command | `0.12` |
| `yaw_cross_track_gain` | Cross-track coupling gain | `0.2` |
| `cross_track_limit` | Clamp for cross-track error (m) | `0.4` |
| `cross_track_slowdown_gain` | Aggressive slowdown per metre of lateral error | `1.5` |
| `cross_track_slowdown_min_scale` | Floor for cross-track slowdown factor | `0.35` |
| `yaw_slowdown_threshold` | Heading error (rad) before forward throttle tapers | `0.2` |
| `yaw_slowdown_min_scale` | Minimum forward multiplier at high yaw error | `0.25` |
| `altitude_limit` | Maximum altitude correction command | `0.7` |
| `servo_trim_left` / `servo_trim_right` | Servo trim offsets (µs) | `0.0` |
| `servo_gain` | Servo throw per unit command (µs) | `280.0` |
| `servo_min_us` / `servo_max_us` | Mechanical limits for servo microseconds | `1000` / `2000` |
| `actuator_startup_check` | Run optional actuator exercise routine on startup | `false` |
| `startup_motor_duration` | Seconds to hold motors in each forward/reverse direction | `3.0` |
| `startup_servo_hold_duration` | Seconds to hold servos at extremes during sweep | `1.0` |
| `startup_neutral_duration` | Neutral dwell between startup segments | `1.0` |
| `yaw_kp`, `yaw_ki`, `yaw_kd`, `yaw_integral_limit` | PID gains for yaw controller | `1.4`, `0.0`, `0.2`, `0.1` |
| `altitude_kp`, `altitude_ki`, `altitude_kd`, `altitude_integral_limit` | PID gains for altitude/thrust vectoring | `0.8`, `0.0`, `0.15`, `0.2` |

Published topics:

- `/navigator/full_path` (`nav_msgs/Path`) – copy of the last received path
- `/navigator/remaining_path` (`nav_msgs/Path`) – dynamic subset representing the unfinished portion
- `/navigator/actuator_debug` (`std_msgs/Float32MultiArray`) – `[left_motor_norm, right_motor_norm, left_servo_us, right_servo_us, heading_error, cross_track_error]`

## Running

```bash
ros2 launch blimp_navigation blimp_navigator.launch.py
```

Set `enable_actuators:=true` only when the blimp’s MSP override link is ready and arming requirements are satisfied. The actuator bridge enforces the MSP reverse-neutral-forward safety dance automatically: when leaving reverse it ramps the thrust command down to 1050 µs, then snaps to neutral (1500 µs) before permitting forward thrust. All commands are rate-limited, clamped to ±20 % of the ESC envelope by default, and translated to servo microseconds with configurable trims and limits. Use the `actuator_debug` topic to monitor the normalised commands and error metrics in RViz or Foxglove while tuning gains.

### Actuator exerciser utility

For bench checks without the full navigation stack, build the package and run the standalone exerciser:

```bash
colcon build --packages-select blimp_navigation
source install/setup.bash
ros2 run blimp_navigation actuator_exerciser --serial /dev/ttyAMA0 --forward-duration 3.0 --reverse-duration 3.0
```

The tool commands both motors forward for the configured duration (default ≈1850 µs), idles, then sends the requested magnitude in reverse (spanning 1450 µs down to 1050 µs) before returning to neutral. It finishes with a servo sweep from neutral (1500 µs) up to the configured high value (default 1800 µs) and down to the low value (default 1200 µs). Pass `--reverse-pwm 1100` to explicitly test a strong reverse thrust pulse, or `--reverse-norm -0.12` for a gentler 1100 µs equivalent. Use `--dry-run` to skip serial writes while still printing the planned sequence. Interrupt with `Ctrl+C`; the utility always finishes by driving the actuators back to neutral.

## Testing

Unit coverage is provided via `colcon test --packages-select blimp_navigation`, which currently exercises the path follower logic. Slew limiting and MSP comms are integration-tested on hardware.

## Next steps

- Wire feedback from the flight controller to detect failsafes or RC override changes
- Integrate wind-compensation feed-forward once airspeed sensing is available
- Extend unit tests with simulated kinematics to validate steady-state tracking errors
