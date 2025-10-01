#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "blimp_navigation/actuator_interface.hpp"
#include "blimp_navigation/path_follower.hpp"

namespace blimp_navigation
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

inline double degToRad(double degrees)
{
  return degrees * kDegToRad;
}

inline double radToDeg(double radians)
{
  return radians * kRadToDeg;
}
}  // namespace

class BlimpNavigatorNode : public rclcpp::Node
{
public:
  BlimpNavigatorNode()
  : rclcpp::Node("blimp_navigator_node"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    declareParameters();
    initialiseInterfaces();
    initialiseSubscriptions();
    initialiseTimer();

    RCLCPP_INFO(get_logger(), "Blimp navigator node initialised");

    if (actuator_config_.enable_actuators && actuator_startup_check_) {
      runActuatorStartupSequence();
    }
  }

private:
  void declareParameters()
  {
    path_topic_ = this->declare_parameter<std::string>("path_topic", "planned_path");
    full_path_topic_ = this->declare_parameter<std::string>("full_path_topic", "navigator/full_path");
    remaining_path_topic_ = this->declare_parameter<std::string>("remaining_path_topic", "navigator/remaining_path");
    debug_topic_ = this->declare_parameter<std::string>("debug_topic", "navigator/actuator_debug");

    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = this->declare_parameter<std::string>("robot_frame", "camera_link");

    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 25.0);

    actuator_config_.serial_device = this->declare_parameter<std::string>("serial_device", "/dev/ttyAMA0");
    actuator_config_.serial_baud = this->declare_parameter<int>("serial_baud", 115200);
    actuator_config_.enable_actuators = this->declare_parameter<bool>("enable_actuators", false);

    actuator_config_.motor_deadband = this->declare_parameter<double>("motor_deadband", 0.02);
    actuator_config_.max_forward_magnitude = this->declare_parameter<double>("max_forward_magnitude", 0.20);
    actuator_config_.max_reverse_magnitude = this->declare_parameter<double>("max_reverse_magnitude", 0.20);
    actuator_config_.motor_rate_limit = this->declare_parameter<double>("motor_rate_limit", 0.25);
    actuator_config_.servo_rate_limit = this->declare_parameter<double>("servo_rate_limit", 120.0);
    actuator_config_.forward_activation_pwm = this->declare_parameter<double>("forward_activation_pwm", 1550.0);
    actuator_config_.forward_max_pwm = this->declare_parameter<double>("forward_max_pwm", 1650.0);
    actuator_config_.reverse_activation_pwm = this->declare_parameter<double>("reverse_activation_pwm", 1375.0);
    actuator_config_.reverse_min_pwm = this->declare_parameter<double>("reverse_min_pwm", 1250.0);
    actuator_config_.reverse_release_hold_time = this->declare_parameter<double>("reverse_release_hold_time", 0.12);
    actuator_config_.reverse_release_neutral_time = this->declare_parameter<double>("reverse_release_neutral_time", 0.05);
    actuator_config_.servo_neutral_us = this->declare_parameter<double>("servo_neutral_us", 1500.0);
    actuator_config_.servo_min_us = this->declare_parameter<double>("servo_min_us", 1000.0);
    actuator_config_.servo_max_us = this->declare_parameter<double>("servo_max_us", 2000.0);
    actuator_config_.servo_gain = this->declare_parameter<double>("servo_gain", 280.0);
    actuator_config_.servo_trim_left = this->declare_parameter<double>("servo_trim_left", 0.0);
    actuator_config_.servo_trim_right = this->declare_parameter<double>("servo_trim_right", 0.0);
    actuator_config_.invert_left_servo = this->declare_parameter<bool>("invert_left_servo", false);
    actuator_config_.invert_right_servo = this->declare_parameter<bool>("invert_right_servo", false);
    actuator_config_.left_servo_index = static_cast<std::size_t>(this->declare_parameter<int>("left_servo_index", 2));
    actuator_config_.right_servo_index = static_cast<std::size_t>(this->declare_parameter<int>("right_servo_index", 3));

    actuator_startup_check_ = this->declare_parameter<bool>("actuator_startup_check", false);
    startup_motor_duration_ = this->declare_parameter<double>("startup_motor_duration", 3.0);
    startup_servo_hold_duration_ = this->declare_parameter<double>("startup_servo_hold_duration", 1.0);
    startup_neutral_duration_ = this->declare_parameter<double>("startup_neutral_duration", 1.0);

    PathFollowerConfig follower_config;
    follower_config.lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.8);
    follower_config.waypoint_tolerance = this->declare_parameter<double>("waypoint_tolerance", 0.6);
    follower_config.goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.8);
    follower_config.max_speed_command = this->declare_parameter<double>("max_speed_command", 0.12);
    follower_config.cruise_speed_command = this->declare_parameter<double>("cruise_speed_command", 0.10);
    follower_config.min_speed_command = this->declare_parameter<double>("min_speed_command", 0.03);
    follower_config.brake_distance = this->declare_parameter<double>("brake_distance", 1.5);
    follower_config.brake_min_scale = this->declare_parameter<double>("brake_min_scale", 0.05);
    follower_config.yaw_limit = this->declare_parameter<double>("yaw_limit", 0.12);
    follower_config.yaw_cross_track_gain = this->declare_parameter<double>("yaw_cross_track_gain", 0.2);
    follower_config.cross_track_limit = this->declare_parameter<double>("cross_track_limit", 0.4);
    follower_config.cross_track_slowdown_gain = this->declare_parameter<double>("cross_track_slowdown_gain", 1.5);
    follower_config.cross_track_slowdown_min_scale = this->declare_parameter<double>("cross_track_slowdown_min_scale", 0.35);
    follower_config.yaw_slowdown_threshold = this->declare_parameter<double>("yaw_slowdown_threshold", 0.2);
    follower_config.yaw_slowdown_min_scale = this->declare_parameter<double>("yaw_slowdown_min_scale", 0.25);
    follower_config.altitude_limit = this->declare_parameter<double>("altitude_limit", 0.7);
    follower_config.max_forward_norm = actuator_config_.max_forward_magnitude;
    follower_config.max_reverse_norm = actuator_config_.max_reverse_magnitude;
    follower_config.pivot_heading_threshold = degToRad(
      this->declare_parameter<double>("pivot_heading_threshold_deg", 60.0));
    follower_config.pivot_min_turn_command = this->declare_parameter<double>("pivot_min_turn_command", 0.08);
    follower_config.single_motor_threshold = degToRad(
      this->declare_parameter<double>("single_motor_threshold_deg", 10.0));

    PID::Gains yaw_gains{};
    yaw_gains.kp = this->declare_parameter<double>("yaw_kp", 1.4);
    yaw_gains.ki = this->declare_parameter<double>("yaw_ki", 0.0);
    yaw_gains.kd = this->declare_parameter<double>("yaw_kd", 0.2);
    yaw_gains.integral_limit = this->declare_parameter<double>("yaw_integral_limit", 0.1);
    follower_config.yaw_gains = yaw_gains;

    PID::Gains altitude_gains{};
    altitude_gains.kp = this->declare_parameter<double>("altitude_kp", 0.8);
    altitude_gains.ki = this->declare_parameter<double>("altitude_ki", 0.0);
    altitude_gains.kd = this->declare_parameter<double>("altitude_kd", 0.15);
    altitude_gains.integral_limit = this->declare_parameter<double>("altitude_integral_limit", 0.2);
    follower_config.altitude_gains = altitude_gains;

    path_follower_ = std::make_unique<PathFollower>(follower_config);
  }

  void initialiseInterfaces()
  {
    actuator_interface_ = std::make_unique<ActuatorInterface>(actuator_config_);
  }

  void initialiseSubscriptions()
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, qos,
      std::bind(&BlimpNavigatorNode::pathCallback, this, std::placeholders::_1));

    full_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(full_path_topic_, 10);
    remaining_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(remaining_path_topic_, 10);
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(debug_topic_, 10);
  }

  void initialiseTimer()
  {
    if (control_rate_hz_ <= 0.0) {
      control_rate_hz_ = 20.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BlimpNavigatorNode::controlLoop, this));
    last_control_time_ = this->now();
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    path_follower_->setPath(*msg);

    nav_msgs::msg::Path full_path = *msg;
    full_path.header.stamp = this->now();
    full_path_publisher_->publish(full_path);
    if (!msg->poses.empty()) {
      const auto &start = msg->poses.front().pose.position;
      const auto &goal = msg->poses.back().pose.position;
      double length = 0.0;
      for (std::size_t i = 1; i < msg->poses.size(); ++i) {
        const auto &prev = msg->poses[i - 1].pose.position;
        const auto &curr = msg->poses[i].pose.position;
        length += std::sqrt(
          std::pow(curr.x - prev.x, 2.0) +
          std::pow(curr.y - prev.y, 2.0) +
          std::pow(curr.z - prev.z, 2.0));
      }

      RCLCPP_INFO(get_logger(),
        "Received new path with %zu poses (length %.2f m) start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)",
        msg->poses.size(),
        length,
        start.x, start.y, start.z,
        goal.x, goal.y, goal.z);
    } else {
      RCLCPP_INFO(get_logger(), "Received empty path");
    }
  }

  bool getCurrentPose(geometry_msgs::msg::PoseStamped &pose)
  {
    try {
      const auto timeout = tf2::durationFromSec(0.1);
      const auto transform = tf_buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero, timeout);
      pose.header.stamp = this->now();
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void controlLoop()
  {
    const auto now_time = this->now();
    double dt = (now_time - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / control_rate_hz_;
    }
    last_control_time_ = now_time;

    geometry_msgs::msg::PoseStamped current_pose;
    ActuatorSetpoint setpoint;
    nav_msgs::msg::Path remaining_path_msg;
    ControlCommand control_command;
    bool have_command = false;

    if (getCurrentPose(current_pose)) {
      control_command = path_follower_->update(current_pose, dt, remaining_path_msg);
      have_command = true;

      if (control_command.has_active_path) {
        setpoint.left_motor_norm = control_command.left_motor_norm;
        setpoint.right_motor_norm = control_command.right_motor_norm;
        setpoint.left_servo_norm = control_command.left_servo_norm;
        setpoint.right_servo_norm = control_command.right_servo_norm;
      } else {
        setpoint = ActuatorSetpoint{};
      }

      remaining_path_msg.header.stamp = now_time;
      remaining_path_publisher_->publish(remaining_path_msg);
    } else {
      remaining_path_msg.header.frame_id = global_frame_;
      remaining_path_msg.header.stamp = now_time;
      remaining_path_publisher_->publish(remaining_path_msg);

      setpoint = ActuatorSetpoint{};
    }

    publishFullPath(now_time);

    actuator_interface_->sendCommand(setpoint, dt);
    const auto actuator_debug = actuator_interface_->debug();

    if (have_command) {
      publishDebug(control_command, actuator_debug, now_time);

      if (control_command.has_active_path || control_command.goal_reached) {
        // Determine what the controller WANTS based on error
        const double heading_err = radToDeg(control_command.heading_error);
        const char* wanted_turn = "STRAIGHT";
        if (std::abs(heading_err) > 5.0) {
          wanted_turn = heading_err > 0 ? "LEFT ←" : "RIGHT →";
        }
        
        const double alt_err = control_command.altitude_error;
        const char* wanted_alt = "LEVEL";
        if (std::abs(alt_err) > 0.05) {
          wanted_alt = alt_err > 0 ? "UP ↑" : "DOWN ↓";
        }
        
        // Determine what the controller THINKS will happen based on commands
        const double motor_diff = control_command.left_motor_norm - control_command.right_motor_norm;
        const char* commanded_turn = "STRAIGHT";
        if (std::abs(motor_diff) > 0.05) {
          // Left motor > Right motor = blimp turns RIGHT (left side pushes harder)
          commanded_turn = motor_diff > 0 ? "RIGHT →" : "LEFT ←";
        }
        
        // Check individual servo deflections (servos move opposite for pitch control)
        const double left_deflection = actuator_debug.left_servo_us - 1500.0;
        const double right_deflection = actuator_debug.right_servo_us - 1500.0;
        const char* commanded_alt = "LEVEL";
        // If servos are opposite (one high, one low), they're commanding pitch
        if (std::abs(left_deflection) > 100.0 || std::abs(right_deflection) > 100.0) {
          // Left low + Right high = pitch up (based on physical mounting)
          commanded_alt = left_deflection < 0 ? "UP ↑" : "DOWN ↓";
        }
        
        RCLCPP_INFO_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "\n"
          "┌─────────────── BLIMP NAVIGATOR ───────────────┐\n"
          "│ Mode: %-38s          │\n"
          "│ Priority: %-13s  Blend: %3.0f%% ALT         │\n"
          "│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   │\n"
          "│ HEADING:                                      │\n"
          "│   Current:  %6.1f°  →  Target: %6.1f°         │\n"
          "│   Error:    %6.1f°  [Want: %s]                │\n"
          "│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   │\n"
          "│ POSITION:                                     │\n"
          "│   Goal Distance:    %5.2f m                   │\n"
          "│   Cross-Track:      %+5.2f m                  │\n"
          "│   Lookahead:        %5.2f m                   │\n"
          "│   Altitude:         %+5.2f m  [Want: %s]      │\n"
          "│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   │\n"
          "│ MOTORS:                                       │\n"
          "│   Left:  %+5.2f  Right: %+5.2f  [Cmd: %s]     │\n"
          "│   Forward:  %+5.2f                            │\n"
          "│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   │\n"
          "│ SERVOS:                                       │\n"
          "│   Left: %4.0f µs  Right: %4.0f µs  [Cmd: %s]  │\n"
          "└───────────────────────────────────────────────┘",
          control_command.state.c_str(),
          control_command.control_priority.c_str(),
          control_command.servo_blend_factor * 100.0,
          radToDeg(control_command.current_heading),
          radToDeg(control_command.desired_heading),
          heading_err,
          wanted_turn,
          control_command.goal_distance,
          control_command.cross_track_error,
          control_command.lookahead_distance,
          current_pose.pose.position.z,
          wanted_alt,
          control_command.left_motor_norm,
          control_command.right_motor_norm,
          commanded_turn,
          control_command.forward_command,
          actuator_debug.left_servo_us,
          actuator_debug.right_servo_us,
          commanded_alt);
      }
    } else {
      publishDebug(ControlCommand{}, actuator_debug, now_time);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "No pose - holding neutral outputs (L=%.2f R=%.2f)",
        actuator_debug.left_motor_norm,
        actuator_debug.right_motor_norm);
    }
  }

  void publishFullPath(const rclcpp::Time &stamp)
  {
    const auto &full_path = path_follower_->fullPath();
    if (full_path.poses.empty()) {
      return;
    }

    nav_msgs::msg::Path path_copy = full_path;
    path_copy.header.stamp = stamp;
    full_path_publisher_->publish(path_copy);
  }

  void publishDebug(const ControlCommand &command, const ActuatorDebug &actuator_debug, const rclcpp::Time &stamp)
  {
    if (!debug_publisher_) {
      return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.clear();
    msg.layout.data_offset = 0;
  msg.data.reserve(10);
  msg.data.push_back(static_cast<float>(command.left_motor_norm));
  msg.data.push_back(static_cast<float>(command.right_motor_norm));
  msg.data.push_back(static_cast<float>(actuator_debug.left_motor_norm));
  msg.data.push_back(static_cast<float>(actuator_debug.right_motor_norm));
  msg.data.push_back(static_cast<float>(actuator_debug.left_servo_us));
  msg.data.push_back(static_cast<float>(actuator_debug.right_servo_us));
  msg.data.push_back(static_cast<float>(command.forward_command));
  msg.data.push_back(static_cast<float>(radToDeg(command.heading_error)));
  msg.data.push_back(static_cast<float>(command.cross_track_error));
  msg.data.push_back(static_cast<float>(command.goal_distance));
    debug_publisher_->publish(msg);
    (void)stamp;
  }

  void runActuatorStartupSequence()
  {
    if (!actuator_config_.enable_actuators) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Running actuator startup sequence");

    auto sleep_for = [this](double seconds) {
      if (seconds <= 0.0) {
        return;
      }
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds)));
    };

    ActuatorSetpoint setpoint;
    const double forward_norm = std::min(actuator_config_.max_forward_magnitude, 0.12);
    const double reverse_norm = -std::min(actuator_config_.max_reverse_magnitude, 0.12);

    // Neutral
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_neutral_duration_);

    // Forward hold
    setpoint.left_motor_norm = forward_norm;
    setpoint.right_motor_norm = forward_norm;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_motor_duration_);

    // Neutral
    setpoint.left_motor_norm = 0.0;
    setpoint.right_motor_norm = 0.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_neutral_duration_);

    // Reverse hold
    setpoint.left_motor_norm = reverse_norm;
    setpoint.right_motor_norm = reverse_norm;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_motor_duration_);

    // Neutral
    setpoint.left_motor_norm = 0.0;
    setpoint.right_motor_norm = 0.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_neutral_duration_);

    // Servo sweep up
    setpoint.left_servo_norm = 1.0;
    setpoint.right_servo_norm = 1.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_servo_hold_duration_);

    // Servo sweep down
    setpoint.left_servo_norm = -1.0;
    setpoint.right_servo_norm = -1.0;
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_servo_hold_duration_);

    // Neutral finish
    setpoint = ActuatorSetpoint{};
    actuator_interface_->sendCommand(setpoint, 0.1);
    sleep_for(startup_neutral_duration_);

    RCLCPP_INFO(get_logger(), "Actuator startup sequence complete");
  }

  std::string path_topic_;
  std::string full_path_topic_;
  std::string remaining_path_topic_;
  std::string debug_topic_;
  std::string global_frame_;
  std::string robot_frame_;

  double control_rate_hz_{20.0};

  ActuatorConfig actuator_config_;
  bool actuator_startup_check_{false};
  double startup_motor_duration_{3.0};
  double startup_servo_hold_duration_{1.0};
  double startup_neutral_duration_{1.0};

  std::unique_ptr<PathFollower> path_follower_;
  std::unique_ptr<ActuatorInterface> actuator_interface_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr remaining_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_publisher_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time last_control_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace blimp_navigation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blimp_navigation::BlimpNavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
