#include "blimp_navigation/path_follower.hpp"
#include "blimp_navigation/blimp_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <vector>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace blimp_navigation
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

PathFollower::PathFollower(const PathFollowerConfig &config)
  : config_(config),
    yaw_pid_(config.yaw_gains),
    altitude_pid_(config.altitude_gains)
{
  full_path_.poses.clear();
  full_path_.header.frame_id.clear();
}

void PathFollower::setPath(const nav_msgs::msg::Path &path)
{
  full_path_ = path;
  active_index_ = 0;
  goal_reached_ = full_path_.poses.empty();
  yaw_pid_.setGains(config_.yaw_gains);
  altitude_pid_.setGains(config_.altitude_gains);
  yaw_pid_.reset();
  altitude_pid_.reset();
}

void PathFollower::clearPath()
{
  full_path_.poses.clear();
  active_index_ = 0;
  goal_reached_ = true;
  yaw_pid_.reset();
  altitude_pid_.reset();
}

ControlCommand PathFollower::update(const geometry_msgs::msg::PoseStamped &pose,
                                    double dt,
                                    nav_msgs::msg::Path &remaining_path)
{
  ControlCommand command;

  remaining_path = nav_msgs::msg::Path{};
  remaining_path.header = full_path_.header;

  if (full_path_.poses.empty()) {
    command.has_active_path = false;
    return command;
  }

  const geometry_msgs::msg::Point &current = pose.pose.position;
  
  // Update velocity estimate from position changes
  updateVelocityEstimate(current, dt);

  // Advance waypoints when we are within tolerance
  while (active_index_ < full_path_.poses.size()) {
    const auto &waypoint = full_path_.poses[active_index_].pose.position;
    if (distance(current, waypoint) <= config_.waypoint_tolerance) {
      if (active_index_ + 1 < full_path_.poses.size()) {
        ++active_index_;
        continue;
      }
    }
    break;
  }

  const auto &goal_point = full_path_.poses.back().pose.position;
  const double goal_distance = distance(current, goal_point);
  if (goal_distance <= config_.goal_tolerance && active_index_ >= full_path_.poses.size() - 1) {
    goal_reached_ = true;
  }

  if (goal_reached_) {
    command.goal_reached = true;
    command.has_active_path = false;
    command.state = "goal_reached";
    remaining_path.poses.clear();
    return command;
  }

  command.has_active_path = true;

  const std::size_t target_index = findLookaheadIndex(current);
  const auto &target_pose = full_path_.poses[target_index].pose.position;
  const double current_yaw = tf2::getYaw(pose.pose.orientation);
  const double target_heading = std::atan2(target_pose.y - current.y, target_pose.x - current.x);
  const double heading_error = normalizeAngle(target_heading - current_yaw);
  const double abs_heading_error = std::abs(heading_error);

  const double cross_track_error = computeCrossTrack(current);
  const double yaw_error = normalizeAngle(heading_error + config_.yaw_cross_track_gain * cross_track_error);
  double yaw_correction = yaw_pid_.update(yaw_error, dt);
  yaw_correction = std::clamp(yaw_correction, -config_.yaw_limit, config_.yaw_limit);

  const double altitude_error = target_pose.z - current.z;
  double altitude_command = altitude_pid_.update(altitude_error, dt);
  altitude_command = std::clamp(altitude_command, -config_.altitude_limit, config_.altitude_limit);
  
  // Apply altitude rate limiting for smoother climb/descent
  // Limit vertical velocity to max_altitude_rate (e.g., 0.3 m/s)
  // This prevents servo slamming and respects helium dynamics
  if (config_.max_altitude_rate > 0.0 && std::abs(altitude_command) > config_.max_altitude_rate) {
    // Preserve the sign, but limit magnitude
    const double sign = (altitude_command > 0) ? 1.0 : -1.0;
    altitude_command = sign * config_.max_altitude_rate;
  }

  // ========== SMART CONTROL MODE ARBITRATION ==========
  // Decide whether to prioritize yaw or altitude based on errors
  const double abs_altitude_error = std::abs(altitude_error);
  
  // Determine priority based on which error is more critical
  bool yaw_critical = abs_heading_error > config_.yaw_priority_threshold;
  bool altitude_critical = abs_altitude_error > config_.altitude_priority_threshold;
  bool yaw_good = abs_heading_error < config_.yaw_good_threshold;
  bool altitude_good = abs_altitude_error < config_.altitude_good_threshold;
  
  // Priority logic:
  // 1. If yaw is critical AND altitude is good → full yaw mode (servos neutral)
  // 2. If altitude is critical AND yaw is good → full altitude mode (servos deflected)
  // 3. If both critical → prioritize yaw first (get pointed right, then worry about altitude)
  // 4. If both good → slight bias toward altitude to maintain height
  
  std::string priority_mode = "blending";
  if (yaw_critical && !altitude_critical) {
    servo_blend_target_ = 0.0;  // Full yaw mode (servos neutral)
    priority_mode = "yaw";
  } else if (altitude_critical && !yaw_critical) {
    servo_blend_target_ = 1.0;  // Full altitude mode (servos active)
    priority_mode = "altitude";
  } else if (yaw_critical && altitude_critical) {
    // Both critical: prioritize yaw fully (must point right direction first)
    servo_blend_target_ = 0.0;  // Full yaw mode - get heading right first
    priority_mode = "yaw_priority";
  } else if (yaw_good && altitude_good) {
    // Both good: maintain altitude with gentle adjustments
    servo_blend_target_ = 0.7;  // Favor altitude maintenance
    priority_mode = "maintain";
  } else {
    // One is borderline: blend based on relative errors
    const double yaw_ratio = abs_heading_error / std::max(config_.yaw_priority_threshold, 1e-3);
    const double alt_ratio = abs_altitude_error / std::max(config_.altitude_priority_threshold, 1e-3);
    const double total = yaw_ratio + alt_ratio;
    if (total > 1e-3) {
      servo_blend_target_ = alt_ratio / total;  // Weight toward larger error
    } else {
      servo_blend_target_ = 0.5;
    }
    priority_mode = "blending";
  }
  
  // Smooth transition using inertia (slow blend to avoid oscillation)
  const double blend_rate = 1.0 / config_.servo_blend_time;  // fraction per second
  const double max_blend_change = blend_rate * dt;
  const double blend_delta = servo_blend_target_ - servo_blend_current_;
  const double blend_step = std::clamp(blend_delta, -max_blend_change, max_blend_change);
  servo_blend_current_ += blend_step;
  servo_blend_current_ = std::clamp(servo_blend_current_, 0.0, 1.0);
  
  // Apply blending: 0.0 = neutral (yaw mode), 1.0 = full deflection (altitude mode)
  const double servo_norm = servo_blend_current_ * 
                           std::clamp(altitude_command / std::max(config_.altitude_limit, 1e-3), -1.0, 1.0);

  const double lookahead_distance = distance(current, target_pose);
  command.lookahead_distance = lookahead_distance;
  command.goal_distance = goal_distance;
  command.target_point = target_pose;
  command.current_heading = current_yaw;
  command.desired_heading = target_heading;
  command.heading_error = heading_error;
  command.cross_track_error = cross_track_error;
  command.altitude_error = altitude_error;
  command.servo_blend_factor = servo_blend_current_;
  command.control_priority = priority_mode;

  bool pivot_mode = abs_heading_error >= config_.pivot_heading_threshold;
  bool single_motor_mode = abs_heading_error <= config_.single_motor_threshold;

  double forward = config_.cruise_speed_command;

  // Proportional altitude boost: more power when far from target altitude, less when close
  // This gives aggressive climb/descent when needed, but gentle approach to avoid overshoot
  if (abs_altitude_error > 0.1) {  // Only boost if error is significant
    // Scale boost from 0 at 0.1m error up to max at 1.0m+ error
    const double error_normalized = std::min(1.0, (abs_altitude_error - 0.1) / 0.9);  // 0.1-1.0m → 0-1
    const double max_altitude_boost = 0.20;  // Maximum boost when far away (increased from 0.10)
    const double altitude_boost = max_altitude_boost * error_normalized * error_normalized;  // Quadratic for smooth ramp
    forward += altitude_boost;
  }

  // Predictive braking: use actual velocity to determine when to start slowing
  // This is smarter than fixed brake_distance because it accounts for current speed
  if (config_.brake_distance > 1e-3 && goal_distance < config_.brake_distance) {
    // Traditional distance-based braking
    const double ratio = std::clamp(goal_distance / config_.brake_distance, 0.0, 1.0);
    forward *= config_.brake_min_scale + (1.0 - config_.brake_min_scale) * ratio;
  } else if (velocity_initialized_ && forward_speed_ > 0.1) {
    // Velocity-based predictive braking
    const double stopping_dist = predictStoppingDistance(forward_speed_);
    if (goal_distance < stopping_dist * 1.5) { // Start slowing at 1.5x stopping distance
      const double brake_ratio = goal_distance / (stopping_dist * 1.5);
      const double velocity_brake_scale = std::max(0.3, brake_ratio); // Min 30% speed
      forward *= velocity_brake_scale;
    }
  }

  // Only apply cross-track slowdown when NOT in altitude priority mode
  if (config_.cross_track_limit > 1e-3 && servo_blend_current_ < 0.8) {
    const double cross_abs = std::min(std::abs(cross_track_error), config_.cross_track_limit);
    double cross_scale = 1.0 - config_.cross_track_slowdown_gain * (cross_abs / config_.cross_track_limit);
    cross_scale = std::clamp(cross_scale, config_.cross_track_slowdown_min_scale, 1.0);
    forward *= cross_scale;
  }

  // Physics-based turn slowdown: reduce speed for tight turns to prevent lateral drift
  // Only apply when NOT in altitude priority mode (need speed for servo airflow when climbing)
  if (servo_blend_current_ < 0.8) {
    // Use actual measured velocity if available, otherwise estimate from command
    const double current_velocity = velocity_initialized_ ? forward_speed_ : (forward * 0.5);
    const double physics_turn_scale = BlimpDynamics::turnSlowdownFactor(heading_error, current_velocity);
    forward *= physics_turn_scale;
  }
  
  // Obstacle proximity slowdown (placeholder for future OctoMap integration)
  const double obstacle_scale = getObstacleProximityFactor(current);
  forward *= obstacle_scale;

  forward = std::clamp(forward, 0.0, config_.max_speed_command);
  if (forward > 0.0) {
    forward = std::max(forward, config_.min_speed_command);
  }

  double left_motor = 0.0;
  double right_motor = 0.0;

  std::vector<std::string> state_tags;

  if (pivot_mode) {
    state_tags.push_back("pivot");
    state_tags.push_back("yaw_align");
    if (goal_distance < config_.brake_distance) {
      state_tags.push_back("brake");
    }
    const double pivot_mag = std::clamp(
      std::max(config_.pivot_min_turn_command, std::abs(yaw_correction)),
      0.0, config_.max_forward_norm);
    const double sign = heading_error >= 0.0 ? 1.0 : -1.0;
    left_motor = sign * pivot_mag;
    right_motor = -sign * pivot_mag;
    forward = 0.0;
  } else {
    left_motor = forward;
    right_motor = forward;

    // Scale yaw authority based on servo deflection
    // Quadratic falloff: 100% effective at blend=0, 0% at blend=1 (prevents roll in altitude mode)
    const double yaw_effectiveness = (1.0 - servo_blend_current_) * (1.0 - servo_blend_current_);
    const double scaled_yaw = yaw_correction * yaw_effectiveness;
    
    if (single_motor_mode) {
      if (scaled_yaw >= 0.0) {
        right_motor += scaled_yaw;
      } else {
        left_motor -= scaled_yaw;
      }
      state_tags.push_back(abs_heading_error < config_.single_motor_threshold * 0.5 ? "trim" : "nudge");
    } else {
      right_motor += scaled_yaw;
      left_motor -= scaled_yaw;
      state_tags.push_back("differential");
    }

    if (goal_distance < config_.brake_distance) {
      state_tags.push_back("brake");
    } else {
      state_tags.push_back("cruise");
    }

    if (abs_heading_error > config_.yaw_slowdown_threshold) {
      state_tags.push_back("yaw_align");
    }
  }

  left_motor = std::clamp(left_motor, -config_.max_reverse_norm, config_.max_forward_norm);
  right_motor = std::clamp(right_motor, -config_.max_reverse_norm, config_.max_forward_norm);

  // Apply deadband to servo command to reduce jitter from small PID oscillations
  double filtered_servo = servo_norm;
  const double servo_deadband = 0.05;  // 5% deadband (50µs with 1000µs gain)
  if (std::abs(servo_norm) < servo_deadband) {
    filtered_servo = 0.0;  // Snap to neutral for tiny movements
  }

  command.forward_command = forward;
  command.left_motor_norm = left_motor;
  command.right_motor_norm = right_motor;
  command.left_servo_norm = filtered_servo;
  command.right_servo_norm = filtered_servo;

  if (state_tags.empty()) {
    state_tags.push_back("cruise");
  }

  std::ostringstream state_builder;
  for (std::size_t i = 0; i < state_tags.size(); ++i) {
    if (i != 0) {
      state_builder << '|';
    }
    state_builder << state_tags[i];
  }
  command.state = state_builder.str();
  
  // Populate velocity estimation & predictive control stats for display
  command.estimated_speed = velocity_initialized_ ? forward_speed_ : 0.0;
  command.stopping_distance = velocity_initialized_ ? predictStoppingDistance(forward_speed_) : 0.0;
  command.physics_slowdown = BlimpDynamics::turnSlowdownFactor(heading_error, 
                                                                velocity_initialized_ ? forward_speed_ : 0.0);

  remaining_path.header = full_path_.header;
  const auto offset = static_cast<std::ptrdiff_t>(active_index_);
  if (offset < static_cast<std::ptrdiff_t>(full_path_.poses.size())) {
    remaining_path.poses.assign(full_path_.poses.begin() + offset, full_path_.poses.end());
  }

  return command;
}

const nav_msgs::msg::Path &PathFollower::fullPath() const
{
  return full_path_;
}

bool PathFollower::hasPath() const
{
  return !full_path_.poses.empty() && !goal_reached_;
}

double PathFollower::distanceSquared(const geometry_msgs::msg::Point &a,
                                      const geometry_msgs::msg::Point &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

double PathFollower::distance(const geometry_msgs::msg::Point &a,
                               const geometry_msgs::msg::Point &b)
{
  return std::sqrt(distanceSquared(a, b));
}

double PathFollower::computeCrossTrack(const geometry_msgs::msg::Point &current) const
{
  if (full_path_.poses.size() < 2 || active_index_ == 0) {
    return 0.0;
  }

  const auto &prev_point = full_path_.poses[active_index_ - 1].pose.position;
  const auto &next_point = full_path_.poses[active_index_].pose.position;

  tf2::Vector3 segment(next_point.x - prev_point.x,
                       next_point.y - prev_point.y,
                       0.0);
  tf2::Vector3 to_current(current.x - prev_point.x,
                          current.y - prev_point.y,
                          0.0);

  const double segment_length = segment.length();
  if (segment_length < 1e-6) {
    return 0.0;
  }

  const double cross = (segment.x() * to_current.y() - segment.y() * to_current.x()) / segment_length;
  return cross;
}

std::size_t PathFollower::findLookaheadIndex(const geometry_msgs::msg::Point &current) const
{
  if (full_path_.poses.empty()) {
    return 0;
  }

  const double lookahead_sq = config_.lookahead_distance * config_.lookahead_distance;
  std::size_t index = active_index_;
  double best_distance_sq = std::numeric_limits<double>::max();
  std::size_t best_index = full_path_.poses.size() - 1;

  for (std::size_t i = active_index_; i < full_path_.poses.size(); ++i) {
    const double dist_sq = distanceSquared(current, full_path_.poses[i].pose.position);
    if (dist_sq >= lookahead_sq) {
      return i;
    }
    if (dist_sq < best_distance_sq) {
      best_distance_sq = dist_sq;
      best_index = i;
    }
  }

  return best_index;
}

double PathFollower::normalizeAngle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void PathFollower::updateVelocityEstimate(const geometry_msgs::msg::Point &current_pos, double dt)
{
  if (!velocity_initialized_) {
    // First call - initialize
    last_position_ = current_pos;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_z_ = 0.0;
    forward_speed_ = 0.0;
    velocity_initialized_ = true;
    return;
  }
  
  if (dt < 1e-6) {
    return; // Avoid division by zero
  }
  
  // Calculate instantaneous velocities
  const double inst_vx = (current_pos.x - last_position_.x) / dt;
  const double inst_vy = (current_pos.y - last_position_.y) / dt;
  const double inst_vz = (current_pos.z - last_position_.z) / dt;
  
  // Apply exponential moving average filter to smooth noise
  const double alpha = config_.velocity_filter_alpha;
  velocity_x_ = alpha * inst_vx + (1.0 - alpha) * velocity_x_;
  velocity_y_ = alpha * inst_vy + (1.0 - alpha) * velocity_y_;
  velocity_z_ = alpha * inst_vz + (1.0 - alpha) * velocity_z_;
  
  // Calculate forward speed (magnitude in XY plane)
  forward_speed_ = std::sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_);
  
  // Update last position
  last_position_ = current_pos;
}

double PathFollower::predictStoppingDistance(double current_speed) const
{
  // With T/W ratio of 1.0, we can decelerate at ~0.8 m/s² (accounting for drag)
  // Using kinematic equation: d = v² / (2a)
  const double max_deceleration = 0.8; // m/s²
  if (max_deceleration < 1e-6) {
    return 0.0;
  }
  
  const double stopping_distance = (current_speed * current_speed) / (2.0 * max_deceleration);
  
  // Add reaction time distance
  const double reaction_distance = current_speed * config_.predictive_brake_time * 0.3;
  
  return stopping_distance + reaction_distance;
}

double PathFollower::getObstacleProximityFactor(const geometry_msgs::msg::Point &current_pos) const
{
  // TODO: Query OctoMap for nearest obstacle distance
  // For now, return 1.0 (no slowdown) as placeholder
  // This will be integrated with OctoMap in the future
  
  // When implemented, this should return:
  // - 1.0 if distance > obstacle_slowdown_distance
  // - Linear scale from 1.0 to obstacle_slowdown_min_scale as distance decreases
  // - obstacle_slowdown_min_scale if very close
  
  return 1.0; // No obstacle slowdown yet (needs OctoMap integration)
}

}  // namespace blimp_navigation
