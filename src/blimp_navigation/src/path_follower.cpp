#include "blimp_navigation/path_follower.hpp"

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
  const double servo_norm = std::clamp(altitude_command / std::max(config_.altitude_limit, 1e-3), -1.0, 1.0);

  const double lookahead_distance = distance(current, target_pose);
  command.lookahead_distance = lookahead_distance;
  command.goal_distance = goal_distance;
  command.target_point = target_pose;
  command.current_heading = current_yaw;
  command.desired_heading = target_heading;
  command.heading_error = heading_error;
  command.cross_track_error = cross_track_error;
  command.altitude_error = altitude_error;

  bool pivot_mode = abs_heading_error >= config_.pivot_heading_threshold;
  bool single_motor_mode = abs_heading_error <= config_.single_motor_threshold;

  double forward = config_.cruise_speed_command;

  if (config_.brake_distance > 1e-3 && goal_distance < config_.brake_distance) {
    const double ratio = std::clamp(goal_distance / config_.brake_distance, 0.0, 1.0);
    forward *= config_.brake_min_scale + (1.0 - config_.brake_min_scale) * ratio;
  }

  if (config_.cross_track_limit > 1e-3) {
    const double cross_abs = std::min(std::abs(cross_track_error), config_.cross_track_limit);
    double cross_scale = 1.0 - config_.cross_track_slowdown_gain * (cross_abs / config_.cross_track_limit);
    cross_scale = std::clamp(cross_scale, config_.cross_track_slowdown_min_scale, 1.0);
    forward *= cross_scale;
  }

  if (abs_heading_error > config_.yaw_slowdown_threshold) {
    const double excess = abs_heading_error - config_.yaw_slowdown_threshold;
    const double range = std::max(kPi - config_.yaw_slowdown_threshold, 1e-3);
    const double normalized = std::clamp(excess / range, 0.0, 1.0);
    const double yaw_scale = 1.0 - (1.0 - config_.yaw_slowdown_min_scale) * normalized;
    forward *= yaw_scale;
  }

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

    if (single_motor_mode) {
      if (yaw_correction >= 0.0) {
        right_motor += yaw_correction;
      } else {
        left_motor -= yaw_correction;
      }
      state_tags.push_back(abs_heading_error < config_.single_motor_threshold * 0.5 ? "trim" : "nudge");
    } else {
      right_motor += yaw_correction;
      left_motor -= yaw_correction;
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

  command.forward_command = forward;
  command.left_motor_norm = left_motor;
  command.right_motor_norm = right_motor;
  command.left_servo_norm = servo_norm;
  command.right_servo_norm = servo_norm;

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

}  // namespace blimp_navigation
