#ifndef BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_
#define BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "blimp_navigation/pid.hpp"

namespace blimp_navigation
{

struct PathFollowerConfig
{
  double lookahead_distance{1.8};
  double waypoint_tolerance{0.6};
  double goal_tolerance{0.8};

  double max_speed_command{0.12};
  double cruise_speed_command{0.10};
  double min_speed_command{0.03};

  double brake_distance{1.5};
  double brake_min_scale{0.05};

  double yaw_limit{0.12};
  double yaw_cross_track_gain{0.2};
  double cross_track_limit{0.4};
  double cross_track_slowdown_gain{1.5};
  double cross_track_slowdown_min_scale{0.35};
  double yaw_slowdown_threshold{0.2};
  double yaw_slowdown_min_scale{0.25};

  double altitude_limit{0.7};

  double max_forward_norm{0.20};
  double pivot_heading_threshold{1.0471975512};          // 60 deg in radians
  double pivot_min_turn_command{0.08};
  double single_motor_threshold{0.1745329252};           // 10 deg in radians
  double max_reverse_norm{0.20};

  PID::Gains yaw_gains{1.4, 0.0, 0.2, 0.1};
  PID::Gains altitude_gains{0.8, 0.0, 0.15, 0.2};
};

struct ControlCommand
{
  double left_motor_norm{0.0};
  double right_motor_norm{0.0};
  double left_servo_norm{0.0};
  double right_servo_norm{0.0};

  double desired_heading{0.0};
  double current_heading{0.0};
  double lookahead_distance{0.0};
  double goal_distance{0.0};
  geometry_msgs::msg::Point target_point{};
  double forward_command{0.0};
  std::string state{"idle"};
  double heading_error{0.0};
  double cross_track_error{0.0};
  double altitude_error{0.0};

  bool has_active_path{false};
  bool goal_reached{false};
};

class PathFollower
{
public:
  explicit PathFollower(const PathFollowerConfig &config = PathFollowerConfig{});

  void setPath(const nav_msgs::msg::Path &path);
  void clearPath();

  ControlCommand update(const geometry_msgs::msg::PoseStamped &pose,
                        double dt,
                        nav_msgs::msg::Path &remaining_path);

  const nav_msgs::msg::Path &fullPath() const;
  bool hasPath() const;

private:
  static double distanceSquared(const geometry_msgs::msg::Point &a,
                                const geometry_msgs::msg::Point &b);

  static double distance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b);

  double computeCrossTrack(const geometry_msgs::msg::Point &current) const;

  std::size_t findLookaheadIndex(const geometry_msgs::msg::Point &current) const;

  double normalizeAngle(double angle) const;

  PathFollowerConfig config_{};
  PID yaw_pid_;
  PID altitude_pid_;

  nav_msgs::msg::Path full_path_;
  std::size_t active_index_{0};
  bool goal_reached_{false};
};

}  // namespace blimp_navigation

#endif  // BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_
