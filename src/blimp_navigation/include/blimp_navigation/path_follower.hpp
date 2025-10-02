#ifndef BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_
#define BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_

#include <cstddef>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"

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
  double max_altitude_rate{0.3};                     // m/s - maximum climb/descent rate
  double velocity_filter_alpha{0.2};                 // EMA filter for velocity estimation
  double predictive_brake_time{2.0};                 // seconds - how far ahead to predict
  double obstacle_slowdown_distance{1.5};            // Start slowing at this distance (m)
  double obstacle_slowdown_min_scale{0.3};           // Minimum speed scale (30%)
  double obstacle_emergency_distance{0.35};          // Emergency stop distance (m)
  double obstacle_reverse_distance{0.33};            // Apply reverse thrust below this (camera min range)
  double emergency_reverse_min_duration{5.0};        // Minimum time to reverse (seconds)
  double obstacle_cone_half_angle{0.524};            // radians (30° = π/6) - detection cone

  double max_forward_norm{0.20};
  double pivot_heading_threshold{1.0471975512};          // 60 deg in radians
  double pivot_min_turn_command{0.08};
  double single_motor_threshold{0.1745329252};           // 10 deg in radians
  double max_reverse_norm{0.20};

  // Control mode arbitration (yaw vs altitude priority)
  double yaw_priority_threshold{0.349};                  // 20 deg - prioritize yaw above this
  double altitude_priority_threshold{0.5};               // 0.5m - prioritize altitude above this  
  double servo_blend_time{0.3};                          // seconds to transition between modes (fast response)
  double yaw_good_threshold{0.0873};                     // 5 deg - heading is "good enough"
  double altitude_good_threshold{0.2};                   // 0.2m - altitude is "good enough"

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
  double servo_blend_factor{0.0};  // 0=yaw_mode, 1=altitude_mode
  std::string control_priority{"yaw"};  // "yaw", "altitude", "blending"
  
  // Velocity estimation & predictive control stats
  double estimated_speed{0.0};      // m/s - forward velocity
  double stopping_distance{0.0};    // m - predicted distance to stop
  double physics_slowdown{1.0};     // 0-1 - turn slowdown factor
  double obstacle_distance{99.9};   // m - nearest obstacle in path

  bool has_active_path{false};
  bool goal_reached{false};
};

class PathFollower
{
public:
  explicit PathFollower(const PathFollowerConfig &config = PathFollowerConfig{});

  void setPath(const nav_msgs::msg::Path &path);
  void clearPath();
  void setDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_msg);

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
  
  // Control mode state
  double servo_blend_target_{0.0};  // Target blend factor
  double servo_blend_current_{0.0}; // Current blend factor (smoothed)
  
  // Velocity estimation state
  geometry_msgs::msg::Point last_position_{};
  double velocity_x_{0.0};  // m/s in x direction
  double velocity_y_{0.0};  // m/s in y direction
  double velocity_z_{0.0};  // m/s in z direction (vertical)
  double forward_speed_{0.0}; // m/s along heading
  bool velocity_initialized_{false};
  
  // Depth image for obstacle detection
  sensor_msgs::msg::Image::SharedPtr latest_depth_;

  // Emergency reverse timer
  std::chrono::steady_clock::time_point emergency_reverse_start_time_;
  bool in_emergency_reverse_{false};
  
  // Helper functions
  void updateVelocityEstimate(const geometry_msgs::msg::Point &current_pos, double dt);
  double predictStoppingDistance(double current_speed) const;
  double getObstacleProximityFactor(const geometry_msgs::msg::Point &current_pos) const;
};

}  // namespace blimp_navigation

#endif  // BLIMP_NAVIGATION__PATH_FOLLOWER_HPP_
