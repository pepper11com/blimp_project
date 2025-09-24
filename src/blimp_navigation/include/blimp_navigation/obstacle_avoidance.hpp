#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <memory>
#include <chrono>

namespace blimp_navigation_cpp {

enum class AvoidanceState {
    CLEAR_PATH,
    OBSTACLE_DETECTED, 
    AVOIDING_LEFT,
    AVOIDING_RIGHT,
    EMERGENCY_STOP,
    BACKING_UP
};

struct BlimpDynamics {
    double length = 0.8;        // 80cm long - Small for home testing
    double width = 0.4;         // 40cm wide - Fits through doorways
    double height = 0.3;        // 30cm height clearance
    double max_speed = 2.0;     // m/s
    double turn_rate = 30.0;    // deg/s
    double response_time = 2.0; // seconds to respond to commands
};

struct AvoidanceParameters {
    double base_lookahead = 2.0;        // Reduced lookahead for home (m)
    double speed_factor = 1.5;          // Reduced additional lookahead per m/s
    double side_clearance = 1.0;        // Reduced side checking distance (m)
    double emergency_threshold = 0.8;   // Reduced distance for emergency stop (m)
    double hysteresis_factor = 1.2;     // Avoid oscillation
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance(rclcpp::Node* node, const BlimpDynamics& dynamics = BlimpDynamics{});
    
    // Main interface
    void update_pose(const geometry_msgs::msg::PoseStamped& pose);
    void update_goal(const geometry_msgs::msg::PoseStamped& goal);  
    void update_velocity(const nav_msgs::msg::Odometry& odom);
    void update_collision_result(bool collision_detected);
    
    // Collision detection
    std::vector<geometry_msgs::msg::Point> build_safety_envelope();
    std::vector<geometry_msgs::msg::Point> predict_trajectory(double time_horizon = 10.0);
    bool check_direction_clearance(double angle_offset); // -45° to +45°
    
    // Avoidance planning
    AvoidanceState plan_avoidance_maneuver();
    double calculate_optimal_turn_direction();
    double calculate_dynamic_lookahead() const;
    
    // State management
    AvoidanceState get_state() const { return current_state_; }
    bool is_obstacle_detected() const { return obstacle_detected_; }
    double get_recommended_turn_rate() const { return recommended_turn_rate_; }
    bool should_stop_forward() const { return stop_forward_; }
    
    // Debug/status
    std::string get_status_string() const;
    size_t get_check_points_count() const { return last_check_points_.size(); }

private:
    // Core state
    rclcpp::Node* node_;
    BlimpDynamics dynamics_;
    AvoidanceParameters params_;
    
    // Current state
    AvoidanceState current_state_{AvoidanceState::CLEAR_PATH};
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    nav_msgs::msg::Odometry current_velocity_;
    bool has_pose_{false};
    bool has_goal_{false};
    bool has_velocity_{false};
    
    // Collision state
    bool obstacle_detected_{false};
    std::chrono::steady_clock::time_point last_obstacle_time_;
    std::vector<geometry_msgs::msg::Point> last_check_points_;
    
    // Control outputs
    double recommended_turn_rate_{0.0};
    bool stop_forward_{false};
    
    // Internal methods
    double get_current_heading() const;
    double get_current_speed() const;
    geometry_msgs::msg::Point predict_position_at_time(double t) const;
    double normalize_angle(double angle) const;
    double calculate_environment_complexity() const;
};

} // namespace blimp_navigation_cpp