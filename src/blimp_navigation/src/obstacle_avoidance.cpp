#include "blimp_navigation/obstacle_avoidance.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <sstream>

namespace blimp_navigation_cpp {

ObstacleAvoidance::ObstacleAvoidance(rclcpp::Node* node, const BlimpDynamics& dynamics)
    : node_(node), dynamics_(dynamics)
{
    RCLCPP_INFO(node_->get_logger(), 
                "ObstacleAvoidance initialized for %.1fm×%.1fm blimp", 
                dynamics_.length, dynamics_.width);
}

void ObstacleAvoidance::update_pose(const geometry_msgs::msg::PoseStamped& pose)
{
    current_pose_ = pose;
    has_pose_ = true;
}

void ObstacleAvoidance::update_goal(const geometry_msgs::msg::PoseStamped& goal)
{
    goal_pose_ = goal;
    has_goal_ = true;
}

void ObstacleAvoidance::update_velocity(const nav_msgs::msg::Odometry& odom)
{
    current_velocity_ = odom;
    has_velocity_ = true;
}

void ObstacleAvoidance::update_collision_result(bool collision_detected)
{
    obstacle_detected_ = collision_detected;
    if (collision_detected) {
        last_obstacle_time_ = std::chrono::steady_clock::now();
    }
    
    // Update state machine
    current_state_ = plan_avoidance_maneuver();
    
    // Calculate recommended turn rate
    if (current_state_ == AvoidanceState::AVOIDING_LEFT) {
        recommended_turn_rate_ = -dynamics_.turn_rate;  // Turn left
    } else if (current_state_ == AvoidanceState::AVOIDING_RIGHT) {
        recommended_turn_rate_ = dynamics_.turn_rate;   // Turn right
    } else if (current_state_ == AvoidanceState::EMERGENCY_STOP) {
        recommended_turn_rate_ = 0.0;
        stop_forward_ = true;
    } else {
        recommended_turn_rate_ = 0.0;
        stop_forward_ = false;
    }
}

std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::build_safety_envelope()
{
    std::vector<geometry_msgs::msg::Point> points;
    
    if (!has_pose_) {
        return points;
    }
    
    auto pos = current_pose_.pose.position;
    double heading = get_current_heading();
    double forward_x = std::cos(heading);
    double forward_y = std::sin(heading);
    double right_x = std::cos(heading - M_PI/2);
    double right_y = std::sin(heading - M_PI/2);
    
    double lookahead = calculate_dynamic_lookahead();
    
    // ULTRA-SIMPLE: With 20cm voxels, we only need minimal points
    
    // Zone 1: Just check the 4 corners of the blimp
    std::vector<std::pair<double, double>> corners = {
        {-dynamics_.length/2, -dynamics_.width/2},  // Back-left
        {-dynamics_.length/2, dynamics_.width/2},   // Back-right  
        {dynamics_.length/2, -dynamics_.width/2},   // Front-left
        {dynamics_.length/2, dynamics_.width/2}     // Front-right
    };
    
    for (auto& corner : corners) {
        geometry_msgs::msg::Point p;
        p.x = pos.x + forward_x * corner.first + right_x * corner.second;
        p.y = pos.y + forward_y * corner.first + right_y * corner.second;
        p.z = pos.z;
        points.push_back(p);
    }
    
    // Zone 2: Just check a few forward points (since voxels are big)
    auto trajectory = predict_trajectory(2.0);  // Only 2 seconds ahead
    for (const auto& traj_point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = traj_point.x;
        p.y = traj_point.y;
        p.z = traj_point.z;
        points.push_back(p);
    }
    
    last_check_points_ = points;
    
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Built safety envelope: %zu points (SIMPLE approach with 20cm voxels), lookahead=%.1fm", 
                 points.size(), lookahead);
    
    return points;
}

std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::predict_trajectory(double time_horizon)
{
    std::vector<geometry_msgs::msg::Point> trajectory;
    
    if (!has_pose_ || !has_velocity_) {
        return trajectory;
    }
    
    // Balanced trajectory prediction frequency
    double dt = 1.0; // 1 second intervals for good coverage
    for (double t = dt; t <= time_horizon; t += dt) {
        auto predicted_pos = predict_position_at_time(t);
        trajectory.push_back(predicted_pos);
    }
    
    return trajectory;
}

geometry_msgs::msg::Point ObstacleAvoidance::predict_position_at_time(double t) const
{
    geometry_msgs::msg::Point predicted;
    auto pos = current_pose_.pose.position;
    
    if (!has_velocity_) {
        // Fallback: assume moving forward at current heading
        double heading = get_current_heading();
        double speed = get_current_speed();
        predicted.x = pos.x + std::cos(heading) * speed * t;
        predicted.y = pos.y + std::sin(heading) * speed * t;
        predicted.z = pos.z;
    } else {
        // Use actual velocity from odometry
        auto vel = current_velocity_.twist.twist.linear;
        predicted.x = pos.x + vel.x * t;
        predicted.y = pos.y + vel.y * t;
        predicted.z = pos.z + vel.z * t;
    }
    
    return predicted;
}

bool ObstacleAvoidance::check_direction_clearance(double angle_offset)
{
    // TODO: Implement directional clearance checking
    // This would send a focused collision check in a specific direction
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Checking clearance at angle offset: %.1f°", 
                 angle_offset * 180.0 / M_PI);
    return true; // Placeholder
}

AvoidanceState ObstacleAvoidance::plan_avoidance_maneuver()
{
    if (!obstacle_detected_) {
        return AvoidanceState::CLEAR_PATH;
    }
    
    // Check both directions for clearance
    bool left_clear = check_direction_clearance(-M_PI/4);   // 45° left
    bool right_clear = check_direction_clearance(M_PI/4);   // 45° right
    
    if (!left_clear && !right_clear) {
        // Emergency situation
        RCLCPP_WARN(node_->get_logger(), "No clear escape route - emergency stop");
        return AvoidanceState::EMERGENCY_STOP;
    }
    
    if (left_clear && right_clear) {
        // Both sides clear - choose direction toward goal
        double goal_direction = calculate_optimal_turn_direction();
        return (goal_direction < 0) ? AvoidanceState::AVOIDING_LEFT : AvoidanceState::AVOIDING_RIGHT;
    }
    
    // One side clear
    return left_clear ? AvoidanceState::AVOIDING_LEFT : AvoidanceState::AVOIDING_RIGHT;
}

double ObstacleAvoidance::calculate_optimal_turn_direction()
{
    if (!has_pose_ || !has_goal_) {
        return -1.0; // Default to left
    }
    
    auto pos = current_pose_.pose.position;
    auto goal = goal_pose_.pose.position;
    double current_heading = get_current_heading();
    
    // Calculate angle to goal
    double angle_to_goal = std::atan2(goal.y - pos.y, goal.x - pos.x);
    double heading_error = normalize_angle(angle_to_goal - current_heading);
    
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Goal direction analysis: heading_error=%.1f°", 
                 heading_error * 180.0 / M_PI);
    
    return (heading_error > 0) ? 1.0 : -1.0; // Positive = right, negative = left
}

double ObstacleAvoidance::calculate_dynamic_lookahead() const
{
    double current_speed = get_current_speed();
    double base = params_.base_lookahead;
    double speed_component = current_speed * params_.speed_factor;
    double complexity_component = calculate_environment_complexity();
    
    return base + speed_component + complexity_component;
}

double ObstacleAvoidance::get_current_heading() const
{
    if (!has_pose_) return 0.0;
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y, 
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double ObstacleAvoidance::get_current_speed() const
{
    if (!has_velocity_) {
        // Estimate from goal distance
        if (!has_pose_ || !has_goal_) return 0.5; // Default speed
        
        auto pos = current_pose_.pose.position;
        auto goal = goal_pose_.pose.position;
        double distance = std::sqrt(
            std::pow(goal.x - pos.x, 2) + std::pow(goal.y - pos.y, 2)
        );
        return std::min(dynamics_.max_speed, distance * 0.1);
    }
    
    auto vel = current_velocity_.twist.twist.linear;
    return std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
}

double ObstacleAvoidance::normalize_angle(double angle) const
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double ObstacleAvoidance::calculate_environment_complexity() const
{
    // TODO: Analyze environment complexity based on obstacle density
    // For now, return base complexity
    return 0.0;
}

std::string ObstacleAvoidance::get_status_string() const
{
    std::stringstream ss;
    
    const char* state_names[] = {
        "CLEAR_PATH", "OBSTACLE_DETECTED", "AVOIDING_LEFT", 
        "AVOIDING_RIGHT", "EMERGENCY_STOP", "BACKING_UP"
    };
    
    ss << "State: " << state_names[static_cast<int>(current_state_)]
       << " | Points: " << last_check_points_.size()
       << " | Speed: " << std::fixed << std::setprecision(1) << get_current_speed() << "m/s"
       << " | Lookahead: " << std::setprecision(1) << calculate_dynamic_lookahead() << "m";
    
    return ss.str();
}

} // namespace blimp_navigation_cpp