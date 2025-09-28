#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "blimp_navigation/srv/check_collision.hpp"
#include "blimp_navigation/srv/plan_path.hpp"
#include "blimp_navigation/pid.hpp"
#include "blimp_navigation/blimp_controller.hpp"
#include "blimp_navigation/obstacle_avoidance.hpp"
#include "nav_msgs/msg/path.hpp"

#include <cmath>
#include <algorithm>
#include <memory>
#include <future>   
#include <sstream>
#include <chrono>
#include <iomanip>
#include <array>

using namespace std::chrono_literals;

class BlimpNavigatorNode : public rclcpp::Node
{
public:
  BlimpNavigatorNode() : Node("blimp_navigator_node")
  {
    // Debug switch
    this->declare_parameter<bool>("debug_mode", true);
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    
    // Parameters for tuning
    this->declare_parameter<double>("altitude_kp", 80.0);   // Reduced for smoother control
    this->declare_parameter<double>("altitude_ki", 5.0);    // Reduced
    this->declare_parameter<double>("altitude_kd", 10.0);   // Reduced
    this->declare_parameter<double>("heading_kp", 0.8);     // Increased for better response
    this->declare_parameter<double>("heading_ki", 0.01);
    this->declare_parameter<double>("heading_kd", 0.05);
    this->declare_parameter<double>("forward_kp", 30.0);    // Increased for more power
    this->declare_parameter<double>("forward_ki", 3.0);
    this->declare_parameter<double>("forward_kd", 8.0);
    
    // TF2 listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Hardware controller
    try {
      controller_ = std::make_unique<blimp_navigation_cpp::BlimpController>("/dev/ttyAMA0", 115200);
      RCLCPP_INFO(this->get_logger(), "Successfully connected to the flight controller");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to flight controller: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // PID controllers with better tuning
    altitude_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("altitude_kp").as_double(), this->get_parameter("altitude_ki").as_double(), this->get_parameter("altitude_kd").as_double(), 0.0, -400.0, 400.0  // Smooth servo range ±400us
    );
    heading_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("heading_kp").as_double(), this->get_parameter("heading_ki").as_double(), this->get_parameter("heading_kd").as_double(), 0.0, -50.0, 50.0  // More turn authority
    );
    forward_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("forward_kp").as_double(), this->get_parameter("forward_ki").as_double(), this->get_parameter("forward_kd").as_double(), 0.0, 0.0, 45.0  // Trimmed max forward power for smoother bursts
    );
    RCLCPP_INFO(this->get_logger(), "PID controllers initialized");

    // Obstacle avoidance system
    obstacle_avoidance_ = std::make_unique<blimp_navigation_cpp::ObstacleAvoidance>(this);
    RCLCPP_INFO(this->get_logger(), "Obstacle avoidance system initialized");

    // ROS2 communications
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&BlimpNavigatorNode::goal_callback, this, std::placeholders::_1));
    
    // Subscribe to OMPL planned paths
    ompl_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/blimp/planned_path", 10, std::bind(&BlimpNavigatorNode::ompl_path_callback, this, std::placeholders::_1));
    
    collision_client_ = this->create_client<blimp_navigation::srv::CheckCollision>("/check_collision");
    path_planning_client_ = this->create_client<blimp_navigation::srv::PlanPath>("/plan_path");

    // Wait for collision service to be available
    if (!collision_client_->wait_for_service(5s)) {
      RCLCPP_WARN(this->get_logger(), "Collision check service not available after waiting for 5s");
    } else {
      RCLCPP_INFO(this->get_logger(), "Collision check service is available");
    }

    // Wait for collision service to become available
    if (!collision_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "Collision check service not available after 5 seconds!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Collision check service is ready");
    }

    if (debug_mode_) {
      status_pub_ = this->create_publisher<std_msgs::msg::String>("/blimp_status", 10);
      last_debug_time_ = this->get_clock()->now();
    }

    // Path visualization publishers
    full_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/blimp/planned_path_full", 10);
    remaining_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/blimp/planned_path_remaining", 10);

    // Main loop
    timer_ = this->create_wall_timer(100ms, std::bind(&BlimpNavigatorNode::navigate_loop, this));
    RCLCPP_INFO(this->get_logger(), "Blimp navigator node started");
  }

private:
  struct AltitudeCommand
  {
    double servo_offset = 0.0;
    double error = 0.0;
    std::string state = "HOLD";
    bool prioritize_altitude = false;
  };

  struct MotorMix
  {
    double left_pct = 0.0;
    double right_pct = 0.0;
    std::string turning_mode;
    double abs_heading_error = 0.0;
  };

  static double normalize_angle(double angle_rad);
  AltitudeCommand compute_altitude_command(double target_z, double current_z);
  void apply_navigation_strategies(double heading_error_deg, double distance_to_target_2d, double altitude_error,
                                   bool obstacle_detected, double recommended_turn,
                                   double & forward_output, double & turn_output);
  MotorMix compute_motor_mix(double heading_error_deg, double forward_output, double turn_output);
  void reset_motor_ramp();
  void publish_debug_status(
    const geometry_msgs::msg::Point & pos,
    const geometry_msgs::msg::Point & target_pos,
    const geometry_msgs::msg::Point & final_goal_pos,
    double distance_to_target_2d,
    double distance_to_target_3d,
    double distance_to_final_goal,
    double heading_error_deg,
    const AltitudeCommand & altitude_cmd,
    double forward_output,
    double turn_output,
    const MotorMix & mix,
    bool obstacle_detected);

  static constexpr int SERVO_NEUTRAL_US = 1500;

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    ompl_path_valid_ = false;  // Wait for OMPL path
    ompl_path_.poses.clear();
    current_waypoint_index_ = 0;
    
    // Update obstacle avoidance system
    obstacle_avoidance_->update_goal(goal_pose_);
    
    RCLCPP_INFO(this->get_logger(), "🎯 New goal received: Pos(x=%.2f, y=%.2f, z=%.2f)", 
      goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for OMPL path...");
    
    // Set timeout for OMPL path planning (10 seconds)
    ompl_path_timeout_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(10.0);
    
    altitude_pid_->set_setpoint(goal_pose_.pose.position.z);
  }

  void ompl_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "❌ Received empty OMPL path!");
      ompl_path_valid_ = false;
      return;
    }
    
    // Check if this is the same path (just republished) or a new path
    bool is_new_path = !ompl_path_valid_ || 
                       ompl_path_.poses.size() != msg->poses.size() ||
                       (ompl_path_.poses.size() > 0 && msg->poses.size() > 0 &&
                        std::abs(ompl_path_.poses[0].pose.position.x - msg->poses[0].pose.position.x) > 0.01);
    
    if (is_new_path) {
      // This is a genuinely new path - reset everything
      ompl_path_ = *msg;
      ompl_path_full_ = *msg;  // Store complete original path
      ompl_path_valid_ = true;
      current_waypoint_index_ = 0;
      
      RCLCPP_INFO(this->get_logger(), "🛤️ NEW OMPL path received with %zu waypoints", ompl_path_.poses.size());
      RCLCPP_INFO(this->get_logger(), "🚁 Starting path following navigation!");
      
      // Set altitude from the path
      if (!ompl_path_.poses.empty()) {
        altitude_pid_->set_setpoint(ompl_path_.poses.back().pose.position.z);
      }
      
      // Publish full path immediately
      publish_path_visualization();
    } else {
      // Same path republished - just update timestamp, don't reset progress
      RCLCPP_DEBUG(this->get_logger(), "📡 Path republish ignored (same path, keeping progress at waypoint %zu/%zu)", 
        current_waypoint_index_ + 1, ompl_path_.poses.size());
    }
  }

  void plan_path_to_goal()
  {
    if (!path_planning_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Path planning service not available");
      return;
    }

    auto request = std::make_shared<blimp_navigation::srv::PlanPath::Request>();
    request->start = current_pose_.pose.position;
    request->goal = goal_pose_.pose.position;

    auto result_future = path_planning_client_->async_send_request(request);
    
    // Waiting for result with short timeout to avoid blocking
    if (result_future.wait_for(200ms) == std::future_status::ready) {
      auto response = result_future.get();
      if (response->success) {
        current_path_ = response->waypoints;
        current_waypoint_index_ = 0;
        path_valid_ = true;
        RCLCPP_INFO(this->get_logger(), "Path planned with %zu waypoints", current_path_.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "Path planning failed: %s", response->message.c_str());
        path_valid_ = false;
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Path planning request timed out");
    }
  }

  geometry_msgs::msg::Point get_current_target()
  {
    // Check if OMPL path timed out
    if (has_goal_ && !ompl_path_valid_ && this->get_clock()->now() > ompl_path_timeout_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "⚠️ OMPL path timeout! Using direct navigation.");
    }
    
    // Use OMPL path if available, otherwise fall back to direct goal navigation
    if (!ompl_path_valid_ || ompl_path_.poses.empty()) {
      return goal_pose_.pose.position;  // Direct navigation fallback
    }

    // Check if we reached the current waypoint
    auto& current_waypoint_pose = ompl_path_.poses[current_waypoint_index_];
    auto& current_pos = current_pose_.pose.position;
    
    double distance_to_waypoint = std::sqrt(
      std::pow(current_waypoint_pose.pose.position.x - current_pos.x, 2) +
      std::pow(current_waypoint_pose.pose.position.y - current_pos.y, 2) +
      std::pow(current_waypoint_pose.pose.position.z - current_pos.z, 2)
    );

    // If we are close to the current waypoint, advance to the next one
    if (distance_to_waypoint < 0.5) {  // 50cm threshold for blimp
      size_t old_index = current_waypoint_index_;
      current_waypoint_index_++;
      if (current_waypoint_index_ >= ompl_path_.poses.size()) {
        current_waypoint_index_ = ompl_path_.poses.size() - 1;  // Stay at final waypoint
      } else {
        RCLCPP_INFO(this->get_logger(), "🎯 Advanced to waypoint %zu/%zu (dist: %.2fm)", 
          current_waypoint_index_ + 1, ompl_path_.poses.size(), distance_to_waypoint);
        
        // Publish updated path visualization (remaining path gets shorter!)
        publish_path_visualization();
      }
    }

    return ompl_path_.poses[current_waypoint_index_].pose.position;
  }

  geometry_msgs::msg::Point get_path_tracking_target()
  {
    if (!ompl_path_valid_ || ompl_path_.poses.empty()) {
      return get_current_target();  // Fallback to waypoint following
    }

    auto& current_pos = current_pose_.pose.position;
    
    // Find closest point on the path (magnetic attraction to path)
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = current_waypoint_index_;
    
    // Look ahead on the path to find the closest point
    size_t search_start = current_waypoint_index_;
    size_t search_end = std::min(current_waypoint_index_ + 10, ompl_path_.poses.size()); // Look 10 waypoints ahead
    
    for (size_t i = search_start; i < search_end; ++i) {
      auto& path_pos = ompl_path_.poses[i].pose.position;
      double distance = std::sqrt(
        std::pow(path_pos.x - current_pos.x, 2) +
        std::pow(path_pos.y - current_pos.y, 2) +
        std::pow(path_pos.z - current_pos.z, 2)
      );
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_index = i;
      }
    }
    
    // Use the closest point or slightly ahead for forward motion
    size_t target_index = std::min(closest_index + 2, ompl_path_.poses.size() - 1);
    
    RCLCPP_DEBUG(this->get_logger(), "🧲 Path tracking: closest=%zu, target=%zu, distance=%.2fm", 
      closest_index, target_index, min_distance);
    
    return ompl_path_.poses[target_index].pose.position;
  }

  void publish_path_visualization()
  {
    if (!ompl_path_valid_ || ompl_path_full_.poses.empty()) {
      return;
    }
    
    // Publish full path (never changes)
    auto full_path = ompl_path_full_;
    full_path.header.stamp = this->get_clock()->now();
    full_path_pub_->publish(full_path);
    
    // Create remaining path (from current waypoint to end)
    nav_msgs::msg::Path remaining_path;
    remaining_path.header = ompl_path_full_.header;
    remaining_path.header.stamp = this->get_clock()->now();
    
    // Add all waypoints from current position onwards
    for (size_t i = current_waypoint_index_; i < ompl_path_.poses.size(); ++i) {
      remaining_path.poses.push_back(ompl_path_.poses[i]);
    }
    
    remaining_path_pub_->publish(remaining_path);
    
    RCLCPP_DEBUG(this->get_logger(), "📊 Published paths: Full=%zu, Remaining=%zu waypoints", 
      full_path.poses.size(), remaining_path.poses.size());
  }

  bool get_current_pose_from_tf()
  {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "camera_link", tf2::TimePointZero);
      current_pose_.header = t.header;
      current_pose_.pose.position.x = t.transform.translation.x;
      current_pose_.pose.position.y = t.transform.translation.y;
      current_pose_.pose.position.z = t.transform.translation.z;
      current_pose_.pose.orientation = t.transform.rotation;

      tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      current_heading_rad_ = yaw;
      
      // Update obstacle avoidance system
      obstacle_avoidance_->update_pose(current_pose_);
      
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not transform camera_link to map: %s", ex.what());
      return false;
    }
  }
  
  void perform_collision_check()
  {
      if (!collision_client_->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "Collision check service not ready!");
          return;
      }
      
      // Get collision points from obstacle avoidance system
      auto check_points = obstacle_avoidance_->build_safety_envelope();
      if (check_points.empty()) {
          RCLCPP_WARN(this->get_logger(), "No collision check points generated!");
          return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Performing collision check with %zu points", check_points.size());
      
      auto request = std::make_shared<blimp_navigation::srv::CheckCollision::Request>();
      request->points_to_check = check_points;

      // Use async call with callback - no executor conflicts
      auto result_future = collision_client_->async_send_request(
          request,
          [this, check_points](rclcpp::Client<blimp_navigation::srv::CheckCollision>::SharedFuture future) {
              try {
                  auto response = future.get();
                  obstacle_avoidance_->update_collision_result(response->is_occupied);
                  RCLCPP_INFO(this->get_logger(), "Collision check: %zu points -> %s", 
                              check_points.size(), response->is_occupied ? "COLLISION" : "clear");
              } catch (const std::exception& e) {
                  RCLCPP_ERROR(this->get_logger(), "Collision service callback error: %s - assuming COLLISION", e.what());
                  obstacle_avoidance_->update_collision_result(true);
              }
          });
  }

  void navigate_loop()
  {
    if (!get_current_pose_from_tf()) return;

    // Run collision detection very infrequently (every 5 seconds) since blimp is slow
    static auto last_collision_check = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_collision_check).count() > 5000) {
        perform_collision_check();
        last_collision_check = now;
    }

    if (!has_goal_) {
      controller_->set_motor_pct(0, 0.0);
      controller_->set_motor_pct(1, 0.0);
      reset_motor_ramp();
    } else {
      auto pos = current_pose_.pose.position;
      
      // First check waypoint advancement (this updates current_waypoint_index_)
      auto waypoint_target = get_current_target();  // This advances waypoints!
      
      // Then get 3D path tracking target (magnetic attraction to path)
      auto target_pos = get_path_tracking_target();
      double dx = target_pos.x - pos.x;
      double dy = target_pos.y - pos.y;
      double dz = target_pos.z - pos.z;
      
      double distance_to_target_2d = std::sqrt(dx*dx + dy*dy);
      double distance_to_target_3d = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      // Check if we reached the final goal
      auto final_goal_pos = goal_pose_.pose.position;
      double final_dx = final_goal_pos.x - pos.x;
      double final_dy = final_goal_pos.y - pos.y;
      double distance_to_final_goal = std::sqrt(final_dx*final_dx + final_dy*final_dy);

      if (distance_to_final_goal < 0.5) {
        RCLCPP_INFO(this->get_logger(), "🎉 Final goal reached!");
        has_goal_ = false;
        ompl_path_valid_ = false;
        controller_->set_motor_pct(0, 0.0);
        controller_->set_motor_pct(1, 0.0);
        reset_motor_ramp();
      } else {
        // 3D Path following with magnetic attraction to the planned route

        auto altitude_cmd = compute_altitude_command(target_pos.z, pos.z);

        const double desired_heading_rad = std::atan2(dy, dx);
        const double heading_error_rad = normalize_angle(desired_heading_rad - current_heading_rad_);
        const double heading_error_deg = heading_error_rad * 180.0 / M_PI;

        double turn_output = heading_pid_->update(heading_error_deg, std::chrono::steady_clock::now());
        forward_pid_->set_setpoint(0);
        double forward_output = forward_pid_->update(-distance_to_target_2d, std::chrono::steady_clock::now());

        const double abs_heading_error = std::abs(heading_error_deg);
        const double distance_cap = 6.0 + std::min(distance_to_target_2d, 3.5) * 4.0;  // 6% to ~20%
        const double heading_cap = 18.0 - std::min(abs_heading_error, 45.0) * 0.3;     // 18% down to ~4%
  const double forward_cap = std::clamp(std::min(distance_cap, heading_cap), 0.0, 18.0);
        forward_output = std::clamp(forward_output, -forward_cap, forward_cap);

        const bool obstacle_detected = obstacle_avoidance_->is_obstacle_detected();
        const double recommended_turn = obstacle_avoidance_->get_recommended_turn_rate();
        apply_navigation_strategies(
          heading_error_deg,
          distance_to_target_2d,
          altitude_cmd.error,
          obstacle_detected,
          recommended_turn,
          forward_output,
          turn_output);

  MotorMix motor_mix = compute_motor_mix(heading_error_deg, forward_output, turn_output);

        controller_->set_servo_us(2, SERVO_NEUTRAL_US + static_cast<int>(altitude_cmd.servo_offset));
        controller_->set_servo_us(3, SERVO_NEUTRAL_US + static_cast<int>(altitude_cmd.servo_offset));
        controller_->set_motor_pct(0, motor_mix.left_pct);
        controller_->set_motor_pct(1, motor_mix.right_pct);

        // Publish path visualization periodically (and after waypoint changes)
        static auto last_path_publish = std::chrono::steady_clock::now();
        static size_t last_waypoint_index = 0;
        auto now_path = std::chrono::steady_clock::now();
        
        bool waypoint_changed = (current_waypoint_index_ != last_waypoint_index);
        bool time_to_publish = std::chrono::duration_cast<std::chrono::milliseconds>(now_path - last_path_publish).count() > 2000;
        
        if (waypoint_changed || time_to_publish) {
            publish_path_visualization();
            last_path_publish = now_path;
            last_waypoint_index = current_waypoint_index_;
        }

        // Publish debug info if needed
        if (debug_mode_ && (this->get_clock()->now() - last_debug_time_).seconds() >= 1.5) {
          last_debug_time_ = this->get_clock()->now();
          publish_debug_status(
            pos,
            target_pos,
            final_goal_pos,
            distance_to_target_2d,
            distance_to_target_3d,
            distance_to_final_goal,
            heading_error_deg,
            altitude_cmd,
            forward_output,
            turn_output,
            motor_mix,
            obstacle_detected);
        }
      }
    }
  }

  // Member variables
  bool debug_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ompl_path_sub_;
  rclcpp::Client<blimp_navigation::srv::CheckCollision>::SharedPtr collision_client_;
  rclcpp::Client<blimp_navigation::srv::PlanPath>::SharedPtr path_planning_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  // Path visualization publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr remaining_path_pub_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::unique_ptr<blimp_navigation_cpp::BlimpController> controller_;
  std::unique_ptr<blimp_navigation_cpp::PID> altitude_pid_, heading_pid_, forward_pid_;
  std::unique_ptr<blimp_navigation_cpp::ObstacleAvoidance> obstacle_avoidance_;

  geometry_msgs::msg::PoseStamped current_pose_;
  double current_heading_rad_ = 0.0;
  geometry_msgs::msg::PoseStamped goal_pose_;

  bool have_last_left_ = false;
  bool have_last_right_ = false;
  double last_left_pct_ = 0.0;
  double last_right_pct_ = 0.0;
  int left_release_ticks_ = 0;
  int right_release_ticks_ = 0;
  int left_reverse_step_index_ = 0;
  int right_reverse_step_index_ = 0;
  
  // OMPL path following
  nav_msgs::msg::Path ompl_path_;         // Current working path
  nav_msgs::msg::Path ompl_path_full_;    // Complete original path (never changes)
  bool ompl_path_valid_ = false;
  size_t current_waypoint_index_ = 0;
  
  // Legacy path planning (kept for fallback)
  std::vector<geometry_msgs::msg::Point> current_path_;
  bool path_valid_ = false;
  
  bool has_goal_ = false;
  rclcpp::Time ompl_path_timeout_;
  rclcpp::Time last_debug_time_;
};

double BlimpNavigatorNode::normalize_angle(double angle_rad)
{
  angle_rad = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (angle_rad < 0.0) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad - M_PI;
}

BlimpNavigatorNode::AltitudeCommand BlimpNavigatorNode::compute_altitude_command(double target_z, double current_z)
{
  AltitudeCommand cmd;
  cmd.error = target_z - current_z;
  altitude_pid_->set_setpoint(target_z);

  if (std::abs(cmd.error) < 0.08) {
    cmd.state = "HOLD";
    cmd.servo_offset = 0.0;
  } else {
    double pid_output = altitude_pid_->update(current_z, std::chrono::steady_clock::now());
    double abs_error = std::abs(cmd.error);
    double scale = 1.0;

    if (abs_error < 0.3) {
      scale = 0.4;
      cmd.state = cmd.error > 0.0 ? "CLIMB_GENTLE" : "DESCEND_GENTLE";
    } else if (abs_error < 0.8) {
      scale = 0.7;
      cmd.state = cmd.error > 0.0 ? "CLIMB_MEDIUM" : "DESCEND_MEDIUM";
    } else {
      cmd.state = cmd.error > 0.0 ? "CLIMB_FULL" : "DESCEND_FULL";
    }

    cmd.servo_offset = pid_output * scale;
  }

  cmd.prioritize_altitude = std::abs(cmd.error) > 1.0;
  return cmd;
}

void BlimpNavigatorNode::apply_navigation_strategies(
  double heading_error_deg,
  double distance_to_target_2d,
  double altitude_error,
  bool obstacle_detected,
  double recommended_turn,
  double & forward_output,
  double & turn_output)
{
  const double abs_heading_error = std::abs(heading_error_deg);

  if (obstacle_detected) {
    forward_output = -5.0;
    turn_output = recommended_turn;
    RCLCPP_DEBUG(this->get_logger(), "🚫 Obstacle detected: REVERSING + turning %.1f", recommended_turn);
    return;
  }

  if (abs_heading_error > 120.0) {
    forward_output = -std::abs(forward_output) * 0.7;
    RCLCPP_DEBUG(this->get_logger(), "🔄 Target behind - REVERSING instead of turning");
  } else if (abs_heading_error > 15.0) {
    forward_output = 0.0;
  }

  if (distance_to_target_2d < 0.3) {
    if (forward_output > 0.0) {
      forward_output = -forward_output * 0.3;
      RCLCPP_DEBUG(this->get_logger(), "🛑 Close to target - REVERSE BRAKING");
    }
  } else if (distance_to_target_2d < 1.0) {
    forward_output *= 0.7;
  }

  if (std::abs(altitude_error) > 1.0) {
    forward_output *= 0.5;
    RCLCPP_DEBUG(this->get_logger(), "🚁 Large altitude deviation (%.2fm) - prioritizing climb/descend", altitude_error);
  }
}

BlimpNavigatorNode::MotorMix BlimpNavigatorNode::compute_motor_mix(double heading_error_deg, double forward_output, double turn_output)
{
  MotorMix mix;
  mix.abs_heading_error = std::abs(heading_error_deg);

  constexpr double fine_steer_threshold = 8.0;
  constexpr double single_motor_threshold = 45.0;
  constexpr double diff_thrust_threshold = 45.0;
  constexpr double counter_rotation_threshold = 110.0;

  auto heading_to_pct = [](double value, double in_min, double in_max, double out_min, double out_max) {
    if (in_max <= in_min) {
      return out_max;
    }
    double t = (value - in_min) / (in_max - in_min);
    t = std::clamp(t, 0.0, 1.0);
    return out_min + t * (out_max - out_min);
  };

  if (mix.abs_heading_error >= diff_thrust_threshold && std::abs(forward_output) < 3.0) {
    double rotation_power = heading_to_pct(mix.abs_heading_error, diff_thrust_threshold, counter_rotation_threshold, 14.0, 30.0);
    double idle_side_power = -heading_to_pct(mix.abs_heading_error, diff_thrust_threshold, counter_rotation_threshold, 8.0, 26.0);
    rotation_power = std::clamp(rotation_power, 14.0, 30.0);
    idle_side_power = std::clamp(idle_side_power, -26.0, -6.0);

    if (mix.abs_heading_error >= counter_rotation_threshold) {
      mix.left_pct = turn_output > 0.0 ? rotation_power : -rotation_power;
      mix.right_pct = turn_output > 0.0 ? -rotation_power : rotation_power;
      mix.turning_mode = "COUNTER_ROTATION";
      RCLCPP_DEBUG(this->get_logger(), "⚔️ AGGRESSIVE COUNTER-ROTATION (%.1f°): L=%.1f, R=%.1f",
                   heading_error_deg, mix.left_pct, mix.right_pct);
    } else {
      if (turn_output > 0.0) {
        mix.left_pct = rotation_power;
        mix.right_pct = idle_side_power;
      } else {
        mix.left_pct = idle_side_power;
        mix.right_pct = rotation_power;
      }
      mix.turning_mode = "DIFFERENTIAL_THRUST";
      RCLCPP_DEBUG(this->get_logger(), "⚡ DIFFERENTIAL-THRUST: %.1f° error, L=%.1f%%, R=%.1f%%",
                   mix.abs_heading_error, mix.left_pct, mix.right_pct);
    }
  } else if (mix.abs_heading_error > fine_steer_threshold) {
    constexpr double min_trailing_scale = 0.1;
    constexpr double min_effective_pct = 6.0;
    const double normalized = std::clamp(
      (mix.abs_heading_error - fine_steer_threshold) / (single_motor_threshold - fine_steer_threshold),
      0.0, 1.0);
    const double trailing_scale = std::clamp(1.0 - (0.75 * normalized) - 0.15, min_trailing_scale, 1.0);

    double lead_power = forward_output;
    double trail_power = forward_output * trailing_scale;

    if (std::abs(trail_power) < min_effective_pct) {
      trail_power = 0.0;
    }

    if (turn_output > 0.0) {
      mix.left_pct = lead_power;
      mix.right_pct = trail_power;
    } else {
      mix.left_pct = trail_power;
      mix.right_pct = lead_power;
    }

    // Keep trailing motor forward only when we are committed to forward translation.
    if (forward_output > 2.0) {
      mix.left_pct = std::max(0.0, mix.left_pct);
      mix.right_pct = std::max(0.0, mix.right_pct);
    }

    if (std::abs(lead_power) < min_effective_pct && std::abs(turn_output) > 1.5) {
      const double spin_power = std::clamp(std::abs(turn_output) * 0.35, 4.0, 12.0);
      if (turn_output > 0.0) {
        mix.left_pct = spin_power;
        mix.right_pct = 0.0;
      } else {
        mix.left_pct = 0.0;
        mix.right_pct = spin_power;
      }
    }

    if (forward_output <= 1.0) {
      double reverse_pct = heading_to_pct(mix.abs_heading_error, fine_steer_threshold, single_motor_threshold, 6.0, 24.0);
      reverse_pct = std::clamp(reverse_pct, 6.0, 24.0);
      double assist_pct = std::clamp(reverse_pct * 0.7, 6.0, 20.0);
      if (turn_output > 0.0) {
        mix.right_pct = -reverse_pct;
        mix.left_pct = std::max(mix.left_pct, assist_pct);
      } else {
        mix.left_pct = -reverse_pct;
        mix.right_pct = std::max(mix.right_pct, assist_pct);
      }
    }

    mix.turning_mode = "SINGLE_MOTOR_STEER";
    RCLCPP_DEBUG(this->get_logger(), "� SINGLE-MOTOR STEER: %.1f° error, L=%.1f%%, R=%.1f%% (trail scale=%.2f)",
                 mix.abs_heading_error, mix.left_pct, mix.right_pct, trailing_scale);
  } else {
    mix.left_pct = forward_output + (turn_output * 0.3);
    mix.right_pct = forward_output - (turn_output * 0.3);
    mix.turning_mode = "FINE_STEERING";
    RCLCPP_DEBUG(this->get_logger(), "🎯 FINE-STEERING: %.1f° error, L=%.1f%%, R=%.1f%%",
                 mix.abs_heading_error, mix.left_pct, mix.right_pct);
  }

  mix.left_pct = std::clamp(mix.left_pct, -30.0, 30.0);
  mix.right_pct = std::clamp(mix.right_pct, -30.0, 30.0);

  const double max_delta_per_cycle = 8.0;
  const double zero_snap_pct = 2.5;
  const double reverse_release_pct = 26.0;
  const int reverse_release_ticks = 2;
  const std::array<double, 5> reverse_step_pct{26.0, 20.0, 14.0, 8.0, 0.0};

  auto apply_ramp = [&](double target, double & last, bool & have_last, int & release_ticks, int & reverse_step_index) {
    if (!have_last) {
      have_last = true;
      last = target;
      reverse_step_index = 0;
      release_ticks = 0;
      return last;
    }

    if (target < 0.0) {
      reverse_step_index = 0;
      release_ticks = 0;
    } else if (last < 0.0) {
      if (release_ticks > 0) {
        --release_ticks;
        target = last;
      } else {
        if (reverse_step_index < static_cast<int>(reverse_step_pct.size())) {
          target = -reverse_step_pct[reverse_step_index];
          ++reverse_step_index;
          release_ticks = reverse_release_ticks;
        } else {
          target = 0.0;
        }
      }
    } else {
      reverse_step_index = 0;
      release_ticks = 0;
    }

    double delta = target - last;
    if (delta > max_delta_per_cycle) {
      delta = max_delta_per_cycle;
    } else if (delta < -max_delta_per_cycle) {
      delta = -max_delta_per_cycle;
    }

    last += delta;

    if (std::abs(last) < zero_snap_pct && std::abs(target) < zero_snap_pct) {
      last = 0.0;
    }

    return last;
  };

  mix.left_pct = apply_ramp(mix.left_pct, last_left_pct_, have_last_left_, left_release_ticks_, left_reverse_step_index_);
  mix.right_pct = apply_ramp(mix.right_pct, last_right_pct_, have_last_right_, right_release_ticks_, right_reverse_step_index_);

  return mix;
}

void BlimpNavigatorNode::reset_motor_ramp()
{
  have_last_left_ = false;
  have_last_right_ = false;
  last_left_pct_ = 0.0;
  last_right_pct_ = 0.0;
  left_release_ticks_ = 0;
  right_release_ticks_ = 0;
  left_reverse_step_index_ = 0;
  right_reverse_step_index_ = 0;
}

void BlimpNavigatorNode::publish_debug_status(
  const geometry_msgs::msg::Point & pos,
  const geometry_msgs::msg::Point & target_pos,
  const geometry_msgs::msg::Point & final_goal_pos,
  double distance_to_target_2d,
  double distance_to_target_3d,
  double distance_to_final_goal,
  double heading_error_deg,
  const AltitudeCommand & altitude_cmd,
  double forward_output,
  double turn_output,
  const MotorMix & mix,
  bool obstacle_detected)
{
  std::stringstream ss;
  ss << "\n=================== BLIMP STATUS (C++) ==================="
     << "\n- OBSTACLE AHEAD: " << (obstacle_detected ? "TRUE - AVOIDING" : "false")
     << "\n- PATH STATUS: " << (ompl_path_valid_ ? "3D Path Tracking" : "Direct Navigation")
     << (ompl_path_valid_ ? (" (waypoint " + std::to_string(current_waypoint_index_ + 1) + "/" + std::to_string(ompl_path_.poses.size()) + ")") : "")
     << (ompl_path_valid_ && std::abs(altitude_cmd.error) > 0.5 ? " - ALTITUDE CORRECTION" : "")
     << "\n----------------------------------------------------"
     << "\n|         |      X     |      Y     |      Z     |"
     << "\n| Current | " << std::fixed << std::setprecision(2) << std::setw(10) << pos.x
     << " | " << std::setw(10) << pos.y << " | " << std::setw(10) << pos.z << " |"
     << "\n| Target  | " << std::setw(10) << target_pos.x << " | " << std::setw(10) << target_pos.y << " | " << std::setw(10) << target_pos.z << " |"
     << "\n| F.Goal  | " << std::setw(10) << final_goal_pos.x << " | " << std::setw(10) << final_goal_pos.y << " | " << std::setw(10) << final_goal_pos.z << " |"
     << "\n----------------------------------------------------"
     << "\n- Navigation:"
     << "\n  - Current Heading: " << std::fixed << std::setprecision(1) << current_heading_rad_ * 180.0 / M_PI << " (deg)"
     << "\n  - Distance to Target:  " << std::fixed << std::setprecision(2) << distance_to_target_2d << " (2D) / " << std::setprecision(2) << distance_to_target_3d << " (3D) m"
     << "\n  - Altitude Error: " << std::fixed << std::setprecision(2) << altitude_cmd.error << " (m) - " << altitude_cmd.state
     << "\n  - Distance to F.Goal:  " << std::fixed << std::setprecision(2) << distance_to_final_goal << " (m)"
     << "\n  - Heading Error:   " << std::fixed << std::setprecision(1) << heading_error_deg << " (deg)"
     << "\n- Control System:"
  << "\n  - TURNING MODE: " << mix.turning_mode << " (" << std::fixed << std::setprecision(1) << mix.abs_heading_error << "° error)"
  << "\n  - Mode Logic: >=110°=COUNTER_ROT, >=45° (low fwd)=DIFF_THRUST, 8-45°=SINGLE-MOTOR, <8°=FINE (SOFT RAMP)"
     << "\n  - Base Turn Power: " << std::setprecision(1)
  << (mix.abs_heading_error > 8.0 ? std::min(20.0, std::max(4.0, mix.abs_heading_error * 0.3)) : 0.0)
     << "% (from " << mix.abs_heading_error << "° error)"
     << "\n  - PID Outputs: Alt=" << std::fixed << std::setprecision(1) << altitude_cmd.servo_offset
     << " | Turn=" << turn_output << " | Fwd=" << forward_output
     << "\n  - Servo Positions: L=" << (SERVO_NEUTRAL_US + static_cast<int>(altitude_cmd.servo_offset))
     << "us | R=" << (SERVO_NEUTRAL_US + static_cast<int>(altitude_cmd.servo_offset)) << "us (1500=neutral)"
  << "\n  - Motor Powers: L=" << std::setprecision(1) << mix.left_pct << "% | R=" << mix.right_pct << "% (REVERSIBLE!)"
  << ((mix.left_pct < 0.0 || mix.right_pct < 0.0) ? " - REVERSE THRUST ACTIVE" : "")
  << "\n  - Motor Status: " << (std::abs(mix.left_pct) < 6.0 && std::abs(mix.right_pct) < 6.0 ? "⚠️  POWER TOO LOW!" : "✅ ADEQUATE POWER")
     << (obstacle_detected ? "\n  - ACTION: SMART OBSTACLE AVOIDANCE ACTIVE" : "")
     << "\n- " << obstacle_avoidance_->get_status_string()
     << "\n====================================================";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  status_pub_->publish(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), msg.data);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlimpNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}