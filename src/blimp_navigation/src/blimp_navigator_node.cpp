#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "blimp_navigation/srv/check_collision.hpp"
#include "blimp_navigation/srv/plan_path.hpp"
#include "blimp_navigation/pid.hpp"
#include "blimp_navigation/blimp_controller.hpp"

#include <cmath>
#include <memory>
#include <future>
#include <sstream>
#include <chrono>

using namespace std::chrono_literals;

class BlimpNavigatorNode : public rclcpp::Node
{
public:
  BlimpNavigatorNode() : Node("blimp_navigator_node")
  {
    // --- Master Debug Switch ---
    this->declare_parameter<bool>("debug_mode", true);
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    
    // --- Parameters for tuning ---
    this->declare_parameter<double>("altitude_kp", 150.0);
    this->declare_parameter<double>("altitude_ki", 10.0);
    this->declare_parameter<double>("altitude_kd", 20.0);
    this->declare_parameter<double>("heading_kp", 0.3);
    this->declare_parameter<double>("heading_ki", 0.05);
    this->declare_parameter<double>("heading_kd", 0.1);
    this->declare_parameter<double>("forward_kp", 20.0);
    this->declare_parameter<double>("forward_ki", 2.0);
    this->declare_parameter<double>("forward_kd", 5.0);
    
    // --- TF2 listener ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // --- Hardware controller ---
    try {
      controller_ = std::make_unique<blimp_navigation_cpp::BlimpController>("/dev/ttyAMA0", 115200);
      RCLCPP_INFO(this->get_logger(), "Successfully connected to the flight controller.");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to flight controller: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // --- PID controllers ---
    altitude_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("altitude_kp").as_double(), this->get_parameter("altitude_ki").as_double(), this->get_parameter("altitude_kd").as_double(), 0.0, -500.0, 500.0
    );
    heading_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("heading_kp").as_double(), this->get_parameter("heading_ki").as_double(), this->get_parameter("heading_kd").as_double(), 0.0, -50.0, 50.0
    );
    forward_pid_ = std::make_unique<blimp_navigation_cpp::PID>(
      this->get_parameter("forward_kp").as_double(), this->get_parameter("forward_ki").as_double(), this->get_parameter("forward_kd").as_double(), 0.0, 0.0, 20.0
    );
    RCLCPP_INFO(this->get_logger(), "PID controllers initialized.");

    // --- ROS2 communications ---
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&BlimpNavigatorNode::goal_callback, this, std::placeholders::_1));
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
      RCLCPP_INFO(this->get_logger(), "Collision check service is ready.");
    }

    if (debug_mode_) {
      status_pub_ = this->create_publisher<std_msgs::msg::String>("/blimp_status", 10);
      last_debug_time_ = this->get_clock()->now();
    }
    
    // --- Main loop ---
    timer_ = this->create_wall_timer(100ms, std::bind(&BlimpNavigatorNode::navigate_loop, this));
    RCLCPP_INFO(this->get_logger(), "Blimp C++ Navigator Node has been started.");
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    path_valid_ = false;  // Need to replan
    current_path_.clear();
    current_waypoint_index_ = 0;
    RCLCPP_INFO(this->get_logger(), "New goal received: Pos(x=%.2f, y=%.2f, z=%.2f)", 
      goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
    altitude_pid_->set_setpoint(goal_pose_.pose.position.z);
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
    
    // Wait for result with short timeout to avoid blocking
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
    if (!path_valid_ || current_path_.empty()) {
      return goal_pose_.pose.position;  // Fall back to direct navigation
    }

    // Check if we reached the current waypoint
    auto& current_waypoint = current_path_[current_waypoint_index_];
    auto& current_pos = current_pose_.pose.position;
    
    double distance_to_waypoint = std::sqrt(
      std::pow(current_waypoint.x - current_pos.x, 2) +
      std::pow(current_waypoint.y - current_pos.y, 2) +
      std::pow(current_waypoint.z - current_pos.z, 2)
    );

    // If we're close to the current waypoint, advance to the next one
    if (distance_to_waypoint < 0.3) {  // 30cm threshold
      current_waypoint_index_++;
      if (current_waypoint_index_ >= current_path_.size()) {
        current_waypoint_index_ = current_path_.size() - 1;  // Stay at final waypoint
      }
      RCLCPP_INFO(this->get_logger(), "Advanced to waypoint %zu/%zu", 
        current_waypoint_index_ + 1, current_path_.size());
    }

    return current_path_[current_waypoint_index_];
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
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not transform camera_link to map: %s", ex.what());
      return false;
    }
  }
  
  void check_for_obstacles()
  {
      if (!collision_client_->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                               "Collision check service not ready!");
          obstacle_detected_ = false; // Fail safe - assume no obstacle if service unavailable
          return;
      }
      
      auto request = std::make_shared<blimp_navigation::srv::CheckCollision::Request>();
      
      // Build points to check in front of the blimp
      auto pos = current_pose_.pose.position;
      double forward_x = std::cos(current_heading_rad_);
      double forward_y = std::sin(current_heading_rad_);
      
      for (double step = 0.3; step <= 1.5; step += 0.12) {
          geometry_msgs::msg::Point p;
          p.x = pos.x + forward_x * step;
          p.y = pos.y + forward_y * step;
          p.z = pos.z;
          request->points_to_check.push_back(p);
      }

      auto result_future = collision_client_->async_send_request(request);
      
      // Use a longer wait time to avoid false negatives
      if (result_future.wait_for(500ms) == std::future_status::ready) {
          auto response = result_future.get();
          obstacle_detected_ = response->is_occupied;
          RCLCPP_DEBUG(this->get_logger(), "Collision check result: %s", 
                      obstacle_detected_ ? "OBSTACLE DETECTED" : "clear");
      } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "Collision service timeout - using fail-safe (no obstacle)");
          obstacle_detected_ = false; // Fail safe
      }
  }

  void navigate_loop()
  {
    if (!get_current_pose_from_tf()) return;

    check_for_obstacles();

    if (!has_goal_) {
      controller_->set_motor_pct(0, 0.0);
      controller_->set_motor_pct(1, 0.0);
    } else {
      auto pos = current_pose_.pose.position;
      auto goal_pos = goal_pose_.pose.position;
      double dx = goal_pos.x - pos.x;
      double dy = goal_pos.y - pos.y;
      double distance_to_goal_2d = std::sqrt(dx*dx + dy*dy);

      if (distance_to_goal_2d < 0.5) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        has_goal_ = false;
        controller_->set_motor_pct(0, 0.0);
        controller_->set_motor_pct(1, 0.0);
      } else {
        double altitude_output = altitude_pid_->update(pos.z, std::chrono::steady_clock::now());
        double desired_heading_rad = std::atan2(dy, dx);
        double heading_error_rad = desired_heading_rad - current_heading_rad_;
        
        while (heading_error_rad > M_PI) heading_error_rad -= 2 * M_PI;
        while (heading_error_rad < -M_PI) heading_error_rad += 2 * M_PI;
        
        double turn_output = heading_pid_->update(heading_error_rad * 180.0 / M_PI, std::chrono::steady_clock::now());
        forward_pid_->set_setpoint(0);
        double forward_output = forward_pid_->update(-distance_to_goal_2d, std::chrono::steady_clock::now());
        
        if (std::abs(heading_error_rad * 180.0 / M_PI) > 25.0) {
            forward_output = 0.0;
        }
        if (obstacle_detected_) {
            forward_output = 0.0;
            turn_output = -40.0;
        }

        controller_->set_servo_us(2, 1500 + static_cast<int>(altitude_output));
        controller_->set_servo_us(3, 1500 + static_cast<int>(altitude_output));
        
        double left_motor = std::clamp(forward_output + turn_output, 0.0, 20.0);
        double right_motor = std::clamp(forward_output - turn_output, 0.0, 20.0);
        
        controller_->set_motor_pct(0, left_motor);
        controller_->set_motor_pct(1, right_motor);

        // --- Publish debug info if needed ---
        if (debug_mode_ && (this->get_clock()->now() - last_debug_time_).seconds() >= 1.5) {
            last_debug_time_ = this->get_clock()->now();
            std::stringstream ss;
            ss << "\n=================== BLIMP STATUS (C++) ==================="
               << "\n- OBSTACLE AHEAD: " << (obstacle_detected_ ? "TRUE - AVOIDING" : "false")
               << "\n----------------------------------------------------"
               << "\n|         |      X     |      Y     |      Z     |"
               << "\n| Current | " << std::fixed << std::setprecision(2) << std::setw(10) << pos.x 
               << " | " << std::setw(10) << pos.y << " | " << std::setw(10) << pos.z << " |"
               << "\n| Goal    | " << std::setw(10) << goal_pos.x << " | " << std::setw(10) << goal_pos.y << " | " << std::setw(10) << goal_pos.z << " |"
               << "\n----------------------------------------------------"
               << "\n- Navigation:"
               << "\n  - Current Heading: " << std::fixed << std::setprecision(1) << current_heading_rad_ * 180.0 / M_PI << " (deg)"
               << "\n  - Distance Error:  " << std::fixed << std::setprecision(2) << distance_to_goal_2d << " (m)"
               << "\n  - Heading Error:   " << std::fixed << std::setprecision(1) << heading_error_rad * 180.0 / M_PI << " (deg)"
               << "\n- Control:"
               << "\n  - PID Outputs: Alt=" << std::fixed << std::setprecision(1) << altitude_output << " | Turn=" << turn_output << " | Fwd=" << forward_output
               << "\n  - Motors: L=" << left_motor << "% | R=" << right_motor << "%"
               << (obstacle_detected_ ? "\n  - ACTION: Stopping forward, turning to avoid obstacle" : "")
               << "\n====================================================";
            
            std_msgs::msg::String msg;
            msg.data = ss.str();
            status_pub_->publish(msg);
            RCLCPP_INFO_STREAM(this->get_logger(), msg.data);
        }
      }
    }
  }

  // Member variables
  bool debug_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Client<blimp_navigation::srv::CheckCollision>::SharedPtr collision_client_;
  rclcpp::Client<blimp_navigation::srv::PlanPath>::SharedPtr path_planning_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::unique_ptr<blimp_navigation_cpp::BlimpController> controller_;
  std::unique_ptr<blimp_navigation_cpp::PID> altitude_pid_, heading_pid_, forward_pid_;

  geometry_msgs::msg::PoseStamped current_pose_;
  double current_heading_rad_ = 0.0;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::vector<geometry_msgs::msg::Point> current_path_;
  size_t current_waypoint_index_ = 0;
  bool path_valid_ = false;
  bool has_goal_ = false;
  bool obstacle_detected_ = false;
  rclcpp::Time last_debug_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlimpNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}