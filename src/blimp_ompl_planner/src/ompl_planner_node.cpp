#include "blimp_ompl_planner/ompl_planner_node.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace blimp_ompl_planner
{

  OmplPlannerNode::OmplPlannerNode()
      : Node("ompl_planner_node"),
        parameters_(declareParameters()),
        octomap_manager_(parameters_.bounds, parameters_.robot_radius),
        path_planner_(parameters_, octomap_manager_, this->get_logger(), this->get_clock())
  {
    RCLCPP_INFO(this->get_logger(), "Starting OMPL Planner Node");

    // Configure safety margins for floor/ceiling detection
    double ceiling_margin = this->declare_parameter("safety_margin_from_ceiling", 0.3);
    double floor_margin = this->declare_parameter("safety_margin_from_floor", 0.3);
    octomap_manager_.setSafetyMarginFromCeiling(ceiling_margin);
    octomap_manager_.setSafetyMarginFromFloor(floor_margin);

    RCLCPP_INFO(this->get_logger(), "Safety margins - Ceiling: %.2f m, Floor: %.2f m", 
                ceiling_margin, floor_margin);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap", 10,
        std::bind(&OmplPlannerNode::octomapCallback, this, std::placeholders::_1));

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", path_qos);

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&OmplPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OmplPlannerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&OmplPlannerNode::executePlan, this, std::placeholders::_1));

    // Disabled path republish timer - navigator handles path following
    // path_republish_timer_ = this->create_wall_timer(
    //     std::chrono::seconds(10),
    //     std::bind(&OmplPlannerNode::republishLastPath, this));

    RCLCPP_INFO(this->get_logger(), "OMPL Planner Node initialized successfully");
  }

  void OmplPlannerNode::octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr &msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(this->get_logger(), "Received null OctoMap message pointer");
      return;
    }

    if (octomap_manager_.updateFromMessage(*msg, this->get_logger()))
    {
      const auto bounds = octomap_manager_.bounds();
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "OctoMap updated. Bounds X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
          bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z);
    }
  }

  rclcpp_action::GoalResponse OmplPlannerNode::handleGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received planning request");

    if (!octomap_manager_.hasMap())
    {
      RCLCPP_ERROR(this->get_logger(), "No OctoMap available for planning");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!isPoseWithinBounds(goal->pose))
    {
      RCLCPP_ERROR(this->get_logger(), "Goal pose is outside planning bounds");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse OmplPlannerNode::handleCancel(
      const std::shared_ptr<GoalHandleNavigateToPose> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void OmplPlannerNode::executePlan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing planning goal");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<NavigateToPose::Result>();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    geometry_msgs::msg::PoseStamped start_pose;
    if (!getCurrentRobotPose(start_pose))
    {
      result->error_code = 1;
      result->error_msg = "Failed to get current robot pose";
      goal_handle->abort(result);
      return;
    }

    feedback->current_pose = start_pose;
    goal_handle->publish_feedback(feedback);

    nav_msgs::msg::Path planned_path;
    std::string error_message;

    if (!path_planner_.plan(start_pose, goal->pose, planned_path, error_message))
    {
      result->error_code = 6;
      result->error_msg = error_message;
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      last_planned_path_ = planned_path;
    }
    path_pub_->publish(planned_path);

    result->error_code = NavigateToPose::Result::NONE;
    result->error_msg.clear();
    goal_handle->succeed(result);
  }

  bool OmplPlannerNode::getCurrentRobotPose(geometry_msgs::msg::PoseStamped &pose)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(
          parameters_.global_frame, parameters_.robot_frame, tf2::TimePointZero);

      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = parameters_.global_frame;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose: %s", ex.what());
      return false;
    }
  }

  PlannerParameters OmplPlannerNode::declareParameters()
  {
    PlannerParameters params;

    params.robot_radius = this->declare_parameter("robot_radius", 0.3);
    params.planning_timeout = this->declare_parameter("planning_timeout", 10.0);
    params.planner_id = this->declare_parameter("planner_id", std::string("RRTConnect"));

    params.global_frame = this->declare_parameter("global_frame", std::string("map"));
    params.robot_frame = this->declare_parameter("robot_frame", std::string("base_link"));

    params.bounds.min_x = this->declare_parameter("bounds_min_x", -10.0);
    params.bounds.max_x = this->declare_parameter("bounds_max_x", 10.0);
    params.bounds.min_y = this->declare_parameter("bounds_min_y", -10.0);
    params.bounds.max_y = this->declare_parameter("bounds_max_y", 10.0);
    params.bounds.min_z = this->declare_parameter("bounds_min_z", 0.0);
    params.bounds.max_z = this->declare_parameter("bounds_max_z", 5.0);

    params.path_processing.enable_simplification =
        this->declare_parameter("enable_path_simplification", true);
    params.path_processing.enable_smoothing =
        this->declare_parameter("enable_path_smoothing", true);
    params.path_processing.max_simplification_steps =
        this->declare_parameter("max_simplification_steps", 100);

    return params;
  }

  bool OmplPlannerNode::isPoseWithinBounds(const geometry_msgs::msg::PoseStamped &pose)
  {
    return octomap_manager_.isWithinBounds(pose);
  }

  void OmplPlannerNode::republishLastPath()
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (last_planned_path_.poses.empty())
    {
      return;
    }

    last_planned_path_.header.stamp = this->get_clock()->now();
    path_pub_->publish(last_planned_path_);
  }

} // namespace blimp_ompl_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blimp_ompl_planner::OmplPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}