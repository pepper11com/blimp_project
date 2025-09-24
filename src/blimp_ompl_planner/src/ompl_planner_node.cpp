#include "blimp_ompl_planner/ompl_planner_node.hpp"
#include "octomap_msgs/conversions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <cmath>
#include <cstdlib>
#include <octomap/ColorOcTree.h>

namespace blimp_ompl_planner
{

OmplPlannerNode::OmplPlannerNode()
    : Node("ompl_planner_node")
    , octree_mutex_(std::make_shared<std::mutex>())
    , has_unsafe_start_(false)
{
    RCLCPP_INFO(this->get_logger(), "Starting OMPL Planner Node");

    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get parameters
    declareParameters();

    // Initialize ROS 2 interfaces
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap", 10,
        std::bind(&OmplPlannerNode::octomapCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    // Create action server
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&OmplPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OmplPlannerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&OmplPlannerNode::executePlan, this, std::placeholders::_1));

    // Create timer to republish paths for visualization
    path_republish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&OmplPlannerNode::republishLastPath, this));

    RCLCPP_INFO(this->get_logger(), "OMPL Planner Node initialized successfully");
}

void OmplPlannerNode::declareParameters()
{
    // Robot parameters
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("planning_timeout", 10.0);
    this->declare_parameter("planner_id", std::string("RRTConnect"));
    
    // Frame parameters
    this->declare_parameter("global_frame", std::string("map"));
    this->declare_parameter("robot_frame", std::string("base_link"));
    
    // Planning bounds
    this->declare_parameter("bounds_min_x", -10.0);
    this->declare_parameter("bounds_max_x", 10.0);
    this->declare_parameter("bounds_min_y", -10.0);
    this->declare_parameter("bounds_max_y", 10.0);
    this->declare_parameter("bounds_min_z", 0.0);
    this->declare_parameter("bounds_max_z", 5.0);
    
    // Path processing parameters
    this->declare_parameter("enable_path_simplification", true);
    this->declare_parameter("enable_path_smoothing", true);
    this->declare_parameter("max_simplification_steps", 100);

    // Get parameter values
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    planning_timeout_ = this->get_parameter("planning_timeout").as_double();
    planner_id_ = this->get_parameter("planner_id").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    
    bounds_min_x_ = this->get_parameter("bounds_min_x").as_double();
    bounds_max_x_ = this->get_parameter("bounds_max_x").as_double();
    bounds_min_y_ = this->get_parameter("bounds_min_y").as_double();
    bounds_max_y_ = this->get_parameter("bounds_max_y").as_double();
    bounds_min_z_ = this->get_parameter("bounds_min_z").as_double();
    bounds_max_z_ = this->get_parameter("bounds_max_z").as_double();
    
    enable_path_simplification_ = this->get_parameter("enable_path_simplification").as_bool();
    enable_path_smoothing_ = this->get_parameter("enable_path_smoothing").as_bool();
    max_simplification_steps_ = this->get_parameter("max_simplification_steps").as_int();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded - Robot radius: %.2f, Timeout: %.1fs, Planner: %s",
                robot_radius_, planning_timeout_, planner_id_.c_str());
}

void OmplPlannerNode::octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr& msg)
{
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received an empty OctoMap message");
        return;
    }

    try {
        // Convert ROS message to OctoMap using binary format (matches RTABMap output)
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        if (!abstract_tree) {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap message");
            return;
        }

        // Try to cast to OcTree (binary octree)
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (!octree) {
            // Try ColorOcTree (RTABMap often uses this)
            octomap::ColorOcTree* color_octree = dynamic_cast<octomap::ColorOcTree*>(abstract_tree);
            if (color_octree) {
                RCLCPP_INFO(this->get_logger(), "Received ColorOcTree, converting to binary OcTree");
                // Create a new binary octree from the color octree
                octree = new octomap::OcTree(color_octree->getResolution());
                for (auto it = color_octree->begin_leafs(); it != color_octree->end_leafs(); ++it) {
                    if (color_octree->isNodeOccupied(*it)) {
                        octree->updateNode(it.getKey(), true);
                    }
                }
                octree->updateInnerOccupancy();
                delete abstract_tree; // Clean up the original
            } else {
                RCLCPP_ERROR(this->get_logger(), "Deserialized map is not of supported type (OcTree or ColorOcTree)");
                delete abstract_tree;
                return;
            }
        }

        // Update the shared octree with thread safety
        {
            std::lock_guard<std::mutex> lock(*octree_mutex_);
            octree_ = std::shared_ptr<octomap::OcTree>(octree);
        }

        // Update validity checker if it exists
        if (validity_checker_) {
            validity_checker_->updateOctoMap(octree_);
        }

        // Update planning bounds based on the new map
        updatePlanningBounds();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Successfully updated OctoMap with %zu nodes",
                            octree_->getNumLeafNodes());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in octomapCallback: %s", e.what());
    }
}

void OmplPlannerNode::updatePlanningBounds()
{
    std::lock_guard<std::mutex> lock(*octree_mutex_);
    if (!octree_) {
        return;
    }

    // Get the bounding box of the OctoMap
    double map_min_x, map_min_y, map_min_z;
    double map_max_x, map_max_y, map_max_z;
    octree_->getMetricMin(map_min_x, map_min_y, map_min_z);
    octree_->getMetricMax(map_max_x, map_max_y, map_max_z);

    // Expand bounds slightly to ensure we don't plan exactly at the map boundary
    double padding = robot_radius_ * 2.0;
    bounds_min_x_ = std::max(bounds_min_x_, map_min_x - padding);
    bounds_max_x_ = std::min(bounds_max_x_, map_max_x + padding);
    bounds_min_y_ = std::max(bounds_min_y_, map_min_y - padding);
    bounds_max_y_ = std::min(bounds_max_y_, map_max_y + padding);
    bounds_min_z_ = std::max(bounds_min_z_, map_min_z - padding);
    bounds_max_z_ = std::min(bounds_max_z_, map_max_z + padding);

    RCLCPP_DEBUG(this->get_logger(), 
                "Updated planning bounds: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]",
                bounds_min_x_, bounds_max_x_, bounds_min_y_, bounds_max_y_, 
                bounds_min_z_, bounds_max_z_);
}

rclcpp_action::GoalResponse OmplPlannerNode::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received planning request");

    // Check if we have an octree
    {
        std::lock_guard<std::mutex> lock(*octree_mutex_);
        if (!octree_) {
            RCLCPP_ERROR(this->get_logger(), "No OctoMap available for planning");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // Validate goal pose is within bounds
    if (!isPoseWithinBounds(goal->pose)) {
        RCLCPP_ERROR(this->get_logger(), "Goal pose is outside planning bounds");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted for planning");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OmplPlannerNode::handleCancel(
    const std::shared_ptr<GoalHandleNavigateToPose> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

bool OmplPlannerNode::isPoseWithinBounds(const geometry_msgs::msg::PoseStamped& pose)
{
    const auto& pos = pose.pose.position;
    return (pos.x >= bounds_min_x_ && pos.x <= bounds_max_x_ &&
            pos.y >= bounds_min_y_ && pos.y <= bounds_max_y_ &&
            pos.z >= bounds_min_z_ && pos.z <= bounds_max_z_);
}

void OmplPlannerNode::executePlan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing planning goal");
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<NavigateToPose::Result>();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    try {
        // Get current robot pose
        geometry_msgs::msg::PoseStamped start_pose;
        if (!getCurrentRobotPose(start_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current robot pose");
            result->error_code = 1;
            result->error_msg = "Failed to get current robot pose";
            goal_handle->abort(result);
            return;
        }

        // Initialize OMPL components
        if (!initializeOMPL()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OMPL components");
            result->error_code = 2;
            result->error_msg = "Failed to initialize OMPL components";
            goal_handle->abort(result);
            return;
        }

        // Create problem definition
        auto pdef = std::make_shared<ompl_base::ProblemDefinition>(space_info_);

        // Set start and goal states
        if (!setStartAndGoalStates(start_pose, goal->pose, pdef)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set start and goal states");
            result->error_code = 3;
            result->error_msg = "Failed to set start and goal states";
            goal_handle->abort(result);
            return;
        }

        // Create and configure planner
        auto planner = createPlanner();
        if (!planner) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create planner");
            result->error_code = 4;
            result->error_msg = "Failed to create planner";
            goal_handle->abort(result);
            return;
        }

        planner->setProblemDefinition(pdef);
        planner->setup();

        // Provide feedback
        feedback->current_pose = start_pose;
        goal_handle->publish_feedback(feedback);

        // Solve the planning problem
        RCLCPP_INFO(this->get_logger(), "Starting to solve planning problem with %s planner", 
                   planner_id_.c_str());
        
        ompl_base::PlannerStatus solved = planner->solve(planning_timeout_);

        if (solved) {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded!");
            
            // Get the solution path
            auto solution_path = pdef->getSolutionPath()->as<ompl_geometric::PathGeometric>();
            
            if (!solution_path) {
                RCLCPP_ERROR(this->get_logger(), "Failed to cast solution to PathGeometric");
                result->error_code = 5;
                result->error_msg = "Failed to cast solution to PathGeometric";
                goal_handle->abort(result);
                return;
            }

            // Simplify the path if enabled
            if (enable_path_simplification_ || enable_path_smoothing_) {
                RCLCPP_INFO(this->get_logger(), "Simplifying and smoothing path...");
                if (!simplifyPath(*solution_path)) {
                    RCLCPP_WARN(this->get_logger(), "Path simplification failed, using original path");
                }
            }

            // Convert to ROS path message
            auto ros_path = convertToNavMsgsPath(*solution_path, global_frame_);
            
            // If we had an unsafe start, prepend it to create a complete path
            if (has_unsafe_start_) {
                // Insert the original unsafe start position at the beginning
                ros_path.poses.insert(ros_path.poses.begin(), unsafe_start_pose_);
                
                RCLCPP_INFO(this->get_logger(), 
                           "✅ Complete path created: %zu waypoints (including unsafe start → safe position)",
                           ros_path.poses.size());
            }
            
            // Store and publish the path
            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                last_planned_path_ = ros_path;
            }
            path_pub_->publish(ros_path);
            
            // Set result
            result->error_code = NavigateToPose::Result::NONE;
            
            RCLCPP_INFO(this->get_logger(), "Planning completed successfully with %zu waypoints", 
                       ros_path.poses.size());
            goal_handle->succeed(result);
        }
        else {
            std::string status_msg;
            if (solved == ompl_base::PlannerStatus::TIMEOUT) {
                status_msg = "Planning timed out";
            } else if (solved == ompl_base::PlannerStatus::ABORT) {
                status_msg = "Planning was aborted";
            } else if (solved == ompl_base::PlannerStatus::CRASH) {
                status_msg = "Planner crashed";
            } else {
                status_msg = "Planning failed";
            }
            
            RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", status_msg.c_str());
            result->error_code = 6;
            result->error_msg = status_msg;
            goal_handle->abort(result);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning execution: %s", e.what());
        result->error_code = 7;
        result->error_msg = std::string("Exception: ") + e.what();
        goal_handle->abort(result);
    }
}

bool OmplPlannerNode::initializeOMPL()
{
    try {
        // Create SE(3) state space
        state_space_ = std::make_shared<ompl_base::SE3StateSpace>();

        // Set bounds for the R^3 (position) component
        ompl_base::RealVectorBounds bounds(3);
        bounds.setLow(0, bounds_min_x_);   // X-axis bounds
        bounds.setHigh(0, bounds_max_x_);
        bounds.setLow(1, bounds_min_y_);   // Y-axis bounds
        bounds.setHigh(1, bounds_max_y_);
        bounds.setLow(2, bounds_min_z_);   // Z-axis bounds
        bounds.setHigh(2, bounds_max_z_);

        state_space_->setBounds(bounds);

        // Create space information
        space_info_ = std::make_shared<ompl_base::SpaceInformation>(state_space_);

        // Create validity checker
        validity_checker_ = std::make_shared<OctoMapValidityChecker>(
            space_info_, octree_, robot_radius_, octree_mutex_);
        
        space_info_->setStateValidityChecker(validity_checker_);
        space_info_->setup();

        RCLCPP_DEBUG(this->get_logger(), "OMPL components initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in initializeOMPL: %s", e.what());
        return false;
    }
}

bool OmplPlannerNode::getCurrentRobotPose(geometry_msgs::msg::PoseStamped& pose)
{
    try {
        auto transform = tf_buffer_->lookupTransform(
            global_frame_, robot_frame_, tf2::TimePointZero);
        
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        
        return true;
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose: %s", ex.what());
        return false;
    }
}

bool OmplPlannerNode::setStartAndGoalStates(
    const geometry_msgs::msg::PoseStamped& start_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    std::shared_ptr<ompl_base::ProblemDefinition> problem_def)
{
    try {
        // Create start state
        ompl_base::ScopedState<> start_state(state_space_);
        auto* se3_start = start_state->as<ompl_base::SE3StateSpace::StateType>();
        
        // Set position
        se3_start->setXYZ(start_pose.pose.position.x, 
                         start_pose.pose.position.y, 
                         start_pose.pose.position.z);
        
        // Set orientation
        auto* rot_start = se3_start->as<ompl_base::SO3StateSpace::StateType>(1);
        rot_start->x = start_pose.pose.orientation.x;
        rot_start->y = start_pose.pose.orientation.y;
        rot_start->z = start_pose.pose.orientation.z;
        rot_start->w = start_pose.pose.orientation.w;

        // Create goal state
        ompl_base::ScopedState<> goal_state(state_space_);
        auto* se3_goal = goal_state->as<ompl_base::SE3StateSpace::StateType>();
        
        // Set position
        se3_goal->setXYZ(goal_pose.pose.position.x, 
                        goal_pose.pose.position.y, 
                        goal_pose.pose.position.z);
        
        // Set orientation
        auto* rot_goal = se3_goal->as<ompl_base::SO3StateSpace::StateType>(1);
        rot_goal->x = goal_pose.pose.orientation.x;
        rot_goal->y = goal_pose.pose.orientation.y;
        rot_goal->z = goal_pose.pose.orientation.z;
        rot_goal->w = goal_pose.pose.orientation.w;

        // Check start state - if invalid, find a safe nearby position
        has_unsafe_start_ = false;  // Reset flag
        
        if (!space_info_->isValid(start_state.get())) {
            RCLCPP_WARN(this->get_logger(), "Start state is too close to obstacles, finding safe position...");
            
            // Store the original unsafe start pose
            has_unsafe_start_ = true;
            const auto* se3_original = start_state->as<ompl_base::SE3StateSpace::StateType>();
            const auto* pos_original = se3_original->as<ompl_base::RealVectorStateSpace::StateType>(0);
            const auto* rot_original = se3_original->as<ompl_base::SO3StateSpace::StateType>(1);
            
            unsafe_start_pose_.header.frame_id = global_frame_;
            unsafe_start_pose_.header.stamp = this->get_clock()->now();
            unsafe_start_pose_.pose.position.x = pos_original->values[0];
            unsafe_start_pose_.pose.position.y = pos_original->values[1];
            unsafe_start_pose_.pose.position.z = pos_original->values[2];
            unsafe_start_pose_.pose.orientation.x = rot_original->x;
            unsafe_start_pose_.pose.orientation.y = rot_original->y;
            unsafe_start_pose_.pose.orientation.z = rot_original->z;
            unsafe_start_pose_.pose.orientation.w = rot_original->w;
            
            // Find the nearest safe position
            ompl_base::ScopedState<> safe_start(state_space_);
            if (!findNearestSafePosition(start_state, safe_start)) {
                RCLCPP_ERROR(this->get_logger(), "Could not find any safe position near start");
                return false;
            }
            
            // Use the safe position for main planning
            start_state = safe_start;
            
            const auto* se3_safe = safe_start->as<ompl_base::SE3StateSpace::StateType>();
            const auto* pos_safe = se3_safe->as<ompl_base::RealVectorStateSpace::StateType>(0);
            
            RCLCPP_INFO(this->get_logger(), 
                       "Will create path: Current(%.2f,%.2f,%.2f) → Safe(%.2f,%.2f,%.2f) → Goal",
                       pos_original->values[0], pos_original->values[1], pos_original->values[2],
                       pos_safe->values[0], pos_safe->values[1], pos_safe->values[2]);
        }
        
        if (!space_info_->isValid(goal_state.get())) {
            RCLCPP_ERROR(this->get_logger(), "Goal state is in collision");
            return false;
        }

        // Set start and goal in problem definition
        problem_def->setStartAndGoalStates(start_state, goal_state);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Set start pose: (%.2f, %.2f, %.2f) and goal pose: (%.2f, %.2f, %.2f)",
                   start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
                   goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in setStartAndGoalStates: %s", e.what());
        return false;
    }
}

ompl_base::PlannerPtr OmplPlannerNode::createPlanner()
{
    ompl_base::PlannerPtr planner;
    
    try {
        if (planner_id_ == "RRTConnect") {
            planner = std::make_shared<ompl_geometric::RRTConnect>(space_info_);
        }
        else if (planner_id_ == "RRTstar" || planner_id_ == "RRT*") {
            auto rrt_star = std::make_shared<ompl_geometric::RRTstar>(space_info_);
            // Set optimization objective (path length by default)
            auto obj = std::make_shared<ompl_base::PathLengthOptimizationObjective>(space_info_);
            rrt_star->getProblemDefinition()->setOptimizationObjective(obj);
            planner = rrt_star;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown planner ID '%s', defaulting to RRTConnect", 
                       planner_id_.c_str());
            planner = std::make_shared<ompl_geometric::RRTConnect>(space_info_);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Created planner: %s", planner->getName().c_str());
        return planner;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in createPlanner: %s", e.what());
        return nullptr;
    }
}

bool OmplPlannerNode::simplifyPath(ompl_geometric::PathGeometric& path)
{
    try {
        // Create path simplifier
        ompl_geometric::PathSimplifier simplifier(space_info_);
        
        // Record original path length and smoothness
        double original_length = path.length();
        double original_smoothness = path.smoothness();
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Original path: length=%.3f, smoothness=%.3f, waypoints=%zu",
                    original_length, original_smoothness, path.getStateCount());

        if (enable_path_simplification_) {
            // Perform path shortcutting
            simplifier.simplifyMax(path);
        }
        
        if (enable_path_smoothing_) {
            // Perform B-spline smoothing
            simplifier.smoothBSpline(path);
        }
        
        // Log improvements
        double new_length = path.length();
        double new_smoothness = path.smoothness();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Path simplified: length %.3f->%.3f (%.1f%%), smoothness %.3f->%.3f, waypoints %zu",
                   original_length, new_length, 
                   (original_length - new_length) / original_length * 100.0,
                   original_smoothness, new_smoothness, path.getStateCount());
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in simplifyPath: %s", e.what());
        return false;
    }
}

nav_msgs::msg::Path OmplPlannerNode::convertToNavMsgsPath(
    const ompl_geometric::PathGeometric& ompl_path,
    const std::string& frame_id)
{
    nav_msgs::msg::Path ros_path;
    ros_path.header.stamp = this->get_clock()->now();
    ros_path.header.frame_id = frame_id;

    // Use getStateCount() and getState(index) for const access
    size_t state_count = ompl_path.getStateCount();
    ros_path.poses.reserve(state_count);

    for (size_t i = 0; i < state_count; ++i) {
        const auto* state = ompl_path.getState(i);
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = ros_path.header;

        const auto* se3_state = state->as<ompl_base::SE3StateSpace::StateType>();
        const auto* pos = se3_state->as<ompl_base::RealVectorStateSpace::StateType>(0);
        const auto* rot = se3_state->as<ompl_base::SO3StateSpace::StateType>(1);

        // Set position
        pose_stamped.pose.position.x = pos->values[0];
        pose_stamped.pose.position.y = pos->values[1];
        pose_stamped.pose.position.z = pos->values[2];

        // Set orientation
        pose_stamped.pose.orientation.x = rot->x;
        pose_stamped.pose.orientation.y = rot->y;
        pose_stamped.pose.orientation.z = rot->z;
        pose_stamped.pose.orientation.w = rot->w;

        ros_path.poses.push_back(pose_stamped);
    }

    return ros_path;
}

void OmplPlannerNode::republishLastPath()
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!last_planned_path_.poses.empty()) {
        // Update timestamp to keep the path fresh in RViz
        last_planned_path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(last_planned_path_);
    }
}

bool OmplPlannerNode::findNearestSafePosition(const ompl_base::ScopedState<>& unsafe_state, ompl_base::ScopedState<>& safe_state)
{
    if (!space_info_) {
        return false;
    }

    const auto* se3_unsafe = unsafe_state->as<ompl_base::SE3StateSpace::StateType>();
    const auto* pos_unsafe = se3_unsafe->as<ompl_base::RealVectorStateSpace::StateType>(0);
    
    double start_x = pos_unsafe->values[0];
    double start_y = pos_unsafe->values[1]; 
    double start_z = pos_unsafe->values[2];
    
    RCLCPP_INFO(this->get_logger(), 
                "Searching for safe position near (%.2f, %.2f, %.2f) with %.1fm clearance...",
                start_x, start_y, start_z, robot_radius_);

    // Search in expanding spheres around the unsafe position
    std::vector<double> search_radii = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0}; // meters
    int samples_per_radius = 50;
    
    for (double search_radius : search_radii) {
        RCLCPP_DEBUG(this->get_logger(), "Searching at radius %.1fm...", search_radius);
        
        for (int i = 0; i < samples_per_radius; i++) {
            // Generate random point on sphere surface
            double theta = 2.0 * M_PI * (double)rand() / RAND_MAX;  // azimuth 0-2π
            double phi = acos(2.0 * (double)rand() / RAND_MAX - 1.0); // polar angle 0-π
            
            // Convert spherical to cartesian
            double dx = search_radius * sin(phi) * cos(theta);
            double dy = search_radius * sin(phi) * sin(theta);  
            double dz = search_radius * cos(phi);
            
            // Create candidate safe position
            ompl_base::ScopedState<> candidate_state(state_space_);
            auto* se3_candidate = candidate_state->as<ompl_base::SE3StateSpace::StateType>();
            
            // Set position
            se3_candidate->setXYZ(start_x + dx, start_y + dy, start_z + dz);
            
            // Keep same orientation as original
            const auto* rot_unsafe = se3_unsafe->as<ompl_base::SO3StateSpace::StateType>(1);
            auto* rot_candidate = se3_candidate->as<ompl_base::SO3StateSpace::StateType>(1);
            rot_candidate->x = rot_unsafe->x;
            rot_candidate->y = rot_unsafe->y;
            rot_candidate->z = rot_unsafe->z;
            rot_candidate->w = rot_unsafe->w;
            
            // Check if this position is valid and within bounds
            if (space_info_->satisfiesBounds(candidate_state.get()) && 
                space_info_->isValid(candidate_state.get())) {
                
                RCLCPP_INFO(this->get_logger(), 
                           "✅ Found safe position: (%.2f, %.2f, %.2f) at distance %.1fm",
                           start_x + dx, start_y + dy, start_z + dz, search_radius);
                safe_state = candidate_state;
                return true;
            }
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "❌ No safe position found within 4m search radius!");
    return false;
}

} // namespace blimp_ompl_planner

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<blimp_ompl_planner::OmplPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}