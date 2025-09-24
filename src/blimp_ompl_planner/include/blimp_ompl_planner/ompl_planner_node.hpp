#ifndef BLIMP_OMPL_PLANNER__OMPL_PLANNER_NODE_HPP_
#define BLIMP_OMPL_PLANNER__OMPL_PLANNER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <octomap/octomap.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include "blimp_ompl_planner/octomap_validity_checker.hpp"

namespace ompl_base = ompl::base;
namespace ompl_geometric = ompl::geometric;

namespace blimp_ompl_planner
{

/**
 * @brief ROS2 Node that provides 3D motion planning using OMPL and OctoMap
 * 
 * This node implements a complete 3D motion planning system for aerial vehicles.
 * It uses OMPL for path planning in SE(3) space and OctoMap for obstacle representation.
 * The node provides an action server interface compatible with nav2_msgs/NavigateToPose.
 */
class OmplPlannerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor
     */
    OmplPlannerNode();

    /**
     * @brief Destructor
     */
    ~OmplPlannerNode() = default;

private:
    // ROS 2 Interfaces
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr path_republish_timer_;
    
    // TF2 for getting robot pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // OctoMap Data
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<std::mutex> octree_mutex_;

    // OMPL components (recreated for each planning request)
    std::shared_ptr<ompl_base::SE3StateSpace> state_space_;
    std::shared_ptr<ompl_base::SpaceInformation> space_info_;
    std::shared_ptr<OctoMapValidityChecker> validity_checker_;
    std::shared_ptr<ompl_geometric::SimpleSetup> simple_setup_;

    // Node Parameters
    double robot_radius_;
    double planning_timeout_;
    std::string planner_id_;
    std::string global_frame_;
    std::string robot_frame_;
    
    // Workspace bounds
    double bounds_min_x_, bounds_max_x_;
    double bounds_min_y_, bounds_max_y_;
    double bounds_min_z_, bounds_max_z_;
    
        // Path processing parameters
    bool enable_path_simplification_;
    bool enable_path_smoothing_;
    int max_simplification_steps_;
    
    // Last planned path for republishing
    nav_msgs::msg::Path last_planned_path_;
    std::mutex path_mutex_;
    
    // Track if we need to prepend unsafe start position to path
    bool has_unsafe_start_;
    geometry_msgs::msg::PoseStamped unsafe_start_pose_;

    /**
     * @brief Callback for receiving OctoMap messages
     * 
     * @param msg OctoMap message from the mapping system
     */
    void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr& msg);

    /**
     * @brief Handle goal requests from action clients
     * 
     * @param uuid Goal UUID
     * @param goal Navigation goal
     * @return Goal response (accept/reject)
     */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);

    /**
     * @brief Handle goal cancellation requests
     * 
     * @param goal_handle Goal handle to cancel
     * @return Cancel response
     */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    /**
     * @brief Execute the planning task
     * 
     * @param goal_handle Goal handle containing start and goal poses
     */
    void executePlan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    /**
     * @brief Initialize OMPL components for planning
     * 
     * @return true if successful, false otherwise
     */
    bool initializeOMPL();

    /**
     * @brief Get the current robot pose from TF
     * 
     * @param pose Output pose
     * @return true if successful, false otherwise
     */
    bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief Set start and goal states for the planner
     * 
     * @param start_pose Start pose
     * @param goal_pose Goal pose
     * @param problem_def Problem definition to configure
     * @return true if successful, false otherwise
     */
    bool setStartAndGoalStates(
        const geometry_msgs::msg::PoseStamped& start_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        std::shared_ptr<ompl_base::ProblemDefinition> problem_def);

    /**
     * @brief Create and configure the specified planner
     * 
     * @return Configured planner instance
     */
    ompl_base::PlannerPtr createPlanner();

    /**
     * @brief Simplify and smooth a path using OMPL's PathSimplifier
     * 
     * @param path Path to simplify
     * @return true if successful, false otherwise
     */
    bool simplifyPath(ompl_geometric::PathGeometric& path);

    /**
     * @brief Convert OMPL path to ROS nav_msgs/Path
     * 
     * @param ompl_path OMPL path to convert
     * @param frame_id Frame ID for the path
     * @return ROS path message
     */
    nav_msgs::msg::Path convertToNavMsgsPath(
        const ompl_geometric::PathGeometric& ompl_path,
        const std::string& frame_id);

    /**
     * @brief Declare and get node parameters
     */
    void declareParameters();

    /**
     * @brief Validate planning bounds based on the current OctoMap
     */
    void updatePlanningBounds();

    /**
     * @brief Check if a pose is within the planning bounds
     * 
     * @param pose Pose to check
     * @return true if within bounds, false otherwise
     */
    bool isPoseWithinBounds(const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief Timer callback to republish the last planned path
     */
    void republishLastPath();

    /**
     * @brief Find the nearest safe position away from obstacles
     * 
     * @param unsafe_state The current unsafe state
     * @return Safe state at least robot_radius away from obstacles, or empty state if none found
     */
    bool findNearestSafePosition(const ompl_base::ScopedState<>& unsafe_state, ompl_base::ScopedState<>& safe_state);
};

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__OMPL_PLANNER_NODE_HPP_