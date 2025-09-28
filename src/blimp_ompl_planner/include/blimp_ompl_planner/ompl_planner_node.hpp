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

#include "blimp_ompl_planner/octomap_manager.hpp"
#include "blimp_ompl_planner/path_planner.hpp"
#include "blimp_ompl_planner/planner_types.hpp"

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

        PlannerParameters parameters_;
        OctomapManager octomap_manager_;
        PathPlanner path_planner_;

        // Last planned path for republishing
        nav_msgs::msg::Path last_planned_path_;
        std::mutex path_mutex_;

        /**
         * @brief Callback for receiving OctoMap messages
         *
         * @param msg OctoMap message from the mapping system
         */
        void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr &msg);

        /**
         * @brief Handle goal requests from action clients
         *
         * @param uuid Goal UUID
         * @param goal Navigation goal
         * @return Goal response (accept/reject)
         */
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &uuid,
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
         * @brief Get the current robot pose from TF
         *
         * @param pose Output pose
         * @return true if successful, false otherwise
         */
        bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped &pose);

        /**
         * @brief Declare and get node parameters
         */
        PlannerParameters declareParameters();

        /**
         * @brief Check if a pose is within the planning bounds
         *
         * @param pose Pose to check
         * @return true if within bounds, false otherwise
         */
        bool isPoseWithinBounds(const geometry_msgs::msg::PoseStamped &pose);

        /**
         * @brief Timer callback to republish the last planned path
         */
        void republishLastPath();
    };

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__OMPL_PLANNER_NODE_HPP_