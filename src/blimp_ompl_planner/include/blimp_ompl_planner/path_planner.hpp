#ifndef BLIMP_OMPL_PLANNER__PATH_PLANNER_HPP_
#define BLIMP_OMPL_PLANNER__PATH_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "blimp_ompl_planner/octomap_manager.hpp"
#include "blimp_ompl_planner/planner_types.hpp"
#include "blimp_ompl_planner/octomap_validity_checker.hpp"

namespace ompl_base = ompl::base;
namespace ompl_geometric = ompl::geometric;

namespace blimp_ompl_planner
{

    class PathPlanner
    {
    public:
        PathPlanner(const PlannerParameters &parameters,
                    OctomapManager &octomap_manager,
                    const rclcpp::Logger &logger,
                    const rclcpp::Clock::SharedPtr &clock);

        bool hasValidMap() const;

        bool plan(const geometry_msgs::msg::PoseStamped &start_pose,
                  const geometry_msgs::msg::PoseStamped &goal_pose,
                  nav_msgs::msg::Path &planned_path,
                  std::string &error_message);

    private:
        bool initializeOmpl();

        ompl_base::PlannerPtr createPlanner();

        bool setStartAndGoalStates(const geometry_msgs::msg::PoseStamped &start_pose,
                                   const geometry_msgs::msg::PoseStamped &goal_pose,
                                   const std::shared_ptr<ompl_base::ProblemDefinition> &problem_def);

        bool simplifyPath(ompl_geometric::PathGeometric &path) const;

        nav_msgs::msg::Path convertToNavMsgsPath(const ompl_geometric::PathGeometric &ompl_path) const;

        bool findNearestSafePosition(const ompl_base::ScopedState<> &unsafe_state,
                                     ompl_base::ScopedState<> &safe_state);

        PlannerParameters parameters_;
        OctomapManager &octomap_manager_;
        rclcpp::Logger logger_;
        rclcpp::Clock::SharedPtr clock_;

        std::shared_ptr<ompl_base::SE3StateSpace> state_space_;
        std::shared_ptr<ompl_base::SpaceInformation> space_info_;
        std::shared_ptr<OctoMapValidityChecker> validity_checker_;

        bool has_unsafe_start_;
        geometry_msgs::msg::PoseStamped unsafe_start_pose_;
    };

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__PATH_PLANNER_HPP_
