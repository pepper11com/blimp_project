#include "blimp_ompl_planner/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace blimp_ompl_planner
{

  PathPlanner::PathPlanner(const PlannerParameters &parameters,
                           OctomapManager &octomap_manager,
                           const rclcpp::Logger &logger,
                           const rclcpp::Clock::SharedPtr &clock)
      : parameters_(parameters),
        octomap_manager_(octomap_manager),
        logger_(logger),
        clock_(clock),
        has_unsafe_start_(false)
  {
  }

  bool PathPlanner::hasValidMap() const
  {
    return octomap_manager_.hasMap();
  }

  bool PathPlanner::plan(const geometry_msgs::msg::PoseStamped &start_pose,
                         const geometry_msgs::msg::PoseStamped &goal_pose,
                         nav_msgs::msg::Path &planned_path,
                         std::string &error_message)
  {
    if (!hasValidMap())
    {
      error_message = "No OctoMap available for planning";
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    if (!initializeOmpl())
    {
      error_message = "Failed to initialise OMPL";
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    auto problem_def = std::make_shared<ompl_base::ProblemDefinition>(space_info_);

    if (!setStartAndGoalStates(start_pose, goal_pose, problem_def))
    {
      error_message = "Failed to configure start and goal states";
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    auto planner = createPlanner();
    if (!planner)
    {
      error_message = "Failed to create planner";
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    planner->setProblemDefinition(problem_def);
    planner->setup();

    RCLCPP_INFO(logger_, "Solving planning problem with %s", parameters_.planner_id.c_str());

    const auto status = planner->solve(parameters_.planning_timeout);

    if (!status)
    {
      if (status == ompl_base::PlannerStatus::TIMEOUT)
      {
        error_message = "Planning timed out";
      }
      else if (status == ompl_base::PlannerStatus::CRASH)
      {
        error_message = "Planner crashed";
      }
      else if (status == ompl_base::PlannerStatus::ABORT)
      {
        error_message = "Planning aborted";
      }
      else
      {
        error_message = "Planning failed";
      }
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    auto solution_path = std::dynamic_pointer_cast<ompl_geometric::PathGeometric>(
        problem_def->getSolutionPath());
    if (!solution_path)
    {
      error_message = "Planner returned an invalid solution path";
      RCLCPP_ERROR(logger_, "%s", error_message.c_str());
      return false;
    }

    if (!simplifyPath(*solution_path))
    {
      RCLCPP_WARN(logger_, "Path simplification failed; using raw solution path");
    }

    planned_path = convertToNavMsgsPath(*solution_path);

    if (has_unsafe_start_)
    {
      planned_path.poses.insert(planned_path.poses.begin(), unsafe_start_pose_);
    }

    RCLCPP_INFO(logger_, "Planning succeeded with %zu waypoints", planned_path.poses.size());
    return true;
  }

  bool PathPlanner::initializeOmpl()
  {
    const auto bounds = octomap_manager_.bounds();
    auto octree = octomap_manager_.octree();
    if (!octree)
    {
      RCLCPP_ERROR(logger_, "Cannot initialise OMPL without an OctoMap");
      return false;
    }

    state_space_ = std::make_shared<ompl_base::SE3StateSpace>();

    ompl_base::RealVectorBounds se3_bounds(3);
    se3_bounds.setLow(0, bounds.min_x);
    se3_bounds.setHigh(0, bounds.max_x);
    se3_bounds.setLow(1, bounds.min_y);
    se3_bounds.setHigh(1, bounds.max_y);
    se3_bounds.setLow(2, bounds.min_z);
    se3_bounds.setHigh(2, bounds.max_z);
    state_space_->setBounds(se3_bounds);

    space_info_ = std::make_shared<ompl_base::SpaceInformation>(state_space_);
    validity_checker_ = std::make_shared<OctoMapValidityChecker>(space_info_, octree,
                                                                 parameters_.robot_radius,
                                                                 octomap_manager_.mutex());
    space_info_->setStateValidityChecker(validity_checker_);
    space_info_->setup();

    return true;
  }

  ompl_base::PlannerPtr PathPlanner::createPlanner()
  {
    if (parameters_.planner_id == "RRTConnect")
    {
      return std::make_shared<ompl_geometric::RRTConnect>(space_info_);
    }

    if (parameters_.planner_id == "RRTstar" || parameters_.planner_id == "RRT*")
    {
      auto planner = std::make_shared<ompl_geometric::RRTstar>(space_info_);
      auto objective = std::make_shared<ompl_base::PathLengthOptimizationObjective>(space_info_);
      planner->getProblemDefinition()->setOptimizationObjective(objective);
      return planner;
    }

    RCLCPP_WARN(logger_, "Unknown planner '%s'; defaulting to RRTConnect", parameters_.planner_id.c_str());
    return std::make_shared<ompl_geometric::RRTConnect>(space_info_);
  }

  bool PathPlanner::setStartAndGoalStates(const geometry_msgs::msg::PoseStamped &start_pose,
                                          const geometry_msgs::msg::PoseStamped &goal_pose,
                                          const std::shared_ptr<ompl_base::ProblemDefinition> &problem_def)
  {
    has_unsafe_start_ = false;

    ompl_base::ScopedState<> start_state(state_space_);
    auto *se3_start = start_state->as<ompl_base::SE3StateSpace::StateType>();
    se3_start->setXYZ(start_pose.pose.position.x,
                      start_pose.pose.position.y,
                      start_pose.pose.position.z);
    auto *rot_start = se3_start->as<ompl_base::SO3StateSpace::StateType>(1);
    rot_start->x = start_pose.pose.orientation.x;
    rot_start->y = start_pose.pose.orientation.y;
    rot_start->z = start_pose.pose.orientation.z;
    rot_start->w = start_pose.pose.orientation.w;

    ompl_base::ScopedState<> goal_state(state_space_);
    auto *se3_goal = goal_state->as<ompl_base::SE3StateSpace::StateType>();
    se3_goal->setXYZ(goal_pose.pose.position.x,
                     goal_pose.pose.position.y,
                     goal_pose.pose.position.z);
    auto *rot_goal = se3_goal->as<ompl_base::SO3StateSpace::StateType>(1);
    rot_goal->x = goal_pose.pose.orientation.x;
    rot_goal->y = goal_pose.pose.orientation.y;
    rot_goal->z = goal_pose.pose.orientation.z;
    rot_goal->w = goal_pose.pose.orientation.w;

    if (!space_info_->isValid(start_state.get()))
    {
      has_unsafe_start_ = true;
      const auto *se3_original = start_state->as<ompl_base::SE3StateSpace::StateType>();
      const auto *pos_original = se3_original->as<ompl_base::RealVectorStateSpace::StateType>(0);
      const auto *rot_original = se3_original->as<ompl_base::SO3StateSpace::StateType>(1);

      unsafe_start_pose_.header = start_pose.header;
      unsafe_start_pose_.pose.position.x = pos_original->values[0];
      unsafe_start_pose_.pose.position.y = pos_original->values[1];
      unsafe_start_pose_.pose.position.z = pos_original->values[2];
      unsafe_start_pose_.pose.orientation.x = rot_original->x;
      unsafe_start_pose_.pose.orientation.y = rot_original->y;
      unsafe_start_pose_.pose.orientation.z = rot_original->z;
      unsafe_start_pose_.pose.orientation.w = rot_original->w;

      ompl_base::ScopedState<> safe_state(state_space_);
      if (!findNearestSafePosition(start_state, safe_state))
      {
        RCLCPP_ERROR(logger_, "Unable to find safe start state away from obstacles");
        return false;
      }
      start_state = safe_state;
    }

    if (!space_info_->isValid(goal_state.get()))
    {
      RCLCPP_ERROR(logger_, "Goal state is in collision");
      return false;
    }

    problem_def->setStartAndGoalStates(start_state, goal_state);
    return true;
  }

  bool PathPlanner::simplifyPath(ompl_geometric::PathGeometric &path) const
  {
    if (!parameters_.path_processing.enable_simplification &&
        !parameters_.path_processing.enable_smoothing)
    {
      return true;
    }

    ompl_geometric::PathSimplifier simplifier(space_info_);
    double original_length = path.length();

    if (parameters_.path_processing.enable_simplification)
    {
      const int max_steps = parameters_.path_processing.max_simplification_steps;
      if (max_steps > 0)
      {
        for (int step = 0; step < max_steps; ++step)
        {
          if (!simplifier.shortcutPath(path))
          {
            break;
          }
        }
      }
      else
      {
        simplifier.shortcutPath(path);
      }
    }

    if (parameters_.path_processing.enable_smoothing)
    {
      simplifier.smoothBSpline(path);
    }

    double new_length = path.length();
    RCLCPP_INFO(logger_, "Path length %.3f -> %.3f", original_length, new_length);
    return true;
  }

  nav_msgs::msg::Path PathPlanner::convertToNavMsgsPath(const ompl_geometric::PathGeometric &ompl_path) const
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = clock_->now();
    path_msg.header.frame_id = parameters_.global_frame;
    path_msg.poses.reserve(ompl_path.getStateCount());

    for (std::size_t i = 0; i < ompl_path.getStateCount(); ++i)
    {
      const auto *state = ompl_path.getState(i);
      const auto *se3 = state->as<ompl_base::SE3StateSpace::StateType>();
      const auto *pos = se3->as<ompl_base::RealVectorStateSpace::StateType>(0);
      const auto *rot = se3->as<ompl_base::SO3StateSpace::StateType>(1);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pos->values[0];
      pose.pose.position.y = pos->values[1];
      pose.pose.position.z = pos->values[2];
      pose.pose.orientation.x = rot->x;
      pose.pose.orientation.y = rot->y;
      pose.pose.orientation.z = rot->z;
      pose.pose.orientation.w = rot->w;

      path_msg.poses.emplace_back(std::move(pose));
    }

    return path_msg;
  }

  bool PathPlanner::findNearestSafePosition(const ompl_base::ScopedState<> &unsafe_state,
                                            ompl_base::ScopedState<> &safe_state)
  {
    if (!space_info_)
    {
      return false;
    }

    const auto *se3_unsafe = unsafe_state->as<ompl_base::SE3StateSpace::StateType>();
    const auto *pos_unsafe = se3_unsafe->as<ompl_base::RealVectorStateSpace::StateType>(0);

    const double start_x = pos_unsafe->values[0];
    const double start_y = pos_unsafe->values[1];
    const double start_z = pos_unsafe->values[2];

    RCLCPP_INFO(logger_, "Searching for safe start near (%.2f, %.2f, %.2f)", start_x, start_y, start_z);

    const std::vector<double> search_radii = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0};
    std::mt19937 rng(static_cast<unsigned>(clock_->now().nanoseconds() & 0xffffffff));
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (double radius : search_radii)
    {
      for (int sample = 0; sample < 50; ++sample)
      {
        const double theta = 2.0 * M_PI * uniform(rng);
        const double phi = std::acos(2.0 * uniform(rng) - 1.0);

        const double dx = radius * std::sin(phi) * std::cos(theta);
        const double dy = radius * std::sin(phi) * std::sin(theta);
        const double dz = radius * std::cos(phi);

        ompl_base::ScopedState<> candidate(state_space_);
        auto *se3_candidate = candidate->as<ompl_base::SE3StateSpace::StateType>();
        se3_candidate->setXYZ(start_x + dx, start_y + dy, start_z + dz);

        auto *rot_candidate = se3_candidate->as<ompl_base::SO3StateSpace::StateType>(1);
        const auto *rot_original = se3_unsafe->as<ompl_base::SO3StateSpace::StateType>(1);
        rot_candidate->x = rot_original->x;
        rot_candidate->y = rot_original->y;
        rot_candidate->z = rot_original->z;
        rot_candidate->w = rot_original->w;

        if (space_info_->satisfiesBounds(candidate.get()) && space_info_->isValid(candidate.get()))
        {
          safe_state = candidate;
          RCLCPP_INFO(logger_, "Found safe start at radius %.2f m", radius);
          return true;
        }
      }
    }

    RCLCPP_ERROR(logger_, "Failed to find safe start position within search radius");
    return false;
  }

} // namespace blimp_ompl_planner
