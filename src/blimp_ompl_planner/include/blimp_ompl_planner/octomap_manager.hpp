#ifndef BLIMP_OMPL_PLANNER__OCTOMAP_MANAGER_HPP_
#define BLIMP_OMPL_PLANNER__OCTOMAP_MANAGER_HPP_

#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/logger.hpp"

#include <octomap/octomap.h>

#include "blimp_ompl_planner/planner_types.hpp"

namespace blimp_ompl_planner
{

  class OctomapManager
  {
  public:
    OctomapManager(const PlanningBounds &initial_bounds, double robot_radius);

    bool updateFromMessage(const octomap_msgs::msg::Octomap &msg, const rclcpp::Logger &logger);

    bool hasMap() const;

    PlanningBounds bounds() const;

    bool isWithinBounds(const geometry_msgs::msg::PoseStamped &pose) const;

    std::shared_ptr<octomap::OcTree> octree() const;

    std::shared_ptr<std::mutex> mutex() const;

    void setRobotRadius(double robot_radius);

  private:
    void updateBoundsLocked(const octomap::OcTree &tree);

    PlanningBounds initial_bounds_;
    PlanningBounds current_bounds_;
    double robot_radius_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<std::mutex> mutex_;
  };

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__OCTOMAP_MANAGER_HPP_
