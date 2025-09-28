#include "blimp_ompl_planner/octomap_manager.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace blimp_ompl_planner
{

  OctomapManager::OctomapManager(const PlanningBounds &initial_bounds, double robot_radius)
      : initial_bounds_(initial_bounds),
        current_bounds_(initial_bounds),
        robot_radius_(robot_radius),
        mutex_(std::make_shared<std::mutex>())
  {
  }

  bool OctomapManager::updateFromMessage(const octomap_msgs::msg::Octomap &msg, const rclcpp::Logger &logger)
  {
    if (msg.data.empty())
    {
      RCLCPP_WARN(logger, "Received an empty OctoMap message");
      return false;
    }

    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::binaryMsgToMap(msg));
    if (!abstract_tree)
    {
      RCLCPP_ERROR(logger, "Failed to deserialize OctoMap message");
      return false;
    }

    std::shared_ptr<octomap::OcTree> new_tree;

    if (auto *octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get()))
    {
      new_tree.reset(octree);
      abstract_tree.release();
    }
    else if (auto *color_tree = dynamic_cast<octomap::ColorOcTree *>(abstract_tree.get()))
    {
      RCLCPP_INFO(logger, "Received ColorOcTree, converting to binary OcTree");
      auto converted_tree = std::make_shared<octomap::OcTree>(color_tree->getResolution());
      for (auto it = color_tree->begin_leafs(); it != color_tree->end_leafs(); ++it)
      {
        if (color_tree->isNodeOccupied(*it))
        {
          converted_tree->updateNode(it.getKey(), true);
        }
      }
      converted_tree->updateInnerOccupancy();
      new_tree = std::move(converted_tree);
    }
    else
    {
      RCLCPP_ERROR(logger, "Deserialized map is not of supported type (OcTree or ColorOcTree)");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(*mutex_);
      octree_ = std::move(new_tree);
      if (octree_)
      {
        updateBoundsLocked(*octree_);
      }
    }

    if (octree_)
    {
      RCLCPP_INFO(logger, "Updated OctoMap with %zu leaf nodes", octree_->getNumLeafNodes());
      return true;
    }

    return false;
  }

  bool OctomapManager::hasMap() const
  {
    return static_cast<bool>(octree_);
  }

  PlanningBounds OctomapManager::bounds() const
  {
    return current_bounds_;
  }

  bool OctomapManager::isWithinBounds(const geometry_msgs::msg::PoseStamped &pose) const
  {
    const auto &position = pose.pose.position;
    return position.x >= current_bounds_.min_x && position.x <= current_bounds_.max_x &&
           position.y >= current_bounds_.min_y && position.y <= current_bounds_.max_y &&
           position.z >= current_bounds_.min_z && position.z <= current_bounds_.max_z;
  }

  std::shared_ptr<octomap::OcTree> OctomapManager::octree() const
  {
    return octree_;
  }

  std::shared_ptr<std::mutex> OctomapManager::mutex() const
  {
    return mutex_;
  }

  void OctomapManager::setRobotRadius(double robot_radius)
  {
    robot_radius_ = robot_radius;
    if (octree_)
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      updateBoundsLocked(*octree_);
    }
  }

  void OctomapManager::updateBoundsLocked(const octomap::OcTree &tree)
  {
    double map_min_x;
    double map_min_y;
    double map_min_z;
    double map_max_x;
    double map_max_y;
    double map_max_z;

    tree.getMetricMin(map_min_x, map_min_y, map_min_z);
    tree.getMetricMax(map_max_x, map_max_y, map_max_z);

    const double padding = robot_radius_ * 2.0;

    current_bounds_.min_x = std::max(initial_bounds_.min_x, map_min_x - padding);
    current_bounds_.max_x = std::min(initial_bounds_.max_x, map_max_x + padding);
    current_bounds_.min_y = std::max(initial_bounds_.min_y, map_min_y - padding);
    current_bounds_.max_y = std::min(initial_bounds_.max_y, map_max_y + padding);
    current_bounds_.min_z = std::max(initial_bounds_.min_z, map_min_z - padding);
    current_bounds_.max_z = std::min(initial_bounds_.max_z, map_max_z + padding);
  }

} // namespace blimp_ompl_planner
