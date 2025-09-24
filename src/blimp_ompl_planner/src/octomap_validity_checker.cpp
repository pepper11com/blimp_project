#include "blimp_ompl_planner/octomap_validity_checker.hpp"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace blimp_ompl_planner
{

OctoMapValidityChecker::OctoMapValidityChecker(
    const ompl::base::SpaceInformationPtr& si,
    std::shared_ptr<octomap::OcTree> tree,
    double robot_radius,
    std::shared_ptr<std::mutex> mutex)
    : ompl::base::StateValidityChecker(si)
    , tree_(tree)
    , robot_radius_(robot_radius)
    , tree_mutex_(mutex)
{
}

bool OctoMapValidityChecker::isValid(const ompl::base::State* state) const
{
    // Lock the mutex for thread-safe access to the tree
    std::lock_guard<std::mutex> lock(*tree_mutex_);
    
    if (!tree_) {
        // If we don't have a map yet, consider all states invalid
        // This prevents planning in an unknown environment
        return false;
    }

    // Cast the OMPL state to the specific SE(3) state type
    const auto* se3_state = state->as<ompl::base::SE3StateSpace::StateType>();
    const auto* position = se3_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // Extract position coordinates
    double x = position->values[0];
    double y = position->values[1];
    double z = position->values[2];
    
    octomap::point3d center(x, y, z);

    // NEW APPROACH: Check if ANY occupied voxel is within robot_radius distance
    // This ensures the robot stays robot_radius meters away from EVERY obstacle point
    
    double resolution = tree_->getResolution();
    octomap::point3d robot_center(x, y, z);
    
    // Debug logging
    static int log_counter = 0;
    if (robot_radius_ > 2.0 && ++log_counter % 100 == 0) {
        std::cout << "🔍 Checking " << robot_radius_ << "m clearance around (" 
                  << x << ", " << y << ", " << z << ")" << std::endl;
    }
    
    // Search in a cubic region around the robot center
    double search_radius = robot_radius_ + resolution; // Add resolution for safety
    
    // Use OctoMap's built-in search functions for efficiency
    octomap::point3d min_point(x - search_radius, y - search_radius, z - search_radius);
    octomap::point3d max_point(x + search_radius, y + search_radius, z + search_radius);
    
    // Iterate through all occupied voxels in the search region
    for (auto it = tree_->begin_leafs_bbx(min_point, max_point); it != tree_->end_leafs_bbx(); ++it) {
        if (tree_->isNodeOccupied(*it)) {
            // Get the coordinate of this occupied voxel
            octomap::point3d voxel_center = it.getCoordinate();
            
            // Calculate distance from robot center to this occupied voxel
            double distance = robot_center.distance(voxel_center);
            
            // If ANY occupied voxel is within robot_radius, REJECT this state
            if (distance <= robot_radius_) {
                static int collision_counter = 0;
                if (robot_radius_ > 2.0 && ++collision_counter % 10 == 0) {
                    std::cout << "🔴 COLLISION: Obstacle at distance " << distance 
                              << "m (need " << robot_radius_ << "m clearance)" << std::endl;
                }
                return false; // TOO CLOSE TO OBSTACLE!
            }
        }
    }
    
    return true; // State is valid (no collisions)
}

void OctoMapValidityChecker::updateOctoMap(std::shared_ptr<octomap::OcTree> new_tree)
{
    std::lock_guard<std::mutex> lock(*tree_mutex_);
    tree_ = new_tree;
}

double OctoMapValidityChecker::clearance(const ompl::base::State* state) const
{
    std::lock_guard<std::mutex> lock(*tree_mutex_);
    
    if (!tree_) {
        return 0.0; // No clearance if no map
    }

    const auto* se3_state = state->as<ompl::base::SE3StateSpace::StateType>();
    const auto* position = se3_state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    
    octomap::point3d center(position->values[0], position->values[1], position->values[2]);
    
    // Search for the nearest obstacle within a reasonable radius
    double search_radius = std::max(5.0, robot_radius_ * 3.0);
    double min_distance = getMinDistanceToObstacles(center, search_radius);
    
    // Subtract robot radius to get clearance (free space between robot surface and obstacles)
    double clearance_value = std::max(0.0, min_distance - robot_radius_);
    
    return std::min(clearance_value, MAX_CLEARANCE);
}

bool OctoMapValidityChecker::isPointOccupied(const octomap::point3d& point) const
{
    octomap::OcTreeNode* node = tree_->search(point);
    
    if (node == nullptr) {
        // Point is in unknown space - treat as free for optimistic planning
        // For conservative planning, you could return true here
        return false;
    }
    
    // Check if the node is occupied
    bool occupied = tree_->isNodeOccupied(node);
    
    // Debug: Log when we find occupied voxels for large robot radius
    static int debug_counter = 0;
    if (occupied && robot_radius_ > 2.0 && ++debug_counter % 50 == 0) {
        std::cout << "🔴 Found occupied voxel at (" << point.x() << ", " << point.y() << ", " << point.z() 
                  << ") - robot should stay " << robot_radius_ << "m away" << std::endl;
    }
    
    return occupied;
}

double OctoMapValidityChecker::getMinDistanceToObstacles(
    const octomap::point3d& center, 
    double radius) const
{
    double min_distance = MAX_CLEARANCE;
    double resolution = tree_->getResolution();
    double step_size = resolution;
    
    // Search in a spherical region around the center point
    for (double dx = -radius; dx <= radius; dx += step_size) {
        for (double dy = -radius; dy <= radius; dy += step_size) {
            for (double dz = -radius; dz <= radius; dz += step_size) {
                
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (distance > radius) {
                    continue;
                }
                
                octomap::point3d query_point(
                    center.x() + dx, 
                    center.y() + dy, 
                    center.z() + dz
                );
                
                if (isPointOccupied(query_point)) {
                    min_distance = std::min(min_distance, distance);
                }
            }
        }
    }
    
    return min_distance;
}

} // namespace blimp_ompl_planner