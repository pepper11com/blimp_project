#ifndef BLIMP_NAVIGATION__PATH_PLANNER_HPP_
#define BLIMP_NAVIGATION__PATH_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/point.hpp"
#include "octomap/octomap.h"

namespace blimp_navigation_cpp
{

struct Node {
    double x, y, z;
    double g_cost;  // Cost from start
    double h_cost;  // Heuristic cost to goal
    double f_cost() const { return g_cost + h_cost; }
    Node* parent;
    
    Node(double x, double y, double z) : x(x), y(y), z(z), g_cost(0), h_cost(0), parent(nullptr) {}
    
    bool operator==(const Node& other) const {
        return std::abs(x - other.x) < 0.1 && std::abs(y - other.y) < 0.1 && std::abs(z - other.z) < 0.1;
    }
};

struct NodeHash {
    std::size_t operator()(const Node& node) const {
        // Simple grid-based hash for 0.1m resolution
        int x_grid = static_cast<int>(node.x * 10);
        int y_grid = static_cast<int>(node.y * 10);
        int z_grid = static_cast<int>(node.z * 10);
        return std::hash<int>{}(x_grid) ^ (std::hash<int>{}(y_grid) << 1) ^ (std::hash<int>{}(z_grid) << 2);
    }
};

struct NodeCompare {
    bool operator()(const Node* a, const Node* b) const {
        return a->f_cost() > b->f_cost();  // Min-heap
    }
};

class PathPlanner {
public:
    PathPlanner(double grid_resolution = 0.2, double safety_margin = 0.5);
    
    /**
     * Plan a path from start to goal using A* algorithm
     * @param start Starting position
     * @param goal Goal position  
     * @param octomap Current octomap for collision checking
     * @return Vector of waypoints from start to goal, empty if no path found
     */
    std::vector<geometry_msgs::msg::Point> plan_path(
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal,
        const std::shared_ptr<octomap::OcTree>& octomap
    );
    
    /**
     * Check if a point is collision-free with safety margin
     */
    bool is_point_free(const geometry_msgs::msg::Point& point, 
                      const std::shared_ptr<octomap::OcTree>& octomap);
    
    /**
     * Validate if entire path is still collision-free
     */
    bool validate_path(const std::vector<geometry_msgs::msg::Point>& path,
                      const std::shared_ptr<octomap::OcTree>& octomap);
    
    /**
     * Simplify path by removing unnecessary waypoints
     */
    std::vector<geometry_msgs::msg::Point> simplify_path(
        const std::vector<geometry_msgs::msg::Point>& path,
        const std::shared_ptr<octomap::OcTree>& octomap
    );

private:
    double grid_resolution_;
    double safety_margin_;
    
    double heuristic(const Node& a, const Node& b);
    std::vector<Node> get_neighbors(const Node& node);
    bool line_of_sight(const geometry_msgs::msg::Point& p1, 
                      const geometry_msgs::msg::Point& p2,
                      const std::shared_ptr<octomap::OcTree>& octomap);
    
    std::vector<geometry_msgs::msg::Point> reconstruct_path(Node* goal_node);
};

}  // namespace blimp_navigation_cpp

#endif  // BLIMP_NAVIGATION__PATH_PLANNER_HPP_