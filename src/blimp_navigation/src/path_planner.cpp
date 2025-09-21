#include "blimp_navigation/path_planner.hpp"
#include <algorithm>

namespace blimp_navigation_cpp
{

PathPlanner::PathPlanner(double grid_resolution, double safety_margin) 
    : grid_resolution_(grid_resolution), safety_margin_(safety_margin) 
{
}

std::vector<geometry_msgs::msg::Point> PathPlanner::plan_path(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    const std::shared_ptr<octomap::OcTree>& octomap)
{
    if (!octomap) {
        return {};
    }
    
    // Check if start and goal are valid
    if (!is_point_free(start, octomap) || !is_point_free(goal, octomap)) {
        return {};
    }
    
    // Initialize A* data structures
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
    std::unordered_set<Node, NodeHash> closed_set;
    std::unordered_map<Node, std::unique_ptr<Node>, NodeHash> all_nodes;
    
    // Create start node
    auto start_node = std::make_unique<Node>(start.x, start.y, start.z);
    start_node->g_cost = 0;
    start_node->h_cost = heuristic(*start_node, Node(goal.x, goal.y, goal.z));
    
    Node* start_ptr = start_node.get();
    all_nodes[*start_node] = std::move(start_node);
    open_set.push(start_ptr);
    
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (std::abs(current->x - goal.x) < grid_resolution_ && 
            std::abs(current->y - goal.y) < grid_resolution_ && 
            std::abs(current->z - goal.z) < grid_resolution_) {
            
            auto path = reconstruct_path(current);
            return simplify_path(path, octomap);
        }
        
        closed_set.insert(*current);
        
        // Explore neighbors
        auto neighbors = get_neighbors(*current);
        for (const auto& neighbor : neighbors) {
            if (closed_set.count(neighbor) > 0) {
                continue;
            }
            
            geometry_msgs::msg::Point neighbor_point;
            neighbor_point.x = neighbor.x;
            neighbor_point.y = neighbor.y; 
            neighbor_point.z = neighbor.z;
            
            if (!is_point_free(neighbor_point, octomap)) {
                continue;
            }
            
            double tentative_g = current->g_cost + 
                std::sqrt(std::pow(neighbor.x - current->x, 2) + 
                         std::pow(neighbor.y - current->y, 2) + 
                         std::pow(neighbor.z - current->z, 2));
            
            Node* neighbor_ptr = nullptr;
            auto it = all_nodes.find(neighbor);
            if (it == all_nodes.end()) {
                auto neighbor_node = std::make_unique<Node>(neighbor.x, neighbor.y, neighbor.z);
                neighbor_node->g_cost = tentative_g;
                neighbor_node->h_cost = heuristic(*neighbor_node, Node(goal.x, goal.y, goal.z));
                neighbor_node->parent = current;
                neighbor_ptr = neighbor_node.get();
                all_nodes[neighbor] = std::move(neighbor_node);
                open_set.push(neighbor_ptr);
            } else {
                neighbor_ptr = it->second.get();
                if (tentative_g < neighbor_ptr->g_cost) {
                    neighbor_ptr->g_cost = tentative_g;
                    neighbor_ptr->parent = current;
                }
            }
        }
    }
    
    return {}; // No path found
}

bool PathPlanner::is_point_free(const geometry_msgs::msg::Point& point, 
                               const std::shared_ptr<octomap::OcTree>& octomap)
{
    if (!octomap) return false;
    
    // Check multiple points around the center point for safety margin
    std::vector<octomap::point3d> check_points;
    check_points.emplace_back(point.x, point.y, point.z);
    
    // Add safety margin checks in all directions
    for (double dx = -safety_margin_; dx <= safety_margin_; dx += safety_margin_) {
        for (double dy = -safety_margin_; dy <= safety_margin_; dy += safety_margin_) {
            for (double dz = -safety_margin_/2; dz <= safety_margin_/2; dz += safety_margin_/2) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                check_points.emplace_back(point.x + dx, point.y + dy, point.z + dz);
            }
        }
    }
    
    for (const auto& check_point : check_points) {
        octomap::OcTreeNode* node = octomap->search(check_point);
        if (node && octomap->isNodeOccupied(node)) {
            return false;
        }
    }
    
    return true;
}

bool PathPlanner::validate_path(const std::vector<geometry_msgs::msg::Point>& path,
                               const std::shared_ptr<octomap::OcTree>& octomap)
{
    for (const auto& point : path) {
        if (!is_point_free(point, octomap)) {
            return false;
        }
    }
    
    // Also check line segments between waypoints
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!line_of_sight(path[i], path[i+1], octomap)) {
            return false;
        }
    }
    
    return true;
}

std::vector<geometry_msgs::msg::Point> PathPlanner::simplify_path(
    const std::vector<geometry_msgs::msg::Point>& path,
    const std::shared_ptr<octomap::OcTree>& octomap)
{
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<geometry_msgs::msg::Point> simplified;
    simplified.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = i + 1;
        
        // Find the farthest point we can reach with line of sight
        while (j < path.size() && line_of_sight(path[i], path[j], octomap)) {
            j++;
        }
        
        simplified.push_back(path[j-1]);
        i = j - 1;
    }
    
    return simplified;
}

double PathPlanner::heuristic(const Node& a, const Node& b)
{
    // 3D Euclidean distance
    return std::sqrt(std::pow(a.x - b.x, 2) + 
                    std::pow(a.y - b.y, 2) + 
                    std::pow(a.z - b.z, 2));
}

std::vector<Node> PathPlanner::get_neighbors(const Node& node)
{
    std::vector<Node> neighbors;
    
    // 26-connected neighbors (3D)
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                
                neighbors.emplace_back(
                    node.x + dx * grid_resolution_,
                    node.y + dy * grid_resolution_, 
                    node.z + dz * grid_resolution_
                );
            }
        }
    }
    
    return neighbors;
}

bool PathPlanner::line_of_sight(const geometry_msgs::msg::Point& p1, 
                               const geometry_msgs::msg::Point& p2,
                               const std::shared_ptr<octomap::OcTree>& octomap)
{
    if (!octomap) return false;
    
    double distance = std::sqrt(std::pow(p2.x - p1.x, 2) + 
                               std::pow(p2.y - p1.y, 2) + 
                               std::pow(p2.z - p1.z, 2));
    
    int num_checks = static_cast<int>(distance / (grid_resolution_ * 0.5)) + 1;
    
    for (int i = 0; i <= num_checks; i++) {
        double t = static_cast<double>(i) / num_checks;
        geometry_msgs::msg::Point check_point;
        check_point.x = p1.x + t * (p2.x - p1.x);
        check_point.y = p1.y + t * (p2.y - p1.y);
        check_point.z = p1.z + t * (p2.z - p1.z);
        
        if (!is_point_free(check_point, octomap)) {
            return false;
        }
    }
    
    return true;
}

std::vector<geometry_msgs::msg::Point> PathPlanner::reconstruct_path(Node* goal_node)
{
    std::vector<geometry_msgs::msg::Point> path;
    Node* current = goal_node;
    
    while (current) {
        geometry_msgs::msg::Point point;
        point.x = current->x;
        point.y = current->y;
        point.z = current->z;
        path.push_back(point);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace blimp_navigation_cpp