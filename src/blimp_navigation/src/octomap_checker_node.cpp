#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include "blimp_navigation/srv/check_collision.hpp"
#include "blimp_navigation/srv/plan_path.hpp"
#include "blimp_navigation/path_planner.hpp"

#include <memory>
#include <chrono>
#include <cmath>

// ros2 service call /check_collision blimp_navigation/srv/CheckCollision \
// "{points_to_check: [ { x: -0.537, y: -0.305, z: -0.024 } ]}"


// ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{ header: { frame_id: 'map'}, pose: { position: { x: 0.818, y: 0.318,  z: 0.145 }, orientation: { x: 0.016, y: 0.069, z: 0.214,   w: 0.974 } }  }" --once

class OctomapCheckerNode : public rclcpp::Node {
public:
    OctomapCheckerNode() : Node("octomap_checker_node") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        // Revert this change from last time for a cleaner data path
        subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/rtabmap/octomap_obstacles", qos,
            std::bind(&OctomapCheckerNode::octomap_callback, this, std::placeholders::_1));

        service_ = this->create_service<blimp_navigation::srv::CheckCollision>(
            "/check_collision",
            std::bind(&OctomapCheckerNode::handle_check_collision, this, std::placeholders::_1, std::placeholders::_2));

        path_planning_service_ = this->create_service<blimp_navigation::srv::PlanPath>(
            "/plan_path",
            std::bind(&OctomapCheckerNode::handle_plan_path, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize path planner
        path_planner_ = std::make_unique<blimp_navigation_cpp::PathPlanner>(0.2, 0.5);

        // Log resolved names to verify remappings
        RCLCPP_INFO(this->get_logger(), "OctoMap Checker Service is ready.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to OctoMap: %s",
                    subscription_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Service available at: %s",
                    service_->get_service_name());

        using namespace std::chrono_literals;
        heartbeat_timer_ = this->create_wall_timer(5s, [this]() {
            if (!tree_ && !color_tree_) {
                RCLCPP_WARN(this->get_logger(), "Heartbeat: no OctoMap received yet; waiting on %s",
                            subscription_ ? subscription_->get_topic_name() : "<unsubscribed>");
                return;
            }

            // Compute age of last received octomap message
            rclcpp::Time now = this->now();
            double age_sec = 0.0;
            if (last_octomap_time_.nanoseconds() > 0) {
                age_sec = (now - last_octomap_time_).seconds();
            }

            if (tree_) {
                RCLCPP_INFO(this->get_logger(),
                            "Heartbeat: OcTree ready (nodes=%zu, depth=%u, res=%.3f), last_msg_age=%.2fs",
                            tree_->size(), tree_->getTreeDepth(), tree_->getResolution(), age_sec);
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Heartbeat: ColorOcTree ready (nodes=%zu, depth=%u, res=%.3f), last_msg_age=%.2fs",
                            color_tree_->size(), color_tree_->getTreeDepth(), color_tree_->getResolution(), age_sec);
            }
        });

        // Parameters for neighborhood collision checking
        this->declare_parameter<double>("check_radius", 0.5);
        this->declare_parameter<double>("check_step", 0.0); // 0.0 => use tree resolution
        this->declare_parameter<bool>("check_use_sphere", true);
        // Log initial values
        double r = this->get_parameter("check_radius").as_double();
        double s = this->get_parameter("check_step").as_double();
        bool sphere = this->get_parameter("check_use_sphere").as_bool();
        RCLCPP_INFO(this->get_logger(),
                    "Params: check_radius=%.3f m, check_step=%s, use_sphere=%s",
                    r, s > 0.0 ? std::to_string(s).c_str() : "<tree_resolution>",
                    sphere ? "true" : "false");
    }

private:
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Convert the ROS message into a C++ OcTree object
        RCLCPP_INFO(this->get_logger(),
                    "OctoMap msg received: frame_id='%s' stamp=%u.%u id='%s' binary=%s res=%.3f data_size=%zu",
                    msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
                    msg->id.c_str(), msg->binary ? "true" : "false", msg->resolution, msg->data.size());
        // Save timestamp for heartbeat age computation
        last_octomap_time_ = rclcpp::Time(msg->header.stamp);
        auto *abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        if (abstract_tree) {
            // Try to cast to standard OcTree first
            if (auto *t = dynamic_cast<octomap::OcTree*>(abstract_tree)) {
                tree_.reset(t);
                color_tree_.reset();
                RCLCPP_INFO(this->get_logger(),
                            "Parsed OcTree: resolution=%.3f size=%zu depth=%u type=%s",
                            tree_->getResolution(), tree_->size(), tree_->getTreeDepth(),
                            abstract_tree->getTreeType().c_str());
                RCLCPP_INFO_ONCE(this->get_logger(), "Successfully received and parsed first OctoMap.");
            }
            // Otherwise try ColorOcTree
            else if (auto *ct = dynamic_cast<octomap::ColorOcTree*>(abstract_tree)) {
                color_tree_.reset(ct);
                tree_.reset();
                RCLCPP_INFO(this->get_logger(),
                            "Parsed ColorOcTree: resolution=%.3f size=%zu depth=%u type=%s",
                            color_tree_->getResolution(), color_tree_->size(), color_tree_->getTreeDepth(),
                            abstract_tree->getTreeType().c_str());
                RCLCPP_INFO_ONCE(this->get_logger(), "Successfully received and parsed first OctoMap (color). ");
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Unsupported OctoMap tree type: %s",
                             abstract_tree->getTreeType().c_str());
                delete abstract_tree; // avoid leak if unsupported type
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap message.");
        }
    }

    void handle_check_collision(
        const std::shared_ptr<blimp_navigation::srv::CheckCollision::Request> request,
        std::shared_ptr<blimp_navigation::srv::CheckCollision::Response> response) {

        response->is_occupied = false; // assume clear

        if (!tree_ && !color_tree_) {
            RCLCPP_WARN(this->get_logger(), "Collision check requested, but no OctoMap received yet.");
            response->is_occupied = true; // fail-safe
            return;
        }

        // Refresh parameters each request so they can be tuned at runtime
        double check_radius = 0.1;
        double check_step = 0.0;
        bool check_use_sphere = true;
        (void)this->get_parameter("check_radius", check_radius);
        (void)this->get_parameter("check_step", check_step);
        (void)this->get_parameter("check_use_sphere", check_use_sphere);
        // Default step to current tree resolution if not set or invalid
        const double tree_res = tree_ ? tree_->getResolution() : color_tree_->getResolution();
        if (check_step <= 0.0) check_step = tree_res;
        if (check_step <= 0.0) check_step = 0.1; // ultimate fallback

        if (tree_) {
            RCLCPP_INFO(this->get_logger(),
                        "Checking %zu point(s) against OcTree (res=%.3f, depth=%u, radius=%.3f, step=%.3f, sphere=%s)...",
                        request->points_to_check.size(), tree_->getResolution(), tree_->getTreeDepth(),
                        check_radius, check_step, check_use_sphere ? "true" : "false");
        } else if (color_tree_) {
            RCLCPP_INFO(this->get_logger(),
                        "Checking %zu point(s) against ColorOcTree (res=%.3f, depth=%u, radius=%.3f, step=%.3f, sphere=%s)...",
                        request->points_to_check.size(), color_tree_->getResolution(), color_tree_->getTreeDepth(),
                        check_radius, check_step, check_use_sphere ? "true" : "false");
        }

        size_t idx = 0;
        for (const auto &point : request->points_to_check) {
            RCLCPP_DEBUG(this->get_logger(), "[%zu] query=(%.3f, %.3f, %.3f)", idx, point.x, point.y, point.z);
            auto is_occupied_at = [&](double x, double y, double z) -> bool {
                if (tree_) {
                    if (auto *n = tree_->search(x, y, z)) {
                        const bool occ = tree_->isNodeOccupied(n);
                        RCLCPP_DEBUG(this->get_logger(), "query(%.3f,%.3f,%.3f): occ=%s p=%.3f",
                                     x, y, z, occ ? "true" : "false", n->getOccupancy());
                        return occ;
                    }
                } else if (color_tree_) {
                    if (auto *n = color_tree_->search(x, y, z)) {
                        const bool occ = color_tree_->isNodeOccupied(n);
                        RCLCPP_DEBUG(this->get_logger(), "query(%.3f,%.3f,%.3f): occ=%s p=%.3f",
                                     x, y, z, occ ? "true" : "false", n->getOccupancy());
                        return occ;
                    }
                }
                return false;
            };

            bool hit = false;
            if (check_radius <= 0.0) {
                // Simple check at the exact point
                hit = is_occupied_at(point.x, point.y, point.z);
            } else {
                const int steps = std::max(1, static_cast<int>(std::ceil(check_radius / check_step)));
                const double r2 = check_radius * check_radius;
                for (int ix = -steps; ix <= steps && !hit; ++ix) {
                    const double ox = ix * check_step;
                    for (int iy = -steps; iy <= steps && !hit; ++iy) {
                        const double oy = iy * check_step;
                        for (int iz = -steps; iz <= steps && !hit; ++iz) {
                            const double oz = iz * check_step;
                            if (check_use_sphere && (ox*ox + oy*oy + oz*oz) > r2) continue; // outside sphere
                            if (is_occupied_at(point.x + ox, point.y + oy, point.z + oz)) {
                                RCLCPP_INFO(this->get_logger(),
                                            "Collision near (%.3f, %.3f, %.3f) at offset (%.3f, %.3f, %.3f)",
                                            point.x, point.y, point.z, ox, oy, oz);
                                hit = true;
                            }
                        }
                    }
                }
            }

            if (hit) {
                response->is_occupied = true;
                return;
            }
            ++idx;
        }
        RCLCPP_INFO(this->get_logger(), "No collisions found.");
    }

    void handle_plan_path(const std::shared_ptr<blimp_navigation::srv::PlanPath::Request> request,
                         std::shared_ptr<blimp_navigation::srv::PlanPath::Response> response)
    {
        if (!tree_ && !color_tree_) {
            response->success = false;
            response->message = "No octomap available for path planning";
            RCLCPP_WARN(this->get_logger(), "Path planning request received but no octomap available");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", 
                    request->start.x, request->start.y, request->start.z,
                    request->goal.x, request->goal.y, request->goal.z);

        // Use the appropriate tree - handle both types properly
        std::shared_ptr<octomap::OcTree> octomap;
        if (tree_) {
            octomap = tree_;
        } else if (color_tree_) {
            // ColorOcTree inherits from OcTree, but we need to be careful with shared_ptr casting
            octomap = std::dynamic_pointer_cast<octomap::OcTree>(color_tree_);
            if (!octomap) {
                response->success = false;
                response->message = "Failed to cast ColorOcTree to OcTree";
                return;
            }
        } else {
            response->success = false;
            response->message = "No octomap available";
            return;
        }
        
        auto path = path_planner_->plan_path(request->start, request->goal, octomap);
        
        if (path.empty()) {
            response->success = false;
            response->message = "No valid path found";
            RCLCPP_WARN(this->get_logger(), "Failed to find path");
        } else {
            response->success = true;
            response->waypoints = path;
            response->message = "Path found with " + std::to_string(path.size()) + " waypoints";
            RCLCPP_INFO(this->get_logger(), "Path planned successfully with %zu waypoints", path.size());
        }
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
    rclcpp::Service<blimp_navigation::srv::CheckCollision>::SharedPtr service_;
    rclcpp::Service<blimp_navigation::srv::PlanPath>::SharedPtr path_planning_service_;
    std::shared_ptr<octomap::OcTree> tree_;
    std::shared_ptr<octomap::ColorOcTree> color_tree_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Time last_octomap_time_{};
    std::unique_ptr<blimp_navigation_cpp::PathPlanner> path_planner_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapCheckerNode>());
    rclcpp::shutdown();
    return 0;
}
