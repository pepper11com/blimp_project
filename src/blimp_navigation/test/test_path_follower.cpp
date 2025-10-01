#include <gtest/gtest.h>

#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "blimp_navigation/path_follower.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace

TEST(PathFollowerTest, GeneratesForwardMotion)
{
  blimp_navigation::PathFollowerConfig config;
  config.lookahead_distance = 1.0;
  config.max_forward_norm = 0.2;
  config.max_reverse_norm = 0.2;

  blimp_navigation::PathFollower follower(config);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0, 0.0, 0.0));
  path.poses.push_back(makePose(5.0, 0.0, 0.0, 0.0));
  follower.setPath(path);

  auto current = makePose(0.0, 0.0, 0.0, 0.0);
  nav_msgs::msg::Path remaining;
  auto command = follower.update(current, 0.05, remaining);

  EXPECT_TRUE(command.has_active_path);
  EXPECT_GT(command.forward_command, 0.0);
  EXPECT_NEAR(command.left_motor_norm, command.right_motor_norm, 1e-6);
  EXPECT_EQ(remaining.poses.size(), path.poses.size() - 1);
}

TEST(PathFollowerTest, GoalReachedStopsMotion)
{
  blimp_navigation::PathFollowerConfig config;
  config.lookahead_distance = 1.0;
  config.goal_tolerance = 0.5;
  config.max_forward_norm = 0.2;
  config.max_reverse_norm = 0.2;

  blimp_navigation::PathFollower follower(config);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0, 0.0, 0.0));
  path.poses.push_back(makePose(5.0, 0.0, 0.0, 0.0));
  follower.setPath(path);

  nav_msgs::msg::Path remaining;
  auto near_start = makePose(0.1, 0.0, 0.0, 0.0);
  follower.update(near_start, 0.05, remaining);

  auto near_goal = makePose(5.1, 0.0, 0.0, 0.0);
  auto command = follower.update(near_goal, 0.05, remaining);

  EXPECT_FALSE(command.has_active_path);
  EXPECT_TRUE(command.goal_reached);
  EXPECT_DOUBLE_EQ(command.forward_command, 0.0);
  EXPECT_TRUE(remaining.poses.empty());
}
