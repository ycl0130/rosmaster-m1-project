#ifndef M1_FAST_PLANNER__STRAIGHT_LINE_HPP_
#define M1_FAST_PLANNER__STRAIGHT_LINE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace m1_fast_planner
{

std::vector<geometry_msgs::msg::Point> sampleStraightLine(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  double path_resolution);

double straightLineYaw(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal);

bool isTraversableCost(unsigned char cost, bool allow_unknown);

bool lineIsCollisionFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & samples,
  bool allow_unknown);

bool framesMatch(
  const std::string & start_frame, const std::string & goal_frame,
  const std::string & global_frame);

}  // namespace m1_fast_planner

#endif  // M1_FAST_PLANNER__STRAIGHT_LINE_HPP_
