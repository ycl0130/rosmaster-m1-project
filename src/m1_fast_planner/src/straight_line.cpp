#include "m1_fast_planner/straight_line.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"

namespace m1_fast_planner
{

std::vector<geometry_msgs::msg::Point> sampleStraightLine(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  double path_resolution)
{
  if (!std::isfinite(path_resolution) || path_resolution <= 0.0) {
    throw std::invalid_argument("path_resolution must be finite and positive");
  }
  if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
    !std::isfinite(goal.x) || !std::isfinite(goal.y))
  {
    throw std::invalid_argument("start and goal positions must be finite");
  }

  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double distance = std::hypot(dx, dy);
  const size_t segments = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / path_resolution)));
  std::vector<geometry_msgs::msg::Point> samples;
  samples.reserve(segments + 1);
  for (size_t index = 0; index <= segments; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(segments);
    geometry_msgs::msg::Point point;
    point.x = start.x + ratio * dx;
    point.y = start.y + ratio * dy;
    point.z = start.z + ratio * (goal.z - start.z);
    samples.push_back(point);
  }
  return samples;
}

double straightLineYaw(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal)
{
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  return std::hypot(dx, dy) > 1e-9 ? std::atan2(dy, dx) : 0.0;
}

bool isTraversableCost(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return false;
  }
  return allow_unknown || cost != nav2_costmap_2d::NO_INFORMATION;
}

bool lineIsCollisionFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & samples,
  bool allow_unknown)
{
  for (const auto & sample : samples) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap.worldToMap(sample.x, sample.y, map_x, map_y) ||
      !isTraversableCost(costmap.getCost(map_x, map_y), allow_unknown))
    {
      return false;
    }
  }
  return true;
}

bool framesMatch(
  const std::string & start_frame, const std::string & goal_frame,
  const std::string & global_frame)
{
  return !global_frame.empty() && start_frame == global_frame && goal_frame == global_frame;
}

}  // namespace m1_fast_planner
