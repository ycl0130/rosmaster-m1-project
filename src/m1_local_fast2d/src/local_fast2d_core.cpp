#include "m1_local_fast2d/local_fast2d_core.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace m1_local_fast2d
{
namespace
{
bool traversable(const uint8_t cost, const bool allow_unknown)
{
  return (allow_unknown || cost != nav2_costmap_2d::NO_INFORMATION) &&
    cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5); q.w = std::cos(yaw * 0.5); return q;
}
}  // namespace

LocalFast2DResult planLocalFast2D(const LocalFast2DRequest & request)
{
  const auto started = std::chrono::steady_clock::now();
  LocalFast2DResult output;
  const auto & map = request.costmap;
  const auto width = map.metadata.size_x, height = map.metadata.size_y;
  const double resolution = map.metadata.resolution;
  if (request.reference_path.poses.empty() || width == 0 || height == 0 || resolution <= 0.0 ||
      map.data.size() != static_cast<size_t>(width) * height) {
    output.failure_reason = "invalid_input"; return output;
  }
  const auto cell = [&map, width, height, resolution](const double x, const double y, uint8_t * value) {
      const double dx = x - map.metadata.origin.position.x;
      const double dy = y - map.metadata.origin.position.y;
      const auto mx = static_cast<long>(std::floor(dx / resolution));
      const auto my = static_cast<long>(std::floor(dy / resolution));
      if (mx < 0 || my < 0 || mx >= static_cast<long>(width) || my >= static_cast<long>(height)) {return false;}
      *value = map.data[static_cast<size_t>(my) * width + static_cast<size_t>(mx)]; return true;
    };
  const auto free = [&cell, &request](double x, double y) {uint8_t cost{}; return cell(x, y, &cost) && traversable(cost, request.config.allow_unknown);};
  size_t nearest = 0; double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < request.reference_path.poses.size(); ++i) {
    const auto & p = request.reference_path.poses[i].pose.position;
    const double d = std::hypot(p.x - request.start.px, p.y - request.start.py);
    if (d < best) {best = d; nearest = i;}
  }
  size_t target = nearest; double distance = 0.0;
  while (target + 1 < request.reference_path.poses.size() && distance < request.config.lookahead_distance) {
    const auto & a = request.reference_path.poses[target].pose.position;
    const auto & b = request.reference_path.poses[++target].pose.position;
    distance += std::hypot(b.x - a.x, b.y - a.y);
  }
  while (target > nearest && !free(request.reference_path.poses[target].pose.position.x, request.reference_path.poses[target].pose.position.y)) {--target;}
  output.selected_local_goal = request.reference_path.poses[target];
  if (!free(output.selected_local_goal.pose.position.x, output.selected_local_goal.pose.position.y)) {
    output.failure_reason = "no_traversable_local_goal"; return output;
  }
  m1_fast_planner::KinodynamicAstarConfig config;
  config.search_timeout_ms = request.config.max_planning_time_ms;
  config.max_expansions = request.config.max_expansions;
  config.local_costmap_cost_weight = request.config.cost_weight;
  const m1_fast_planner::PlanarState goal{output.selected_local_goal.pose.position.x, output.selected_local_goal.pose.position.y, 0.0, 0.0};
  const auto risk = [&cell](double x, double y) {uint8_t cost{}; return cell(x, y, &cost) ? static_cast<double>(cost) / 252.0 : std::numeric_limits<double>::infinity();};
  const auto result = m1_fast_planner::KinodynamicAstar(config).search(request.start, goal, free, risk);
  output.latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  output.timeout = result.telemetry.timeout; output.expanded_nodes = result.telemetry.expanded_nodes; output.generated_nodes = result.telemetry.generated_nodes;
  output.cycle_class = result.telemetry.generated_nodes ? "full_search" : "connector_shortcut";
  if (!result.telemetry.success || result.trajectory.empty()) {output.failure_reason = result.telemetry.timeout ? "timeout" : "no_path"; return output;}
  output.success = true; output.failure_reason = "ok";
  output.local_path.header = request.reference_path.header;
  for (size_t i = 0; i < result.trajectory.size(); ++i) {
    geometry_msgs::msg::PoseStamped point; point.header = output.local_path.header;
    point.pose.position.x = result.trajectory[i].state.px; point.pose.position.y = result.trajectory[i].state.py;
    double yaw = request.start_yaw;
    if (i + 1 < result.trajectory.size()) {yaw = std::atan2(result.trajectory[i + 1].state.py - result.trajectory[i].state.py, result.trajectory[i + 1].state.px - result.trajectory[i].state.px);}
    point.pose.orientation = yawQuaternion(yaw); output.local_path.poses.push_back(point);
  }
  return output;
}
}  // namespace m1_local_fast2d
