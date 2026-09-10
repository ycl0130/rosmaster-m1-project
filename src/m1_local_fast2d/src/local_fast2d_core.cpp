#include "m1_local_fast2d/local_fast2d_core.hpp"

#include <chrono>
#include <algorithm>
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

double meanNearestSeparation(const std::vector<TimedTrajectoryPoint> & a,
  const std::vector<TimedTrajectoryPoint> & b)
{
  if (a.empty() || b.empty()) {return 0.0;}
  double total = 0.0;
  for (const auto & point : a) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & other : b) {
      nearest = std::min(nearest, std::hypot(point.x - other.x, point.y - other.y));
    }
    total += nearest;
  }
  return total / static_cast<double>(a.size());
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
  // Candidate zero intentionally retains the Phase-1 call verbatim.  Further
  // searches use the same snapshot and only add a soft cost around accepted
  // trajectories; collision and A* propagation are unchanged.
  const int requested = std::clamp(request.config.num_candidates, 1, 5);
  output.requested_candidates = requested;
  const auto primary_started = std::chrono::steady_clock::now();
  const auto result = m1_fast_planner::KinodynamicAstar(config).search(request.start, goal, free, risk);
  output.primary_planning_latency_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - primary_started).count();
  output.latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  output.timeout = result.telemetry.timeout; output.expanded_nodes = result.telemetry.expanded_nodes; output.generated_nodes = result.telemetry.generated_nodes;
  output.cycle_class = result.telemetry.generated_nodes ? "full_search" : "connector_shortcut";
  if (!result.telemetry.success || result.trajectory.empty()) {output.failure_reason = result.telemetry.timeout ? "timeout" : "no_path"; return output;}
  const auto make_candidate = [&request](const m1_fast_planner::SearchResult & search,
      std::size_t index, double elapsed_ms) {
      LocalFast2DResult::Candidate candidate; candidate.index = index;
      candidate.local_path.header = request.reference_path.header;
      candidate.planner_cost = search.telemetry.trajectory_cost;
      candidate.duration = search.telemetry.trajectory_duration;
      candidate.generation_latency_ms = elapsed_ms;
      for (size_t i = 0; i < search.trajectory.size(); ++i) {
        geometry_msgs::msg::PoseStamped point; point.header = candidate.local_path.header;
        point.pose.position.x = search.trajectory[i].state.px; point.pose.position.y = search.trajectory[i].state.py;
    double yaw = request.start_yaw;
        if (i + 1 < search.trajectory.size()) {yaw = std::atan2(search.trajectory[i + 1].state.py - search.trajectory[i].state.py, search.trajectory[i + 1].state.px - search.trajectory[i].state.px);}
        point.pose.orientation = yawQuaternion(yaw); candidate.local_path.poses.push_back(point);
        const auto & raw = search.trajectory[i];
        candidate.timed_trajectory.push_back(TimedTrajectoryPoint{
      raw.state.px, raw.state.py, yaw, raw.time_from_start,
          raw.state.vx, raw.state.vy, raw.acceleration.ax, raw.acceleration.ay});
      }
      return candidate;
    };
  output.success = true; output.failure_reason = "ok";
  output.candidates.push_back(make_candidate(result, 0, output.primary_planning_latency_ms));
  const double dx = goal.px - request.start.px, dy = goal.py - request.start.py;
  const double length = std::hypot(dx, dy);
  // Only branch when the direct reference chord is genuinely blocked. Gates
  // are expressed in the start-to-goal longitudinal/lateral frame.
  bool direct_blocked = false; double first_blocked = 1.0, last_blocked = 0.0;
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.05) {if (!free(request.start.px + ratio * dx, request.start.py + ratio * dy)) {direct_blocked = true; first_blocked = std::min(first_blocked, ratio); last_blocked = std::max(last_blocked, ratio);}}
  for (int index = 1; index < requested && direct_blocked && length > 1e-6; ++index) {
    const auto alternative_started = std::chrono::steady_clock::now();
    const double side = index % 2 ? 1.0 : -1.0;
    const double anchor = 0.5 * (first_blocked + last_blocked);
    m1_fast_planner::PlanarState gate; bool valid_gate = false;
    for (double offset = request.config.candidate_gate_initial_offset;
      offset <= request.config.candidate_gate_max_offset + 1e-9;
      offset += request.config.candidate_gate_offset_step) {
      gate = {request.start.px + anchor * dx - side * offset * dy / length,
        request.start.py + anchor * dy + side * offset * dx / length, 0.0, 0.0};
      valid_gate = free(gate.px, gate.py);
      for (int sample = 0; valid_gate && sample < 8; ++sample) {
        const double angle = sample * 2.0 * M_PI / 8.0;
        valid_gate = free(gate.px + request.config.candidate_gate_clearance * std::cos(angle),
          gate.py + request.config.candidate_gate_clearance * std::sin(angle));
      }
      if (valid_gate) {break;}
    }
    if (!valid_gate) {continue;}
    const auto first = m1_fast_planner::KinodynamicAstar(config).search(request.start, gate, free, risk);
    if (!first.telemetry.success || first.trajectory.empty()) {continue;}
    const auto second = m1_fast_planner::KinodynamicAstar(config).search(
      first.trajectory.back().state, goal, free, risk);
    if (!second.telemetry.success || second.trajectory.empty()) {continue;}
    auto alternate = first;
    for (std::size_t point = 1; point < second.trajectory.size(); ++point) {
      auto joined = second.trajectory[point]; joined.time_from_start += first.trajectory.back().time_from_start;
      alternate.trajectory.push_back(joined);
    }
    alternate.telemetry.success = true;
    alternate.telemetry.trajectory_duration = alternate.trajectory.back().time_from_start;
    alternate.telemetry.trajectory_cost = first.telemetry.trajectory_cost + second.telemetry.trajectory_cost;
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - alternative_started).count();
    if (!alternate.telemetry.success || alternate.trajectory.empty()) {continue;}
    auto candidate = make_candidate(alternate, output.candidates.size(), elapsed);
    candidate.minimum_diversity = std::numeric_limits<double>::infinity();
    for (const auto & accepted : output.candidates) {
      candidate.minimum_diversity = std::min(candidate.minimum_diversity,
        std::min(meanNearestSeparation(candidate.timed_trajectory, accepted.timed_trajectory),
          meanNearestSeparation(accepted.timed_trajectory, candidate.timed_trajectory)));
    }
    if (candidate.minimum_diversity >= request.config.candidate_diversity_threshold) {
      output.candidates.push_back(std::move(candidate));
    }
  }
  output.local_path = output.candidates.front().local_path;
  output.timed_trajectory = output.candidates.front().timed_trajectory;
  output.latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  return output;
}
}  // namespace m1_local_fast2d
