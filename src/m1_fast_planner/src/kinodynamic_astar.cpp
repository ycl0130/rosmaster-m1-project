#include "m1_fast_planner/kinodynamic_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace m1_fast_planner
{
namespace
{

struct StateKey
{
  long long px;
  long long py;
  long long vx;
  long long vy;
  bool operator==(const StateKey & other) const
  {
    return px == other.px && py == other.py && vx == other.vx && vy == other.vy;
  }
};

struct StateKeyHash
{
  std::size_t operator()(const StateKey & key) const
  {
    std::size_t value = std::hash<long long>{}(key.px);
    for (const auto item : {key.py, key.vx, key.vy}) {
      value ^= std::hash<long long>{}(item) + 0x9e3779b9 + (value << 6U) + (value >> 2U);
    }
    return value;
  }
};

struct Node
{
  PlanarState state;
  PlanarAcceleration acceleration;
  double g{0.0};
  double time_from_start{0.0};
  std::size_t parent{std::numeric_limits<std::size_t>::max()};
};

struct OpenEntry
{
  double f;
  double h;
  double g;
  std::size_t insertion_id;
  std::size_t index;
  bool operator<(const OpenEntry & other) const
  {
    // priority_queue is a max heap.  Make equal-cost searches reproducible:
    // unordered_map iteration never selects successors, but equal entries in
    // the heap otherwise have unspecified ordering.
    if (f != other.f) {
      return f > other.f;
    }
    if (h != other.h) {
      return h > other.h;
    }
    if (g != other.g) {
      return g > other.g;
    }
    return insertion_id > other.insertion_id;
  }
};

double speed(const PlanarState & state)
{
  return std::hypot(state.vx, state.vy);
}

double positionError(const PlanarState & state, const PlanarState & goal)
{
  return std::hypot(goal.px - state.px, goal.py - state.py);
}

bool finiteState(const PlanarState & state)
{
  return std::isfinite(state.px) && std::isfinite(state.py) &&
         std::isfinite(state.vx) && std::isfinite(state.vy);
}

}  // namespace

KinodynamicAstar::KinodynamicAstar(KinodynamicAstarConfig config)
: config_(std::move(config))
{
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid kinodynamic A* configuration");
  }
}

PlanarState KinodynamicAstar::propagate(
  const PlanarState & state, const PlanarAcceleration & acceleration, double time)
{
  return PlanarState{
    state.px + state.vx * time + 0.5 * acceleration.ax * time * time,
    state.py + state.vy * time + 0.5 * acceleration.ay * time * time,
    state.vx + acceleration.ax * time,
    state.vy + acceleration.ay * time};
}

bool KinodynamicAstar::validConfig(const KinodynamicAstarConfig & c)
{
  return c.max_velocity_x > 0.0 && c.max_velocity_y > 0.0 &&
         c.max_accel_x > 0.0 && c.max_accel_y > 0.0 &&
         c.primitive_duration > 0.0 && c.collision_check_dt > 0.0 &&
         c.collision_check_distance > 0.0 && c.position_resolution > 0.0 &&
         c.velocity_resolution > 0.0 && c.goal_position_tolerance >= 0.0 &&
         c.goal_velocity_tolerance >= 0.0 && c.search_timeout_ms > 0 &&
         c.max_expansions > 0 && c.time_cost_weight >= 0.0 &&
         c.control_cost_weight >= 0.0 && c.local_costmap_cost_weight >= 0.0 &&
         c.path_resolution > 0.0;
}

long long KinodynamicAstar::quantize(double value, double resolution)
{
  return static_cast<long long>(std::llround(value / resolution));
}

SearchResult KinodynamicAstar::search(
  const PlanarState & start, const PlanarState & goal, const CollisionChecker & collision_free,
  const TraversalCostChecker & traversal_cost) const
{
  if (!finiteState(start) || !finiteState(goal) || !collision_free) {
    throw std::invalid_argument("search requires finite states and a collision checker");
  }
  SearchResult result;
  const auto make_key = [this](const PlanarState & state) {
      return StateKey{quantize(state.px, config_.position_resolution),
        quantize(state.py, config_.position_resolution),
        quantize(state.vx, config_.velocity_resolution),
        quantize(state.vy, config_.velocity_resolution)};
    };
  const auto heuristic = [this, &goal](const PlanarState & state) {
      // The omni base can move on both axes simultaneously.  The maximum of
      // the independent axis travel times is therefore an admissible lower
      // bound; Euclidean distance divided by one component limit is not.
      const double x_time = std::abs(goal.px - state.px) / config_.max_velocity_x;
      const double y_time = std::abs(goal.py - state.py) / config_.max_velocity_y;
      return config_.time_cost_weight * std::max(x_time, y_time);
    };
  const auto primitive_cost = [this, &collision_free, &traversal_cost](
      const PlanarState & state, const PlanarAcceleration & acceleration) {
      PlanarState previous = state;
      double time = 0.0;
      double integrated_cost = 0.0;
      while (time < config_.primitive_duration - 1e-9) {
        const double remaining = config_.primitive_duration - time;
        const double velocity_bound = std::max(1e-3, speed(previous) +
          std::hypot(acceleration.ax, acceleration.ay) * remaining);
        const double step = std::min({config_.collision_check_dt, remaining,
            config_.collision_check_distance / velocity_bound});
        time += step;
        const auto sample = propagate(state, acceleration, time);
        if (!collision_free(sample.px, sample.py)) {
          return std::numeric_limits<double>::infinity();
        }
        if (traversal_cost) {
          const double cost = traversal_cost(sample.px, sample.py);
          if (!std::isfinite(cost) || cost < 0.0) {
            return std::numeric_limits<double>::infinity();
          }
          integrated_cost += cost * step;
        }
        previous = sample;
      }
      return integrated_cost;
    };
  const auto connector_is_free = [this, &collision_free](const PlanarState & from,
      const PlanarState & to) {
      const double distance = std::hypot(to.px - from.px, to.py - from.py);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / config_.path_resolution)));
      for (int i = 1; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / steps;
        if (!collision_free(from.px + ratio * (to.px - from.px),
          from.py + ratio * (to.py - from.py)))
        {
          return false;
        }
      }
      return true;
    };

  if (!collision_free(start.px, start.py) || !collision_free(goal.px, goal.py)) {
    return result;
  }
  std::vector<Node> nodes;
  nodes.push_back(Node{start, {}, 0.0, 0.0, std::numeric_limits<std::size_t>::max()});
  std::priority_queue<OpenEntry> open;
  std::size_t next_insertion_id = 0;
  open.push(OpenEntry{heuristic(start), heuristic(start), 0.0, next_insertion_id++, 0});
  std::unordered_map<StateKey, double, StateKeyHash> best_g;
  best_g.emplace(make_key(start), 0.0);
  const std::vector<PlanarAcceleration> controls{
    {-config_.max_accel_x, -config_.max_accel_y}, {-config_.max_accel_x, 0.0},
    {-config_.max_accel_x, config_.max_accel_y}, {0.0, -config_.max_accel_y}, {0.0, 0.0},
    {0.0, config_.max_accel_y}, {config_.max_accel_x, -config_.max_accel_y},
    {config_.max_accel_x, 0.0}, {config_.max_accel_x, config_.max_accel_y}};
  const auto started = std::chrono::steady_clock::now();
  std::size_t goal_index = std::numeric_limits<std::size_t>::max();

  while (!open.empty()) {
    if (result.telemetry.expanded_nodes >= static_cast<std::size_t>(config_.max_expansions)) {
      break;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started).count();
    if (elapsed >= config_.search_timeout_ms) {
      result.telemetry.timeout = true;
      break;
    }
    const auto entry = open.top();
    open.pop();
    // Keep a value copy: appending successors can reallocate nodes.
    const Node node = nodes[entry.index];
    const auto best = best_g.find(make_key(node.state));
    if (best == best_g.end() || node.g > best->second + 1e-9) {
      continue;
    }
    ++result.telemetry.expanded_nodes;
    if (positionError(node.state, goal) <= config_.goal_position_tolerance &&
      speed(node.state) <= config_.goal_velocity_tolerance && connector_is_free(node.state, goal))
    {
      goal_index = entry.index;
      break;
    }
    for (const auto & control : controls) {
      const auto next = propagate(node.state, control, config_.primitive_duration);
      ++result.telemetry.generated_nodes;
      // The measured initial velocity is authoritative.  If it is already
      // outside our nominal envelope, only admit controls that reduce that
      // component's magnitude; once back inside, enforce the normal limit.
      const auto velocity_component_valid = [](double current, double next_value, double limit) {
          if (std::abs(current) <= limit + 1e-9) {
            return std::abs(next_value) <= limit + 1e-9;
          }
          return std::abs(next_value) <= std::abs(current) + 1e-9;
        };
      if (!finiteState(next) ||
        !velocity_component_valid(node.state.vx, next.vx, config_.max_velocity_x) ||
        !velocity_component_valid(node.state.vy, next.vy, config_.max_velocity_y) ||
        !std::isfinite(primitive_cost(node.state, control)))
      {
        continue;
      }
      const double edge = config_.time_cost_weight * config_.primitive_duration +
        config_.control_cost_weight * (control.ax * control.ax + control.ay * control.ay) *
        config_.primitive_duration +
        config_.local_costmap_cost_weight * primitive_cost(node.state, control);
      const double g = node.g + edge;
      const auto key = make_key(next);
      const auto previous = best_g.find(key);
      if (previous != best_g.end() && g >= previous->second - 1e-9) {
        continue;
      }
      best_g[key] = g;
      nodes.push_back(Node{next, control, g, node.time_from_start + config_.primitive_duration, entry.index});
      const double h = heuristic(next);
      open.push(OpenEntry{g + h, h, g, next_insertion_id++, nodes.size() - 1});
    }
  }

  if (goal_index == std::numeric_limits<std::size_t>::max()) {
    return result;
  }
  std::vector<std::size_t> chain;
  for (auto index = goal_index; index != std::numeric_limits<std::size_t>::max(); index = nodes[index].parent) {
    chain.push_back(index);
  }
  std::reverse(chain.begin(), chain.end());
  result.trajectory.push_back(TrajectoryPoint{nodes[chain.front()].state, {}, 0.0});
  for (std::size_t chain_index = 1; chain_index < chain.size(); ++chain_index) {
    const auto & parent = nodes[chain[chain_index - 1]];
    const auto & child = nodes[chain[chain_index]];
    const double endpoint_distance = std::hypot(
      child.state.px - parent.state.px, child.state.py - parent.state.py);
    const int samples = std::max(1, static_cast<int>(
        std::ceil(endpoint_distance / config_.path_resolution)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double time = config_.primitive_duration * static_cast<double>(sample) / samples;
      result.trajectory.push_back(TrajectoryPoint{
        propagate(parent.state, child.acceleration, time), child.acceleration,
        parent.time_from_start + time});
    }
  }
  const auto & terminal = nodes[goal_index];
  if (positionError(terminal.state, goal) > 1e-9) {
    result.trajectory.push_back(TrajectoryPoint{goal, {}, terminal.time_from_start});
  }
  result.telemetry.success = true;
  result.telemetry.trajectory_duration = terminal.time_from_start;
  result.telemetry.goal_position_error = positionError(terminal.state, goal);
  result.telemetry.goal_speed = speed(terminal.state);
  for (const auto & point : result.trajectory) {
    result.telemetry.max_speed = std::max(result.telemetry.max_speed, speed(point.state));
    result.telemetry.max_acceleration = std::max(result.telemetry.max_acceleration,
      std::hypot(point.acceleration.ax, point.acceleration.ay));
    result.telemetry.max_abs_vx = std::max(result.telemetry.max_abs_vx, std::abs(point.state.vx));
    result.telemetry.max_abs_vy = std::max(result.telemetry.max_abs_vy, std::abs(point.state.vy));
    result.telemetry.max_abs_ax = std::max(result.telemetry.max_abs_ax, std::abs(point.acceleration.ax));
    result.telemetry.max_abs_ay = std::max(result.telemetry.max_abs_ay, std::abs(point.acceleration.ay));
  }
  return result;
}

}  // namespace m1_fast_planner
