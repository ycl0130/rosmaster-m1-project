#include <chrono>
#include <cstdio>
#include <cmath>

#include "m1_fast_planner/kinodynamic_astar.hpp"

namespace
{

void runCase(const char * name, const m1_fast_planner::PlanarState & start,
  const m1_fast_planner::PlanarState & goal, const m1_fast_planner::CollisionChecker & collision_free)
{
  m1_fast_planner::KinodynamicAstarConfig config;
  config.search_timeout_ms = 200;
  config.max_expansions = 100000;
  const auto begin = std::chrono::steady_clock::now();
  const auto result = m1_fast_planner::KinodynamicAstar(config).search(start, goal, collision_free);
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  std::printf("%s success=%s timeout=%s latency_ms=%.3f expanded=%zu generated=%zu "
    "points=%zu duration_s=%.3f max_speed=%.3f max_accel=%.3f\n", name,
    result.telemetry.success ? "true" : "false", result.telemetry.timeout ? "true" : "false",
    elapsed, result.telemetry.expanded_nodes, result.telemetry.generated_nodes, result.trajectory.size(),
    result.telemetry.trajectory_duration, result.telemetry.max_speed, result.telemetry.max_acceleration);
}

}  // namespace

int main()
{
  runCase("free_space", {-2.5, -1.5, 0.0, 0.0}, {-2.5, 1.5, 0.0, 0.0},
    [](double x, double y) {return x >= -4.0 && x <= 4.0 && y >= -3.0 && y <= 3.0;});
  runCase("center_obstacle_detour", {-2.5, -1.5, 0.0, 0.0}, {2.5, 1.5, 0.0, 0.0},
    [](double x, double y) {
      return x >= -4.0 && x <= 4.0 && y >= -3.0 && y <= 3.0 && std::hypot(x, y) > 0.45;
    });
  return 0;
}
