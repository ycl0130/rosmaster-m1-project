#ifndef M1_FAST_PLANNER__TRAJECTORY_TYPES_HPP_
#define M1_FAST_PLANNER__TRAJECTORY_TYPES_HPP_

#include <cstddef>
#include <vector>

namespace m1_fast_planner
{

struct PlanarState
{
  double px{0.0};
  double py{0.0};
  double vx{0.0};
  double vy{0.0};
};

struct PlanarAcceleration
{
  double ax{0.0};
  double ay{0.0};
};

struct TrajectoryPoint
{
  PlanarState state;
  PlanarAcceleration acceleration;
  double time_from_start{0.0};
};

struct SearchTelemetry
{
  bool success{false};
  bool timeout{false};
  std::size_t expanded_nodes{0};
  std::size_t generated_nodes{0};
  double trajectory_duration{0.0};
  double max_speed{0.0};
  double max_acceleration{0.0};
  double max_abs_vx{0.0};
  double max_abs_vy{0.0};
  double max_abs_ax{0.0};
  double max_abs_ay{0.0};
  double goal_position_error{0.0};
  double goal_speed{0.0};
};

struct SearchResult
{
  std::vector<TrajectoryPoint> trajectory;
  SearchTelemetry telemetry;
};

}  // namespace m1_fast_planner

#endif  // M1_FAST_PLANNER__TRAJECTORY_TYPES_HPP_
