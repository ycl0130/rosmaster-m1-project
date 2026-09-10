#ifndef M1_FAST_PLANNER__KINODYNAMIC_ASTAR_HPP_
#define M1_FAST_PLANNER__KINODYNAMIC_ASTAR_HPP_

#include <functional>
#include <vector>

#include "m1_fast_planner/trajectory_types.hpp"

namespace m1_fast_planner
{

struct KinodynamicAstarConfig
{
  double max_velocity_x{0.45};
  double max_velocity_y{0.45};
  double max_accel_x{0.6};
  double max_accel_y{0.6};
  double primitive_duration{0.25};
  double collision_check_dt{0.05};
  double collision_check_distance{0.05};
  double position_resolution{0.1};
  double velocity_resolution{0.1};
  double goal_position_tolerance{0.15};
  double goal_velocity_tolerance{0.2};
  int search_timeout_ms{200};
  int max_expansions{20000};
  double time_cost_weight{1.0};
  double control_cost_weight{0.1};
  double local_costmap_cost_weight{0.0};
  double path_resolution{0.05};
};

using CollisionChecker = std::function<bool(double, double)>;
using TraversalCostChecker = std::function<double(double, double)>;

class KinodynamicAstar
{
public:
  explicit KinodynamicAstar(KinodynamicAstarConfig config);

  static PlanarState propagate(
    const PlanarState & state, const PlanarAcceleration & acceleration, double time);
  static bool validConfig(const KinodynamicAstarConfig & config);
  static long long quantize(double value, double resolution);

  SearchResult search(
    const PlanarState & start, const PlanarState & goal, const CollisionChecker & collision_free,
    const TraversalCostChecker & traversal_cost = {}) const;

private:
  KinodynamicAstarConfig config_;
};

}  // namespace m1_fast_planner

#endif  // M1_FAST_PLANNER__KINODYNAMIC_ASTAR_HPP_
