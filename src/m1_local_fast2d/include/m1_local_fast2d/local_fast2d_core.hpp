#ifndef M1_LOCAL_FAST2D__LOCAL_FAST2D_CORE_HPP_
#define M1_LOCAL_FAST2D__LOCAL_FAST2D_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "m1_fast_planner/kinodynamic_astar.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"

namespace m1_local_fast2d
{

// Deliberately ROS-control-free: both the Nav2 wrapper and the causality test
// use this exact snapshot-to-guide implementation.
struct LocalFast2DConfig
{
  double lookahead_distance{2.0};
  double cost_weight{2.0};
  int max_planning_time_ms{80};
  int max_expansions{12000};
  bool allow_unknown{true};
};

struct LocalFast2DRequest
{
  nav2_msgs::msg::Costmap costmap;
  m1_fast_planner::PlanarState start;
  double start_yaw{0.0};
  nav_msgs::msg::Path reference_path;
  LocalFast2DConfig config;
};

struct LocalFast2DResult
{
  bool success{false};
  bool timeout{false};
  std::string failure_reason{"not_run"};
  nav_msgs::msg::Path local_path;
  geometry_msgs::msg::PoseStamped selected_local_goal;
  double latency_ms{0.0};
  std::size_t expanded_nodes{0};
  std::size_t generated_nodes{0};
  std::string cycle_class{"not_run"};
};

LocalFast2DResult planLocalFast2D(const LocalFast2DRequest & request);

}  // namespace m1_local_fast2d

#endif
