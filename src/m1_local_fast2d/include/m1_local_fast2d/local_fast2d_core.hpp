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
  int num_candidates{3};
  double candidate_diversity_threshold{0.25};
  double candidate_gate_initial_offset{0.6};
  double candidate_gate_offset_step{0.2};
  double candidate_gate_max_offset{1.8};
  double candidate_gate_clearance{0.2};
};

struct LocalFast2DRequest
{
  nav2_msgs::msg::Costmap costmap;
  m1_fast_planner::PlanarState start;
  double start_yaw{0.0};
  nav_msgs::msg::Path reference_path;
  LocalFast2DConfig config;
};

// This is deliberately ROS-control-free like LocalFast2DResult.  Every field
// except yaw is copied from one m1_fast_planner::TrajectoryPoint.  yaw uses
// the exact geometric orientation rule used for local_path below.
struct TimedTrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double time_from_start{0.0};
  double vx{0.0};
  double vy{0.0};
  // Incoming primitive control: acceleration that propagated the parent to
  // this point. The start point has no incoming primitive and is zero.
  double ax{0.0};
  double ay{0.0};
};

struct LocalFast2DResult
{
  bool success{false};
  bool timeout{false};
  std::string failure_reason{"not_run"};
  nav_msgs::msg::Path local_path;
  std::vector<TimedTrajectoryPoint> timed_trajectory;
  geometry_msgs::msg::PoseStamped selected_local_goal;
  double latency_ms{0.0};
  std::size_t expanded_nodes{0};
  std::size_t generated_nodes{0};
  std::string cycle_class{"not_run"};
  struct Candidate {
    nav_msgs::msg::Path local_path;
    std::vector<TimedTrajectoryPoint> timed_trajectory;
    double planner_cost{0.0};
    double duration{0.0};
    double generation_latency_ms{0.0};
    double minimum_diversity{0.0};
    std::size_t index{0};
  };
  std::vector<Candidate> candidates;
  int requested_candidates{1};
  double primary_planning_latency_ms{0.0};
};

LocalFast2DResult planLocalFast2D(const LocalFast2DRequest & request);

}  // namespace m1_local_fast2d

#endif
