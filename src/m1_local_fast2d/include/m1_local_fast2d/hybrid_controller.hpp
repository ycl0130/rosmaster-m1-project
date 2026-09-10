#ifndef M1_LOCAL_FAST2D__HYBRID_CONTROLLER_HPP_
#define M1_LOCAL_FAST2D__HYBRID_CONTROLLER_HPP_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "nav2_core/controller.hpp"
#include "m1_local_fast2d/restricted_mppi_adapter.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "m1_local_fast2d/msg/timed_trajectory.hpp"
#include "m1_local_fast2d/msg/local_trajectory_candidate_array.hpp"
#include "m1_local_fast2d/msg/candidate_risk_score_array.hpp"
#include "m1_local_fast2d/msg/selected_local_trajectory.hpp"
#include "m1_local_fast2d/msg/selection_diagnostic_array.hpp"
#include "m1_local_fast2d/candidate_selector.hpp"
#include "m1_local_fast2d/restricted_mppi_reference.hpp"
#include "m1_scope_msgs/msg/scope_prediction_sequence.hpp"
#include "m1_scope_risk/risk_field_snapshot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace m1_local_fast2d
{

class HybridController : public nav2_core::Controller
{
public:
  HybridController() = default;
  ~HybridController() override;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;
  void setSpeedLimit(const double &, const bool &) override;

private:
  struct Input {nav_msgs::msg::Path plan; geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Twist velocity; uint64_t generation{0};};
  // snapshot is the immutable master-map copy given to planLocalFast2D.  It is
  // retained solely so an accepted guide can be observed against the exact
  // map it was planned in; MPPI never receives or consults it.
  struct Guide {nav_msgs::msg::Path path; nav_msgs::msg::Path mppi_plan;
    m1_local_fast2d::msg::TimedTrajectory timed_trajectory;
    nav2_msgs::msg::Costmap snapshot; uint64_t id{0}; uint64_t generation{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME}; uint32_t selected_candidate_index{0}; bool valid{false};};
  void workerLoop();
  void planOnce(const Input & input);
  void publishDiagnostic(bool success, bool timeout, std::size_t expanded,
    std::size_t generated, double latency_ms, std::size_t path_points,
    const std::string & cycle_class);
  void publishAcceptedDiagnostic(
    const Guide & guide, const std::shared_ptr<const RestrictedTimedReference> & reference,
    bool switched, uint64_t previous_planning_result_id);
  void publishRestrictedDiagnostic(const RestrictedMPPIDiagnostics & diagnostics);
  struct LatencyStats {
    std::size_t count{0};
    double sum_ms{0.0};
    double max_ms{0.0};
    std::deque<double> samples_ms;
  };
  void recordLatency(LatencyStats & stats, double latency_ms);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_mppi_controller::MPPIController> mppi_;
  std::unique_ptr<RestrictedMPPIAdapter> restricted_mppi_adapter_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, goal_pub_, accepted_pub_;
  rclcpp::Publisher<m1_local_fast2d::msg::TimedTrajectory>::SharedPtr accepted_timed_trajectory_pub_;
  rclcpp::Publisher<m1_local_fast2d::msg::LocalTrajectoryCandidateArray>::SharedPtr candidates_pub_;
  rclcpp::Publisher<m1_local_fast2d::msg::CandidateRiskScoreArray>::SharedPtr risk_scores_pub_;
  rclcpp::Publisher<m1_local_fast2d::msg::SelectedLocalTrajectory>::SharedPtr selected_trajectory_pub_;
  rclcpp::Publisher<m1_local_fast2d::msg::SelectionDiagnosticArray>::SharedPtr selection_diagnostics_pub_;
  rclcpp::Subscription<m1_scope_msgs::msg::ScopePredictionSequence>::SharedPtr scope_sequence_sub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr accepted_costmap_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::string name_, mode_{"shadow"}, local_frame_;
  double update_rate_{8.0}, stale_timeout_{0.5}, lookahead_{2.0}, cost_weight_{2.0}, candidate_diversity_threshold_{0.25}, candidate_gate_initial_offset_{0.6}, candidate_gate_offset_step_{0.2}, candidate_gate_max_offset_{1.8}, candidate_gate_clearance_{0.2};
  int num_candidates_{3};
  double risk_scoring_dt_{0.1}, risk_high_threshold_{0.7}, minimum_coverage_ratio_{0.25}, prediction_fresh_timeout_{1.0};
  RestrictedMPPIConfig restricted_mppi_config_;
  CandidateSelectorConfig selector_config_;
  int max_planning_time_ms_{80};
  bool allow_unknown_{true}, active_{false}, stop_worker_{false};
  // Test-only, default-off switch used to verify the production fallback
  // branch without changing any planner or safety behavior.
  bool test_force_fallback_{false};
  std::mutex mutex_;
  std::shared_ptr<const m1_scope_risk::RiskFieldSnapshot> latest_risk_snapshot_;
  std::optional<m1_local_fast2d::msg::TimedTrajectory> previous_selected_trajectory_;
  std::shared_ptr<const RestrictedTimedReference> latest_reference_;
  std::condition_variable wake_;
  std::thread worker_;
  Input latest_input_;
  Guide latest_guide_;
  uint64_t input_generation_{0}, forwarded_generation_{0}, next_guide_id_{0}, reference_generation_{0};
  uint64_t fallback_count_{0}, stale_count_{0}, no_path_count_{0}, timeout_count_{0};
  uint64_t guide_generated_count_{0}, guide_accepted_count_{0};
  std::string reference_source_{"global_fallback"};
  // These are written by the sole local-planning worker and exposed in its
  // diagnostics; keeping a bounded sample history permits p50/p95 without
  // changing planning behavior.
  LatencyStats full_search_stats_, shortcut_stats_, other_stats_;
  uint64_t path_reuse_count_{0}, no_op_count_{0};
};

}  // namespace m1_local_fast2d

#endif
