#ifndef M1_LOCAL_FAST2D__RESTRICTED_MPPI_ADAPTER_HPP_
#define M1_LOCAL_FAST2D__RESTRICTED_MPPI_ADAPTER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "m1_local_fast2d/restricted_mppi_internal.hpp"
#include "m1_local_fast2d/restricted_mppi_phase_tracker.hpp"
#include "m1_local_fast2d/restricted_mppi_policy.hpp"

namespace m1_local_fast2d
{

struct RestrictedMPPIConfig
{
  bool enabled{false};
  double delta_vx_max{0.10};
  double delta_vy_max{0.10};
  double delta_wz_max{0.16};
  double position_tube_radius{0.15};
  double phase_search_back_s{0.20};
  double phase_search_forward_s{0.35};
  double max_phase_lead_s{0.15};
  double phase_projection_resolution_s{0.01};
  bool consider_footprint{true};
};

struct RestrictedMPPIDiagnostics
{
  bool enabled{false};
  bool version_guard_ok{false};
  bool reference_switched{false};
  bool reset_performed{false};
  uint64_t planning_result_id{0};
  uint64_t reference_generation{0};
  uint32_t selected_candidate_index{0};
  uint64_t reference_id{0};
  double reference_age_sec{0.0};
  double previous_tracking_phase_s{0.0};
  double projected_phase_s{0.0};
  double tracking_phase_s{0.0};
  double phase_lead_s{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  int64_t reference_source_stamp_ns{0};
  int64_t reference_activation_stamp_ns{0};
  double reference_length_m{0.0};
  double reference_start_x{0.0};
  double reference_start_y{0.0};
  double reference_last_x{0.0};
  double reference_last_y{0.0};
  double initial_reference_error_m{0.0};
  double reference_dt_s{0.0};
  double reference0_yaw{0.0}, reference0_vx_world{0.0}, reference0_vy_world{0.0}, reference0_wz{0.0};
  double robot_yaw{0.0}, robot_vx{0.0}, robot_vy{0.0}, robot_wz{0.0};
  double rollout0_x{0.0}, rollout0_y{0.0}, rollout0_yaw{0.0};
  double tube0_reference_x{0.0}, tube0_reference_y{0.0}, tube0_deviation_m{0.0};
  std::string horizon_state_json;
  std::string original_horizon_state_json;
  std::string conditioned_horizon_state_json;
  std::string conditioning_input_check_json;
  double original_max_kinematic_residual_m{0.0};
  double conditioned_max_kinematic_residual_m{0.0};
  double original_max_yaw_residual_rad{0.0};
  double conditioned_max_yaw_residual_rad{0.0};
  std::string mppi_limits_json;
  std::string first_rollout_json;
  std::string deviation_inputs_json;
  uint32_t rollout_count{0};
  std::size_t feasible_sample_count{0};
  double feasible_sample_ratio{0.0};
  std::size_t tube_rejected_count{0};
  std::size_t collision_rejected_count{0};
  // First tube breach in this compute cycle. This is observational only and
  // deliberately does not alter the rejection predicate or selected control.
  bool first_tube_breach_captured{false};
  uint32_t first_tube_breach_batch{0}, first_tube_breach_step{0};
  double first_tube_breach_deviation_m{0.0};
  double first_tube_breach_rollout_x{0.0}, first_tube_breach_rollout_y{0.0};
  double first_tube_breach_reference_x{0.0}, first_tube_breach_reference_y{0.0};
  bool collision_full_mask{false};
  int64_t collision_full_mask_stamp_ns{0};
  std::string collision_rollout_evidence_json;
  std::string collision_costmap_json;
  std::size_t control_bound_clamped_count{0};
  // The legacy count above is a sample-step "any axis" count (and also
  // included post-update sequence clamps).  These fields make the sampling
  // denominator explicit and are deliberately restricted to generated MPPI
  // controls: batch_size * time_steps * 3 scalar velocity controls.
  std::size_t proposed_scalar_control_count{0};
  std::size_t clamped_scalar_control_count{0};
  std::size_t proposed_sample_step_count{0};
  std::size_t clamped_sample_step_count{0};
  std::size_t proposed_sample_count{0};
  std::size_t clamped_sample_count{0};
  std::size_t vx_clamp_count{0}, vy_clamp_count{0}, wz_clamp_count{0};
  std::size_t lower_bound_clamp_count{0}, upper_bound_clamp_count{0};
  std::size_t velocity_bound_clamp_count{0};
  // This seam clamps velocity controls only. Acceleration is propagated by
  // stock MPPI and has no explicit clamp here, so this is expected to be 0.
  std::size_t acceleration_bound_clamp_count{0};
  double controller_total_ms{0.0};
  double restricted_mppi_total_ms{0.0};
  double phase_tracking_ms{0.0};
  double sampling_ms{0.0};
  double control_bounds_ms{0.0};
  double rollout_ms{0.0};
  double tube_filter_ms{0.0};
  double collision_filter_ms{0.0};
  double critic_ms{0.0};
  double costmap_access_ms{0.0};
  double command_finalization_ms{0.0};
  uint32_t batch_size{0};
  uint32_t time_steps{0};
  double model_dt{0.0};
  double maximum_accepted_tube_deviation{0.0};
  double maximum_generated_tube_deviation{0.0};
  RestrictedControl nominal_command;
  RestrictedControl selected_correction;
  RestrictedControl final_command;
  std::string comparison_frame;
  std::string status{"NO_REFERENCE"};
  std::string upstream_version;
};

// Adapter around the stock MPPIController.  Feature OFF never calls this
// class; it continues through MPPIController::computeVelocityCommands().
class RestrictedMPPIAdapter
{
public:
  explicit RestrictedMPPIAdapter(nav2_mppi_controller::MPPIController & controller);

  bool versionCompatible() const {return version_guard_ok_;}
  const std::string & upstreamVersion() const {return upstream_version_;}
  const std::string & versionFailure() const {return version_failure_;}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker,
    const std::shared_ptr<const RestrictedTimedReference> & reference,
    uint64_t reference_generation, const RestrictedMPPIConfig & config,
    RestrictedMPPIDiagnostics & diagnostics);

private:
  bool checkInstalledVersion();
  geometry_msgs::msg::TwistStamped zeroCommand(const builtin_interfaces::msg::Time & stamp) const;
  bool transformReference(
    const std::vector<ReferenceState> & source, const std::string & source_frame,
    const std::string & target_frame, std::vector<ReferenceState> & output) const;
  bool collisionAtPose(float x, float y, float yaw, bool consider_footprint,
    std::string * source = nullptr, std::string * evidence_json = nullptr) const;

  nav2_mppi_controller::MPPIController * controller_{nullptr};
  bool version_guard_ok_{false};
  std::string upstream_version_;
  std::string version_failure_;
  uint64_t last_reference_generation_{0};
  uint64_t last_reference_id_{0};
  bool optimizer_input_validation_logged_{false};
  const float * control_sequence_before_reset_vx_{nullptr};
  const float * control_sequence_before_reset_vy_{nullptr};
  const float * control_sequence_before_reset_wz_{nullptr};
  RestrictedMPPIPhaseTracker phase_tracker_;
};

}  // namespace m1_local_fast2d

#endif
