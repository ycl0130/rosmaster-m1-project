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
  uint32_t rollout_count{0};
  std::size_t feasible_sample_count{0};
  std::size_t tube_rejected_count{0};
  std::size_t collision_rejected_count{0};
  std::size_t control_bound_clamped_count{0};
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
  bool collisionAtPose(float x, float y, float yaw, bool consider_footprint) const;

  nav2_mppi_controller::MPPIController * controller_{nullptr};
  bool version_guard_ok_{false};
  std::string upstream_version_;
  std::string version_failure_;
  uint64_t last_reference_generation_{0};
  uint64_t last_reference_id_{0};
  RestrictedMPPIPhaseTracker phase_tracker_;
};

}  // namespace m1_local_fast2d

#endif
