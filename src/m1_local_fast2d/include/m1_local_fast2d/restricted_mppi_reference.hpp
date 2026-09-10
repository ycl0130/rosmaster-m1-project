#ifndef M1_LOCAL_FAST2D__RESTRICTED_MPPI_REFERENCE_HPP_
#define M1_LOCAL_FAST2D__RESTRICTED_MPPI_REFERENCE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "m1_local_fast2d/msg/timed_trajectory.hpp"

namespace m1_local_fast2d
{

// Fast2D's planning frame is the frame of the local planning snapshot
// (normally odom).  Its vx/vy state is expressed in that frame, not in the
// robot body frame: the planner receives odometry velocity rotated into the
// planning frame and propagates it with a world-frame double integrator.
constexpr const char * kTimedTrajectoryVelocityFrame = "planning_frame_world";

struct RestrictedTimedReferenceConfig
{
  double time_epsilon{1e-9};
  double omega_max{4.0};
};

struct ReferenceState
{
  double x_ref{0.0};
  double y_ref{0.0};
  double yaw_ref{0.0};
  double vx_world_ref{0.0};
  double vy_world_ref{0.0};
  double vx_body_ref{0.0};
  double vy_body_ref{0.0};
  double omega_ref{0.0};
  double omega_raw{0.0};
  double ax_ref{0.0};
  double ay_ref{0.0};
  bool reference_valid{false};
  bool terminal_hold{false};
};

struct RestrictedTimedReferenceMetadata
{
  uint64_t reference_id{0};
  uint64_t planning_result_id{0};
  uint64_t input_generation{0};
  uint32_t selected_candidate_index{0};
  std::string frame_id;
  int64_t source_stamp_ns{0};
  int64_t activation_stamp_ns{0};
};

struct RestrictedTimedReferenceDiagnostics
{
  RestrictedTimedReferenceMetadata metadata;
  double reference_age_sec{0.0};
  double reference_duration_sec{0.0};
  double mppi_horizon_duration_sec{0.0};
  std::size_t horizon_sample_count{0};
  std::string velocity_frame{kTimedTrajectoryVelocityFrame};
  ReferenceState first_reference;
  std::string terminal_handling_mode{"terminal_pose_hold_zero_velocity"};
  bool reference_switched{false};
  uint64_t previous_planning_result_id{0};
};

// Immutable, time-origin-normalized view of one selected TimedTrajectory.
// The object is deliberately independent of MPPI.  Phase 6A uses it only for
// validation and diagnostics; no optimizer, sampler, critic, or command path
// consumes this class.
class RestrictedTimedReference
{
public:
  RestrictedTimedReference(
    const msg::TimedTrajectory & trajectory,
    RestrictedTimedReferenceMetadata metadata,
    const RestrictedTimedReferenceConfig & config = {});

  ReferenceState sample(double elapsed_sec) const;

  std::vector<ReferenceState> sampleHorizon(
    double elapsed_sec, std::size_t sample_count, double model_dt) const;

  RestrictedTimedReferenceDiagnostics diagnostics(
    int64_t now_stamp_ns, double horizon_duration_sec,
    std::size_t horizon_sample_count, bool switched = false,
    uint64_t previous_planning_result_id = 0) const;

  bool valid() const {return valid_;}
  double duration() const {return duration_sec_;}
  const RestrictedTimedReferenceMetadata & metadata() const {return metadata_;}
  const RestrictedTimedReferenceConfig & config() const {return config_;}

private:
  struct Sample
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double time{0.0};
    double vx{0.0};
    double vy{0.0};
    double ax{0.0};
    double ay{0.0};
    double omega_raw{0.0};
    double omega{0.0};
  };

  static double clamp(double value, double lower, double upper);
  static double shortestAngleDelta(double from, double to);
  ReferenceState terminalState() const;
  ReferenceState stateFromSample(const Sample & sample, bool terminal_hold) const;

  RestrictedTimedReferenceMetadata metadata_;
  RestrictedTimedReferenceConfig config_;
  std::vector<Sample> samples_;
  Sample terminal_sample_;
  double duration_sec_{0.0};
  bool terminal_is_zero_duration_{false};
  bool valid_{false};
};

}  // namespace m1_local_fast2d

#endif
