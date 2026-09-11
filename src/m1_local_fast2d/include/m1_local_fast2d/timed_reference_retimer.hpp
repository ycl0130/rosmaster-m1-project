#ifndef M1_LOCAL_FAST2D__TIMED_REFERENCE_RETIMER_HPP_
#define M1_LOCAL_FAST2D__TIMED_REFERENCE_RETIMER_HPP_

#include "m1_local_fast2d/msg/timed_trajectory.hpp"

namespace m1_local_fast2d
{

struct TimedReferenceRetimingLimits
{
  double vx_max{0.5};
  double vy_max{0.5};
  double wz_max{0.8};
  double ax_max{0.6};
  double ay_max{0.6};
  double awz_max{0.7};
};

struct TimedReferenceRetimingDiagnostics
{
  double time_scale{1.0};
  double before_duration{0.0};
  double after_duration{0.0};
  double before_max_vx{0.0}, before_max_vy{0.0}, before_max_wz{0.0};
  double after_max_vx{0.0}, after_max_vy{0.0}, after_max_wz{0.0};
  double before_max_ax{0.0}, before_max_ay{0.0}, before_max_awz{0.0};
  double after_max_ax{0.0}, after_max_ay{0.0}, after_max_awz{0.0};
  double geometry_error{0.0};
  // Terminal approach diagnostics. A terminal heading discontinuity can be
  // represented as an in-place turn at the already-reached terminal pose so
  // it cannot stretch the preceding translational approach.
  bool terminal_heading_split{false};
  double terminal_approach_distance{0.0};
  double terminal_approach_speed{0.0};
  double terminal_rotation_duration{0.0};
  double terminal_time_scale{1.0};
  std::size_t ignored_duplicate_segments{0};
  std::size_t reprocessed_zero_duration_segments{0};
  bool sanity_guard_triggered{false};
  bool valid{false};
};

// Retimes a controller-owned copy only.  Each input pose is copied exactly;
// timestamps and derivative fields are regenerated from the pose sequence.
m1_local_fast2d::msg::TimedTrajectory retimeTimedReference(
  const m1_local_fast2d::msg::TimedTrajectory & input,
  const TimedReferenceRetimingLimits & limits,
  TimedReferenceRetimingDiagnostics * diagnostics = nullptr);

}  // namespace m1_local_fast2d

#endif
