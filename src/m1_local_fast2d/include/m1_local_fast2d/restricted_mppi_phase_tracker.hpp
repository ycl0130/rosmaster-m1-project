#ifndef M1_LOCAL_FAST2D__RESTRICTED_MPPI_PHASE_TRACKER_HPP_
#define M1_LOCAL_FAST2D__RESTRICTED_MPPI_PHASE_TRACKER_HPP_

#include <cstdint>

#include "m1_local_fast2d/restricted_mppi_reference.hpp"

namespace m1_local_fast2d
{

// This is deliberately a pose-progress clock, not a wall clock.  The local
// projection window prevents a self-near/crossing trajectory from changing
// topology while the controller is tracking a selected candidate.
struct RestrictedMPPIPhaseConfig
{
  double phase_search_back_s{0.20};
  double phase_search_forward_s{0.35};
  double max_phase_lead_s{0.15};
  double projection_resolution_s{0.01};
};

struct RestrictedMPPIPhaseResult
{
  double previous_phase_s{0.0};
  double projected_phase_s{0.0};
  double tracking_phase_s{0.0};
  bool reference_reset{false};
};

class RestrictedMPPIPhaseTracker
{
public:
  RestrictedMPPIPhaseResult update(
    const RestrictedTimedReference & reference, double robot_x, double robot_y,
    uint64_t reference_generation, double controller_dt,
    const RestrictedMPPIPhaseConfig & config);

  void reset();

private:
  uint64_t reference_generation_{0};
  uint64_t reference_id_{0};
  double tracking_phase_s_{0.0};
  bool initialized_{false};
};

}  // namespace m1_local_fast2d

#endif
