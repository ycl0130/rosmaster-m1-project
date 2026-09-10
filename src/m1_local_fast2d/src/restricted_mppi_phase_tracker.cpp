#include "m1_local_fast2d/restricted_mppi_phase_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace m1_local_fast2d
{
namespace
{
double bounded(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}
}  // namespace

void RestrictedMPPIPhaseTracker::reset()
{
  reference_generation_ = 0;
  reference_id_ = 0;
  tracking_phase_s_ = 0.0;
  initialized_ = false;
}

RestrictedMPPIPhaseResult RestrictedMPPIPhaseTracker::update(
  const RestrictedTimedReference & reference, double robot_x, double robot_y,
  uint64_t reference_generation, double controller_dt,
  const RestrictedMPPIPhaseConfig & config)
{
  RestrictedMPPIPhaseResult result;
  const double duration = std::max(0.0, reference.duration());
  const bool valid_config = std::isfinite(config.phase_search_back_s) &&
    std::isfinite(config.phase_search_forward_s) && std::isfinite(config.max_phase_lead_s) &&
    std::isfinite(config.projection_resolution_s) && config.phase_search_back_s >= 0.0 &&
    config.phase_search_forward_s >= 0.0 && config.max_phase_lead_s >= 0.0 &&
    config.projection_resolution_s > 0.0;
  if (!valid_config || !std::isfinite(robot_x) || !std::isfinite(robot_y)) {
    result.tracking_phase_s = tracking_phase_s_;
    result.projected_phase_s = tracking_phase_s_;
    return result;
  }

  const auto reference_id = reference.metadata().reference_id;
  const bool changed = !initialized_ || reference_generation_ != reference_generation ||
    reference_id_ != reference_id;
  result.reference_reset = changed;
  result.previous_phase_s = changed ? 0.0 : tracking_phase_s_;

  // A new selection starts with a deliberately small local window.  It may
  // align a robot already a little way into the candidate, but never skips to
  // an unrelated later self-crossing segment.
  const double search_start = changed ? 0.0 :
    bounded(result.previous_phase_s - config.phase_search_back_s, 0.0, duration);
  const double search_end = changed ?
    std::min(duration, config.phase_search_forward_s) :
    bounded(result.previous_phase_s + config.phase_search_forward_s, 0.0, duration);
  const std::size_t samples = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil((search_end - search_start) /
    config.projection_resolution_s)));
  double best_phase = search_start;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(samples);
    const double phase = search_start + (search_end - search_start) * ratio;
    const auto state = reference.sample(phase);
    const double dx = state.x_ref - robot_x;
    const double dy = state.y_ref - robot_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_phase = phase;
    }
  }
  result.projected_phase_s = bounded(best_phase, 0.0, duration);

  const double dt = std::max(0.0, std::isfinite(controller_dt) ? controller_dt : 0.0);
  const double nominal_advance = result.previous_phase_s + dt;
  const double progress_ceiling = result.projected_phase_s + config.max_phase_lead_s;
  // Never regress on noisy nearest-point measurements.  If physical motion
  // stops, the projection ceiling catches the nominal advance within the
  // configured lead and holds the phase there.
  tracking_phase_s_ = bounded(
    std::max(result.previous_phase_s, std::min(nominal_advance, progress_ceiling)),
    0.0, duration);
  result.tracking_phase_s = tracking_phase_s_;
  reference_generation_ = reference_generation;
  reference_id_ = reference_id;
  initialized_ = true;
  return result;
}

}  // namespace m1_local_fast2d
