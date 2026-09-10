#include "m1_local_fast2d/restricted_mppi_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace m1_local_fast2d
{
namespace
{
double clampScalar(double value, double lower, double upper, bool & clamped)
{
  if (lower > upper) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double output = std::max(lower, std::min(upper, value));
  clamped = clamped || output != value;
  return output;
}
}

ClampResult clampCorrection(
  const RestrictedControl & nominal, const RestrictedControl & correction,
  const RestrictedCorrectionBounds & correction_bounds,
  const RestrictedAbsoluteBounds & absolute_bounds)
{
  ClampResult output;
  output.control.vx = nominal.vx + correction.vx;
  output.control.vy = nominal.vy + correction.vy;
  output.control.wz = nominal.wz + correction.wz;
  bool clamped = false;
  bool feasible = true;
  const auto apply = [&clamped, &feasible](double nominal_value, double correction_value,
      double correction_limit, double absolute_lower, double absolute_upper) {
      if (!std::isfinite(nominal_value) || !std::isfinite(correction_value) ||
        !std::isfinite(correction_limit) || correction_limit < 0.0) {
        feasible = false;
        return std::numeric_limits<double>::quiet_NaN();
      }
      const double lower = std::max(absolute_lower, nominal_value - correction_limit);
      const double upper = std::min(absolute_upper, nominal_value + correction_limit);
      if (lower > upper) {
        feasible = false;
        return std::max(absolute_lower, std::min(absolute_upper, nominal_value));
      }
      return clampScalar(nominal_value + correction_value, lower, upper, clamped);
    };
  output.control.vx = apply(
    nominal.vx, correction.vx, correction_bounds.delta_vx_max,
    absolute_bounds.vx_min, absolute_bounds.vx_max);
  output.control.vy = apply(
    nominal.vy, correction.vy, correction_bounds.delta_vy_max,
    -absolute_bounds.vy_max, absolute_bounds.vy_max);
  output.control.wz = apply(
    nominal.wz, correction.wz, correction_bounds.delta_wz_max,
    -absolute_bounds.wz_max, absolute_bounds.wz_max);
  output.clamped = clamped;
  output.feasible_intersection = feasible;
  return output;
}

std::vector<OmniPose> propagateOmni(
  OmniPose initial, const std::vector<RestrictedControl> & controls, double dt)
{
  std::vector<OmniPose> output;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return output;
  }
  output.reserve(controls.size());
  for (const auto & control : controls) {
    initial.x += (control.vx * std::cos(initial.yaw) - control.vy * std::sin(initial.yaw)) * dt;
    initial.y += (control.vx * std::sin(initial.yaw) + control.vy * std::cos(initial.yaw)) * dt;
    initial.yaw += control.wz * dt;
    output.push_back(initial);
  }
  return output;
}

RolloutEvaluation evaluateRollout(
  const std::vector<RolloutPosition> & rollout,
  const std::vector<ReferenceState> & reference,
  double position_tube_radius, bool collision_rejected)
{
  RolloutEvaluation output;
  if (!std::isfinite(position_tube_radius) || position_tube_radius < 0.0 ||
    rollout.size() != reference.size()) {
    output.tube_feasible = false;
    output.collision_free = !collision_rejected;
    return output;
  }
  output.tube_feasible = true;
  output.collision_free = !collision_rejected;
  for (std::size_t index = 0; index < rollout.size(); ++index) {
    const double dx = rollout[index].x - reference[index].x_ref;
    const double dy = rollout[index].y - reference[index].y_ref;
    const double deviation = std::hypot(dx, dy);
    if (!std::isfinite(deviation)) {
      output.tube_feasible = false;
      continue;
    }
    output.maximum_tube_deviation = std::max(output.maximum_tube_deviation, deviation);
    if (deviation > position_tube_radius) {
      output.tube_feasible = false;
    }
  }
  output.feasible = output.tube_feasible && output.collision_free;
  return output;
}

RolloutBatchEvaluation evaluateRollouts(
  const std::vector<std::vector<RolloutPosition>> & rollouts,
  const std::vector<ReferenceState> & reference, double position_tube_radius,
  const std::vector<bool> & collision_rejected)
{
  RolloutBatchEvaluation output;
  output.rollouts.reserve(rollouts.size());
  for (std::size_t index = 0; index < rollouts.size(); ++index) {
    const bool collision = index < collision_rejected.size() && collision_rejected[index];
    auto evaluation = evaluateRollout(rollouts[index], reference, position_tube_radius, collision);
    output.maximum_generated_tube_deviation = std::max(
      output.maximum_generated_tube_deviation, evaluation.maximum_tube_deviation);
    if (!evaluation.tube_feasible) {
      ++output.tube_rejected_count;
    }
    if (!evaluation.collision_free) {
      ++output.collision_rejected_count;
    }
    if (evaluation.feasible) {
      ++output.feasible_sample_count;
      output.maximum_accepted_tube_deviation = std::max(
        output.maximum_accepted_tube_deviation, evaluation.maximum_tube_deviation);
    }
    output.rollouts.push_back(evaluation);
  }
  return output;
}

}  // namespace m1_local_fast2d
