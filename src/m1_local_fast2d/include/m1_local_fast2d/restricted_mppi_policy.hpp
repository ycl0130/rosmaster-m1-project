#ifndef M1_LOCAL_FAST2D__RESTRICTED_MPPI_POLICY_HPP_
#define M1_LOCAL_FAST2D__RESTRICTED_MPPI_POLICY_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include "m1_local_fast2d/restricted_mppi_reference.hpp"

namespace m1_local_fast2d
{

struct RestrictedControl
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct RestrictedCorrectionBounds
{
  double delta_vx_max{0.10};
  double delta_vy_max{0.10};
  double delta_wz_max{0.16};
};

struct RestrictedAbsoluteBounds
{
  double vx_min{-0.5};
  double vx_max{0.5};
  double vy_max{0.5};
  double wz_max{0.8};
};

struct ClampResult
{
  RestrictedControl control;
  bool clamped{false};
  bool feasible_intersection{true};
};

ClampResult clampCorrection(
  const RestrictedControl & nominal, const RestrictedControl & correction,
  const RestrictedCorrectionBounds & correction_bounds,
  const RestrictedAbsoluteBounds & absolute_bounds);

struct OmniPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

std::vector<OmniPose> propagateOmni(
  OmniPose initial, const std::vector<RestrictedControl> & controls, double dt);

struct RolloutPosition
{
  double x{0.0};
  double y{0.0};
};

struct RolloutEvaluation
{
  bool feasible{false};
  bool tube_feasible{false};
  bool collision_free{false};
  double maximum_tube_deviation{0.0};
};

RolloutEvaluation evaluateRollout(
  const std::vector<RolloutPosition> & rollout,
  const std::vector<ReferenceState> & reference,
  double position_tube_radius, bool collision_rejected);

struct RolloutBatchEvaluation
{
  std::vector<RolloutEvaluation> rollouts;
  std::size_t feasible_sample_count{0};
  std::size_t tube_rejected_count{0};
  std::size_t collision_rejected_count{0};
  double maximum_accepted_tube_deviation{0.0};
  double maximum_generated_tube_deviation{0.0};
};

RolloutBatchEvaluation evaluateRollouts(
  const std::vector<std::vector<RolloutPosition>> & rollouts,
  const std::vector<ReferenceState> & reference, double position_tube_radius,
  const std::vector<bool> & collision_rejected);

}  // namespace m1_local_fast2d

#endif
