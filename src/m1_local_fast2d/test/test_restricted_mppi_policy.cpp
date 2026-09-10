#include "m1_local_fast2d/restricted_mppi_policy.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using m1_local_fast2d::ReferenceState;
using m1_local_fast2d::RestrictedAbsoluteBounds;
using m1_local_fast2d::RestrictedControl;
using m1_local_fast2d::RestrictedCorrectionBounds;
using m1_local_fast2d::RolloutPosition;

ReferenceState reference(double x, double y)
{
  ReferenceState output;
  output.x_ref = x;
  output.y_ref = y;
  output.reference_valid = true;
  return output;
}

std::vector<RolloutPosition> straight(double y)
{
  return {{0.0, y}, {0.1, y}, {0.2, y}};
}
}

TEST(RestrictedMPPIPolicy, ZeroAndSignedCorrectionAreMathematicallyExact)
{
  const RestrictedControl nominal{0.2, -0.1, 0.3};
  const RestrictedCorrectionBounds limits{0.1, 0.1, 0.16};
  const RestrictedAbsoluteBounds absolute{-0.5, 0.5, 0.5, 0.8};
  const auto zero = m1_local_fast2d::clampCorrection(nominal, {}, limits, absolute);
  EXPECT_DOUBLE_EQ(zero.control.vx, nominal.vx);
  EXPECT_DOUBLE_EQ(zero.control.vy, nominal.vy);
  EXPECT_DOUBLE_EQ(zero.control.wz, nominal.wz);
  const auto signed_correction = m1_local_fast2d::clampCorrection(
    nominal, {-0.05, 0.04, -0.08}, limits, absolute);
  EXPECT_DOUBLE_EQ(signed_correction.control.vx, 0.15);
  EXPECT_DOUBLE_EQ(signed_correction.control.vy, -0.06);
  EXPECT_DOUBLE_EQ(signed_correction.control.wz, 0.22);
}

TEST(RestrictedMPPIPolicy, CorrectionIsHardClampedOnEveryAxis)
{
  const auto result = m1_local_fast2d::clampCorrection(
    {0.2, -0.2, 0.1}, {0.9, -0.9, 0.9},
    {0.1, 0.1, 0.16}, {-0.5, 0.5, 0.5, 0.8});
  EXPECT_DOUBLE_EQ(result.control.vx, 0.3);
  EXPECT_DOUBLE_EQ(result.control.vy, -0.3);
  EXPECT_DOUBLE_EQ(result.control.wz, 0.26);
  EXPECT_TRUE(result.clamped);
  EXPECT_TRUE(result.feasible_intersection);
}

TEST(RestrictedMPPIPolicy, OmniLateralPropagationIsNotDropped)
{
  const auto yaw_zero = m1_local_fast2d::propagateOmni(
    {}, {{0.0, 0.4, 0.0}, {0.0, 0.4, 0.0}}, 0.05);
  ASSERT_EQ(yaw_zero.size(), 2U);
  EXPECT_NEAR(yaw_zero.back().x, 0.0, 1e-12);
  EXPECT_NEAR(yaw_zero.back().y, 0.04, 1e-12);

  constexpr double pi = 3.14159265358979323846;
  const auto yaw_ninety = m1_local_fast2d::propagateOmni(
    {0.0, 0.0, pi / 2.0}, {{1.0, 0.0, 0.0}}, 0.05);
  ASSERT_EQ(yaw_ninety.size(), 1U);
  EXPECT_NEAR(yaw_ninety.front().x, 0.0, 1e-12);
  EXPECT_NEAR(yaw_ninety.front().y, 0.05, 1e-12);
}

TEST(RestrictedMPPIPolicy, PositionTubeIsHardAndTimeIndexed)
{
  const std::vector<ReferenceState> reference_path = {
    reference(0.0, 0.0), reference(0.1, 0.0), reference(0.2, 0.0)};
  EXPECT_TRUE(m1_local_fast2d::evaluateRollout(
    straight(0.10), reference_path, 0.15, false).feasible);
  EXPECT_FALSE(m1_local_fast2d::evaluateRollout(
    straight(0.16), reference_path, 0.15, false).feasible);

  // The point at k=1 is close to a different time on the crossing path, but
  // far from R(t_1). Only the corresponding time-indexed point may be used.
  const std::vector<ReferenceState> crossing = {
    reference(0.0, 0.0), reference(1.0, 0.0), reference(0.0, 0.0)};
  const std::vector<RolloutPosition> self_near = {
    {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  EXPECT_FALSE(m1_local_fast2d::evaluateRollout(
    self_near, crossing, 0.15, false).feasible);
}

TEST(RestrictedMPPIPolicy, CollisionAndOppositeTopologyCannotContribute)
{
  const std::vector<ReferenceState> reference_path = {
    reference(0.0, 0.0), reference(0.1, 0.0), reference(0.2, 0.0)};
  const std::vector<std::vector<RolloutPosition>> rollouts = {
    straight(0.05), straight(0.20), straight(-0.20)};
  const auto result = m1_local_fast2d::evaluateRollouts(
    rollouts, reference_path, 0.15, {false, true, false});
  ASSERT_EQ(result.rollouts.size(), 3U);
  EXPECT_EQ(result.feasible_sample_count, 1U);
  EXPECT_EQ(result.tube_rejected_count, 2U);
  EXPECT_EQ(result.collision_rejected_count, 1U);
  EXPECT_FALSE(result.rollouts[1].feasible);
  EXPECT_FALSE(result.rollouts[2].feasible);
}

TEST(RestrictedMPPIPolicy, SmallDisturbanceRemainsFeasible)
{
  const std::vector<ReferenceState> reference_path = {
    reference(0.0, 0.0), reference(0.1, 0.0), reference(0.2, 0.0)};
  const auto result = m1_local_fast2d::evaluateRollouts(
    {straight(0.05), straight(0.10)}, reference_path, 0.15, {false, false});
  EXPECT_EQ(result.feasible_sample_count, 2U);
  EXPECT_LE(result.maximum_accepted_tube_deviation, 0.15);
}

TEST(RestrictedMPPIPolicy, AllInfeasibleHasNoUnrestrictedFallback)
{
  const std::vector<ReferenceState> reference_path = {
    reference(0.0, 0.0), reference(0.1, 0.0), reference(0.2, 0.0)};
  const auto result = m1_local_fast2d::evaluateRollouts(
    {straight(0.2), straight(-0.2)}, reference_path, 0.15, {true, false});
  EXPECT_EQ(result.feasible_sample_count, 0U);
  EXPECT_TRUE(result.rollouts[0].collision_free == false ||
    result.rollouts[0].tube_feasible == false);
  EXPECT_FALSE(result.rollouts[1].feasible);
}

TEST(RestrictedMPPIPolicy, CandidateOneTubeCannotCrossBackToCandidateZero)
{
  // Candidate 1 is the selected upper route.  A rollout on candidate 0's
  // lower route is spatially valid for that other topology, but cannot be
  // accepted against candidate 1's time-indexed tube.
  const std::vector<ReferenceState> candidate_one = {
    reference(0.0, 1.0), reference(0.1, 1.0), reference(0.2, 1.0)};
  const std::vector<RolloutPosition> candidate_zero_rollout = {
    {0.0, -1.0}, {0.1, -1.0}, {0.2, -1.0}};
  const auto evaluation = m1_local_fast2d::evaluateRollout(
    candidate_zero_rollout, candidate_one, 0.15, false);
  EXPECT_FALSE(evaluation.tube_feasible);
  EXPECT_FALSE(evaluation.feasible);
}

TEST(RestrictedMPPIPolicy, ReferenceSwitchStartsAtNewNominalWithoutOldCorrection)
{
  const RestrictedCorrectionBounds limits{0.1, 0.1, 0.16};
  const RestrictedAbsoluteBounds absolute{-0.5, 0.5, 0.5, 0.8};
  const RestrictedControl old_nominal{0.25, 0.10, 0.12};
  const RestrictedControl old_correction{0.08, -0.05, 0.10};
  const auto old_control = m1_local_fast2d::clampCorrection(
    old_nominal, old_correction, limits, absolute);
  ASSERT_TRUE(old_control.feasible_intersection);

  // A generation switch is reset, rather than translating an old absolute
  // control into the new topology. Zero correction must therefore equal T1's
  // nominal control exactly.
  const RestrictedControl new_nominal{-0.15, 0.20, -0.10};
  const auto switched = m1_local_fast2d::clampCorrection(
    new_nominal, {}, limits, absolute);
  EXPECT_DOUBLE_EQ(switched.control.vx, new_nominal.vx);
  EXPECT_DOUBLE_EQ(switched.control.vy, new_nominal.vy);
  EXPECT_DOUBLE_EQ(switched.control.wz, new_nominal.wz);
  EXPECT_NE(switched.control.vx, old_control.control.vx);
}
