#include "m1_local_fast2d/restricted_mppi_reference.hpp"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <memory>

#include <gtest/gtest.h>

namespace
{
using m1_local_fast2d::RestrictedTimedReference;
using m1_local_fast2d::RestrictedTimedReferenceConfig;
using m1_local_fast2d::RestrictedTimedReferenceMetadata;
using m1_local_fast2d::msg::TimedTrajectory;
using m1_local_fast2d::msg::TimedTrajectoryPoint;

TimedTrajectory trajectory(std::initializer_list<TimedTrajectoryPoint> points)
{
  TimedTrajectory output;
  output.header.frame_id = "odom";
  output.header.stamp.sec = 10;
  output.planning_result_id = 7;
  output.input_generation = 3;
  output.points.assign(points.begin(), points.end());
  return output;
}

TimedTrajectoryPoint point(double t, double x, double y, double yaw,
  double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0)
{
  TimedTrajectoryPoint output;
  output.time_from_start = t;
  output.x = x;
  output.y = y;
  output.yaw = yaw;
  output.vx = vx;
  output.vy = vy;
  output.ax = ax;
  output.ay = ay;
  return output;
}

RestrictedTimedReferenceMetadata metadata(uint64_t id = 1, int64_t activation = 20)
{
  RestrictedTimedReferenceMetadata output;
  output.reference_id = id;
  output.planning_result_id = id + 10;
  output.input_generation = id + 20;
  output.selected_candidate_index = 1;
  output.frame_id = "odom";
  output.source_stamp_ns = 10'000'000'000LL;
  output.activation_stamp_ns = activation * 1'000'000'000LL;
  return output;
}
}

TEST(RestrictedTimedReference, ExactTimeAndInterpolation)
{
  auto input = trajectory({point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    point(0.1, 1.0, 2.0, 0.1, 1.0, 2.0), point(0.2, 2.0, 4.0, 0.2, 2.0, 4.0)});
  RestrictedTimedReference reference(input, metadata());
  ASSERT_TRUE(reference.valid());
  const auto exact = reference.sample(0.1);
  EXPECT_DOUBLE_EQ(exact.x_ref, 1.0);
  EXPECT_DOUBLE_EQ(exact.y_ref, 2.0);
  EXPECT_DOUBLE_EQ(exact.vx_world_ref, 1.0);
  EXPECT_DOUBLE_EQ(exact.vy_world_ref, 2.0);
  const auto middle = reference.sample(0.05);
  EXPECT_DOUBLE_EQ(middle.x_ref, 0.5);
  EXPECT_DOUBLE_EQ(middle.y_ref, 1.0);
  EXPECT_DOUBLE_EQ(middle.vx_world_ref, 0.5);
  EXPECT_DOUBLE_EQ(middle.vy_world_ref, 1.0);
}

TEST(RestrictedTimedReference, YawUnwrapAndOmegaWrap)
{
  constexpr double pi = 3.14159265358979323846;
  auto input = trajectory({point(0.0, 0.0, 0.0, 179.0 * pi / 180.0),
    point(0.1, 1.0, 0.0, -179.0 * pi / 180.0),
    point(0.2, 2.0, 0.0, -178.0 * pi / 180.0)});
  RestrictedTimedReference reference(input, metadata());
  const auto middle = reference.sample(0.05);
  EXPECT_NEAR(std::abs(middle.yaw_ref), pi, 1e-9);
  EXPECT_LT(std::abs(middle.yaw_ref), 2.0 * pi);
  EXPECT_NEAR(reference.sample(0.05).omega_raw, 2.0 * pi / 180.0 / 0.1, 1e-9);
  EXPECT_LT(std::abs(reference.sample(0.05).omega_raw), 10.0);
}

TEST(RestrictedTimedReference, WorldToBodyVelocity)
{
  constexpr double pi = 3.14159265358979323846;
  auto input = trajectory({point(0.0, 0.0, 0.0, pi / 2.0, 1.0, 0.0),
    point(0.1, 0.1, 0.0, pi / 2.0, 1.0, 0.0)});
  RestrictedTimedReference reference(input, metadata());
  const auto state = reference.sample(0.0);
  EXPECT_NEAR(state.vx_body_ref, 0.0, 1e-9);
  EXPECT_NEAR(state.vy_body_ref, -1.0, 1e-9);
}

TEST(RestrictedTimedReference, OmegaAndTerminalZeroDuration)
{
  auto input = trajectory({point(0.0, 0.0, 0.0, 0.0),
    point(0.1, 1.0, 0.0, 0.1), point(0.1, 2.0, 0.0, 3.0, 10.0, 10.0, 50.0, 50.0)});
  RestrictedTimedReference reference(input, metadata());
  EXPECT_NEAR(reference.sample(0.05).omega_ref, 1.0, 1e-9);
  const auto terminal = reference.sample(0.1);
  EXPECT_DOUBLE_EQ(terminal.x_ref, 2.0);
  EXPECT_DOUBLE_EQ(terminal.vx_world_ref, 0.0);
  EXPECT_DOUBLE_EQ(terminal.vy_world_ref, 0.0);
  EXPECT_DOUBLE_EQ(terminal.omega_ref, 0.0);
  EXPECT_TRUE(terminal.terminal_hold);
  EXPECT_TRUE(std::isfinite(terminal.omega_ref));

  RestrictedTimedReferenceConfig bounded_config;
  bounded_config.omega_max = 0.5;
  RestrictedTimedReference bounded(input, metadata(), bounded_config);
  EXPECT_NEAR(bounded.sample(0.05).omega_raw, 1.0, 1e-9);
  EXPECT_NEAR(bounded.sample(0.05).omega_ref, 0.5, 1e-9);
}

TEST(RestrictedTimedReference, ShortTrajectoryHoldsTerminalWithZeroVelocity)
{
  auto input = trajectory({point(0.0, 0.0, 0.0, 0.0, 0.5, 0.5),
    point(0.8, 0.4, 0.4, 0.0, 0.5, 0.5)});
  RestrictedTimedReference reference(input, metadata());
  const auto terminal = reference.sample(1.5);
  EXPECT_NEAR(terminal.x_ref, 0.4, 1e-12);
  EXPECT_NEAR(terminal.y_ref, 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(terminal.vx_world_ref, 0.0);
  EXPECT_DOUBLE_EQ(terminal.vy_world_ref, 0.0);
  EXPECT_DOUBLE_EQ(terminal.omega_ref, 0.0);
}

TEST(RestrictedTimedReference, LongTrajectoryAndRollingHorizon)
{
  auto input = trajectory({point(0.0, 0.0, 0.0, 0.0), point(1.0, 1.0, 0.0, 0.0),
    point(2.0, 2.0, 0.0, 0.0), point(3.0, 3.0, 0.0, 0.0)});
  RestrictedTimedReference reference(input, metadata());
  const auto first = reference.sampleHorizon(0.0, 40, 0.05);
  ASSERT_EQ(first.size(), 40U);
  EXPECT_NEAR(first.front().x_ref, 0.0, 1e-12);
  EXPECT_NEAR(first.back().x_ref, 1.95, 1e-12);
  const auto rolling = reference.sampleHorizon(1.5, 40, 0.05);
  ASSERT_EQ(rolling.size(), 40U);
  EXPECT_NEAR(rolling.front().x_ref, 1.5, 1e-12);
  EXPECT_TRUE(rolling.back().terminal_hold);
}

TEST(RestrictedTimedReference, ReferenceSwitchHasIndependentTimeOrigin)
{
  auto old_input = trajectory({point(0.0, 0.0, 0.0, 0.0), point(1.0, 1.0, 0.0, 0.0)});
  auto new_input = trajectory({point(0.0, 10.0, 0.0, 0.0), point(1.0, 11.0, 0.0, 0.0)});
  auto old_reference = std::make_shared<const RestrictedTimedReference>(old_input, metadata(1, 20));
  auto new_metadata = metadata(2, 30);
  new_metadata.planning_result_id = 99;
  auto new_reference = std::make_shared<const RestrictedTimedReference>(new_input, new_metadata);
  EXPECT_EQ(new_reference->metadata().reference_id, 2U);
  EXPECT_EQ(new_reference->metadata().planning_result_id, 99U);
  EXPECT_EQ(new_reference->metadata().activation_stamp_ns, 30'000'000'000LL);
  EXPECT_NE(old_reference->sample(0.0).x_ref, new_reference->sample(0.0).x_ref);
  EXPECT_EQ(new_reference->sampleHorizon(0.0, 1, 0.05).size(), 1U);
}

TEST(RestrictedTimedReference, CandidateOneMetadataSelectsOnlyItsReference)
{
  auto candidate_zero = trajectory({point(0.0, 0.0, -1.0, 0.0), point(1.0, 1.0, -1.0, 0.0)});
  auto candidate_one = trajectory({point(0.0, 0.0, 1.0, 0.0), point(1.0, 1.0, 1.0, 0.0)});
  auto selected_metadata = metadata(42, 30);
  selected_metadata.selected_candidate_index = 1;
  RestrictedTimedReference selected(candidate_one, selected_metadata);
  RestrictedTimedReference unselected(candidate_zero, metadata(41, 30));
  ASSERT_TRUE(selected.valid());
  EXPECT_EQ(selected.metadata().selected_candidate_index, 1U);
  EXPECT_NEAR(selected.sample(0.5).y_ref, 1.0, 1e-12);
  EXPECT_NEAR(unselected.sample(0.5).y_ref, -1.0, 1e-12);
}
