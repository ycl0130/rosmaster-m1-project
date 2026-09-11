#include <gtest/gtest.h>

#include "m1_local_fast2d/timed_reference_retimer.hpp"

namespace
{
m1_local_fast2d::msg::TimedTrajectoryPoint point(double t, double x, double y, double yaw)
{
  m1_local_fast2d::msg::TimedTrajectoryPoint p; p.time_from_start = t;
  p.x = x; p.y = y; p.yaw = yaw; return p;
}
}  // namespace

TEST(TimedReferenceRetimer, PreservesGeometryAndRespectsDynamics)
{
  m1_local_fast2d::msg::TimedTrajectory input;
  input.points = {point(0.0, 0.0, 0.0, 0.0), point(0.1, 0.04, 0.0, 0.18),
    point(0.2, 0.08, 0.04, 0.36), point(0.3, 0.12, 0.04, 0.54)};
  m1_local_fast2d::TimedReferenceRetimingLimits limits;
  m1_local_fast2d::TimedReferenceRetimingDiagnostics diagnostics;
  const auto output = m1_local_fast2d::retimeTimedReference(input, limits, &diagnostics);
  ASSERT_TRUE(diagnostics.valid); ASSERT_EQ(output.points.size(), input.points.size());
  EXPECT_GT(diagnostics.time_scale, 1.0);
  EXPECT_LE(diagnostics.after_max_vx, limits.vx_max + 1e-9);
  EXPECT_LE(diagnostics.after_max_vy, limits.vy_max + 1e-9);
  EXPECT_LE(diagnostics.after_max_wz, limits.wz_max + 1e-9);
  EXPECT_LE(diagnostics.after_max_ax, limits.ax_max + 1e-9);
  EXPECT_LE(diagnostics.after_max_ay, limits.ay_max + 1e-9);
  EXPECT_LE(diagnostics.after_max_awz, limits.awz_max + 1e-9);
  EXPECT_NEAR(diagnostics.geometry_error, 0.0, 1e-12);
  for (std::size_t i = 1; i < output.points.size(); ++i) {
    const auto dt = output.points[i].time_from_start - output.points[i - 1].time_from_start;
    EXPECT_GT(dt, 0.0);
    EXPECT_NEAR((output.points[i].x - output.points[i - 1].x) / dt, output.points[i - 1].vx, 1e-12);
    EXPECT_NEAR((output.points[i].y - output.points[i - 1].y) / dt, output.points[i - 1].vy, 1e-12);
    EXPECT_DOUBLE_EQ(output.points[i].x, input.points[i].x);
    EXPECT_DOUBLE_EQ(output.points[i].y, input.points[i].y);
    EXPECT_DOUBLE_EQ(output.points[i].yaw, input.points[i].yaw);
  }
}

TEST(TimedReferenceRetimer, IgnoresDuplicateTerminalConnectorAndKeepsGoalHold)
{
  m1_local_fast2d::msg::TimedTrajectory input;
  input.points = {point(0.0, 0.0, 0.0, 0.0), point(0.25, 0.10, 0.0, 0.0),
    point(0.25, 0.10, 0.0, 1.4)};
  m1_local_fast2d::TimedReferenceRetimingDiagnostics diagnostics;
  const auto output = m1_local_fast2d::retimeTimedReference(input, {}, &diagnostics);
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_EQ(diagnostics.ignored_duplicate_segments, 1U);
  EXPECT_LT(diagnostics.time_scale, 10.0);
  EXPECT_NEAR(output.points[2].time_from_start, output.points[1].time_from_start, 1e-12);
  EXPECT_DOUBLE_EQ(output.points[1].vx, 0.0);
  EXPECT_DOUBLE_EQ(output.points[2].vx, 0.0);
  EXPECT_DOUBLE_EQ(output.points[2].x, input.points[2].x);
  EXPECT_DOUBLE_EQ(output.points[2].y, input.points[2].y);
  EXPECT_DOUBLE_EQ(output.points[2].yaw, input.points[2].yaw);
}

TEST(TimedReferenceRetimer, ReprocessesZeroDurationMovingSegmentWithoutOneMillisecondDerivative)
{
  m1_local_fast2d::msg::TimedTrajectory input;
  input.points = {point(0.0, 0.0, 0.0, 0.0), point(0.0, 0.10, 0.0, 0.0),
    point(0.25, 0.20, 0.0, 0.0)};
  m1_local_fast2d::TimedReferenceRetimingDiagnostics diagnostics;
  const auto output = m1_local_fast2d::retimeTimedReference(input, {}, &diagnostics);
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_EQ(diagnostics.reprocessed_zero_duration_segments, 1U);
  EXPECT_LT(diagnostics.time_scale, 10.0);
  EXPECT_GT(output.points[1].time_from_start, output.points[0].time_from_start);
  EXPECT_NEAR((output.points[1].x - output.points[0].x) /
    (output.points[1].time_from_start - output.points[0].time_from_start), output.points[0].vx, 1e-12);
}

TEST(TimedReferenceRetimer, SplitsTerminalHeadingWithoutSlowingApproachGeometry)
{
  m1_local_fast2d::msg::TimedTrajectory input;
  // The final yaw is a requested terminal heading, not the tangent heading.
  // A direct retime would uniformly slow the 20 cm final approach to satisfy
  // the 2 rad yaw change. It should instead arrive at x/y, then turn in place.
  input.points = {point(0.0, 0.0, 0.0, 0.0), point(0.25, 0.20, 0.0, 0.0),
    point(0.50, 0.40, 0.0, 2.0)};
  m1_local_fast2d::TimedReferenceRetimingDiagnostics diagnostics;
  const auto output = m1_local_fast2d::retimeTimedReference(input, {}, &diagnostics);
  ASSERT_TRUE(diagnostics.valid);
  ASSERT_TRUE(diagnostics.terminal_heading_split);
  ASSERT_EQ(output.points.size(), input.points.size() + 1);
  EXPECT_NEAR(output.points[2].x, input.points.back().x, 1e-12);
  EXPECT_NEAR(output.points[2].y, input.points.back().y, 1e-12);
  EXPECT_NEAR(output.points[2].yaw, input.points[1].yaw, 1e-12);
  EXPECT_NEAR(output.points.back().yaw, input.points.back().yaw, 1e-12);
  EXPECT_GT(diagnostics.terminal_approach_speed, 0.05);
  EXPECT_GT(diagnostics.terminal_rotation_duration, 0.0);
  EXPECT_LE(diagnostics.after_max_wz, 0.8 + 1e-9);
  EXPECT_NEAR(diagnostics.geometry_error, 0.0, 1e-12);
}
