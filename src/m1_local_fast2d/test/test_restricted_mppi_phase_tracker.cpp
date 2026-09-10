#include "m1_local_fast2d/restricted_mppi_phase_tracker.hpp"

#include <memory>

#include <gtest/gtest.h>

namespace
{
using m1_local_fast2d::RestrictedMPPIPhaseConfig;
using m1_local_fast2d::RestrictedMPPIPhaseTracker;
using m1_local_fast2d::RestrictedTimedReference;
using m1_local_fast2d::RestrictedTimedReferenceMetadata;
using m1_local_fast2d::msg::TimedTrajectory;
using m1_local_fast2d::msg::TimedTrajectoryPoint;

TimedTrajectoryPoint point(double time, double x, double y)
{
  TimedTrajectoryPoint result;
  result.time_from_start = time;
  result.x = x;
  result.y = y;
  result.yaw = 0.0;
  result.vx = 1.0;
  return result;
}

RestrictedTimedReference makeReference(std::initializer_list<TimedTrajectoryPoint> points,
  uint64_t id = 1)
{
  TimedTrajectory trajectory;
  trajectory.header.frame_id = "odom";
  trajectory.points.assign(points.begin(), points.end());
  RestrictedTimedReferenceMetadata metadata;
  metadata.reference_id = id;
  metadata.frame_id = "odom";
  return RestrictedTimedReference(trajectory, metadata);
}

RestrictedMPPIPhaseConfig config()
{
  RestrictedMPPIPhaseConfig result;
  result.phase_search_back_s = 0.20;
  result.phase_search_forward_s = 0.35;
  result.max_phase_lead_s = 0.15;
  result.projection_resolution_s = 0.005;
  return result;
}
}  // namespace

TEST(RestrictedMPPIPhaseTracker, StoppedRobotCannotRunAwayWithWallTime)
{
  auto reference = makeReference({point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0)});
  RestrictedMPPIPhaseTracker tracker;
  const auto parameters = config();
  for (int index = 0; index <= 4; ++index) {
    const double t = 0.05 * index;
    tracker.update(reference, t, 0.0, 1, 0.05, parameters);
  }
  m1_local_fast2d::RestrictedMPPIPhaseResult held;
  for (int index = 0; index < 26; ++index) {
    held = tracker.update(reference, 0.20, 0.0, 1, 0.05, parameters);
  }
  EXPECT_LE(held.tracking_phase_s, 0.35 + 1e-9);
  EXPECT_NEAR(held.projected_phase_s, 0.20, 0.01);
  EXPECT_LE(reference.sample(held.tracking_phase_s).x_ref, 0.35 + 1e-9);
}

TEST(RestrictedMPPIPhaseTracker, ResumeAfterStopAdvancesWithoutReset)
{
  auto reference = makeReference({point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0)});
  RestrictedMPPIPhaseTracker tracker;
  const auto parameters = config();
  for (int index = 0; index <= 4; ++index) {
    tracker.update(reference, 0.05 * index, 0.0, 1, 0.05, parameters);
  }
  for (int index = 0; index < 10; ++index) {
    tracker.update(reference, 0.20, 0.0, 1, 0.05, parameters);
  }
  const auto held = tracker.update(reference, 0.20, 0.0, 1, 0.05, parameters);
  const auto resumed = tracker.update(reference, 0.40, 0.0, 1, 0.05, parameters);
  EXPECT_FALSE(resumed.reference_reset);
  EXPECT_GE(resumed.tracking_phase_s, held.tracking_phase_s);
  EXPECT_LE(resumed.tracking_phase_s, resumed.projected_phase_s + parameters.max_phase_lead_s + 1e-9);
}

TEST(RestrictedMPPIPhaseTracker, LocalProjectionCannotJumpAcrossSelfCrossing)
{
  auto reference = makeReference({point(0.0, 0.0, 0.0), point(1.0, 1.0, 0.0),
    point(2.0, 0.0, 0.0), point(3.0, 1.0, 0.0)});
  RestrictedMPPIPhaseTracker tracker;
  const auto parameters = config();
  auto result = tracker.update(reference, 0.0, 0.0, 1, 0.05, parameters);
  for (int index = 0; index < 3; ++index) {
    result = tracker.update(reference, 0.0, 0.0, 1, 0.05, parameters);
  }
  EXPECT_LT(result.projected_phase_s, 0.36);
  EXPECT_LT(result.tracking_phase_s, 0.36);
}

TEST(RestrictedMPPIPhaseTracker, NormalProgressTracksControllerTime)
{
  auto reference = makeReference({point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0)});
  RestrictedMPPIPhaseTracker tracker;
  const auto parameters = config();
  m1_local_fast2d::RestrictedMPPIPhaseResult result;
  for (int index = 0; index <= 20; ++index) {
    const double t = 0.05 * index;
    result = tracker.update(reference, t, 0.0, 1, 0.05, parameters);
  }
  EXPECT_NEAR(result.tracking_phase_s, 1.0, 0.06);
  EXPECT_LE(result.tracking_phase_s, result.projected_phase_s + parameters.max_phase_lead_s + 1e-9);
}

TEST(RestrictedMPPIPhaseTracker, InitialAlignmentIsLocalAndDoesNotUseWallTime)
{
  auto reference = makeReference({point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0)});
  RestrictedMPPIPhaseTracker tracker;
  const auto parameters = config();
  const auto result = tracker.update(reference, 0.06, 0.0, 1, 0.05, parameters);
  EXPECT_TRUE(result.reference_reset);
  EXPECT_NEAR(result.projected_phase_s, 0.06, 0.01);
  EXPECT_LE(result.tracking_phase_s, 0.11 + 1e-9);
}
