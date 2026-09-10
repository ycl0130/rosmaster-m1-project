#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "m1_fast_planner/kinodynamic_astar.hpp"
#include "m1_fast_planner/fast2d_planner.hpp"

namespace
{

using m1_fast_planner::KinodynamicAstar;
using m1_fast_planner::KinodynamicAstarConfig;
using m1_fast_planner::PlanarAcceleration;
using m1_fast_planner::PlanarState;

KinodynamicAstarConfig testConfig()
{
  KinodynamicAstarConfig config;
  config.search_timeout_ms = 1000;
  config.max_expansions = 100000;
  return config;
}

TEST(KinodynamicAstar, DoubleIntegratorPropagationIsExact)
{
  const auto state = KinodynamicAstar::propagate({1.0, 2.0, 3.0, -1.0}, {2.0, 4.0}, 0.5);
  EXPECT_DOUBLE_EQ(state.px, 2.75);
  EXPECT_DOUBLE_EQ(state.py, 2.0);
  EXPECT_DOUBLE_EQ(state.vx, 4.0);
  EXPECT_DOUBLE_EQ(state.vy, 1.0);
}

TEST(KinodynamicAstar, ZeroAccelerationKeepsVelocity)
{
  const auto state = KinodynamicAstar::propagate({0.0, 0.0, 0.3, -0.2}, {}, 0.4);
  EXPECT_NEAR(state.px, 0.12, 1e-12);
  EXPECT_NEAR(state.py, -0.08, 1e-12);
  EXPECT_DOUBLE_EQ(state.vx, 0.3);
  EXPECT_DOUBLE_EQ(state.vy, -0.2);
}

TEST(KinodynamicAstar, StateHashIncludesVelocity)
{
  EXPECT_NE(KinodynamicAstar::quantize(0.0, 0.1), KinodynamicAstar::quantize(0.11, 0.1));
  EXPECT_NE(KinodynamicAstar::quantize(0.0, 0.1), KinodynamicAstar::quantize(0.2, 0.1));
}

TEST(KinodynamicAstar, InvalidConfigIsRejected)
{
  auto config = testConfig();
  config.primitive_duration = 0.0;
  EXPECT_FALSE(KinodynamicAstar::validConfig(config));
  EXPECT_THROW({KinodynamicAstar planner(config);}, std::invalid_argument);
}

TEST(KinodynamicAstar, OpenSpaceFindsDynamicallyFeasibleTrajectory)
{
  KinodynamicAstar planner(testConfig());
  const auto result = planner.search({-2.5, -1.5, 0.0, 0.0}, {-2.5, 1.5, 0.0, 0.0},
      [](double x, double y) {return x >= -4.0 && x <= 4.0 && y >= -3.0 && y <= 3.0;});
  ASSERT_TRUE(result.telemetry.success);
  EXPECT_GT(result.telemetry.expanded_nodes, 0u);
  EXPECT_GT(result.trajectory.size(), 2u);
  for (const auto & point : result.trajectory) {
    EXPECT_LE(std::abs(point.state.vx), 0.45 + 1e-9);
    EXPECT_LE(std::abs(point.state.vy), 0.45 + 1e-9);
  }
  EXPECT_LE(result.telemetry.max_acceleration, std::hypot(0.6, 0.6) + 1e-9);
  EXPECT_NEAR(result.trajectory.back().state.py, 1.5, 1e-12);
}

TEST(KinodynamicAstar, FindsDetourAroundCenterObstacle)
{
  KinodynamicAstar planner(testConfig());
  const auto result = planner.search({-2.5, -1.5, 0.0, 0.0}, {2.5, 1.5, 0.0, 0.0},
      [](double x, double y) {
        return x >= -4.0 && x <= 4.0 && y >= -3.0 && y <= 3.0 &&
               std::hypot(x, y) > 0.45;
      });
  ASSERT_TRUE(result.telemetry.success);
  for (const auto & point : result.trajectory) {
    EXPECT_GT(std::hypot(point.state.px, point.state.py), 0.44);
  }
}

TEST(KinodynamicAstar, SealedGoalFails)
{
  KinodynamicAstar planner(testConfig());
  const auto result = planner.search({-2.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0},
      [](double x, double y) {return std::hypot(x, y) > 0.8 && x > -3.0 && x < 3.0 && y > -3.0 && y < 3.0;});
  EXPECT_FALSE(result.telemetry.success);
}

TEST(KinodynamicAstar, SubstepCollisionRejectsEndpointFreePrimitive)
{
  auto config = testConfig();
  config.primitive_duration = 1.0;
  config.collision_check_dt = 0.05;
  config.max_velocity_x = 1.0;
  config.max_velocity_y = 1.0;
  config.max_accel_x = 0.1;
  config.max_accel_y = 0.1;
  KinodynamicAstar planner(config);
  const auto result = planner.search({-1.0, 0.0, 1.0, 0.0}, {1.0, 0.0, 0.0, 0.0},
      [](double x, double y) {return std::hypot(x, y) > 0.12 && std::abs(y) < 0.2;});
  EXPECT_FALSE(result.telemetry.success);
}

TEST(KinodynamicAstar, BoundaryAndUnknownPoliciesAreProvidedByCollisionChecker)
{
  KinodynamicAstar planner(testConfig());
  const auto blocked = planner.search({0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0},
      [](double x, double) {return x < 0.45;});
  EXPECT_FALSE(blocked.telemetry.success);
  const auto allowed = planner.search({0.0, 0.0, 0.0, 0.0}, {0.2, 0.0, 0.0, 0.0},
      [](double, double) {return true;});
  EXPECT_TRUE(allowed.telemetry.success);
}

TEST(KinodynamicAstar, GoalVelocityToleranceAndConnectorAreRespected)
{
  auto config = testConfig();
  config.goal_position_tolerance = 0.2;
  config.goal_velocity_tolerance = 0.01;
  KinodynamicAstar planner(config);
  const auto result = planner.search({0.0, 0.0, 0.0, 0.0}, {0.8, 0.0, 0.0, 0.0},
      [](double, double) {return true;});
  ASSERT_TRUE(result.telemetry.success);
  EXPECT_LE(result.telemetry.goal_speed, config.goal_velocity_tolerance + 1e-9);
  EXPECT_NEAR(result.trajectory.back().state.px, 0.8, 1e-12);
}

TEST(KinodynamicAstar, TimeoutAndExpansionCapsTerminateSearch)
{
  auto config = testConfig();
  config.max_expansions = 1;
  KinodynamicAstar expansion_limited(config);
  const auto result = expansion_limited.search({0.0, 0.0, 0.0, 0.0}, {2.0, 0.0, 0.0, 0.0},
      [](double, double) {return true;});
  EXPECT_FALSE(result.telemetry.success);
  EXPECT_EQ(result.telemetry.expanded_nodes, 1u);
}

TEST(KinodynamicAstar, OverspeedInitialStateCanBrakeButCannotAccelerateFurther)
{
  auto config = testConfig();
  config.goal_position_tolerance = 0.01;
  config.goal_velocity_tolerance = 0.01;
  KinodynamicAstar planner(config);
  // From +0.60 m/s with a 0.45 m/s envelope, the only useful x primitive
  // must brake.  The returned trajectory must monotonically recover first.
  const auto result = planner.search({0.0, 0.0, 0.60, 0.0}, {0.6, 0.0, 0.0, 0.0},
      [](double, double) {return true;});
  ASSERT_TRUE(result.telemetry.success);
  ASSERT_GT(result.trajectory.size(), 1u);
  EXPECT_TRUE(std::any_of(result.trajectory.begin(), result.trajectory.end(), [](const auto & point) {
      return point.state.vx < 0.60 - 1e-9;
    }));
  EXPECT_TRUE(std::any_of(result.trajectory.begin(), result.trajectory.end(), [](const auto & point) {
      return std::abs(point.state.vx) <= 0.45 + 1e-9;
    }));
  for (const auto & point : result.trajectory) {
    EXPECT_LE(std::abs(point.state.vx), 0.60 + 1e-9);
  }
}

TEST(KinodynamicAstar, EqualInputProducesDeterministicTrajectory)
{
  KinodynamicAstar planner(testConfig());
  const auto collision_free = [](double x, double y) {
      return x >= -4.0 && x <= 4.0 && y >= -3.0 && y <= 3.0 && std::hypot(x, y) > 0.45;
    };
  const auto first = planner.search({-2.5, -1.5, 0.0, 0.0}, {2.5, 1.5, 0.0, 0.0}, collision_free);
  const auto second = planner.search({-2.5, -1.5, 0.0, 0.0}, {2.5, 1.5, 0.0, 0.0}, collision_free);
  ASSERT_TRUE(first.telemetry.success);
  ASSERT_TRUE(second.telemetry.success);
  ASSERT_EQ(first.trajectory.size(), second.trajectory.size());
  for (std::size_t i = 0; i < first.trajectory.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.trajectory[i].state.px, second.trajectory[i].state.px);
    EXPECT_DOUBLE_EQ(first.trajectory[i].state.py, second.trajectory[i].state.py);
  }
}

TEST(KinodynamicAstar, OptionalSoftTraversalCostChangesOnlyLocalObjective)
{
  auto config = testConfig();
  config.local_costmap_cost_weight = 8.0;
  KinodynamicAstar planner(config);
  const auto free = [](double x, double y) {
      return x >= -3.0 && x <= 3.0 && y >= -3.0 && y <= 3.0;
    };
  // A high-risk central strip is traversable, not lethal. The local objective
  // must nevertheless prefer one of the free side corridors.
  const auto result = planner.search({-1.5, 0.0, 0.0, 0.0}, {1.5, 0.0, 0.0, 0.0}, free,
      [](double x, double y) {return std::abs(x) < 0.5 && std::abs(y) < 0.20 ? 10.0 : 0.0;});
  ASSERT_TRUE(result.telemetry.success);
  EXPECT_TRUE(std::any_of(result.trajectory.begin(), result.trajectory.end(), [](const auto & point) {
      return std::abs(point.state.py) > 0.20;
    }));
}

TEST(KinodynamicAstar, ReconstructedTrajectoryIsFiniteAndTimeOrdered)
{
  KinodynamicAstar planner(testConfig());
  const auto result = planner.search({0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0},
      [](double, double) {return true;});
  ASSERT_TRUE(result.telemetry.success);
  double last_time = -1.0;
  for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
    const auto & point = result.trajectory[i];
    EXPECT_TRUE(std::isfinite(point.state.px));
    EXPECT_TRUE(std::isfinite(point.state.py));
    EXPECT_TRUE(std::isfinite(point.state.vx));
    EXPECT_TRUE(std::isfinite(point.state.vy));
    EXPECT_TRUE(std::isfinite(point.acceleration.ax));
    EXPECT_TRUE(std::isfinite(point.acceleration.ay));
    EXPECT_GE(point.time_from_start, last_time);
    if (i > 0) {
      const auto & previous = result.trajectory[i - 1];
      const double dt = point.time_from_start - previous.time_from_start;
      // Acceleration is the incoming control of the sample.  A zero-duration
      // terminal connector is geometric only and is not dynamically checked.
      if (dt > 1e-12) {
        const auto expected = KinodynamicAstar::propagate(previous.state, point.acceleration, dt);
        EXPECT_NEAR(point.state.px, expected.px, 1e-10);
        EXPECT_NEAR(point.state.py, expected.py, 1e-10);
        EXPECT_NEAR(point.state.vx, expected.vx, 1e-10);
        EXPECT_NEAR(point.state.vy, expected.vy, 1e-10);
      }
    }
    last_time = point.time_from_start;
  }
}

TEST(KinodynamicAstar, BodyVelocityRotationUsesPlanningFrame)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto forward = m1_fast_planner::Fast2DPlanner::rotateBodyVelocityToPlanningFrame(0.3, -0.2, 0.0);
  EXPECT_NEAR(forward.vx, 0.3, 1e-12);
  EXPECT_NEAR(forward.vy, -0.2, 1e-12);
  const auto left = m1_fast_planner::Fast2DPlanner::rotateBodyVelocityToPlanningFrame(0.3, 0.0, kPi * 0.5);
  EXPECT_NEAR(left.vx, 0.0, 1e-12);
  EXPECT_NEAR(left.vy, 0.3, 1e-12);
  const auto right = m1_fast_planner::Fast2DPlanner::rotateBodyVelocityToPlanningFrame(0.3, 0.0, -kPi * 0.5);
  EXPECT_NEAR(right.vx, 0.0, 1e-12);
  EXPECT_NEAR(right.vy, -0.3, 1e-12);
}

}  // namespace
