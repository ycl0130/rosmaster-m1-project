#include <cmath>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

#include "m1_fast_planner/straight_line.hpp"

namespace
{

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  return result;
}

TEST(StraightLine, SamplesAtRequestedResolution)
{
  const auto samples = m1_fast_planner::sampleStraightLine(point(0.0, 0.0), point(0.12, 0.0), 0.05);
  ASSERT_EQ(samples.size(), 4u);
  EXPECT_DOUBLE_EQ(samples.front().x, 0.0);
  EXPECT_DOUBLE_EQ(samples.back().x, 0.12);
  for (size_t index = 1; index < samples.size(); ++index) {
    EXPECT_LE(samples[index].x - samples[index - 1].x, 0.05 + 1e-12);
  }
}

TEST(StraightLine, CoincidentEndpointsRemainFinite)
{
  const auto samples = m1_fast_planner::sampleStraightLine(point(1.0, -2.0), point(1.0, -2.0), 0.05);
  ASSERT_EQ(samples.size(), 2u);
  for (const auto & sample : samples) {
    EXPECT_TRUE(std::isfinite(sample.x));
    EXPECT_TRUE(std::isfinite(sample.y));
  }
  EXPECT_DOUBLE_EQ(m1_fast_planner::straightLineYaw(samples.front(), samples.back()), 0.0);
  EXPECT_TRUE(std::isfinite(m1_fast_planner::straightLineYaw(samples.front(), samples.back())));
}

TEST(StraightLine, YawFollowsTheLineDirection)
{
  EXPECT_NEAR(
    m1_fast_planner::straightLineYaw(point(0.0, 0.0), point(0.0, 1.0)),
    std::acos(-1.0) * 0.5, 1e-12);
}

TEST(StraightLine, RejectsInvalidResolution)
{
  EXPECT_THROW(
    m1_fast_planner::sampleStraightLine(point(0.0, 0.0), point(1.0, 0.0), 0.0),
    std::invalid_argument);
}

TEST(StraightLine, CostRulesRespectUnknownParameter)
{
  EXPECT_FALSE(m1_fast_planner::isTraversableCost(nav2_costmap_2d::LETHAL_OBSTACLE, true));
  EXPECT_FALSE(m1_fast_planner::isTraversableCost(
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true));
  EXPECT_FALSE(m1_fast_planner::isTraversableCost(nav2_costmap_2d::NO_INFORMATION, false));
  EXPECT_TRUE(m1_fast_planner::isTraversableCost(nav2_costmap_2d::NO_INFORMATION, true));
}

TEST(StraightLine, CollisionCheckingRejectsLethalAndUnknownCells)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto samples = m1_fast_planner::sampleStraightLine(point(0.5, 0.5), point(3.5, 0.5), 0.25);
  EXPECT_TRUE(m1_fast_planner::lineIsCollisionFree(costmap, samples, false));
  costmap.setCost(2, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(m1_fast_planner::lineIsCollisionFree(costmap, samples, false));
  costmap.setCost(2, 0, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(m1_fast_planner::lineIsCollisionFree(costmap, samples, false));
  EXPECT_TRUE(m1_fast_planner::lineIsCollisionFree(costmap, samples, true));
}

TEST(StraightLine, FramesMustExactlyMatchGlobalFrame)
{
  EXPECT_TRUE(m1_fast_planner::framesMatch("map", "map", "map"));
  EXPECT_FALSE(m1_fast_planner::framesMatch("odom", "map", "map"));
  EXPECT_FALSE(m1_fast_planner::framesMatch("map", "odom", "map"));
}

}  // namespace
