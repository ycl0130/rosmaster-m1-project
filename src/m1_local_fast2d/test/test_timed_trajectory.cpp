#include <algorithm>
#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "tf2/utils.h"

#include "m1_local_fast2d/local_fast2d_core.hpp"

namespace
{

m1_local_fast2d::LocalFast2DRequest openSpaceRequest()
{
  m1_local_fast2d::LocalFast2DRequest request;
  request.costmap.header.frame_id = "map";
  request.costmap.metadata.resolution = 0.1;
  request.costmap.metadata.size_x = 100;
  request.costmap.metadata.size_y = 100;
  request.costmap.metadata.origin.position.x = -5.0;
  request.costmap.metadata.origin.position.y = -5.0;
  request.costmap.metadata.origin.orientation.w = 1.0;
  request.costmap.data.assign(10000, 0);
  request.reference_path.header = request.costmap.header;
  for (double x = 0.0; x <= 1.2; x += 0.1) {
    geometry_msgs::msg::PoseStamped point;
    point.header = request.reference_path.header;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    request.reference_path.poses.push_back(point);
  }
  request.start = {0.0, 0.0, 0.0, 0.0};
  request.start_yaw = 0.0;
  request.config.max_planning_time_ms = 1000;
  request.config.max_expansions = 100000;
  return request;
}

TEST(LocalFast2DTimedTrajectory, PreservesRawSearchTrajectoryAndLegacyGeometricPath)
{
  const auto result = m1_local_fast2d::planLocalFast2D(openSpaceRequest());
  ASSERT_TRUE(result.success) << result.failure_reason;
  ASSERT_GT(result.timed_trajectory.size(), 1U);
  ASSERT_EQ(result.timed_trajectory.size(), result.local_path.poses.size());

  double previous_time = -1.0;
  double max_position_error = 0.0;
  for (std::size_t i = 0; i < result.timed_trajectory.size(); ++i) {
    const auto & timed = result.timed_trajectory[i];
    const auto & pose = result.local_path.poses[i].pose;
    EXPECT_TRUE(std::isfinite(timed.x));
    EXPECT_TRUE(std::isfinite(timed.y));
    EXPECT_TRUE(std::isfinite(timed.yaw));
    EXPECT_TRUE(std::isfinite(timed.time_from_start));
    EXPECT_TRUE(std::isfinite(timed.vx));
    EXPECT_TRUE(std::isfinite(timed.vy));
    EXPECT_TRUE(std::isfinite(timed.ax));
    EXPECT_TRUE(std::isfinite(timed.ay));
    EXPECT_GE(timed.time_from_start, previous_time);
    EXPECT_DOUBLE_EQ(timed.x, pose.position.x);
    EXPECT_DOUBLE_EQ(timed.y, pose.position.y);
    max_position_error = std::max(max_position_error,
      std::hypot(timed.x - pose.position.x, timed.y - pose.position.y));
    EXPECT_NEAR(timed.yaw, tf2::getYaw(pose.orientation), 1e-12);
    // This repeats the pre-contract Path yaw rule exactly, proving that the
    // parallel contract did not alter geometric Path construction.
    const double expected_yaw = i + 1 < result.timed_trajectory.size() ?
      std::atan2(result.timed_trajectory[i + 1].y - timed.y,
      result.timed_trajectory[i + 1].x - timed.x) : 0.0;
    EXPECT_NEAR(timed.yaw, expected_yaw, 1e-12);
    previous_time = timed.time_from_start;
  }
  ::testing::Test::RecordProperty("point_count", std::to_string(result.timed_trajectory.size()));
  ::testing::Test::RecordProperty("first_time_from_start", std::to_string(result.timed_trajectory.front().time_from_start));
  ::testing::Test::RecordProperty("last_time_from_start", std::to_string(result.timed_trajectory.back().time_from_start));
  ::testing::Test::RecordProperty("max_path_position_error", std::to_string(max_position_error));
}

TEST(LocalFast2DTimedTrajectory, IncomingControlsMatchDoubleIntegratorPropagation)
{
  const auto result = m1_local_fast2d::planLocalFast2D(openSpaceRequest());
  ASSERT_TRUE(result.success) << result.failure_reason;
  const auto & points = result.timed_trajectory;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & previous = points[i - 1];
    const auto & current = points[i];
    const double dt = current.time_from_start - previous.time_from_start;
    // The optional final goal connector has zero duration; it is not a
    // propagated dynamic sample and is intentionally excluded.
    if (dt <= 1e-12) {continue;}
    EXPECT_GT(dt, 0.0);
    EXPECT_NEAR(current.vx, previous.vx + current.ax * dt, 1e-10);
    EXPECT_NEAR(current.vy, previous.vy + current.ay * dt, 1e-10);
    EXPECT_NEAR(current.x, previous.x + previous.vx * dt + 0.5 * current.ax * dt * dt, 1e-10);
    EXPECT_NEAR(current.y, previous.y + previous.vy * dt + 0.5 * current.ay * dt * dt, 1e-10);
  }
}

TEST(LocalFast2DCandidates, KOnePreservesPrimaryAndCandidateCountIsBounded)
{
  auto single_request = openSpaceRequest(); single_request.config.num_candidates = 1;
  const auto single = m1_local_fast2d::planLocalFast2D(single_request);
  auto multi_request = openSpaceRequest(); multi_request.config.num_candidates = 3;
  const auto multi = m1_local_fast2d::planLocalFast2D(multi_request);
  ASSERT_TRUE(single.success); ASSERT_TRUE(multi.success);
  ASSERT_EQ(single.candidates.size(), 1U);
  ASSERT_GE(multi.candidates.size(), 1U); ASSERT_LE(multi.candidates.size(), 3U);
  ASSERT_EQ(single.local_path.poses.size(), multi.local_path.poses.size());
  for (std::size_t i = 0; i < single.local_path.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(single.local_path.poses[i].pose.position.x, multi.local_path.poses[i].pose.position.x);
    EXPECT_DOUBLE_EQ(single.local_path.poses[i].pose.position.y, multi.local_path.poses[i].pose.position.y);
  }
  auto clamped_request = openSpaceRequest(); clamped_request.config.num_candidates = 99;
  const auto clamped = m1_local_fast2d::planLocalFast2D(clamped_request);
  ASSERT_TRUE(clamped.success); EXPECT_EQ(clamped.requested_candidates, 5);
  EXPECT_LE(clamped.candidates.size(), 5U);
  for (std::size_t i = 0; i < multi.candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < multi.candidates.size(); ++j) {
      EXPECT_GE(multi.candidates[j].minimum_diversity,
        multi_request.config.candidate_diversity_threshold);
    }
  }
}

TEST(LocalFast2DCandidates, CentralObstacleProducesTwoDistinctSides)
{
  auto request = openSpaceRequest();
  request.costmap.metadata.size_x = request.costmap.metadata.size_y = 60;
  request.costmap.metadata.origin.position.x = request.costmap.metadata.origin.position.y = -3.0;
  request.costmap.data.assign(3600, 0);
  request.reference_path.poses.clear(); request.config.lookahead_distance = 5.0;
  request.config.num_candidates = 3; request.config.max_planning_time_ms = 5000;
  request.start = {-2.0, 0.0, 0.0, 0.0};
  for (int ix = -5; ix <= 5; ++ix) for (int iy = -5; iy <= 5; ++iy) {
    request.costmap.data[static_cast<std::size_t>(iy + 30) * 60 + static_cast<std::size_t>(ix + 30)] = 254;
  }
  for (double x = -2.0; x <= 2.0; x += 0.1) { geometry_msgs::msg::PoseStamped pose; pose.header = request.reference_path.header; pose.pose.position.x=x; pose.pose.orientation.w=1.0; request.reference_path.poses.push_back(pose); }
  const auto result = m1_local_fast2d::planLocalFast2D(request);
  ASSERT_TRUE(result.success); ASSERT_GE(result.candidates.size(), 2U);
  const auto side = [](const auto & candidate) { double sum=0; int count=0; for(const auto & p:candidate.timed_trajectory) if(std::abs(p.x)<0.8){sum+=p.y;++count;} return sum/count; };
  const double first=side(result.candidates[0]), second=side(result.candidates[1]);
  EXPECT_LT(first * second, 0.0);
  EXPECT_GE(result.candidates[1].minimum_diversity, request.config.candidate_diversity_threshold);
}

TEST(LocalFast2DCandidates, VerticalAndObstacleSizesUseLocalLateralGates)
{
  for (const int half_size : {3, 9}) {
    auto request = openSpaceRequest(); request.costmap.metadata.size_x=request.costmap.metadata.size_y=80;
    request.costmap.metadata.origin.position.x=request.costmap.metadata.origin.position.y=-4.0;
    request.costmap.data.assign(6400, 0); request.reference_path.poses.clear();
    request.config.lookahead_distance=7.0; request.config.num_candidates=3; request.config.max_planning_time_ms=2000;
    request.start={0.0,-3.0,0.0,0.0};
    for(int ix=-half_size;ix<=half_size;++ix) for(int iy=-half_size;iy<=half_size;++iy)
      request.costmap.data[static_cast<std::size_t>(iy+40)*80+static_cast<std::size_t>(ix+40)]=254;
    for(double y=-3;y<=3;y+=.1){geometry_msgs::msg::PoseStamped p;p.header=request.reference_path.header;p.pose.position.y=y;p.pose.orientation.w=1;request.reference_path.poses.push_back(p);}
    const auto result=m1_local_fast2d::planLocalFast2D(request);
    ASSERT_TRUE(result.success); ASSERT_GE(result.candidates.size(),2U);
    const auto lateral=[](const auto & c){double s=0;int n=0;for(const auto&p:c.timed_trajectory)if(std::abs(p.y)<1.5){s+=-p.x;++n;}return s/n;};
    EXPECT_LT(lateral(result.candidates[0])*lateral(result.candidates[1]),0.0);
  }
}

TEST(LocalFast2DCandidates, NearEdgeDoesNotInventUnavailableBranch)
{
  auto request=openSpaceRequest(); request.costmap.metadata.size_x=60;request.costmap.metadata.size_y=30;
  request.costmap.metadata.origin.position.x=-3;request.costmap.metadata.origin.position.y=-1.5;request.costmap.data.assign(1800,0);
  request.reference_path.poses.clear();request.config.lookahead_distance=5;request.config.num_candidates=3;request.start={-2,0,0,0};
  for(int ix=-5;ix<=5;++ix)for(int iy=-5;iy<=5;++iy)request.costmap.data[static_cast<std::size_t>(iy+15)*60+static_cast<std::size_t>(ix+30)]=254;
  for(double x=-2;x<=2;x+=.1){geometry_msgs::msg::PoseStamped p;p.header=request.reference_path.header;p.pose.position.x=x;p.pose.orientation.w=1;request.reference_path.poses.push_back(p);}
  const auto result=m1_local_fast2d::planLocalFast2D(request); ASSERT_TRUE(result.success); EXPECT_LE(result.candidates.size(),2U);
}

TEST(LocalFast2DCandidates, DiagonalFortyFiveDegreeUsesLocalLateralSides)
{
  auto request=openSpaceRequest();request.costmap.metadata.size_x=request.costmap.metadata.size_y=80;request.costmap.metadata.origin.position.x=request.costmap.metadata.origin.position.y=-4;request.costmap.data.assign(6400,0);request.reference_path.poses.clear();request.config.lookahead_distance=7;request.config.num_candidates=3;request.start={-2,-2,0,0};
  for(int ix=-6;ix<=6;++ix)for(int iy=-6;iy<=6;++iy)request.costmap.data[static_cast<std::size_t>(iy+40)*80+static_cast<std::size_t>(ix+40)]=254;
  for(double d=-2;d<=2;d+=.1){geometry_msgs::msg::PoseStamped p;p.header=request.reference_path.header;p.pose.position.x=d;p.pose.position.y=d;p.pose.orientation.w=1;request.reference_path.poses.push_back(p);}
  const auto result=m1_local_fast2d::planLocalFast2D(request);ASSERT_TRUE(result.success);ASSERT_GE(result.candidates.size(),2U);
  const auto lateral=[](const auto&c){double s=0;int n=0;for(const auto&p:c.timed_trajectory)if(std::hypot(p.x,p.y)<1.5){s+=(p.y-p.x)/std::sqrt(2.0);++n;}return s/n;}; bool opposite=false;for(std::size_t i=0;i<result.candidates.size();++i)for(std::size_t j=i+1;j<result.candidates.size();++j)opposite|=lateral(result.candidates[i])*lateral(result.candidates[j])<0;EXPECT_TRUE(opposite);
}

TEST(LocalFast2DCandidates, SingleCorridorEmitsOnlyPrimary)
{
  auto request=openSpaceRequest(); request.costmap.metadata.size_x=request.costmap.metadata.size_y=60;
  request.costmap.metadata.origin.position.x=request.costmap.metadata.origin.position.y=-3; request.costmap.data.assign(3600,254);
  request.reference_path.poses.clear();request.config.lookahead_distance=5;request.config.num_candidates=3;request.config.max_planning_time_ms=2000;request.start={-2,0,0,0};
  // Carve exactly one 0.7 m-wide U-shaped upper corridor; all lower cells
  // remain hard obstacles. This is independent of candidate selection.
  const auto carve=[&](double x0,double x1,double y0,double y1){for(int x=0;x<60;++x)for(int y=0;y<60;++y){double wx=-3+(x+.5)*.1,wy=-3+(y+.5)*.1;if(wx>=x0&&wx<=x1&&wy>=y0&&wy<=y1)request.costmap.data[static_cast<std::size_t>(y)*60+x]=0;}};
  carve(-2.2,-.7,-.35,.35); carve(-1.0,-.3,-.35,1.35); carve(-.7,.7,.95,1.35); carve(.3,1.0,-.35,1.35); carve(.7,2.2,-.35,.35);
  for(double x=-2;x<=2;x+=.1){geometry_msgs::msg::PoseStamped p;p.header=request.reference_path.header;p.pose.position.x=x;p.pose.orientation.w=1;request.reference_path.poses.push_back(p);}
  const auto result=m1_local_fast2d::planLocalFast2D(request); ASSERT_TRUE(result.success); EXPECT_EQ(result.candidates.size(),1U);
}

}  // namespace
