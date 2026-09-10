#include <gtest/gtest.h>
#include <cmath>

#include "m1_scope_risk/risk_field_snapshot.hpp"

namespace
{
m1_scope_msgs::msg::ScopePredictionSequence makeSequence()
{
  m1_scope_msgs::msg::ScopePredictionSequence sequence;
  sequence.header.frame_id = "odom";
  sequence.header.stamp.sec = 10;
  sequence.prediction_id = 7;
  for (int i = 1; i <= 2; ++i) {
    m1_scope_msgs::msg::ScopePredictionSlice slice;
    slice.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000);
    for (auto * grid : {&slice.probability, &slice.uncertainty}) {
      grid->header.frame_id = "odom";
      grid->header.stamp.sec = 10;
      grid->header.stamp.nanosec = slice.time_from_start.nanosec;
      grid->info.resolution = 1.0F;
      grid->info.width = 2;
      grid->info.height = 2;
      grid->info.origin.orientation.w = 1.0;
      grid->data = {20, -1, 0, 0};
    }
    slice.probability.data[0] = i == 1 ? 20 : 60;
    slice.uncertainty.data[0] = 0;
    sequence.slices.push_back(slice);
  }
  return sequence;
}
}

TEST(RiskFieldSnapshot, QueriesAndInterpolatesDeterministically)
{
  auto value = m1_scope_risk::RiskFieldSnapshot::fromMessage(makeSequence(), {});
  ASSERT_TRUE(value);
  EXPECT_EQ(value->predictionId(), 7u);
  EXPECT_NEAR(*value->query(0.2, 0.2, 0.1, 10'100'000'000LL), 0.2F, 1e-6F);
  EXPECT_NEAR(*value->query(0.2, 0.2, 0.15, 10'100'000'000LL), 0.4F, 1e-6F);
  EXPECT_NEAR(*value->query(0.2, 0.2, 0.2, 10'100'000'000LL), 0.6F, 1e-6F);
  EXPECT_FALSE(value->query(1.2, 0.2, 0.1, 10'100'000'000LL));
  EXPECT_FALSE(value->query(3.0, 0.2, 0.1, 10'100'000'000LL));
  EXPECT_FALSE(value->query(0.2, 0.2, 0.0, 10'100'000'000LL));
  EXPECT_FALSE(value->query(0.2, 0.2, 0.3, 10'100'000'000LL));
}

TEST(RiskFieldSnapshot, HandlesUncertaintyClampStalenessAndReset)
{
  auto sequence = makeSequence();
  sequence.slices[0].probability.data[0] = 90;
  sequence.slices[0].uncertainty.data[0] = 100;
  m1_scope_risk::RiskFieldConfig config;
  config.uncertainty_gain = 1.0;
  config.uncertainty_encoding_scale = 0.5;
  auto value = m1_scope_risk::RiskFieldSnapshot::fromMessage(sequence, config);
  ASSERT_TRUE(value);
  EXPECT_FLOAT_EQ(*value->query(0.2, 0.2, 0.1, 10'100'000'000LL), 1.0F);
  EXPECT_FALSE(value->query(0.2, 0.2, 0.1, 10'600'000'001LL));
  EXPECT_FALSE(value->query(0.2, 0.2, 0.1, 9'999'999'999LL));
}

TEST(RiskFieldSnapshot, RejectsMalformedContracts)
{
  std::string reason;
  auto bad_time = makeSequence(); bad_time.slices[1].time_from_start = bad_time.slices[0].time_from_start;
  EXPECT_FALSE(m1_scope_risk::RiskFieldSnapshot::fromMessage(bad_time, {}, &reason));
  auto bad_geometry = makeSequence(); bad_geometry.slices[0].uncertainty.info.width = 3;
  EXPECT_FALSE(m1_scope_risk::RiskFieldSnapshot::fromMessage(bad_geometry, {}, &reason));
  auto bad_stamp = makeSequence(); ++bad_stamp.slices[0].uncertainty.header.stamp.nanosec;
  EXPECT_FALSE(m1_scope_risk::RiskFieldSnapshot::fromMessage(bad_stamp, {}, &reason));
  auto bad_encoding = makeSequence(); bad_encoding.slices[0].probability.data[0] = 101;
  EXPECT_FALSE(m1_scope_risk::RiskFieldSnapshot::fromMessage(bad_encoding, {}, &reason));
}
