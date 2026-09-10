#include "m1_scope_risk/risk_field_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace m1_scope_risk
{
namespace
{
constexpr int64_t kNsPerSecond = 1000000000LL;

int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNsPerSecond + stamp.nanosec;
}

int64_t durationNs(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<int64_t>(duration.sec) * kNsPerSecond + duration.nanosec;
}

bool samePose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return a.position.x == b.position.x && a.position.y == b.position.y &&
         a.position.z == b.position.z && a.orientation.x == b.orientation.x &&
         a.orientation.y == b.orientation.y && a.orientation.z == b.orientation.z &&
         a.orientation.w == b.orientation.w;
}

bool validGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (grid.header.frame_id.empty() || grid.header.stamp.sec < 0 ||
    grid.header.stamp.nanosec >= static_cast<uint32_t>(kNsPerSecond) ||
    grid.info.width == 0 || grid.info.height == 0 || !std::isfinite(grid.info.resolution) ||
    grid.info.resolution <= 0.0 ||
    grid.data.size() != static_cast<size_t>(grid.info.width) * grid.info.height)
  {
    return false;
  }
  return std::all_of(grid.data.begin(), grid.data.end(), [](int8_t value) {
    return value >= -1 && value <= 100;
  });
}

bool matchingGrid(const nav_msgs::msg::OccupancyGrid & probability,
  const nav_msgs::msg::OccupancyGrid & uncertainty)
{
  return probability.header.stamp == uncertainty.header.stamp &&
         probability.header.frame_id == uncertainty.header.frame_id &&
         probability.info.width == uncertainty.info.width &&
         probability.info.height == uncertainty.info.height &&
         probability.info.resolution == uncertainty.info.resolution &&
         samePose(probability.info.origin, uncertainty.info.origin);
}

std::optional<float> riskAt(const nav_msgs::msg::OccupancyGrid & probability,
  const nav_msgs::msg::OccupancyGrid & uncertainty, double x, double y,
  const RiskFieldConfig & config)
{
  const auto & origin = probability.info.origin;
  const double yaw = std::atan2(2.0 * (origin.orientation.w * origin.orientation.z +
    origin.orientation.x * origin.orientation.y), 1.0 - 2.0 *
    (origin.orientation.y * origin.orientation.y + origin.orientation.z * origin.orientation.z));
  const double dx = x - origin.position.x;
  const double dy = y - origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const int cell_x = static_cast<int>(std::floor(local_x / probability.info.resolution));
  const int cell_y = static_cast<int>(std::floor(local_y / probability.info.resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(probability.info.width) ||
    cell_y >= static_cast<int>(probability.info.height))
  {
    return std::nullopt;
  }
  const auto index = static_cast<size_t>(cell_y) * probability.info.width + cell_x;
  if (probability.data[index] < 0 || uncertainty.data[index] < 0) {
    return std::nullopt;
  }
  const double p = static_cast<double>(probability.data[index]) / 100.0;
  const double sigma = config.uncertainty_encoding_scale *
    static_cast<double>(uncertainty.data[index]) / 100.0;
  return static_cast<float>(std::clamp(p + config.uncertainty_gain * sigma, 0.0, 1.0));
}
}  // namespace

std::optional<RiskFieldSnapshot> RiskFieldSnapshot::fromMessage(
  const m1_scope_msgs::msg::ScopePredictionSequence & sequence,
  const RiskFieldConfig & config, std::string * reason)
{
  auto reject = [reason](const std::string & value) -> std::optional<RiskFieldSnapshot> {
      if (reason) {*reason = value;}
      return std::nullopt;
    };
  if (!std::isfinite(config.uncertainty_gain) || config.uncertainty_gain < 0.0 ||
    !std::isfinite(config.uncertainty_encoding_scale) || config.uncertainty_encoding_scale < 0.0 ||
    config.stale_timeout_ns <= 0 || sequence.header.frame_id.empty() ||
    sequence.header.stamp.sec < 0 || sequence.header.stamp.nanosec >= static_cast<uint32_t>(kNsPerSecond) ||
    sequence.slices.size() < 2)
  {
    return reject("invalid sequence header, configuration, or slice count");
  }
  const int64_t anchor = stampNs(sequence.header.stamp);
  int64_t previous = 0;
  for (const auto & slice : sequence.slices) {
    const int64_t offset = durationNs(slice.time_from_start);
    if (offset <= previous || offset <= 0 || !validGrid(slice.probability) ||
      !validGrid(slice.uncertainty) || !matchingGrid(slice.probability, slice.uncertainty) ||
      slice.probability.header.frame_id != sequence.header.frame_id ||
      stampNs(slice.probability.header.stamp) != anchor + offset)
    {
      return reject("invalid slice timing, grid pair, frame, or encoding");
    }
    previous = offset;
  }
  RiskFieldSnapshot snapshot;
  snapshot.sequence_ = sequence;
  snapshot.config_ = config;
  snapshot.anchor_stamp_ns_ = anchor;
  snapshot.latest_slice_ns_ = anchor + previous;
  snapshot.prediction_id_ = sequence.prediction_id;
  if (reason) {reason->clear();}
  return snapshot;
}

bool RiskFieldSnapshot::isFresh(int64_t now_ns) const
{
  return now_ns >= anchor_stamp_ns_ && now_ns - anchor_stamp_ns_ <= config_.stale_timeout_ns;
}

std::optional<float> RiskFieldSnapshot::query(double x, double y, double time, int64_t now_ns) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(time) || time <= 0.0 || !isFresh(now_ns)) {
    return std::nullopt;
  }
  const int64_t query_ns = static_cast<int64_t>(std::llround(time * kNsPerSecond));
  const auto & slices = sequence_.slices;
  const int64_t first = durationNs(slices.front().time_from_start);
  const int64_t last = durationNs(slices.back().time_from_start);
  if (query_ns < first || query_ns > last) {return std::nullopt;}
  for (size_t i = 0; i < slices.size(); ++i) {
    const int64_t upper = durationNs(slices[i].time_from_start);
    if (query_ns == upper) {
      return riskAt(slices[i].probability, slices[i].uncertainty, x, y, config_);
    }
    if (query_ns < upper) {
      const auto lower_risk = riskAt(slices[i - 1].probability, slices[i - 1].uncertainty, x, y, config_);
      const auto upper_risk = riskAt(slices[i].probability, slices[i].uncertainty, x, y, config_);
      if (!lower_risk || !upper_risk) {return std::nullopt;}
      const int64_t lower = durationNs(slices[i - 1].time_from_start);
      const float alpha = static_cast<float>(query_ns - lower) / static_cast<float>(upper - lower);
      return *lower_risk + alpha * (*upper_risk - *lower_risk);
    }
  }
  return std::nullopt;
}
}  // namespace m1_scope_risk
