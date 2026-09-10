#ifndef M1_SCOPE_RISK__RISK_FIELD_SNAPSHOT_HPP_
#define M1_SCOPE_RISK__RISK_FIELD_SNAPSHOT_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "m1_scope_msgs/msg/scope_prediction_sequence.hpp"

namespace m1_scope_risk
{

struct RiskFieldConfig
{
  double uncertainty_gain{1.0};
  // The legacy transport encodes min(sigma / scale, 1) in [0,100].
  double uncertainty_encoding_scale{0.5};
  int64_t stale_timeout_ns{500000000};
};

// Immutable, validated decoded sequence. It deliberately contains no costmap
// state: current obstacles remain a separate, ordinary costmap concern.
class RiskFieldSnapshot
{
public:
  static std::optional<RiskFieldSnapshot> fromMessage(
    const m1_scope_msgs::msg::ScopePredictionSequence & sequence,
    const RiskFieldConfig & config, std::string * reason = nullptr);

  // `now_ns` is supplied explicitly to make freshness and ROS time resets
  // deterministic. Returns nullopt for stale, reset, unknown, out-of-grid, or
  // out-of-horizon queries. Time is seconds from the sequence anchor.
  std::optional<float> query(double world_x, double world_y,
    double time_from_anchor_seconds, int64_t now_ns) const;

  int64_t anchorStampNs() const {return anchor_stamp_ns_;}
  int64_t latestSliceNs() const {return latest_slice_ns_;}
  uint64_t predictionId() const {return prediction_id_;}
  const std::string & frameId() const {return sequence_.header.frame_id;}
  bool isFresh(int64_t now_ns) const;

private:
  m1_scope_msgs::msg::ScopePredictionSequence sequence_;
  RiskFieldConfig config_;
  int64_t anchor_stamp_ns_{0};
  int64_t latest_slice_ns_{0};
  uint64_t prediction_id_{0};
};

}  // namespace m1_scope_risk

#endif
