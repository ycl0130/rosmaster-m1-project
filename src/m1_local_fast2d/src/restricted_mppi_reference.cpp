#include "m1_local_fast2d/restricted_mppi_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace m1_local_fast2d
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double RestrictedTimedReference::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double RestrictedTimedReference::shortestAngleDelta(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

RestrictedTimedReference::RestrictedTimedReference(
  const msg::TimedTrajectory & trajectory,
  RestrictedTimedReferenceMetadata metadata,
  const RestrictedTimedReferenceConfig & config)
: metadata_(std::move(metadata)), config_(config)
{
  if (!std::isfinite(config_.time_epsilon) || config_.time_epsilon <= 0.0 ||
    !std::isfinite(config_.omega_max) || config_.omega_max <= 0.0 || trajectory.points.empty())
  {
    return;
  }

  Sample previous;
  bool have_previous = false;
  double previous_raw_yaw = 0.0;
  double last_time = -std::numeric_limits<double>::infinity();
  for (const auto & point : trajectory.points) {
    const double values[] = {point.x, point.y, point.yaw, point.time_from_start,
      point.vx, point.vy, point.ax, point.ay};
    bool finite = true;
    for (const auto value : values) {finite = finite && std::isfinite(value);}
    if (!finite || point.time_from_start < last_time - config_.time_epsilon) {
      samples_.clear();
      return;
    }

    double unwrapped_yaw = point.yaw;
    if (have_previous) {
      unwrapped_yaw = previous_raw_yaw + shortestAngleDelta(previous_raw_yaw, point.yaw);
    }
    previous_raw_yaw = unwrapped_yaw;

    Sample current{point.x, point.y, unwrapped_yaw, point.time_from_start,
      point.vx, point.vy, point.ax, point.ay, 0.0, 0.0};
    terminal_sample_ = current;
    if (!have_previous || current.time > samples_.back().time + config_.time_epsilon) {
      samples_.push_back(current);
    }
    previous = current;
    have_previous = true;
    last_time = std::max(last_time, current.time);
  }

  if (samples_.empty()) {return;}
  duration_sec_ = std::max(0.0, terminal_sample_.time);
  terminal_is_zero_duration_ = terminal_sample_.time <= samples_.back().time + config_.time_epsilon &&
    terminal_sample_.time < samples_.back().time + config_.time_epsilon;

  // A duplicate-time terminal connector is retained as a pose, but never as
  // a dynamic interval.  It is represented by terminal_sample_ only.
  if (terminal_sample_.time < samples_.back().time + config_.time_epsilon) {
    duration_sec_ = samples_.back().time;
  }

  if (samples_.size() == 1) {
    samples_.front().omega_raw = 0.0;
    samples_.front().omega = 0.0;
  } else {
    for (std::size_t index = 0; index + 1 < samples_.size(); ++index) {
      const double dt = samples_[index + 1].time - samples_[index].time;
      const double raw = dt > config_.time_epsilon ?
        (samples_[index + 1].yaw - samples_[index].yaw) / dt : 0.0;
      samples_[index].omega_raw = std::isfinite(raw) ? raw : 0.0;
      samples_[index].omega = clamp(samples_[index].omega_raw, -config_.omega_max, config_.omega_max);
    }
    samples_.back().omega_raw = 0.0;
    samples_.back().omega = 0.0;
  }
  valid_ = metadata_.frame_id.size() > 0;
}

ReferenceState RestrictedTimedReference::stateFromSample(const Sample & sample, bool terminal_hold) const
{
  ReferenceState output;
  output.x_ref = sample.x;
  output.y_ref = sample.y;
  output.yaw_ref = sample.yaw;
  output.vx_world_ref = terminal_hold ? 0.0 : sample.vx;
  output.vy_world_ref = terminal_hold ? 0.0 : sample.vy;
  output.vx_body_ref = terminal_hold ? 0.0 :
    std::cos(sample.yaw) * output.vx_world_ref + std::sin(sample.yaw) * output.vy_world_ref;
  output.vy_body_ref = terminal_hold ? 0.0 :
    -std::sin(sample.yaw) * output.vx_world_ref + std::cos(sample.yaw) * output.vy_world_ref;
  output.omega_raw = terminal_hold ? 0.0 : sample.omega_raw;
  output.omega_ref = terminal_hold ? 0.0 : sample.omega;
  output.ax_ref = terminal_hold ? 0.0 : sample.ax;
  output.ay_ref = terminal_hold ? 0.0 : sample.ay;
  output.reference_valid = valid_;
  output.terminal_hold = terminal_hold;
  return output;
}

ReferenceState RestrictedTimedReference::terminalState() const
{
  // Terminal velocity and acceleration are targets, not an instantaneous
  // command.  The future controller remains responsible for rate limiting.
  return stateFromSample(terminal_sample_, true);
}

ReferenceState RestrictedTimedReference::sample(double elapsed_sec) const
{
  if (!valid_ || samples_.empty() || !std::isfinite(elapsed_sec) || elapsed_sec < -config_.time_epsilon) {
    return {};
  }
  const double t = std::max(0.0, elapsed_sec);
  if (t >= duration_sec_ - config_.time_epsilon) {
    return terminalState();
  }
  if (samples_.size() == 1) {
    return stateFromSample(samples_.front(), false);
  }
  if (t <= samples_.front().time + config_.time_epsilon) {
    return stateFromSample(samples_.front(), false);
  }

  const auto upper = std::upper_bound(
    samples_.begin(), samples_.end(), t,
    [](double value, const Sample & sample) {return value < sample.time;});
  const auto right = upper == samples_.end() ? std::prev(samples_.end()) : upper;
  const auto left = right == samples_.begin() ? right : std::prev(right);
  const double dt = right->time - left->time;
  const double ratio = dt > config_.time_epsilon ? clamp((t - left->time) / dt, 0.0, 1.0) : 0.0;
  Sample interpolated;
  interpolated.time = t;
  interpolated.x = left->x + ratio * (right->x - left->x);
  interpolated.y = left->y + ratio * (right->y - left->y);
  interpolated.yaw = left->yaw + ratio * (right->yaw - left->yaw);
  interpolated.vx = left->vx + ratio * (right->vx - left->vx);
  interpolated.vy = left->vy + ratio * (right->vy - left->vy);
  interpolated.ax = left->ax + ratio * (right->ax - left->ax);
  interpolated.ay = left->ay + ratio * (right->ay - left->ay);
  interpolated.omega_raw = left->omega_raw;
  interpolated.omega = left->omega;
  return stateFromSample(interpolated, false);
}

std::vector<ReferenceState> RestrictedTimedReference::sampleHorizon(
  double elapsed_sec, std::size_t sample_count, double model_dt) const
{
  std::vector<ReferenceState> output;
  if (!std::isfinite(model_dt) || model_dt <= 0.0) {return output;}
  output.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    output.push_back(sample(elapsed_sec + static_cast<double>(index) * model_dt));
  }
  return output;
}

double RestrictedTimedReference::spatialLength() const
{
  double length = 0.0;
  for (std::size_t index = 1; index < samples_.size(); ++index) {
    length += std::hypot(samples_[index].x - samples_[index - 1].x,
      samples_[index].y - samples_[index - 1].y);
  }
  return length;
}

double RestrictedTimedReference::nominalDt() const
{
  return samples_.size() > 1 ? samples_[1].time - samples_[0].time : 0.0;
}

RestrictedTimedReferenceDiagnostics RestrictedTimedReference::diagnostics(
  int64_t now_stamp_ns, double horizon_duration_sec,
  std::size_t horizon_sample_count, bool switched,
  uint64_t previous_planning_result_id) const
{
  RestrictedTimedReferenceDiagnostics output;
  output.metadata = metadata_;
  output.reference_age_sec = static_cast<double>(now_stamp_ns - metadata_.activation_stamp_ns) / 1e9;
  output.reference_duration_sec = duration_sec_;
  output.mppi_horizon_duration_sec = horizon_duration_sec;
  output.horizon_sample_count = horizon_sample_count;
  output.first_reference = sample(0.0);
  output.reference_switched = switched;
  output.previous_planning_result_id = previous_planning_result_id;
  return output;
}

}  // namespace m1_local_fast2d
