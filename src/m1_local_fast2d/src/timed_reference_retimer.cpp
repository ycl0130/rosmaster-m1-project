#include "m1_local_fast2d/timed_reference_retimer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace m1_local_fast2d
{
namespace
{
constexpr double kPositionEpsilon = 1e-6;
constexpr double kYawEpsilon = 1e-6;
constexpr double kTimeEpsilon = 1e-6;
constexpr double kMinimumMovingDt = 0.05;
constexpr double kReasonableTimeScale = 20.0;
constexpr double kTerminalApproachZone = 0.35;
struct Segment {std::size_t start; double dt, vx, vy, wz;};
double angleDelta(double from, double to) {return std::atan2(std::sin(to - from), std::cos(to - from));}
double requiredDt(double dx, double dy, double dyaw, const TimedReferenceRetimingLimits & limits)
{
  return std::max({kMinimumMovingDt, std::abs(dx) / limits.vx_max,
    std::abs(dy) / limits.vy_max, std::abs(dyaw) / limits.wz_max});
}

void extrema(const std::vector<Segment> & segments, double & vx, double & vy, double & wz,
  double & ax, double & ay, double & awz)
{
  vx = vy = wz = ax = ay = awz = 0.0;
  double previous_vx = 0.0, previous_vy = 0.0, previous_wz = 0.0;
  for (const auto & segment : segments) {
    vx = std::max(vx, std::abs(segment.vx)); vy = std::max(vy, std::abs(segment.vy));
    wz = std::max(wz, std::abs(segment.wz));
    ax = std::max(ax, std::abs(segment.vx - previous_vx) / segment.dt);
    ay = std::max(ay, std::abs(segment.vy - previous_vy) / segment.dt);
    awz = std::max(awz, std::abs(segment.wz - previous_wz) / segment.dt);
    previous_vx = segment.vx; previous_vy = segment.vy; previous_wz = segment.wz;
  }
  if (!segments.empty()) {
    const auto & last = segments.back();
    ax = std::max(ax, std::abs(last.vx) / last.dt);
    ay = std::max(ay, std::abs(last.vy) / last.dt);
    awz = std::max(awz, std::abs(last.wz) / last.dt);
  }
}
}  // namespace

m1_local_fast2d::msg::TimedTrajectory retimeTimedReference(
  const m1_local_fast2d::msg::TimedTrajectory & input,
  const TimedReferenceRetimingLimits & limits, TimedReferenceRetimingDiagnostics * diagnostics)
{
  TimedReferenceRetimingDiagnostics result;
  auto output = input;
  std::optional<std::size_t> terminal_rotation_start;
  std::optional<std::size_t> terminal_rotation_segment;
  std::optional<std::size_t> terminal_approach_segment;
  if (input.points.size() < 2 || limits.vx_max <= 0.0 || limits.vy_max <= 0.0 ||
    limits.wz_max <= 0.0 || limits.ax_max <= 0.0 || limits.ay_max <= 0.0 || limits.awz_max <= 0.0)
  {if (diagnostics) {*diagnostics = result;} return output;}

  // Fast Planner's final pose has the requested terminal heading, whereas
  // the preceding points use path-tangent headings.  When that final heading
  // changes sharply, coupling the turn to the final translation forces one
  // uniform time scale over every approach segment.  Preserve every supplied
  // x/y pose and the final yaw, but insert a same-position turn after the
  // terminal x/y has been reached.  This is a temporal representation change
  // only: the spatial path is identical.
  if (output.points.size() >= 2) {
    const auto & before_terminal = output.points[output.points.size() - 2];
    const auto & terminal = output.points.back();
    const double terminal_distance = std::hypot(
      terminal.x - before_terminal.x, terminal.y - before_terminal.y);
    const double terminal_yaw_delta = angleDelta(before_terminal.yaw, terminal.yaw);
    const double terminal_dt = terminal.time_from_start - before_terminal.time_from_start;
    const double terminal_wz = terminal_dt > kTimeEpsilon ?
      std::abs(terminal_yaw_delta) / terminal_dt : std::numeric_limits<double>::infinity();
    if (terminal_distance > kPositionEpsilon && terminal_distance <= kTerminalApproachZone &&
      std::abs(terminal_yaw_delta) > 1.0 && terminal_wz > limits.wz_max + kYawEpsilon) {
      auto approach_terminal = terminal;
      approach_terminal.yaw = before_terminal.yaw;
      // Its supplied stamp is intentionally ignored below; a fresh, bounded
      // duration is regenerated for both the approach and the turn.
      approach_terminal.time_from_start = before_terminal.time_from_start;
      approach_terminal.vx = approach_terminal.vy = 0.0;
      approach_terminal.ax = approach_terminal.ay = 0.0;
      output.points.insert(output.points.end() - 1, approach_terminal);
      terminal_rotation_start = output.points.size() - 2;
      result.terminal_heading_split = true;
      result.terminal_approach_distance = terminal_distance;
    }
  }

  std::vector<Segment> original;
  original.reserve(output.points.size() - 1);
  for (std::size_t i = 1; i < output.points.size(); ++i) {
    const auto & a = output.points[i - 1]; const auto & b = output.points[i];
    const double dx = b.x - a.x, dy = b.y - a.y, dyaw = angleDelta(a.yaw, b.yaw);
    const bool no_translation = std::hypot(dx, dy) < kPositionEpsilon;
    const bool terminal_rotation = terminal_rotation_start &&
      i - 1 == *terminal_rotation_start;
    // A duplicate pose/yaw connector is a terminal hold, not a 1 ms motion
    // primitive. A duplicate position with a changed yaw is instead an
    // explicit in-place turn and must retain a finite, capability-limited
    // duration.
    if (no_translation && !terminal_rotation) {
      ++result.ignored_duplicate_segments;
      continue;
    }
    double dt = b.time_from_start - a.time_from_start;
    if (!std::isfinite(dt) || dt <= kTimeEpsilon ||
      (terminal_rotation &&
      std::abs(dyaw) / dt > limits.wz_max + kYawEpsilon)) {
      dt = requiredDt(dx, dy, dyaw, limits);
      ++result.reprocessed_zero_duration_segments;
    }
    if (terminal_rotation) {
      terminal_rotation_segment = original.size();
    } else if (terminal_rotation_start && i == *terminal_rotation_start) {
      terminal_approach_segment = original.size();
    }
    original.push_back({i - 1, dt, dx / dt, dy / dt, dyaw / dt});
  }
  extrema(original, result.before_max_vx, result.before_max_vy, result.before_max_wz,
    result.before_max_ax, result.before_max_ay, result.before_max_awz);
  result.before_duration = input.points.back().time_from_start - input.points.front().time_from_start;
  result.time_scale = std::max({1.0,
    result.before_max_vx / limits.vx_max, result.before_max_vy / limits.vy_max,
    result.before_max_wz / limits.wz_max,
    std::sqrt(result.before_max_ax / limits.ax_max), std::sqrt(result.before_max_ay / limits.ay_max),
    std::sqrt(result.before_max_awz / limits.awz_max)});

  // A scale above this guard is only admissible after each pathological
  // zero-duration moving segment has been given a finite capability-derived
  // duration above.  Rebuild such segments rather than treating their
  // timestamp defect as an arbitrarily fast physical primitive.
  if (result.time_scale > kReasonableTimeScale && result.reprocessed_zero_duration_segments > 0) {
    result.sanity_guard_triggered = true;
    extrema(original, result.before_max_vx, result.before_max_vy, result.before_max_wz,
      result.before_max_ax, result.before_max_ay, result.before_max_awz);
    result.time_scale = std::max({1.0,
      result.before_max_vx / limits.vx_max, result.before_max_vy / limits.vy_max,
      result.before_max_wz / limits.wz_max,
      std::sqrt(result.before_max_ax / limits.ax_max), std::sqrt(result.before_max_ay / limits.ay_max),
      std::sqrt(result.before_max_awz / limits.awz_max)});
  }

  std::vector<Segment> retimed = original;
  for (auto & segment : retimed) {
    segment.dt *= result.time_scale;
    segment.vx /= result.time_scale; segment.vy /= result.time_scale; segment.wz /= result.time_scale;
  }
  double time = 0.0;
  output.points.front().time_from_start = time;
  std::size_t dynamic = 0;
  for (std::size_t i = 1; i < output.points.size(); ++i) {
    const auto & input_a = output.points[i - 1]; const auto & input_b = output.points[i];
    const double dx = input_b.x - input_a.x;
    const double dy = input_b.y - input_a.y;
    const bool terminal_rotation = terminal_rotation_start &&
      i - 1 == *terminal_rotation_start;
    if (std::hypot(dx, dy) < kPositionEpsilon && !terminal_rotation) {
      output.points[i - 1].vx = output.points[i - 1].vy = 0.0;
      output.points[i - 1].ax = output.points[i - 1].ay = 0.0;
      output.points[i].time_from_start = time;
      continue;
    }
    auto & point = output.points[i - 1];
    const auto & segment = retimed[dynamic++];
    point.vx = segment.vx; point.vy = segment.vy;
    time += segment.dt;
    output.points[i].time_from_start = time;
  }
  for (std::size_t i = 0; i < retimed.size(); ++i) {
    const auto previous_vx = i == 0 ? 0.0 : retimed[i - 1].vx;
    const auto previous_vy = i == 0 ? 0.0 : retimed[i - 1].vy;
    auto & point = output.points[retimed[i].start];
    point.ax = (retimed[i].vx - previous_vx) / retimed[i].dt;
    point.ay = (retimed[i].vy - previous_vy) / retimed[i].dt;
  }
  output.points.back().vx = output.points.back().vy = output.points.back().ax = output.points.back().ay = 0.0;
  result.after_duration = time;
  extrema(retimed, result.after_max_vx, result.after_max_vy, result.after_max_wz,
    result.after_max_ax, result.after_max_ay, result.after_max_awz);
  if (terminal_rotation_segment) {
    result.terminal_rotation_duration = retimed[*terminal_rotation_segment].dt;
  }
  if (terminal_approach_segment) {
    const auto & approach = retimed[*terminal_approach_segment];
    result.terminal_approach_speed = std::hypot(approach.vx, approach.vy);
    // Use the already-sanitized source segment, rather than the final raw
    // connector stamp (which may correctly be zero-duration).
    result.terminal_time_scale = approach.dt /
      std::max(kTimeEpsilon, original[*terminal_approach_segment].dt);
  }
  // The optional inserted terminal turn is at the final supplied x/y. Check
  // every original point and the final endpoint against the unconditioned
  // geometry rather than treating that representation point as geometry.
  const std::size_t prefix = result.terminal_heading_split ? input.points.size() - 1 : input.points.size();
  for (std::size_t i = 0; i < prefix; ++i) {
    result.geometry_error = std::max(result.geometry_error, std::hypot(
      input.points[i].x - output.points[i].x, input.points[i].y - output.points[i].y));
  }
  if (result.terminal_heading_split) {
    result.geometry_error = std::max(result.geometry_error, std::hypot(
      input.points.back().x - output.points.back().x, input.points.back().y - output.points.back().y));
  }
  result.valid = true;
  if (diagnostics) {*diagnostics = result;}
  return output;
}

}  // namespace m1_local_fast2d
