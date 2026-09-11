#include "m1_local_fast2d/restricted_mppi_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace m1_local_fast2d
{
namespace
{
double angleDelta(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

std::string horizonJson(const std::vector<ReferenceState> & horizon)
{
  std::ostringstream json;
  json << "[";
  for (std::size_t i = 0; i < horizon.size(); ++i) {
    if (i) {json << ",";}
    const auto & s = horizon[i];
    json << "{\"i\":" << i << ",\"x\":" << s.x_ref << ",\"y\":" << s.y_ref
         << ",\"yaw\":" << s.yaw_ref << ",\"vx\":" << s.vx_world_ref
         << ",\"vy\":" << s.vy_world_ref << ",\"wz\":" << s.omega_ref << "}";
  }
  json << "]";
  return json.str();
}

// MPPI rolls a zero-order-held control forward by model_dt.  The planner
// reference is interpolated at that same grid, so derive each controller
// nominal directly from the next sampled pose.  This leaves every sampled
// pose (and therefore Fast2D geometry) untouched while enforcing the exact
// discrete kinematic contract consumed by the rollout integrator.
void conditionHorizonKinematics(std::vector<ReferenceState> & horizon, double dt,
  double & original_position_residual, double & conditioned_position_residual,
  double & original_yaw_residual, double & conditioned_yaw_residual)
{
  original_position_residual = conditioned_position_residual = 0.0;
  original_yaw_residual = conditioned_yaw_residual = 0.0;
  if (!std::isfinite(dt) || dt <= 0.0 || horizon.empty()) {return;}
  for (std::size_t i = 0; i + 1 < horizon.size(); ++i) {
    const auto & current = horizon[i];
    const auto & next = horizon[i + 1];
    original_position_residual = std::max(original_position_residual, std::hypot(
      next.x_ref - current.x_ref - current.vx_world_ref * dt,
      next.y_ref - current.y_ref - current.vy_world_ref * dt));
    original_yaw_residual = std::max(original_yaw_residual, std::abs(
      angleDelta(current.yaw_ref, next.yaw_ref) - current.omega_ref * dt));

    auto & conditioned = horizon[i];
    conditioned.vx_world_ref = (next.x_ref - conditioned.x_ref) / dt;
    conditioned.vy_world_ref = (next.y_ref - conditioned.y_ref) / dt;
    conditioned.omega_ref = angleDelta(conditioned.yaw_ref, next.yaw_ref) / dt;
    conditioned.omega_raw = conditioned.omega_ref;
    conditioned.vx_body_ref = std::cos(conditioned.yaw_ref) * conditioned.vx_world_ref +
      std::sin(conditioned.yaw_ref) * conditioned.vy_world_ref;
    conditioned.vy_body_ref = -std::sin(conditioned.yaw_ref) * conditioned.vx_world_ref +
      std::cos(conditioned.yaw_ref) * conditioned.vy_world_ref;
    conditioned_position_residual = std::max(conditioned_position_residual, std::hypot(
      next.x_ref - conditioned.x_ref - conditioned.vx_world_ref * dt,
      next.y_ref - conditioned.y_ref - conditioned.vy_world_ref * dt));
    conditioned_yaw_residual = std::max(conditioned_yaw_residual, std::abs(
      angleDelta(conditioned.yaw_ref, next.yaw_ref) - conditioned.omega_ref * dt));
  }
  // A terminal hold is stationary by definition.  Keeping its command zero
  // prevents a synthetic velocity after the fixed Fast2D geometry ends.
  auto & terminal = horizon.back();
  terminal.vx_world_ref = terminal.vy_world_ref = terminal.vx_body_ref = terminal.vy_body_ref = 0.0;
  terminal.omega_ref = terminal.omega_raw = 0.0;
}
constexpr char kExpectedVersion[] = "1.1.20";

double clampScalar(double value, double lower, double upper, bool & clamped)
{
  if (lower > upper) {
    return std::max(lower, std::min(upper, value));
  }
  const double result = std::max(lower, std::min(upper, value));
  clamped = clamped || result != value;
  return result;
}

bool finite(double value)
{
  return std::isfinite(value);
}

struct TensorFiniteCounts
{
  std::size_t finite{0};
  std::size_t nan{0};
  std::size_t infinity{0};
};

template<typename Tensor>
TensorFiniteCounts countTensorValues(const Tensor & tensor)
{
  TensorFiniteCounts result;
  for (const auto value : tensor) {
    if (std::isfinite(value)) {
      ++result.finite;
    } else if (std::isnan(value)) {
      ++result.nan;
    } else {
      ++result.infinity;
    }
  }
  return result;
}

}  // namespace

RestrictedMPPIAdapter::RestrictedMPPIAdapter(nav2_mppi_controller::MPPIController & controller)
: controller_(&controller)
{
  version_guard_ok_ = checkInstalledVersion();
}

bool RestrictedMPPIAdapter::checkInstalledVersion()
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("nav2_mppi_controller");
    std::ifstream package_xml(share + "/package.xml");
    std::string line;
    while (std::getline(package_xml, line)) {
      const auto begin = line.find("<version>");
      const auto end = line.find("</version>");
      if (begin != std::string::npos && end != std::string::npos && end > begin + 9) {
        upstream_version_ = line.substr(begin + 9, end - (begin + 9));
        break;
      }
    }
  } catch (const std::exception & error) {
    version_failure_ = std::string("unable to read nav2_mppi_controller package version: ") + error.what();
    return false;
  }

  if (upstream_version_ != kExpectedVersion) {
    version_failure_ = "restricted seam requires nav2_mppi_controller 1.1.20, found " +
      (upstream_version_.empty() ? std::string("<unknown>") : upstream_version_);
    return false;
  }
  return true;
}

geometry_msgs::msg::TwistStamped RestrictedMPPIAdapter::zeroCommand(
  const builtin_interfaces::msg::Time & stamp) const
{
  return mppi::utils::toTwistStamped(
    0.0F, 0.0F, 0.0F, stamp, controller_->costmap_ros_->getBaseFrameID());
}

bool RestrictedMPPIAdapter::transformReference(
  const std::vector<ReferenceState> & source, const std::string & source_frame,
  const std::string & target_frame, std::vector<ReferenceState> & output) const
{
  if (source_frame.empty() || target_frame.empty()) {
    return false;
  }
  if (source_frame == target_frame) {
    output = source;
    return true;
  }
  if (!controller_->tf_buffer_) {
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = controller_->tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double transform_yaw = tf2::getYaw(transform.transform.rotation);
  const double c = std::cos(transform_yaw);
  const double s = std::sin(transform_yaw);
  output.clear();
  output.reserve(source.size());
  for (auto state : source) {
    const double x = state.x_ref;
    const double y = state.y_ref;
    state.x_ref = transform.transform.translation.x + c * x - s * y;
    state.y_ref = transform.transform.translation.y + s * x + c * y;
    state.yaw_ref += transform_yaw;

    const double vx = state.vx_world_ref;
    const double vy = state.vy_world_ref;
    state.vx_world_ref = c * vx - s * vy;
    state.vy_world_ref = s * vx + c * vy;
    const double ax = state.ax_ref;
    const double ay = state.ay_ref;
    state.ax_ref = c * ax - s * ay;
    state.ay_ref = s * ax + c * ay;
    state.vx_body_ref = std::cos(state.yaw_ref) * state.vx_world_ref +
      std::sin(state.yaw_ref) * state.vy_world_ref;
    state.vy_body_ref = -std::sin(state.yaw_ref) * state.vx_world_ref +
      std::cos(state.yaw_ref) * state.vy_world_ref;
    output.push_back(state);
  }
  return true;
}

bool RestrictedMPPIAdapter::collisionAtPose(
  float x, float y, float yaw, bool consider_footprint, std::string * source,
  std::string * evidence_json) const
{
  if (source) {*source = "NONE";}
  if (evidence_json) {evidence_json->clear();}
  auto * costmap = controller_->costmap_ros_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  const bool inside = checker.worldToMap(x, y, map_x, map_y);
  float point_cost = nav2_costmap_2d::NO_INFORMATION;
  if (inside) {
    point_cost = static_cast<float>(checker.pointCost(map_x, map_y));
  }

  const bool tracking_unknown =
    controller_->costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  if (static_cast<unsigned char>(point_cost) == nav2_costmap_2d::NO_INFORMATION) {
    if (!tracking_unknown) {
      if (source) {*source = "UNKNOWN";}
      return true;
    }
  }
  if (static_cast<unsigned char>(point_cost) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    if (source) {*source = "LETHAL_CELL";}
    if (evidence_json) {
      std::ostringstream out;
      out << "{\"robot_pose\":{\"x\":" << x << ",\"y\":" << y << ",\"yaw\":" << yaw
          << "},\"collision_cell\":{\"mx\":" << map_x << ",\"my\":" << map_y
          << ",\"cost\":" << static_cast<unsigned int>(point_cost) << "}}";
      *evidence_json = out.str();
    }
    return true;
  }

  // CostCritic's footprint check is the authoritative collision model.  The
  // adapter repeats that check only to turn its finite collision penalty into
  // an explicit infeasible mask; it does not replace CostCritic.
  if (consider_footprint && inside && point_cost > 0.0F) {
    const double footprint_cost = checker.footprintCostAtPose(
      x, y, yaw, controller_->costmap_ros_->getRobotFootprint());
    if (!std::isfinite(footprint_cost) ||
      static_cast<unsigned char>(footprint_cost) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      if (source) {*source = "FOOTPRINT_LETHAL";}
      if (evidence_json) {
        const auto footprint = controller_->costmap_ros_->getRobotFootprint();
        const double c = std::cos(yaw), s = std::sin(yaw);
        std::ostringstream out;
        out << "{\"robot_pose\":{\"x\":" << x << ",\"y\":" << y << ",\"yaw\":" << yaw
            << "},\"footprint\":[";
        for (std::size_t i = 0; i < footprint.size(); ++i) {
          if (i) {out << ',';}
          out << "{\"x\":" << x + c * footprint[i].x - s * footprint[i].y
              << ",\"y\":" << y + s * footprint[i].x + c * footprint[i].y << "}";
        }
        // Nav2's API does not expose the exact edge cell chosen by its
        // footprint checker.  The centre cell is explicitly recorded as
        // context; the retained master snapshot is authoritative for exact
        // post-run cell attribution.
        out << "],\"centre_cell\":{\"mx\":" << map_x << ",\"my\":" << map_y
            << ",\"cost\":" << static_cast<unsigned int>(point_cost) << "}}";
        *evidence_json = out.str();
      }
      return true;
    }
    if (static_cast<unsigned char>(footprint_cost) == nav2_costmap_2d::NO_INFORMATION) {
      if (source && !tracking_unknown) {*source = "FOOTPRINT_UNKNOWN";}
      return !tracking_unknown;
    }
  }
  return false;
}

geometry_msgs::msg::TwistStamped RestrictedMPPIAdapter::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker,
  const std::shared_ptr<const RestrictedTimedReference> & reference,
  uint64_t reference_generation, const RestrictedMPPIConfig & config,
  RestrictedMPPIDiagnostics & diagnostics)
{
  const auto cycle_started = std::chrono::steady_clock::now();
  diagnostics = {};
  diagnostics.enabled = config.enabled;
  diagnostics.version_guard_ok = version_guard_ok_;
  diagnostics.upstream_version = upstream_version_;
  diagnostics.reference_generation = reference_generation;

  const auto now = controller_->clock_->now();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  const auto setReferenceIdentity = [&diagnostics, &reference]() {
      if (reference) {
        diagnostics.planning_result_id = reference->metadata().planning_result_id;
        diagnostics.reference_id = reference->metadata().reference_id;
        diagnostics.selected_candidate_index = reference->metadata().selected_candidate_index;
      }
    };
  setReferenceIdentity();
  if (reference) {
    diagnostics.reference_source_stamp_ns = reference->metadata().source_stamp_ns;
    diagnostics.reference_activation_stamp_ns = reference->metadata().activation_stamp_ns;
    diagnostics.reference_length_m = reference->spatialLength();
    const auto first = reference->sample(0.0);
    const auto last = reference->sample(reference->duration());
    diagnostics.reference_start_x = first.x_ref;
    diagnostics.reference_start_y = first.y_ref;
    diagnostics.reference_last_x = last.x_ref;
    diagnostics.reference_last_y = last.y_ref;
    diagnostics.reference_dt_s = reference->nominalDt();
    diagnostics.reference0_yaw = first.yaw_ref;
    diagnostics.reference0_vx_world = first.vx_world_ref;
    diagnostics.reference0_vy_world = first.vy_world_ref;
    diagnostics.reference0_wz = first.omega_ref;
  }
  if (!reference || !reference->valid()) {
    diagnostics.status = "NO_REFERENCE";
    return zeroCommand(stamp);
  }
  if (!finite(config.delta_vx_max) || config.delta_vx_max < 0.0 ||
    !finite(config.delta_vy_max) || config.delta_vy_max < 0.0 ||
    !finite(config.delta_wz_max) || config.delta_wz_max < 0.0 ||
    !finite(config.position_tube_radius) || config.position_tube_radius <= 0.0)
  {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(stamp);
  }

  diagnostics.reference_age_sec = static_cast<double>(
    now.nanoseconds() - reference->metadata().activation_stamp_ns) / 1e9;
  diagnostics.reference_switched =
    last_reference_generation_ != 0 &&
    (last_reference_generation_ != reference_generation ||
    last_reference_id_ != reference->metadata().reference_id);
  diagnostics.reset_performed = diagnostics.reference_switched;
  if (diagnostics.reference_switched) {
    last_reference_generation_ = reference_generation;
    last_reference_id_ = reference->metadata().reference_id;
  } else if (last_reference_generation_ == 0) {
    last_reference_generation_ = reference_generation;
    last_reference_id_ = reference->metadata().reference_id;
  }

  std::lock_guard<std::mutex> parameter_lock(*controller_->parameters_handler_->getLock());
  // MPPIController::computeVelocityCommands() holds this same mutex around
  // Optimizer::evalControl().  The restricted seam invokes the optimizer and
  // the CostCritic directly, plus its explicit footprint mask, so it must
  // retain the stock synchronization contract with costmap update threads.
  auto * costmap = controller_->costmap_ros_->getCostmap();
  if (!costmap) {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(stamp);
  }
  const auto costmap_access_started = std::chrono::steady_clock::now();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
  diagnostics.costmap_access_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - costmap_access_started).count();
  if (controller_->clock_->now() - controller_->last_time_called_ >
    rclcpp::Duration::from_seconds(controller_->reset_period_))
  {
    RCLCPP_INFO(controller_->logger_, "Restricted MPPI reset begin");
    controller_->reset();
    RCLCPP_INFO(controller_->logger_, "Restricted MPPI reset end");
  }
  controller_->last_time_called_ = controller_->clock_->now();

  nav_msgs::msg::Path transformed_plan;
  try {
    transformed_plan = controller_->path_handler_.transformPath(robot_pose);
  } catch (const std::exception &) {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(stamp);
  }

  const auto target_frame = controller_->costmap_ros_->getGlobalFrameID();
  diagnostics.comparison_frame = target_frame;
  // Project the actual robot pose in the immutable reference frame before
  // preparing the rollout horizon.  This is outside the 500x40 hot loop and
  // is the only TF operation needed for phase synchronization.
  geometry_msgs::msg::PoseStamped reference_pose = robot_pose;
  if (reference_pose.header.frame_id != reference->metadata().frame_id) {
    try {
      geometry_msgs::msg::PoseStamped transformed_pose;
      controller_->tf_buffer_->transform(
        reference_pose, transformed_pose, reference->metadata().frame_id,
        tf2::durationFromSec(0.2));
      reference_pose = transformed_pose;
      reference_pose.header.frame_id = reference->metadata().frame_id;
    } catch (const tf2::TransformException &) {
      diagnostics.status = "NO_FEASIBLE_CONTROL";
      return zeroCommand(transformed_plan.header.stamp);
    }
  }
  auto & optimizer = controller_->optimizer_;
  const auto & settings = optimizer.settings_;
  const RestrictedMPPIPhaseConfig phase_config{
    config.phase_search_back_s, config.phase_search_forward_s,
    config.max_phase_lead_s, config.phase_projection_resolution_s};
  const auto phase_started = std::chrono::steady_clock::now();
  const auto phase = phase_tracker_.update(
    *reference, reference_pose.pose.position.x, reference_pose.pose.position.y,
    reference_generation, settings.model_dt, phase_config);
  diagnostics.previous_tracking_phase_s = phase.previous_phase_s;
  diagnostics.projected_phase_s = phase.projected_phase_s;
  diagnostics.tracking_phase_s = phase.tracking_phase_s;
  diagnostics.phase_lead_s = phase.tracking_phase_s - phase.projected_phase_s;
  std::vector<ReferenceState> horizon = reference->sampleHorizon(
    diagnostics.tracking_phase_s, settings.time_steps, settings.model_dt);
  if (horizon.size() != controller_->optimizer_.settings_.time_steps) {
    diagnostics.status = "NO_REFERENCE";
    return zeroCommand(transformed_plan.header.stamp);
  }
  for (const auto & sample : horizon) {
    if (!sample.reference_valid) {
      diagnostics.status = "NO_REFERENCE";
      return zeroCommand(transformed_plan.header.stamp);
    }
  }
  if (!transformReference(
      horizon, reference->metadata().frame_id, target_frame, horizon))
  {
    diagnostics.status = "NO_REFERENCE";
    return zeroCommand(transformed_plan.header.stamp);
  }
  diagnostics.original_horizon_state_json = horizonJson(horizon);
  conditionHorizonKinematics(horizon, settings.model_dt,
    diagnostics.original_max_kinematic_residual_m,
    diagnostics.conditioned_max_kinematic_residual_m,
    diagnostics.original_max_yaw_residual_rad,
    diagnostics.conditioned_max_yaw_residual_rad);
  diagnostics.conditioned_horizon_state_json = horizonJson(horizon);
  // Retain the legacy key as the actual controller input for existing
  // evidence collectors.
  diagnostics.horizon_state_json = diagnostics.conditioned_horizon_state_json;
  {
    bool all_finite = true;
    double max_abs_vx = 0.0, max_abs_vy = 0.0, max_abs_wz = 0.0;
    double max_yaw_step = 0.0;
    for (std::size_t i = 0; i < horizon.size(); ++i) {
      const auto & sample = horizon[i];
      all_finite = all_finite && finite(sample.x_ref) && finite(sample.y_ref) &&
        finite(sample.yaw_ref) && finite(sample.vx_world_ref) &&
        finite(sample.vy_world_ref) && finite(sample.omega_ref);
      max_abs_vx = std::max(max_abs_vx, std::abs(sample.vx_body_ref));
      max_abs_vy = std::max(max_abs_vy, std::abs(sample.vy_body_ref));
      max_abs_wz = std::max(max_abs_wz, std::abs(sample.omega_ref));
      if (i) {max_yaw_step = std::max(max_yaw_step, std::abs(angleDelta(
            horizon[i - 1].yaw_ref, sample.yaw_ref)));}
    }
    const bool limits_ok = max_abs_vx <= std::max(
      std::abs(settings.constraints.vx_min), std::abs(settings.constraints.vx_max)) &&
      max_abs_vy <= settings.constraints.vy && max_abs_wz <= settings.constraints.wz;
    std::ostringstream check;
    check << "{\"reference_size\":" << horizon.size() << ",\"model_dt\":" << settings.model_dt
          << ",\"horizon_duration\":" << (horizon.empty() ? 0.0 : (horizon.size() - 1) * settings.model_dt)
          << ",\"all_finite\":" << (all_finite ? "true" : "false")
          << ",\"max_abs_vx_body\":" << max_abs_vx << ",\"max_abs_vy_body\":" << max_abs_vy
          << ",\"max_abs_wz\":" << max_abs_wz << ",\"max_yaw_step_rad\":" << max_yaw_step
          << ",\"velocity_limits_ok\":" << (limits_ok ? "true" : "false")
          << ",\"comparison_frame\":\"" << target_frame << "\"}";
    diagnostics.conditioning_input_check_json = check.str();
    RCLCPP_INFO_ONCE(controller_->logger_, "Restricted MPPI conditioned input: %s",
      diagnostics.conditioning_input_check_json.c_str());
  }
  diagnostics.mppi_limits_json =
    "{\"max_vx\":0.5,\"max_vy\":0.5,\"max_wz\":0.8,\"max_accel_x\":0.6,\"max_accel_y\":0.6,\"max_accel_wz\":0.7}";
  diagnostics.phase_tracking_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - phase_started).count();

  geometry_msgs::msg::PoseStamped optimizer_pose = robot_pose;
  if (optimizer_pose.header.frame_id != target_frame) {
    try {
      geometry_msgs::msg::PoseStamped transformed_pose;
      controller_->tf_buffer_->transform(
        optimizer_pose, transformed_pose, target_frame,
        tf2::durationFromSec(0.2));
      optimizer_pose = transformed_pose;
      optimizer_pose.header.frame_id = target_frame;
    } catch (const tf2::TransformException &) {
      diagnostics.status = "NO_FEASIBLE_CONTROL";
      return zeroCommand(transformed_plan.header.stamp);
    }
  }
  diagnostics.robot_x = optimizer_pose.pose.position.x;
  diagnostics.robot_y = optimizer_pose.pose.position.y;
  diagnostics.robot_yaw = tf2::getYaw(optimizer_pose.pose.orientation);
  diagnostics.robot_vx = robot_speed.linear.x;
  diagnostics.robot_vy = robot_speed.linear.y;
  diagnostics.robot_wz = robot_speed.angular.z;
  if (reference->metadata().frame_id == target_frame) {
    diagnostics.initial_reference_error_m = std::hypot(
      diagnostics.robot_x - diagnostics.reference_start_x,
      diagnostics.robot_y - diagnostics.reference_start_y);
  }

  diagnostics.rollout_count = settings.batch_size;
  diagnostics.batch_size = settings.batch_size;
  diagnostics.time_steps = settings.time_steps;
  diagnostics.model_dt = settings.model_dt;
  const RestrictedAbsoluteBounds absolute_bounds{
    settings.constraints.vx_min, settings.constraints.vx_max,
    settings.constraints.vy, settings.constraints.wz};
  const RestrictedCorrectionBounds correction_bounds{
    config.delta_vx_max, config.delta_vy_max, config.delta_wz_max};

  optimizer.prepare(optimizer_pose, robot_speed, transformed_plan, goal_checker);
  control_sequence_before_reset_vx_ = optimizer.control_sequence_.vx.data();
  control_sequence_before_reset_vy_ = optimizer.control_sequence_.vy.data();
  control_sequence_before_reset_wz_ = optimizer.control_sequence_.wz.data();
  // Optimizer::reset() owns this storage lifecycle.  The stock compute path
  // preserves the allocated sequence between prepare() and its internal
  // update; replacing the tensors here invalidates internal xtensor storage
  // assumptions even though the replacement has the same shape.  The nominal
  // center below overwrites every element in-place.
  if (optimizer.control_sequence_.vx.size() != settings.time_steps ||
    optimizer.control_sequence_.vy.size() != settings.time_steps ||
    optimizer.control_sequence_.wz.size() != settings.time_steps)
  {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(transformed_plan.header.stamp);
  }
  std::vector<RestrictedControl> nominal(settings.time_steps);
  bool reference_control_intersection_valid = true;
  for (std::size_t index = 0; index < settings.time_steps; ++index) {
    const auto & sample = horizon[index];
    nominal[index] = {sample.vx_body_ref, sample.vy_body_ref, sample.omega_ref};
    reference_control_intersection_valid = reference_control_intersection_valid &&
      finite(nominal[index].vx) && finite(nominal[index].vy) && finite(nominal[index].wz);
    bool clamped = false;
    const auto vx = clampScalar(
      nominal[index].vx, absolute_bounds.vx_min, absolute_bounds.vx_max, clamped);
    const auto vy = clampScalar(
      nominal[index].vy, -absolute_bounds.vy_max, absolute_bounds.vy_max, clamped);
    const auto wz = clampScalar(
      nominal[index].wz, -absolute_bounds.wz_max, absolute_bounds.wz_max, clamped);
    if (!finite(vx) || !finite(vy) || !finite(wz)) {
      reference_control_intersection_valid = false;
    }
    // Keep the actual nominal center in the absolute envelope.  Every
    // generated value is subsequently intersected with this envelope and the
    // configured correction corridor.
    optimizer.control_sequence_.vx(index) = finite(vx) ? static_cast<float>(vx) : 0.0F;
    optimizer.control_sequence_.vy(index) = finite(vy) ? static_cast<float>(vy) : 0.0F;
    optimizer.control_sequence_.wz(index) = finite(wz) ? static_cast<float>(wz) : 0.0F;
  }

  const auto history_center = nominal.empty() ? RestrictedControl{} : nominal.front();
  for (auto & history : controller_->optimizer_.control_history_) {
    history = {
      static_cast<float>(history_center.vx), static_cast<float>(history_center.vy),
      static_cast<float>(history_center.wz)};
  }

  std::size_t iterations = settings.iteration_count;
  if (iterations == 0) {
    iterations = 1;
  }
  for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
    // This is the official generateNoisedTrajectories sequence expanded at
    // the two required seams.  Noise remains Gaussian with the official
    // baseline std, but is interpreted as correction noise and hard-clamped
    // before state propagation.
    const auto sampling_started = std::chrono::steady_clock::now();
    optimizer.noise_generator_.setNoisedControls(
      optimizer.state_, optimizer.control_sequence_);
    optimizer.noise_generator_.generateNextNoises();
    diagnostics.sampling_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - sampling_started).count();

    bool all_control_corridors_valid = reference_control_intersection_valid;
    const auto bounds_started = std::chrono::steady_clock::now();
    for (std::size_t batch = 0; batch < settings.batch_size; ++batch) {
      bool sample_clamped = false;
      for (std::size_t index = 0; index < settings.time_steps; ++index) {
        const RestrictedControl center = nominal[index];
        // Each axis has a different nominal and absolute envelope.
        bool clamped = false;
        const double vx_lower = std::max(
          absolute_bounds.vx_min, center.vx - correction_bounds.delta_vx_max);
        const double vx_upper = std::min(
          absolute_bounds.vx_max, center.vx + correction_bounds.delta_vx_max);
        const double vy_lower = std::max(
          -absolute_bounds.vy_max, center.vy - correction_bounds.delta_vy_max);
        const double vy_upper = std::min(
          absolute_bounds.vy_max, center.vy + correction_bounds.delta_vy_max);
        const double wz_lower = std::max(
          -absolute_bounds.wz_max, center.wz - correction_bounds.delta_wz_max);
        const double wz_upper = std::min(
          absolute_bounds.wz_max, center.wz + correction_bounds.delta_wz_max);
        if (vx_lower > vx_upper || vy_lower > vy_upper || wz_lower > wz_upper) {
          all_control_corridors_valid = false;
        }
        auto & vx = optimizer.state_.cvx(batch, index);
        auto & vy = optimizer.state_.cvy(batch, index);
        auto & wz = optimizer.state_.cwz(batch, index);
        const double proposed_vx = vx, proposed_vy = vy, proposed_wz = wz;
        const auto bounded_vx = clampScalar(vx, vx_lower, vx_upper, clamped);
        const auto bounded_vy = clampScalar(vy, vy_lower, vy_upper, clamped);
        const auto bounded_wz = clampScalar(wz, wz_lower, wz_upper, clamped);
        diagnostics.proposed_scalar_control_count += 3;
        ++diagnostics.proposed_sample_step_count;
        const auto record_scalar = [&diagnostics](double proposed, double bounded,
            double lower, double upper, std::size_t & axis_count) {
            if (proposed == bounded) {return;}
            ++diagnostics.clamped_scalar_control_count;
            ++diagnostics.velocity_bound_clamp_count;
            ++axis_count;
            if (proposed < lower) {++diagnostics.lower_bound_clamp_count;}
            if (proposed > upper) {++diagnostics.upper_bound_clamp_count;}
          };
        record_scalar(proposed_vx, bounded_vx, vx_lower, vx_upper, diagnostics.vx_clamp_count);
        record_scalar(proposed_vy, bounded_vy, vy_lower, vy_upper, diagnostics.vy_clamp_count);
        record_scalar(proposed_wz, bounded_wz, wz_lower, wz_upper, diagnostics.wz_clamp_count);
        if (clamped) {
          ++diagnostics.control_bound_clamped_count;
          ++diagnostics.clamped_sample_step_count;
          sample_clamped = true;
        }
        vx = finite(bounded_vx) ? static_cast<float>(bounded_vx) : 0.0F;
        vy = finite(bounded_vy) ? static_cast<float>(bounded_vy) : 0.0F;
        wz = finite(bounded_wz) ? static_cast<float>(bounded_wz) : 0.0F;
      }
      ++diagnostics.proposed_sample_count;
      if (sample_clamped) {++diagnostics.clamped_sample_count;}
    }
    diagnostics.control_bounds_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - bounds_started).count();

    const auto rollout_started = std::chrono::steady_clock::now();
    optimizer.updateStateVelocities(optimizer.state_);
    optimizer.integrateStateVelocities(
      optimizer.generated_trajectories_, optimizer.state_);
    diagnostics.rollout0_x = optimizer.generated_trajectories_.x(0, 0);
    diagnostics.rollout0_y = optimizer.generated_trajectories_.y(0, 0);
    diagnostics.rollout0_yaw = optimizer.generated_trajectories_.yaws(0, 0);
    diagnostics.tube0_reference_x = horizon[0].x_ref;
    diagnostics.tube0_reference_y = horizon[0].y_ref;
    diagnostics.tube0_deviation_m = std::hypot(
      diagnostics.rollout0_x - diagnostics.tube0_reference_x,
      diagnostics.rollout0_y - diagnostics.tube0_reference_y);
    if (iteration == 0) {
      std::ostringstream rollout;
      rollout << "[";
      for (std::size_t i = 0; i < settings.time_steps; ++i) {
        if (i) {rollout << ",";}
        rollout << "{\"i\":" << i << ",\"x\":" << optimizer.generated_trajectories_.x(0, i)
                << ",\"y\":" << optimizer.generated_trajectories_.y(0, i)
                << ",\"yaw\":" << optimizer.generated_trajectories_.yaws(0, i) << "}";
      }
      rollout << "]"; diagnostics.first_rollout_json = rollout.str();
      std::ostringstream deviation;
      deviation << "{\"frame\":\"" << target_frame << "\",\"formula\":\"hypot(rollout.x-reference.x,rollout.y-reference.y)\",\"horizon0_x\":" << horizon[0].x_ref << ",\"horizon0_y\":" << horizon[0].y_ref << "}";
      diagnostics.deviation_inputs_json = deviation.str();
    }
    diagnostics.rollout_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rollout_started).count();

    const auto critic_started = std::chrono::steady_clock::now();
    optimizer.critic_manager_.evalTrajectoriesScores(optimizer.critics_data_);
    diagnostics.critic_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - critic_started).count();
    diagnostics.feasible_sample_count = 0;
    diagnostics.tube_rejected_count = 0;
    diagnostics.collision_rejected_count = 0;
    diagnostics.maximum_accepted_tube_deviation = 0.0;
    diagnostics.maximum_generated_tube_deviation = 0.0;
    std::vector<std::string> collision_evidence;
    bool any_feasible = false;
    for (std::size_t batch = 0; batch < settings.batch_size; ++batch) {
      bool tube_feasible = all_control_corridors_valid;
      bool collision_free = true;
      bool first_collision_recorded = false;
      double maximum_deviation = 0.0;
      for (std::size_t index = 0; index < settings.time_steps; ++index) {
        const auto tube_step_started = std::chrono::steady_clock::now();
        const double dx = optimizer.generated_trajectories_.x(batch, index) - horizon[index].x_ref;
        const double dy = optimizer.generated_trajectories_.y(batch, index) - horizon[index].y_ref;
        const double deviation = std::hypot(dx, dy);
        maximum_deviation = std::max(maximum_deviation, deviation);
        if (!finite(deviation) || deviation > config.position_tube_radius) {
          if (!diagnostics.first_tube_breach_captured) {
            diagnostics.first_tube_breach_captured = true;
            diagnostics.first_tube_breach_batch = static_cast<uint32_t>(batch);
            diagnostics.first_tube_breach_step = static_cast<uint32_t>(index);
            diagnostics.first_tube_breach_deviation_m = deviation;
            diagnostics.first_tube_breach_rollout_x = optimizer.generated_trajectories_.x(batch, index);
            diagnostics.first_tube_breach_rollout_y = optimizer.generated_trajectories_.y(batch, index);
            diagnostics.first_tube_breach_reference_x = horizon[index].x_ref;
            diagnostics.first_tube_breach_reference_y = horizon[index].y_ref;
          }
          tube_feasible = false;
        }
        const auto collision_started = std::chrono::steady_clock::now();
        diagnostics.tube_filter_ms += std::chrono::duration<double, std::milli>(
          collision_started - tube_step_started).count();
        std::string collision_source;
        std::string collision_detail;
        if (collisionAtPose(
            optimizer.generated_trajectories_.x(batch, index),
            optimizer.generated_trajectories_.y(batch, index),
            optimizer.generated_trajectories_.yaws(batch, index),
            config.consider_footprint, &collision_source, &collision_detail))
        {
          collision_free = false;
          if (!first_collision_recorded) {
            std::ostringstream event;
            event << "{\"batch\":" << batch << ",\"first_collision_step\":" << index
                  << ",\"x\":" << optimizer.generated_trajectories_.x(batch, index)
                  << ",\"y\":" << optimizer.generated_trajectories_.y(batch, index)
                  << ",\"source\":\"" << collision_source
                  << "\",\"minimum_obstacle_distance_m\":null,\"footprint_evidence\":"
                  << (collision_detail.empty() ? "null" : collision_detail) << "}";
            collision_evidence.push_back(event.str());
            first_collision_recorded = true;
          }
        }
        diagnostics.collision_filter_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - collision_started).count();
      }
      diagnostics.maximum_generated_tube_deviation = std::max(
        diagnostics.maximum_generated_tube_deviation, maximum_deviation);
      if (!tube_feasible) {
        ++diagnostics.tube_rejected_count;
      }
      if (!collision_free) {
        ++diagnostics.collision_rejected_count;
      }
      const bool feasible = tube_feasible && collision_free &&
        finite(optimizer.costs_(batch));
      if (!feasible) {
        optimizer.costs_(batch) = std::numeric_limits<float>::infinity();
      } else {
        any_feasible = true;
        ++diagnostics.feasible_sample_count;
        diagnostics.maximum_accepted_tube_deviation = std::max(
          diagnostics.maximum_accepted_tube_deviation, maximum_deviation);
      }
    }
    diagnostics.feasible_sample_ratio = settings.batch_size ?
      static_cast<double>(diagnostics.feasible_sample_count) / settings.batch_size : 0.0;
    if (diagnostics.collision_rejected_count == settings.batch_size) {
      diagnostics.collision_full_mask = true;
      diagnostics.collision_full_mask_stamp_ns = controller_->clock_->now().nanoseconds();
      std::ostringstream events;
      events << "[";
      for (std::size_t i = 0; i < collision_evidence.size(); ++i) {
        if (i) {events << ",";} events << collision_evidence[i];
      }
      events << "]";
      diagnostics.collision_rollout_evidence_json = events.str();
      std::ostringstream map;
      map << "{\"frame\":\"" << target_frame << "\",\"resolution\":"
          << costmap->getResolution() << ",\"width\":" << costmap->getSizeInCellsX()
          << ",\"height\":" << costmap->getSizeInCellsY() << ",\"origin_x\":"
          << costmap->getOriginX() << ",\"origin_y\":" << costmap->getOriginY()
          << ",\"data\":[";
      const auto size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
      const auto * data = costmap->getCharMap();
      for (unsigned int i = 0; i < size; ++i) {if (i) {map << ",";} map << static_cast<unsigned int>(data[i]);}
      map << "]}";
      diagnostics.collision_costmap_json = map.str();
    }

    // +infinity is an explicit infeasible mask.  It is not a large soft
    // penalty, and the all-infeasible case is handled before the official
    // softmax update to avoid NaN normalization.
    if (!any_feasible) {
      break;
    }
    if (!optimizer_input_validation_logged_) {
      const auto cvx = countTensorValues(optimizer.state_.cvx);
      const auto cvy = countTensorValues(optimizer.state_.cvy);
      const auto cwz = countTensorValues(optimizer.state_.cwz);
      const auto controls_vx = countTensorValues(optimizer.control_sequence_.vx);
      const auto controls_vy = countTensorValues(optimizer.control_sequence_.vy);
      const auto controls_wz = countTensorValues(optimizer.control_sequence_.wz);
      const auto costs = countTensorValues(optimizer.costs_);
      const auto noises_vx = countTensorValues(optimizer.noise_generator_.noises_vx_);
      const auto noises_vy = countTensorValues(optimizer.noise_generator_.noises_vy_);
      const auto noises_wz = countTensorValues(optimizer.noise_generator_.noises_wz_);
      RCLCPP_INFO(
        controller_->logger_,
        "Before updateControlSequence: optimizer=%s initialized=%s batch=%u steps=%u "
        "control_dim=%u state_shape=%zux%zu/%zux%zu/%zux%zu "
        "control_sequence_shape=%zu/%zu/%zu reference_shape=%zu "
        "state_finite=%zu,%zu,%zu state_nan=%zu,%zu,%zu state_inf=%zu,%zu,%zu "
        "control_finite=%zu,%zu,%zu control_nan=%zu,%zu,%zu control_inf=%zu,%zu,%zu "
        "cost_finite=%zu cost_nan=%zu cost_inf=%zu",
        controller_ ? "true" : "false",
        optimizer.motion_model_ && optimizer.costs_.shape(0) == settings.batch_size ? "true" : "false",
        settings.batch_size, settings.time_steps, optimizer.isHolonomic() ? 3U : 2U,
        optimizer.state_.cvx.shape(0), optimizer.state_.cvx.shape(1),
        optimizer.state_.cvy.shape(0), optimizer.state_.cvy.shape(1),
        optimizer.state_.cwz.shape(0), optimizer.state_.cwz.shape(1),
        optimizer.control_sequence_.vx.shape(0), optimizer.control_sequence_.vy.shape(0),
        optimizer.control_sequence_.wz.shape(0), horizon.size(),
        cvx.finite, cvy.finite, cwz.finite, cvx.nan, cvy.nan, cwz.nan,
        cvx.infinity, cvy.infinity, cwz.infinity,
        controls_vx.finite, controls_vy.finite, controls_wz.finite,
        controls_vx.nan, controls_vy.nan, controls_wz.nan,
        controls_vx.infinity, controls_vy.infinity, controls_wz.infinity,
        costs.finite, costs.nan, costs.infinity);
      RCLCPP_INFO(
        controller_->logger_,
        "Control sequence storage: pre_reset=%p/%p/%p current=%p/%p/%p "
        "size=%zu/%zu/%zu capacity=%zu/%zu/%zu",
        static_cast<const void *>(control_sequence_before_reset_vx_),
        static_cast<const void *>(control_sequence_before_reset_vy_),
        static_cast<const void *>(control_sequence_before_reset_wz_),
        static_cast<const void *>(optimizer.control_sequence_.vx.data()),
        static_cast<const void *>(optimizer.control_sequence_.vy.data()),
        static_cast<const void *>(optimizer.control_sequence_.wz.data()),
        optimizer.control_sequence_.vx.size(), optimizer.control_sequence_.vy.size(),
        optimizer.control_sequence_.wz.size(), optimizer.control_sequence_.vx.storage().capacity(),
        optimizer.control_sequence_.vy.storage().capacity(), optimizer.control_sequence_.wz.storage().capacity());
      RCLCPP_INFO(
        controller_->logger_,
        "Optimizer internals: optimizer=%p model=%p critics=%zu trajectories=%p/%p/%p "
        "noise=%p/%p/%p noise_shape=%zux%zu/%zux%zu/%zux%zu "
        "noise_finite=%zu,%zu,%zu noise_nan=%zu,%zu,%zu noise_inf=%zu,%zu,%zu",
        static_cast<void *>(&optimizer), static_cast<void *>(optimizer.motion_model_.get()),
        optimizer.critic_manager_.critics_.size(),
        static_cast<void *>(optimizer.generated_trajectories_.x.data()),
        static_cast<void *>(optimizer.generated_trajectories_.y.data()),
        static_cast<void *>(optimizer.generated_trajectories_.yaws.data()),
        static_cast<void *>(optimizer.noise_generator_.noises_vx_.data()),
        static_cast<void *>(optimizer.noise_generator_.noises_vy_.data()),
        static_cast<void *>(optimizer.noise_generator_.noises_wz_.data()),
        optimizer.noise_generator_.noises_vx_.shape(0), optimizer.noise_generator_.noises_vx_.shape(1),
        optimizer.noise_generator_.noises_vy_.shape(0), optimizer.noise_generator_.noises_vy_.shape(1),
        optimizer.noise_generator_.noises_wz_.shape(0), optimizer.noise_generator_.noises_wz_.shape(1),
        noises_vx.finite, noises_vy.finite, noises_wz.finite,
        noises_vx.nan, noises_vy.nan, noises_wz.nan,
        noises_vx.infinity, noises_vy.infinity, noises_wz.infinity);
      optimizer_input_validation_logged_ = true;
    }
    RCLCPP_INFO_ONCE(controller_->logger_, "Restricted MPPI first compute updateControlSequence begin");
    optimizer.updateControlSequence();
    RCLCPP_INFO_ONCE(controller_->logger_, "Restricted MPPI first compute updateControlSequence end");

    // The official update is retained, but the final nominal sequence is
    // intersected with the same hard correction corridor before it can be
    // emitted or shifted.
    for (std::size_t index = 0; index < settings.time_steps; ++index) {
      bool clamped = false;
      const double vx_lower = std::max(
        absolute_bounds.vx_min, nominal[index].vx - correction_bounds.delta_vx_max);
      const double vx_upper = std::min(
        absolute_bounds.vx_max, nominal[index].vx + correction_bounds.delta_vx_max);
      const double vy_lower = std::max(
        -absolute_bounds.vy_max, nominal[index].vy - correction_bounds.delta_vy_max);
      const double vy_upper = std::min(
        absolute_bounds.vy_max, nominal[index].vy + correction_bounds.delta_vy_max);
      const double wz_lower = std::max(
        -absolute_bounds.wz_max, nominal[index].wz - correction_bounds.delta_wz_max);
      const double wz_upper = std::min(
        absolute_bounds.wz_max, nominal[index].wz + correction_bounds.delta_wz_max);
      optimizer.control_sequence_.vx(index) = static_cast<float>(clampScalar(
          optimizer.control_sequence_.vx(index), vx_lower, vx_upper, clamped));
      optimizer.control_sequence_.vy(index) = static_cast<float>(clampScalar(
          optimizer.control_sequence_.vy(index), vy_lower, vy_upper, clamped));
      optimizer.control_sequence_.wz(index) = static_cast<float>(clampScalar(
          optimizer.control_sequence_.wz(index), wz_lower, wz_upper, clamped));
      if (clamped) {
        ++diagnostics.control_bound_clamped_count;
      }
    }
  }

  if (diagnostics.feasible_sample_count == 0 || !reference_control_intersection_valid) {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(transformed_plan.header.stamp);
  }

  // Retain official output smoothing, with a reference-centered history. It
  // cannot leak an old absolute route because the history is reseeded above.
  const auto finalization_started = std::chrono::steady_clock::now();
  mppi::utils::savitskyGolayFilter(
    optimizer.control_sequence_, optimizer.control_history_, optimizer.settings_);
  for (std::size_t index = 0; index < settings.time_steps; ++index) {
    bool clamped = false;
    const double vx_lower = std::max(
      absolute_bounds.vx_min, nominal[index].vx - correction_bounds.delta_vx_max);
    const double vx_upper = std::min(
      absolute_bounds.vx_max, nominal[index].vx + correction_bounds.delta_vx_max);
    const double vy_lower = std::max(
      -absolute_bounds.vy_max, nominal[index].vy - correction_bounds.delta_vy_max);
    const double vy_upper = std::min(
      absolute_bounds.vy_max, nominal[index].vy + correction_bounds.delta_vy_max);
    const double wz_lower = std::max(
      -absolute_bounds.wz_max, nominal[index].wz - correction_bounds.delta_wz_max);
    const double wz_upper = std::min(
      absolute_bounds.wz_max, nominal[index].wz + correction_bounds.delta_wz_max);
    optimizer.control_sequence_.vx(index) = static_cast<float>(clampScalar(
        optimizer.control_sequence_.vx(index), vx_lower, vx_upper, clamped));
    optimizer.control_sequence_.vy(index) = static_cast<float>(clampScalar(
        optimizer.control_sequence_.vy(index), vy_lower, vy_upper, clamped));
    optimizer.control_sequence_.wz(index) = static_cast<float>(clampScalar(
        optimizer.control_sequence_.wz(index), wz_lower, wz_upper, clamped));
    if (clamped) {
      ++diagnostics.control_bound_clamped_count;
    }
  }

  const std::size_t output_index = settings.shift_control_sequence ? 1U : 0U;
  if (output_index >= settings.time_steps) {
    diagnostics.status = "NO_FEASIBLE_CONTROL";
    return zeroCommand(transformed_plan.header.stamp);
  }
  const auto command = optimizer.getControlFromSequenceAsTwist(transformed_plan.header.stamp);
  diagnostics.nominal_command = nominal[output_index];
  diagnostics.final_command = {
    command.twist.linear.x, command.twist.linear.y, command.twist.angular.z};
  diagnostics.selected_correction = {
    diagnostics.final_command.vx - diagnostics.nominal_command.vx,
    diagnostics.final_command.vy - diagnostics.nominal_command.vy,
    diagnostics.final_command.wz - diagnostics.nominal_command.wz};
  diagnostics.status = diagnostics.reference_switched ? "REFERENCE_SWITCH" : "TRACKING";

  // Match the stock controller's post-command horizon shift. The next tick
  // reseeds this sequence from the current reference, so this is continuity
  // bookkeeping only and never becomes a new topology source.
  if (settings.shift_control_sequence) {
    optimizer.shiftControlSequence();
  }
  diagnostics.command_finalization_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - finalization_started).count();
  diagnostics.restricted_mppi_total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - cycle_started).count();
  diagnostics.controller_total_ms = diagnostics.restricted_mppi_total_ms;
  return command;
}

}  // namespace m1_local_fast2d
