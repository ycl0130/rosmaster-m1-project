#include "m1_local_fast2d/restricted_mppi_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
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
  float x, float y, float yaw, bool consider_footprint) const
{
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
      return true;
    }
  }
  if (static_cast<unsigned char>(point_cost) == nav2_costmap_2d::LETHAL_OBSTACLE) {
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
      return true;
    }
    if (static_cast<unsigned char>(footprint_cost) == nav2_costmap_2d::NO_INFORMATION) {
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
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
  if (controller_->clock_->now() - controller_->last_time_called_ >
    rclcpp::Duration::from_seconds(controller_->reset_period_))
  {
    controller_->reset();
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

  diagnostics.rollout_count = settings.batch_size;
  const RestrictedAbsoluteBounds absolute_bounds{
    settings.constraints.vx_min, settings.constraints.vx_max,
    settings.constraints.vy, settings.constraints.wz};
  const RestrictedCorrectionBounds correction_bounds{
    config.delta_vx_max, config.delta_vy_max, config.delta_wz_max};

  optimizer.prepare(optimizer_pose, robot_speed, transformed_plan, goal_checker);
  optimizer.control_sequence_.reset(settings.time_steps);
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
    optimizer.noise_generator_.setNoisedControls(
      optimizer.state_, optimizer.control_sequence_);
    optimizer.noise_generator_.generateNextNoises();

    bool all_control_corridors_valid = reference_control_intersection_valid;
    for (std::size_t batch = 0; batch < settings.batch_size; ++batch) {
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
        const auto bounded_vx = clampScalar(vx, vx_lower, vx_upper, clamped);
        const auto bounded_vy = clampScalar(vy, vy_lower, vy_upper, clamped);
        const auto bounded_wz = clampScalar(wz, wz_lower, wz_upper, clamped);
        if (clamped) {
          ++diagnostics.control_bound_clamped_count;
        }
        vx = finite(bounded_vx) ? static_cast<float>(bounded_vx) : 0.0F;
        vy = finite(bounded_vy) ? static_cast<float>(bounded_vy) : 0.0F;
        wz = finite(bounded_wz) ? static_cast<float>(bounded_wz) : 0.0F;
      }
    }

    optimizer.updateStateVelocities(optimizer.state_);
    optimizer.integrateStateVelocities(
      optimizer.generated_trajectories_, optimizer.state_);

    optimizer.critic_manager_.evalTrajectoriesScores(optimizer.critics_data_);
    diagnostics.feasible_sample_count = 0;
    diagnostics.tube_rejected_count = 0;
    diagnostics.collision_rejected_count = 0;
    diagnostics.maximum_accepted_tube_deviation = 0.0;
    diagnostics.maximum_generated_tube_deviation = 0.0;
    bool any_feasible = false;
    for (std::size_t batch = 0; batch < settings.batch_size; ++batch) {
      bool tube_feasible = all_control_corridors_valid;
      bool collision_free = true;
      double maximum_deviation = 0.0;
      for (std::size_t index = 0; index < settings.time_steps; ++index) {
        const double dx = optimizer.generated_trajectories_.x(batch, index) - horizon[index].x_ref;
        const double dy = optimizer.generated_trajectories_.y(batch, index) - horizon[index].y_ref;
        const double deviation = std::hypot(dx, dy);
        maximum_deviation = std::max(maximum_deviation, deviation);
        if (!finite(deviation) || deviation > config.position_tube_radius) {
          tube_feasible = false;
        }
        if (collisionAtPose(
            optimizer.generated_trajectories_.x(batch, index),
            optimizer.generated_trajectories_.y(batch, index),
            optimizer.generated_trajectories_.yaws(batch, index),
            config.consider_footprint))
        {
          collision_free = false;
        }
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

    // +infinity is an explicit infeasible mask.  It is not a large soft
    // penalty, and the all-infeasible case is handled before the official
    // softmax update to avoid NaN normalization.
    if (!any_feasible) {
      break;
    }
    optimizer.updateControlSequence();

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
  return command;
}

}  // namespace m1_local_fast2d
