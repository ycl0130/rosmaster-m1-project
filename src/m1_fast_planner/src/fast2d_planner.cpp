#include "m1_fast_planner/fast2d_planner.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "m1_fast_planner/kinodynamic_astar.hpp"
#include "m1_fast_planner/straight_line.hpp"

namespace m1_fast_planner
{
namespace
{

geometry_msgs::msg::Quaternion normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & input, double fallback_yaw)
{
  const double norm_squared = input.x * input.x + input.y * input.y +
    input.z * input.z + input.w * input.w;
  if (std::isfinite(norm_squared) && norm_squared > 1e-12) {
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    geometry_msgs::msg::Quaternion output;
    output.x = input.x * inverse_norm;
    output.y = input.y * inverse_norm;
    output.z = input.z * inverse_norm;
    output.w = input.w * inverse_norm;
    return output;
  }
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(fallback_yaw * 0.5);
  output.w = std::cos(fallback_yaw * 0.5);
  return output;
}

}  // namespace

void Fast2DPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node || !costmap_ros_) {
    throw nav2_core::PlannerException("Fast2DPlanner requires a lifecycle node and global costmap");
  }
  logger_ = node->get_logger();
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  if (!costmap_) {
    throw nav2_core::PlannerException("Fast2DPlanner received a null global costmap");
  }

  const std::string resolution_parameter = name_ + ".path_resolution";
  const std::string unknown_parameter = name_ + ".allow_unknown";
  if (!node->has_parameter(resolution_parameter)) {
    node->declare_parameter(resolution_parameter, path_resolution_);
  }
  if (!node->has_parameter(unknown_parameter)) {
    node->declare_parameter(unknown_parameter, allow_unknown_);
  }
  node->get_parameter(resolution_parameter, path_resolution_);
  node->get_parameter(unknown_parameter, allow_unknown_);
  const auto declare_double = [&node, this](const std::string & suffix, double & value) {
      const auto parameter = name_ + "." + suffix;
      if (!node->has_parameter(parameter)) {
        node->declare_parameter(parameter, value);
      }
      node->get_parameter(parameter, value);
    };
  declare_double("max_velocity_x", search_config_.max_velocity_x);
  declare_double("max_velocity_y", search_config_.max_velocity_y);
  declare_double("max_accel_x", search_config_.max_accel_x);
  declare_double("max_accel_y", search_config_.max_accel_y);
  declare_double("primitive_duration", search_config_.primitive_duration);
  declare_double("collision_check_dt", search_config_.collision_check_dt);
  declare_double("collision_check_distance", search_config_.collision_check_distance);
  declare_double("position_resolution", search_config_.position_resolution);
  declare_double("velocity_resolution", search_config_.velocity_resolution);
  declare_double("goal_position_tolerance", search_config_.goal_position_tolerance);
  declare_double("goal_velocity_tolerance", search_config_.goal_velocity_tolerance);
  declare_double("time_cost_weight", search_config_.time_cost_weight);
  declare_double("control_cost_weight", search_config_.control_cost_weight);
  search_config_.path_resolution = path_resolution_;
  const auto timeout_parameter = name_ + ".search_timeout_ms";
  const auto expansions_parameter = name_ + ".max_expansions";
  if (!node->has_parameter(timeout_parameter)) {
    node->declare_parameter(timeout_parameter, search_config_.search_timeout_ms);
  }
  if (!node->has_parameter(expansions_parameter)) {
    node->declare_parameter(expansions_parameter, search_config_.max_expansions);
  }
  node->get_parameter(timeout_parameter, search_config_.search_timeout_ms);
  node->get_parameter(expansions_parameter, search_config_.max_expansions);
  const auto odom_topic_parameter = name_ + ".odom_topic";
  const auto odom_timeout_parameter = name_ + ".odom_timeout";
  if (!node->has_parameter(odom_topic_parameter)) {
    node->declare_parameter(odom_topic_parameter, odom_topic_);
  }
  if (!node->has_parameter(odom_timeout_parameter)) {
    node->declare_parameter(odom_timeout_parameter, odom_timeout_);
  }
  node->get_parameter(odom_topic_parameter, odom_topic_);
  node->get_parameter(odom_timeout_parameter, odom_timeout_);
  if (!std::isfinite(path_resolution_) || path_resolution_ <= 0.0) {
    throw nav2_core::PlannerException("Fast2DPlanner path_resolution must be finite and positive");
  }
  if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
    !KinodynamicAstar::validConfig(search_config_))
  {
    throw nav2_core::PlannerException("Fast2DPlanner has invalid kinodynamic search parameters");
  }
  odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10), [this, node](nav_msgs::msg::Odometry::SharedPtr message) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_odom_ = std::move(message);
      latest_odom_received_ = node->now();
    });
  RCLCPP_INFO(
    logger_, "Configured Fast2DPlanner kinodynamic A*: resolution=%.3f v=(%.2f,%.2f) a=(%.2f,%.2f)",
    path_resolution_, search_config_.max_velocity_x, search_config_.max_velocity_y,
    search_config_.max_accel_x, search_config_.max_accel_y);
}

void Fast2DPlanner::cleanup()
{
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
  odom_subscription_.reset();
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_.reset();
  active_ = false;
}

void Fast2DPlanner::activate()
{
  active_ = true;
}

void Fast2DPlanner::deactivate()
{
  active_ = false;
}

nav_msgs::msg::Path Fast2DPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto planning_start = std::chrono::steady_clock::now();
  if (!active_ || !costmap_) {
    throw nav2_core::PlannerException("Fast2DPlanner is not active or configured");
  }
  if (!framesMatch(start.header.frame_id, goal.header.frame_id, global_frame_)) {
    throw nav2_core::PlannerException(
            "Fast2DPlanner requires start and goal in global costmap frame '" + global_frame_ + "'");
  }

  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Fast2DPlanner lifecycle node expired");
  }
  PlanarState start_state{start.pose.position.x, start.pose.position.y, 0.0, 0.0};
  bool zero_velocity_fallback = true;
  nav_msgs::msg::Odometry::SharedPtr odom;
  rclcpp::Time odom_received(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = latest_odom_;
    odom_received = latest_odom_received_;
  }
  if (odom && (node->now() - odom_received).seconds() <= odom_timeout_) {
    try {
      const auto transform = tf_->lookupTransform(
        global_frame_, odom->child_frame_id, rclcpp::Time(odom->header.stamp), rclcpp::Duration::from_seconds(0.05));
      start_state = rotateBodyVelocityToPlanningFrame(
        odom->twist.twist.linear.x, odom->twist.twist.linear.y,
        tf2::getYaw(transform.transform.rotation));
      start_state.px = start.pose.position.x;
      start_state.py = start.pose.position.y;
      // Do not alter a measured state merely because it is transiently above
      // the planning envelope.  KinodynamicAstar admits braking-only motion
      // for such a component and restores the regular bound once recovered.
      zero_velocity_fallback = false;
    } catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "Fast2DPlanner using zero start velocity: odom TF failed: %s", error.what());
    }
  } else {
    RCLCPP_WARN(logger_, "Fast2DPlanner using zero start velocity: odom missing or stale");
  }
  const PlanarState goal_state{goal.pose.position.x, goal.pose.position.y, 0.0, 0.0};
  const auto collision_free = [this](double x, double y) {
      unsigned int map_x = 0;
      unsigned int map_y = 0;
      return costmap_->worldToMap(x, y, map_x, map_y) &&
             isTraversableCost(costmap_->getCost(map_x, map_y), allow_unknown_);
    };
  SearchResult result;
  try {
    result = KinodynamicAstar(search_config_).search(start_state, goal_state, collision_free);
  } catch (const std::exception & error) {
    throw nav2_core::PlannerException(std::string("Fast2DPlanner kinodynamic search error: ") + error.what());
  }
  if (!result.telemetry.success || result.trajectory.empty()) {
    throw nav2_core::PlannerException(
            result.telemetry.timeout ? "Fast2DPlanner kinodynamic search timeout" :
            "Fast2DPlanner kinodynamic search found no collision-free trajectory");
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  path.poses.reserve(result.trajectory.size());
  double previous_yaw = straightLineYaw(start.pose.position, goal.pose.position);
  for (std::size_t index = 0; index < result.trajectory.size(); ++index) {
    const auto & sample = result.trajectory[index].state;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sample.px;
    pose.pose.position.y = sample.py;
    if (index + 1 < result.trajectory.size()) {
      const auto & next = result.trajectory[index + 1].state;
      if (std::hypot(next.px - sample.px, next.py - sample.py) > 1e-6) {
        previous_yaw = std::atan2(next.py - sample.py, next.px - sample.px);
      }
    }
    pose.pose.orientation.z = std::sin(previous_yaw * 0.5);
    pose.pose.orientation.w = std::cos(previous_yaw * 0.5);
    path.poses.push_back(pose);
  }
  path.poses.back().pose.orientation = normalizedQuaternion(goal.pose.orientation, previous_yaw);
  const auto planning_end = std::chrono::steady_clock::now();
  double plan_length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    plan_length += std::hypot(path.poses[index].pose.position.x - path.poses[index - 1].pose.position.x,
      path.poses[index].pose.position.y - path.poses[index - 1].pose.position.y);
  }
  const double planning_time_ms =
    std::chrono::duration<double, std::milli>(planning_end - planning_start).count();
  RCLCPP_INFO(
    logger_, "Fast2DPlanner kinodynamic: success=true points=%zu length=%.3f m planning_time=%.3f ms "
    "expanded=%zu generated=%zu duration=%.3f max_speed=%.3f max_accel=%.3f "
    "max_v=(%.3f,%.3f) max_a=(%.3f,%.3f) "
    "start_v=(%.3f,%.3f) zero_start_v=%s goal_error=%.3f goal_speed=%.3f",
    path.poses.size(), plan_length, planning_time_ms, result.telemetry.expanded_nodes,
    result.telemetry.generated_nodes, result.telemetry.trajectory_duration, result.telemetry.max_speed,
    result.telemetry.max_acceleration, result.telemetry.max_abs_vx, result.telemetry.max_abs_vy,
    result.telemetry.max_abs_ax, result.telemetry.max_abs_ay, start_state.vx, start_state.vy,
    zero_velocity_fallback ? "true" : "false", result.telemetry.goal_position_error,
    result.telemetry.goal_speed);
  return path;
}

PlanarState Fast2DPlanner::rotateBodyVelocityToPlanningFrame(
  double body_vx, double body_vy, double yaw)
{
  return PlanarState{0.0, 0.0,
    std::cos(yaw) * body_vx - std::sin(yaw) * body_vy,
    std::sin(yaw) * body_vx + std::cos(yaw) * body_vy};
}

}  // namespace m1_fast_planner

PLUGINLIB_EXPORT_CLASS(m1_fast_planner::Fast2DPlanner, nav2_core::GlobalPlanner)
