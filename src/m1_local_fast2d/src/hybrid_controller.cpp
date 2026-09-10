#include "m1_local_fast2d/hybrid_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "m1_local_fast2d/local_fast2d_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace m1_local_fast2d
{
namespace
{
}  // namespace

HybridController::~HybridController() {cleanup();}

void HybridController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent; name_ = std::move(name); tf_ = std::move(tf); costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node || !costmap_ros_) {throw std::runtime_error("HybridController needs local Costmap2DROS");}
  const auto declare = [&node, this](const char * suffix, auto & value) {
      const std::string key = name_ + ".local_fast2d." + suffix;
      if (!node->has_parameter(key)) {node->declare_parameter(key, value);}
      node->get_parameter(key, value);
    };
  declare("mode", mode_); declare("update_rate", update_rate_); declare("path_stale_timeout", stale_timeout_);
  declare("lookahead_distance", lookahead_); declare("local_costmap_cost_weight", cost_weight_);
  declare("allow_unknown", allow_unknown_);
  declare("max_planning_time_ms", max_planning_time_ms_);
  declare("test_force_fallback", test_force_fallback_);
  if ((mode_ != "shadow" && mode_ != "active") || update_rate_ <= 0.0 || stale_timeout_ <= 0.0 ||
    lookahead_ <= 0.0 || cost_weight_ < 0.0 || max_planning_time_ms_ <= 0) {throw std::runtime_error("invalid local_fast2d parameters");}
  local_frame_ = costmap_ros_->getGlobalFrameID();
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/path", rclcpp::QoS(1));
  goal_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/local_goal", rclcpp::QoS(1));
  // Test-only observation of precisely the local guide forwarded to MPPI.
  accepted_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/accepted_path", rclcpp::QoS(10));
  accepted_costmap_pub_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    "/local_fast2d/accepted_costmap", rclcpp::QoS(10));
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/local_fast2d/diagnostics", rclcpp::QoS(10));
  // Keep MPPI's established namespace and parameters exactly intact.
  mppi_ = std::make_unique<nav2_mppi_controller::MPPIController>();
  mppi_->configure(parent, name_, tf_, costmap_ros_);
}

void HybridController::activate() {active_ = true; mppi_->activate(); stop_worker_ = false; worker_ = std::thread(&HybridController::workerLoop, this);}
void HybridController::deactivate() {active_ = false; {std::lock_guard<std::mutex> l(mutex_); stop_worker_ = true;} wake_.notify_all(); if (worker_.joinable()) {worker_.join();} if (mppi_) {mppi_->deactivate();}}
void HybridController::cleanup() {deactivate(); if (mppi_) {mppi_->cleanup(); mppi_.reset();} path_pub_.reset(); goal_pub_.reset(); accepted_pub_.reset(); accepted_costmap_pub_.reset(); diagnostics_pub_.reset(); costmap_ros_.reset(); tf_.reset(); node_.reset();}

void HybridController::setPlan(const nav_msgs::msg::Path & path)
{
  {std::lock_guard<std::mutex> l(mutex_); latest_input_.plan = path; ++input_generation_; latest_input_.generation = input_generation_; latest_guide_.valid = false;}
  // An original plan is always ready before the first local result.
  mppi_->setPlan(path); forwarded_generation_ = 0; wake_.notify_one();
}

geometry_msgs::msg::TwistStamped HybridController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  Guide guide; nav_msgs::msg::Path original; bool use_guide = false;
  auto node = node_.lock();
  // This default-off diagnostic hook is read at the controller boundary so a
  // test can exercise fallback without restarting or altering normal plans.
  bool force_fallback = false;
  if (node) {node->get_parameter(name_ + ".local_fast2d.test_force_fallback", force_fallback);}
  {std::lock_guard<std::mutex> l(mutex_);
    latest_input_.pose = pose; latest_input_.velocity = velocity;
    original = latest_input_.plan; guide = latest_guide_;
    use_guide = mode_ == "active" && !force_fallback && guide.valid && node &&
      (node->now() - guide.stamp).seconds() <= stale_timeout_;
    if (mode_ == "active" && !use_guide) {++fallback_count_; if (guide.valid) {++stale_count_;}}
  }
  if (use_guide && guide.generation != forwarded_generation_) {
    mppi_->setPlan(guide.mppi_plan); forwarded_generation_ = guide.generation;
    if (accepted_pub_) {accepted_pub_->publish(guide.path);}
    if (accepted_costmap_pub_) {accepted_costmap_pub_->publish(guide.snapshot);}
    publishAcceptedDiagnostic(guide);
    std::lock_guard<std::mutex> l(mutex_); ++guide_accepted_count_; reference_source_ = "local_fast2d";
  }
  if (!use_guide && forwarded_generation_ != 0) {mppi_->setPlan(original); forwarded_generation_ = 0;}
  if (!use_guide) {std::lock_guard<std::mutex> l(mutex_); reference_source_ = "global_fallback";}
  wake_.notify_one();
  return mppi_->computeVelocityCommands(pose, velocity, goal_checker);
}

void HybridController::setSpeedLimit(const double & speed_limit, const bool & percentage) {mppi_->setSpeedLimit(speed_limit, percentage);}

void HybridController::workerLoop()
{
  uint64_t seen = 0;
  while (true) {
    Input input;
    {std::unique_lock<std::mutex> l(mutex_);
      wake_.wait_for(l, std::chrono::duration<double>(1.0 / update_rate_), [this, seen] {return stop_worker_ || input_generation_ != seen;});
      if (stop_worker_) {return;} input = latest_input_; seen = input.generation;
    }
    if (!input.plan.poses.empty()) {planOnce(input);}
  }
}

void HybridController::planOnce(const Input & input)
{
  auto node = node_.lock(); if (!node || input.pose.header.frame_id.empty()) {return;}
  nav_msgs::msg::Path plan = input.plan;
  try {
    if (plan.header.frame_id != local_frame_) {
      const auto transform = tf_->lookupTransform(local_frame_, plan.header.frame_id, tf2::TimePointZero);
      for (auto & pose : plan.poses) {tf2::doTransform(pose, pose, transform); pose.header.frame_id = local_frame_;}
      plan.header.frame_id = local_frame_;
    }
  } catch (const std::exception &) {std::lock_guard<std::mutex> l(mutex_); ++no_path_count_; return;}
  geometry_msgs::msg::PoseStamped robot = input.pose;
  try {
    if (robot.header.frame_id != local_frame_) {tf2::doTransform(robot, robot, tf_->lookupTransform(local_frame_, robot.header.frame_id, tf2::TimePointZero));}
  } catch (const std::exception &) {return;}
  nav2_msgs::msg::Costmap snapshot;
  auto * master = costmap_ros_->getCostmap();
  {std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l(*master->getMutex());
    snapshot.header.frame_id = local_frame_; snapshot.header.stamp = node->now();
    snapshot.metadata.resolution = master->getResolution(); snapshot.metadata.size_x = master->getSizeInCellsX(); snapshot.metadata.size_y = master->getSizeInCellsY();
    snapshot.metadata.origin.position.x = master->getOriginX(); snapshot.metadata.origin.position.y = master->getOriginY(); snapshot.metadata.origin.orientation.w = 1.0;
    const auto count = static_cast<size_t>(snapshot.metadata.size_x) * snapshot.metadata.size_y;
    // Costmap's ROS message uses a byte sequence; materialize it explicitly
    // instead of relying on iterator conversion from Costmap2D's char map.
    snapshot.data.resize(count);
    std::copy_n(master->getCharMap(), count, snapshot.data.begin());
  }
  const double yaw = tf2::getYaw(robot.pose.orientation);
  // Keep this same immutable byte-for-byte snapshot alive for accepted-guide
  // evidence.  Moving it into the request would leave the diagnostic mirror
  // with correct metadata but an empty data payload.
  LocalFast2DRequest request; request.costmap = snapshot; request.reference_path = plan;
  request.config = LocalFast2DConfig{lookahead_, cost_weight_, max_planning_time_ms_, 12000, allow_unknown_}; request.start_yaw = yaw;
  request.start = {robot.pose.position.x, robot.pose.position.y,
    std::cos(yaw)*input.velocity.linear.x - std::sin(yaw)*input.velocity.linear.y,
    std::sin(yaw)*input.velocity.linear.x + std::cos(yaw)*input.velocity.linear.y};
  const auto result = planLocalFast2D(request);
  if (!result.success) {
    {std::lock_guard<std::mutex> l(mutex_); ++no_path_count_; if (result.timeout) {++timeout_count_;}}
    publishDiagnostic(false, result.timeout, result.expanded_nodes,
      result.generated_nodes, result.latency_ms, 0, result.cycle_class);
    return;
  }
  // Keep the planning snapshot stamp on the observer path.  Along with the
  // identical Costmap header stamp this is the stable pairing key for the
  // diagnostic validator, not a later service lookup.
  nav_msgs::msg::Path guide = result.local_path; guide.header.stamp = snapshot.header.stamp;
  for (auto & p : guide.poses) {p.header = guide.header;}
  // Preserve the pre-instrumentation MPPI input timestamps.  The diagnostic
  // path above is never passed to MPPI; only its coordinates are mirrored.
  nav_msgs::msg::Path mppi_guide = guide; mppi_guide.header.stamp = node->now();
  for (auto & p : mppi_guide.poses) {p.header = mppi_guide.header;}
  nav_msgs::msg::Path stitched = mppi_guide; const auto target = std::find_if(plan.poses.begin(), plan.poses.end(), [&result](const auto & p) {return p.pose.position.x == result.selected_local_goal.pose.position.x && p.pose.position.y == result.selected_local_goal.pose.position.y;}); if (target != plan.poses.end()) {stitched.poses.insert(stitched.poses.end(), std::next(target), plan.poses.end());}
  nav_msgs::msg::Path goal_msg; goal_msg.header = guide.header; goal_msg.poses.push_back(result.selected_local_goal); path_pub_->publish(guide); goal_pub_->publish(goal_msg);
  {std::lock_guard<std::mutex> l(mutex_);
    latest_guide_ = Guide{guide, stitched, std::move(snapshot), ++next_guide_id_, input.generation,
      node->now(), true};
    ++guide_generated_count_;
  }
  // A generated primitive means the kinodynamic graph was expanded. The
  // generated==0 success is the A* start-node / collision-free connector
  // shortcut responsible for the very small latency mode in prior runs.
  publishDiagnostic(true, false, result.expanded_nodes,
    result.generated_nodes, result.latency_ms, guide.poses.size(), result.cycle_class);
}

void HybridController::publishAcceptedDiagnostic(const Guide & guide)
{
  auto node = node_.lock(); if (!node || !diagnostics_pub_) {return;}
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "local_fast2d_accepted_snapshot";
  status.hardware_id = "m1";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "accepted guide and exact planning snapshot published";
  const auto add = [&status](const char * key, const auto & value) {
      diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = std::to_string(value);
      status.values.push_back(pair);
    };
  add("accepted_guide_id", guide.id);
  add("planning_result_id", guide.id);
  const auto stamp_ns = static_cast<int64_t>(guide.snapshot.header.stamp.sec) * 1000000000LL +
    guide.snapshot.header.stamp.nanosec;
  add("snapshot_stamp_ns", stamp_ns);
  add("snapshot_size_x", guide.snapshot.metadata.size_x);
  add("snapshot_size_y", guide.snapshot.metadata.size_y);
  diagnostic_msgs::msg::DiagnosticArray array;
  // This header is intentionally the pairing stamp, rather than publication
  // time, so all three diagnostic artifacts identify one planning result.
  array.header = guide.snapshot.header;
  array.status.push_back(std::move(status)); diagnostics_pub_->publish(array);
}

void HybridController::publishDiagnostic(bool success, bool timeout, std::size_t expanded,
  std::size_t generated, double latency_ms, std::size_t path_points,
  const std::string & cycle_class)
{
  auto node = node_.lock(); if (!node || !diagnostics_pub_) {return;}
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "local_fast2d"; status.hardware_id = "m1";
  status.level = success ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = success ? "guide valid" : (timeout ? "planning timeout" : "no local path");
  if (cycle_class == "full_search") {recordLatency(full_search_stats_, latency_ms);}
  else if (cycle_class == "connector_shortcut") {recordLatency(shortcut_stats_, latency_ms);}
  else {recordLatency(other_stats_, latency_ms);}
  const auto percentile = [](const LatencyStats & stats, double fraction) {
      if (stats.samples_ms.empty()) {return 0.0;}
      std::vector<double> values(stats.samples_ms.begin(), stats.samples_ms.end());
      std::sort(values.begin(), values.end());
      const double index = (values.size() - 1) * fraction;
      const auto low = static_cast<std::size_t>(std::floor(index));
      const auto high = static_cast<std::size_t>(std::ceil(index));
      return values[low] + (values[high] - values[low]) * (index - low);
    };
  const auto add = [&status](const char * key, const auto & value) {diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = std::to_string(value); status.values.push_back(pair);};
  diagnostic_msgs::msg::KeyValue mode; mode.key = "mode"; mode.value = mode_; status.values.push_back(mode);
  diagnostic_msgs::msg::KeyValue category; category.key = "cycle_class"; category.value = cycle_class; status.values.push_back(category);
  const auto add_stats = [&add, &percentile](const char * prefix, const LatencyStats & stats) {
      const std::string name(prefix);
      add((name + "_count").c_str(), stats.count);
      add((name + "_mean_ms").c_str(), stats.count ? stats.sum_ms / stats.count : 0.0);
      add((name + "_p50_ms").c_str(), percentile(stats, 0.50));
      add((name + "_p95_ms").c_str(), percentile(stats, 0.95));
      add((name + "_max_ms").c_str(), stats.max_ms);
    };
  std::string reference_source;
  uint64_t fallback_count, stale_count, no_path_count, timeout_count, guide_generated_count, guide_accepted_count;
  {std::lock_guard<std::mutex> l(mutex_);
    reference_source = reference_source_; fallback_count = fallback_count_; stale_count = stale_count_;
    no_path_count = no_path_count_; timeout_count = timeout_count_;
    guide_generated_count = guide_generated_count_; guide_accepted_count = guide_accepted_count_;
  }
  diagnostic_msgs::msg::KeyValue source; source.key = "reference_source"; source.value = reference_source; status.values.push_back(source);
  add("latency_ms", latency_ms); add("expanded_nodes", expanded); add("generated_nodes", generated); add("path_points", path_points); add("guide_generated_count", guide_generated_count); add("guide_accepted_count", guide_accepted_count); add("fallback_count", fallback_count); add("stale_count", stale_count); add("no_path_count", no_path_count); add("timeout_count", timeout_count); add("path_reuse_count", path_reuse_count_); add("no_op_count", no_op_count_);
  add_stats("full_search", full_search_stats_);
  add_stats("connector_shortcut", shortcut_stats_);
  add_stats("other", other_stats_);
  diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = node->now(); array.status.push_back(std::move(status)); diagnostics_pub_->publish(array);
}

void HybridController::recordLatency(LatencyStats & stats, double latency_ms)
{
  constexpr std::size_t kMaximumSamples = 4096;
  ++stats.count; stats.sum_ms += latency_ms; stats.max_ms = std::max(stats.max_ms, latency_ms);
  stats.samples_ms.push_back(latency_ms);
  if (stats.samples_ms.size() > kMaximumSamples) {stats.samples_ms.pop_front();}
}

}  // namespace m1_local_fast2d

PLUGINLIB_EXPORT_CLASS(m1_local_fast2d::HybridController, nav2_core::Controller)
