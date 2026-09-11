#include "m1_local_fast2d/hybrid_controller.hpp"

#include <algorithm>
#include <array>
#include <csignal>
#include <chrono>
#include <cmath>
#include <execinfo.h>
#include <cstdio>
#include <ucontext.h>
#include <unistd.h>
#include <limits>
#include <utility>
#include <vector>

#include "m1_local_fast2d/local_fast2d_core.hpp"
#include "m1_local_fast2d/candidate_risk_scorer.hpp"
#include "m1_local_fast2d/candidate_selector.hpp"
#include "m1_local_fast2d/timed_reference_retimer.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace m1_local_fast2d
{
namespace
{
std::once_flag crash_backtrace_once;

void writeCrashBacktrace(int signal, siginfo_t * info, void * context)
{
  constexpr char header[] = "m1_local_fast2d SIGSEGV backtrace:\n";
  ::write(STDERR_FILENO, header, sizeof(header) - 1);
  const auto * ucontext = static_cast<ucontext_t *>(context);
  char registers[512];
  const auto bytes = std::snprintf(
    registers, sizeof(registers),
    "signal=%d fault=%p rip=%#llx rdi=%#llx rsi=%#llx rdx=%#llx rcx=%#llx r8=%#llx\n",
    signal, info ? info->si_addr : nullptr,
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_RIP]),
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_RDI]),
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_RSI]),
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_RDX]),
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_RCX]),
    static_cast<unsigned long long>(ucontext->uc_mcontext.gregs[REG_R8]));
  if (bytes > 0) {
    ::write(STDERR_FILENO, registers, static_cast<std::size_t>(bytes));
  }
  std::array<void *, 64> frames{};
  const int count = ::backtrace(frames.data(), static_cast<int>(frames.size()));
  ::backtrace_symbols_fd(frames.data(), count, STDERR_FILENO);
  struct sigaction default_action {};
  default_action.sa_handler = SIG_DFL;
  sigemptyset(&default_action.sa_mask);
  sigaction(signal, &default_action, nullptr);
  std::raise(signal);
}

void installCrashBacktraceHandler()
{
  struct sigaction action {};
  action.sa_sigaction = writeCrashBacktrace;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_SIGINFO;
  sigaction(SIGSEGV, &action, nullptr);
}

// Conditions only the controller-owned timed-reference copy. Positions and
// timestamps are deliberately untouched: the planner's geometry/horizon stay
// authoritative, while its initial state is made continuous with the state
// from which MPPI will propagate.
m1_local_fast2d::msg::TimedTrajectory conditionTimedReference(
  m1_local_fast2d::msg::TimedTrajectory trajectory, double robot_yaw,
  const geometry_msgs::msg::Twist & robot_twist, TimedReferenceRetimingDiagnostics * retiming)
{
  (void)robot_yaw;
  (void)robot_twist;
  TimedReferenceRetimingDiagnostics result;
  auto output = retimeTimedReference(trajectory, {}, &result);
  if (retiming) {*retiming = result;}
  RCLCPP_INFO(rclcpp::get_logger("TimedReferenceRetimer"),
    "retimed points=%zu scale=%.6f duration=%.6f->%.6f v=(%.6f,%.6f,%.6f)->(%.6f,%.6f,%.6f) a=(%.6f,%.6f,%.6f)->(%.6f,%.6f,%.6f) geometry_error=%.9f terminal_split=%d terminal_distance=%.6f terminal_speed=%.6f terminal_turn_s=%.6f terminal_scale=%.6f",
    output.points.size(), result.time_scale, result.before_duration, result.after_duration,
    result.before_max_vx, result.before_max_vy, result.before_max_wz,
    result.after_max_vx, result.after_max_vy, result.after_max_wz,
    result.before_max_ax, result.before_max_ay, result.before_max_awz,
    result.after_max_ax, result.after_max_ay, result.after_max_awz, result.geometry_error,
    result.terminal_heading_split ? 1 : 0, result.terminal_approach_distance,
    result.terminal_approach_speed, result.terminal_rotation_duration, result.terminal_time_scale);
  return output;
}
}  // namespace

HybridController::~HybridController() {cleanup();}

void HybridController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent; name_ = std::move(name); tf_ = std::move(tf); costmap_ros_ = std::move(costmap_ros);
  std::call_once(crash_backtrace_once, installCrashBacktraceHandler);
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
  declare("num_candidates", num_candidates_); declare("candidate_diversity_threshold", candidate_diversity_threshold_);
  declare("candidate_gate_initial_offset", candidate_gate_initial_offset_); declare("candidate_gate_offset_step", candidate_gate_offset_step_); declare("candidate_gate_max_offset", candidate_gate_max_offset_); declare("candidate_gate_clearance", candidate_gate_clearance_);
  declare("test_force_fallback", test_force_fallback_);
  declare("risk_scoring_dt", risk_scoring_dt_); declare("risk_high_threshold", risk_high_threshold_); declare("minimum_coverage_ratio", minimum_coverage_ratio_); declare("prediction_fresh_timeout", prediction_fresh_timeout_);
  declare("selector_w_mean", selector_config_.w_mean); declare("selector_w_max", selector_config_.w_max); declare("selector_w_high", selector_config_.w_high); declare("candidate_baseline_preference", selector_config_.baseline_preference); declare("candidate_switch_margin", selector_config_.switch_margin); declare("candidate_switch_penalty", selector_config_.switch_penalty); declare("route_match_threshold", selector_config_.route_match_threshold); declare("minimum_valid_queries", selector_config_.minimum_valid_queries);
  const auto declareRestricted = [&node, this](const char * suffix, auto & value) {
      const std::string key = name_ + ".restricted_mppi." + suffix;
      if (!node->has_parameter(key)) {node->declare_parameter(key, value);}
      node->get_parameter(key, value);
    };
  declareRestricted("enabled", restricted_mppi_config_.enabled);
  declareRestricted("delta_vx_max", restricted_mppi_config_.delta_vx_max);
  declareRestricted("delta_vy_max", restricted_mppi_config_.delta_vy_max);
  declareRestricted("delta_wz_max", restricted_mppi_config_.delta_wz_max);
  declareRestricted("position_tube_radius", restricted_mppi_config_.position_tube_radius);
  declareRestricted("phase_search_back_s", restricted_mppi_config_.phase_search_back_s);
  declareRestricted("phase_search_forward_s", restricted_mppi_config_.phase_search_forward_s);
  declareRestricted("max_phase_lead_s", restricted_mppi_config_.max_phase_lead_s);
  declareRestricted("phase_projection_resolution_s", restricted_mppi_config_.phase_projection_resolution_s);
  declareRestricted("consider_footprint", restricted_mppi_config_.consider_footprint);
  selector_config_.minimum_coverage_ratio = minimum_coverage_ratio_;
  if ((mode_ != "shadow" && mode_ != "active") || update_rate_ <= 0.0 || stale_timeout_ <= 0.0 ||
    lookahead_ <= 0.0 || cost_weight_ < 0.0 || max_planning_time_ms_ <= 0 || num_candidates_ < 1 || num_candidates_ > 5 || candidate_diversity_threshold_ <= 0.0 || candidate_gate_initial_offset_ <= 0.0 || candidate_gate_offset_step_ <= 0.0 || candidate_gate_max_offset_ < candidate_gate_initial_offset_ || candidate_gate_clearance_ < 0.0 ||
    !std::isfinite(restricted_mppi_config_.delta_vx_max) || restricted_mppi_config_.delta_vx_max < 0.0 ||
    !std::isfinite(restricted_mppi_config_.delta_vy_max) || restricted_mppi_config_.delta_vy_max < 0.0 ||
    !std::isfinite(restricted_mppi_config_.delta_wz_max) || restricted_mppi_config_.delta_wz_max < 0.0 ||
    !std::isfinite(restricted_mppi_config_.position_tube_radius) || restricted_mppi_config_.position_tube_radius <= 0.0 ||
    !std::isfinite(restricted_mppi_config_.phase_search_back_s) || restricted_mppi_config_.phase_search_back_s < 0.0 ||
    !std::isfinite(restricted_mppi_config_.phase_search_forward_s) || restricted_mppi_config_.phase_search_forward_s < 0.0 ||
    !std::isfinite(restricted_mppi_config_.max_phase_lead_s) || restricted_mppi_config_.max_phase_lead_s < 0.0 ||
    !std::isfinite(restricted_mppi_config_.phase_projection_resolution_s) || restricted_mppi_config_.phase_projection_resolution_s <= 0.0) {throw std::runtime_error("invalid local_fast2d or restricted_mppi parameters");}
  local_frame_ = costmap_ros_->getGlobalFrameID();
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/path", rclcpp::QoS(1));
  goal_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/local_goal", rclcpp::QoS(1));
  // Test-only observation of precisely the local guide forwarded to MPPI.
  accepted_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_fast2d/accepted_path", rclcpp::QoS(10));
  accepted_costmap_pub_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    "/local_fast2d/accepted_costmap", rclcpp::QoS(10));
  accepted_timed_trajectory_pub_ = node->create_publisher<m1_local_fast2d::msg::TimedTrajectory>(
    "/local_fast2d/accepted_timed_trajectory", rclcpp::QoS(10));
  candidates_pub_ = node->create_publisher<m1_local_fast2d::msg::LocalTrajectoryCandidateArray>(
    "/local_fast2d/candidate_trajectories", rclcpp::QoS(10));
  risk_scores_pub_ = node->create_publisher<m1_local_fast2d::msg::CandidateRiskScoreArray>(
    "/local_fast2d/candidate_risk_scores", rclcpp::QoS(10));
  selected_trajectory_pub_ = node->create_publisher<m1_local_fast2d::msg::SelectedLocalTrajectory>(
    "/local_fast2d/selected_trajectory", rclcpp::QoS(10));
  selection_diagnostics_pub_ = node->create_publisher<m1_local_fast2d::msg::SelectionDiagnosticArray>(
    "/local_fast2d/selection_diagnostics", rclcpp::QoS(10));
  scope_sequence_sub_ = node->create_subscription<m1_scope_msgs::msg::ScopePredictionSequence>(
    "/scope/prediction_sequence", rclcpp::QoS(1), [this](m1_scope_msgs::msg::ScopePredictionSequence::ConstSharedPtr message) {
      m1_scope_risk::RiskFieldConfig config;
      // Match the predictor's established max_prediction_age contract. This
      // is a freshness acceptance bound, not a change to R(x,y,t).
      config.stale_timeout_ns = 1000000000LL;
      auto snapshot = m1_scope_risk::RiskFieldSnapshot::fromMessage(*message, config);
      if (snapshot) {std::lock_guard<std::mutex> lock(mutex_); latest_risk_snapshot_ = std::make_shared<m1_scope_risk::RiskFieldSnapshot>(std::move(*snapshot));}
    });
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/local_fast2d/diagnostics", rclcpp::QoS(10));
  // Keep MPPI's established namespace and parameters exactly intact.
  mppi_ = std::make_unique<nav2_mppi_controller::MPPIController>();
  mppi_->configure(parent, name_, tf_, costmap_ros_);
  restricted_mppi_adapter_ = std::make_unique<RestrictedMPPIAdapter>(*mppi_);
  if (restricted_mppi_config_.enabled && !restricted_mppi_adapter_->versionCompatible()) {
    RCLCPP_ERROR(
      node->get_logger(), "Restricted MPPI disabled by version guard: %s",
      restricted_mppi_adapter_->versionFailure().c_str());
  }
}

void HybridController::activate() {mppi_->activate(); mppi_active_ = true; active_ = true; stop_worker_ = false; worker_ = std::thread(&HybridController::workerLoop, this);}
void HybridController::deactivate() {active_ = false; {std::lock_guard<std::mutex> l(mutex_); stop_worker_ = true; previous_selected_trajectory_.reset();} wake_.notify_all(); if (worker_.joinable()) {worker_.join();} if (mppi_ && mppi_active_) {mppi_->deactivate(); mppi_active_ = false;}}
void HybridController::cleanup() {deactivate(); restricted_mppi_adapter_.reset(); if (mppi_) {mppi_->cleanup(); mppi_.reset();} path_pub_.reset(); goal_pub_.reset(); accepted_pub_.reset(); accepted_costmap_pub_.reset(); accepted_timed_trajectory_pub_.reset(); candidates_pub_.reset(); risk_scores_pub_.reset(); selected_trajectory_pub_.reset(); selection_diagnostics_pub_.reset(); scope_sequence_sub_.reset(); diagnostics_pub_.reset(); {std::lock_guard<std::mutex> lock(mutex_); latest_reference_.reset();} costmap_ros_.reset(); tf_.reset(); node_.reset();}

void HybridController::setPlan(const nav_msgs::msg::Path & path)
{
  // Keep the last accepted timed reference until the next selected guide is
  // accepted.  A new global plan invalidates the pending guide, but the old
  // reference is needed to diagnose an atomic reference switch.
  {std::lock_guard<std::mutex> l(mutex_); latest_input_.plan = path; ++input_generation_; latest_input_.generation = input_generation_; latest_guide_.valid = false; previous_selected_trajectory_.reset();}
  // An original plan is always ready before the first local result.
  mppi_->setPlan(path); forwarded_generation_ = 0; wake_.notify_one();
}

geometry_msgs::msg::TwistStamped HybridController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  RCLCPP_INFO_ONCE(rclcpp::get_logger("HybridController"),
    "Restricted MPPI controller callback begin (first invocation)");
  // This boundary is the actual Nav2 controller callback.  The adapter also
  // records its own restricted-MPPI duration; keep the two measurements
  // separate so controller_total_ms includes only the small HybridController
  // handoff around the frozen MPPI computation, without timing diagnostics
  // publication itself.
  const auto controller_cycle_started = std::chrono::steady_clock::now();
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
    std::shared_ptr<const RestrictedTimedReference> reference;
    bool switched = false;
    uint64_t previous_planning_result_id = 0;
    const auto node_now_ns = node->now().nanoseconds();
    RestrictedTimedReferenceMetadata metadata;
    metadata.reference_id = guide.id;
    metadata.planning_result_id = guide.timed_trajectory.planning_result_id;
    metadata.input_generation = guide.timed_trajectory.input_generation;
    metadata.selected_candidate_index = guide.selected_candidate_index;
    metadata.frame_id = guide.timed_trajectory.header.frame_id;
    metadata.source_stamp_ns = rclcpp::Time(guide.timed_trajectory.header.stamp).nanoseconds();
    metadata.activation_stamp_ns = node_now_ns;
    geometry_msgs::msg::PoseStamped reference_pose = pose;
    if (reference_pose.header.frame_id != metadata.frame_id) {
      try {
        geometry_msgs::msg::PoseStamped transformed;
        tf_->transform(reference_pose, transformed, metadata.frame_id, tf2::durationFromSec(0.2));
        reference_pose = transformed;
      } catch (const tf2::TransformException &) {
        reference_pose = pose;
      }
    }
    TimedReferenceRetimingDiagnostics retiming;
    const auto conditioned = conditionTimedReference(
      guide.timed_trajectory, tf2::getYaw(reference_pose.pose.orientation), velocity, &retiming);
    reference = std::make_shared<const RestrictedTimedReference>(conditioned, metadata);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_reference_) {
        previous_planning_result_id = latest_reference_->metadata().planning_result_id;
        switched = previous_planning_result_id != metadata.planning_result_id;
      }
      latest_reference_ = reference;
      ++reference_generation_;
    }
    mppi_->setPlan(guide.mppi_plan); forwarded_generation_ = guide.generation;
    if (accepted_pub_) {accepted_pub_->publish(guide.path);}
    if (accepted_costmap_pub_) {accepted_costmap_pub_->publish(guide.snapshot);}
    if (accepted_timed_trajectory_pub_) {accepted_timed_trajectory_pub_->publish(guide.timed_trajectory);}
    publishAcceptedDiagnostic(guide, reference, switched, previous_planning_result_id, retiming);
    std::lock_guard<std::mutex> l(mutex_); ++guide_accepted_count_; reference_source_ = "local_fast2d";
  }
  if (!use_guide && forwarded_generation_ != 0) {mppi_->setPlan(original); forwarded_generation_ = 0;}
  if (!use_guide) {std::lock_guard<std::mutex> l(mutex_); reference_source_ = "global_fallback";}
  wake_.notify_one();
  bool restricted_enabled = false;
  if (node) {
    node->get_parameter(name_ + ".restricted_mppi.enabled", restricted_enabled);
  }
  if (mode_ == "active" && restricted_enabled && restricted_mppi_adapter_ &&
    restricted_mppi_adapter_->versionCompatible())
  {
    std::shared_ptr<const RestrictedTimedReference> active_reference;
    uint64_t active_reference_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (use_guide) {
        active_reference = latest_reference_;
        active_reference_generation = reference_generation_;
      }
    }
    RestrictedMPPIDiagnostics restricted_diagnostics;
    auto restricted_config = restricted_mppi_config_;
    restricted_config.enabled = true;
    auto command = restricted_mppi_adapter_->computeVelocityCommands(
      pose, velocity, goal_checker, active_reference, active_reference_generation,
      restricted_config, restricted_diagnostics);
    RCLCPP_INFO_ONCE(rclcpp::get_logger("HybridController"),
      "Restricted MPPI adapter returned from first invocation with status=%s",
      restricted_diagnostics.status.c_str());
    restricted_diagnostics.controller_total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - controller_cycle_started).count();
    publishRestrictedDiagnostic(restricted_diagnostics);
    return command;
  }
  if (mode_ == "active" && restricted_enabled && restricted_mppi_adapter_ &&
    !restricted_mppi_adapter_->versionCompatible())
  {
    RestrictedMPPIDiagnostics restricted_diagnostics;
    restricted_diagnostics.enabled = true;
    restricted_diagnostics.version_guard_ok = false;
    restricted_diagnostics.status = "FEATURE_UNAVAILABLE_VERSION_GUARD";
    restricted_diagnostics.upstream_version = restricted_mppi_adapter_->upstreamVersion();
    publishRestrictedDiagnostic(restricted_diagnostics);
  }
  // This is intentionally observability-only.  The command below is the
  // unmodified stock controller path, so a disabled feature cannot retain a
  // nominal sequence, tube mask, or any other restricted-MPPI influence.
  if (!restricted_enabled) {
    RestrictedMPPIDiagnostics restricted_diagnostics;
    restricted_diagnostics.enabled = false;
    restricted_diagnostics.version_guard_ok = restricted_mppi_adapter_ &&
      restricted_mppi_adapter_->versionCompatible();
    if (restricted_mppi_adapter_) {
      restricted_diagnostics.upstream_version = restricted_mppi_adapter_->upstreamVersion();
    }
    restricted_diagnostics.status = "FEATURE_DISABLED";
    publishRestrictedDiagnostic(restricted_diagnostics);
  }
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
  request.config = LocalFast2DConfig{lookahead_, cost_weight_, max_planning_time_ms_, 12000, allow_unknown_, num_candidates_, candidate_diversity_threshold_, candidate_gate_initial_offset_, candidate_gate_offset_step_, candidate_gate_max_offset_, candidate_gate_clearance_}; request.start_yaw = yaw;
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
  m1_local_fast2d::msg::TimedTrajectory timed_trajectory;
  timed_trajectory.header = guide.header;
  timed_trajectory.input_generation = input.generation;
  timed_trajectory.planning_result_id = next_guide_id_ + 1;
  timed_trajectory.points.reserve(result.timed_trajectory.size());
  for (std::size_t index = 0; index < result.timed_trajectory.size(); ++index) {
    const auto & source = result.timed_trajectory[index];
    m1_local_fast2d::msg::TimedTrajectoryPoint point;
    point.point_index = static_cast<uint32_t>(index);
    point.x = source.x; point.y = source.y; point.yaw = source.yaw;
    point.time_from_start = source.time_from_start;
    point.vx = source.vx; point.vy = source.vy; point.ax = source.ax; point.ay = source.ay;
    timed_trajectory.points.push_back(std::move(point));
  }
  m1_local_fast2d::msg::LocalTrajectoryCandidateArray candidate_set;
  candidate_set.header = guide.header; candidate_set.planning_result_id = timed_trajectory.planning_result_id;
  candidate_set.input_generation = input.generation; candidate_set.requested_candidate_count = result.requested_candidates;
  candidate_set.valid_candidate_count = result.candidates.size(); candidate_set.primary_planning_latency_ms = result.primary_planning_latency_ms; candidate_set.total_planning_latency_ms = result.latency_ms;
  for (const auto & source : result.candidates) { m1_local_fast2d::msg::LocalTrajectoryCandidate candidate; candidate.candidate_index = source.index; candidate.candidate_rank = source.index; candidate.valid = true; candidate.planner_cost = source.planner_cost; candidate.duration = source.duration; candidate.generation_latency_ms = source.generation_latency_ms; candidate.minimum_diversity = source.minimum_diversity; candidate.trajectory = timed_trajectory; candidate.trajectory.points.clear(); candidate.trajectory.planning_result_id = candidate_set.planning_result_id; for (std::size_t i=0;i<source.timed_trajectory.size();++i) {m1_local_fast2d::msg::TimedTrajectoryPoint p; const auto & q=source.timed_trajectory[i]; p.point_index=i;p.x=q.x;p.y=q.y;p.yaw=q.yaw;p.time_from_start=q.time_from_start;p.vx=q.vx;p.vy=q.vy;p.ax=q.ax;p.ay=q.ay;candidate.trajectory.points.push_back(p);} candidate_set.candidates.push_back(candidate); }
  std::shared_ptr<const m1_scope_risk::RiskFieldSnapshot> risk_snapshot;
  {std::lock_guard<std::mutex> lock(mutex_); risk_snapshot = latest_risk_snapshot_;}
  m1_local_fast2d::msg::CandidateRiskScoreArray scores;
  scores.header = guide.header; scores.planning_result_id = candidate_set.planning_result_id;
  const int64_t planning_stamp_ns = rclcpp::Time(snapshot.header.stamp).nanoseconds();
  const bool scope_present = static_cast<bool>(risk_snapshot);
  const bool frame_valid = scope_present && risk_snapshot->frameId() == guide.header.frame_id;
  const bool causal = scope_present && risk_snapshot->anchorStampNs() <= planning_stamp_ns;
  const bool fresh = scope_present && risk_snapshot->isFresh(node->now().nanoseconds()) &&
    (node->now().nanoseconds() - risk_snapshot->anchorStampNs()) <= static_cast<int64_t>(prediction_fresh_timeout_ * 1e9);
  if (scope_present) {
    scores.scope_prediction_id = risk_snapshot->predictionId();
    scores.scope_anchor_stamp.sec = risk_snapshot->anchorStampNs() / 1000000000LL;
    scores.scope_anchor_stamp.nanosec = risk_snapshot->anchorStampNs() % 1000000000LL;
    scores.pairing_age_ms = (planning_stamp_ns - risk_snapshot->anchorStampNs()) / 1e6;
  }
  if (scope_present && frame_valid && causal) {
    CandidateRiskScoringConfig config{risk_scoring_dt_, risk_high_threshold_, minimum_coverage_ratio_};
    for (const auto & candidate : candidate_set.candidates) {scores.scores.push_back(CandidateRiskScorer::score(candidate.trajectory, candidate.candidate_index, *risk_snapshot, planning_stamp_ns, node->now().nanoseconds(), config));}
  } else {
    for (const auto & candidate : candidate_set.candidates) {m1_local_fast2d::msg::CandidateRiskScore score; score.planning_result_id = candidate_set.planning_result_id; score.candidate_index = candidate.candidate_index; scores.scores.push_back(score);}
  }
  if (risk_scores_pub_) {risk_scores_pub_->publish(scores);}
  std::optional<m1_local_fast2d::msg::TimedTrajectory> previous;
  {std::lock_guard<std::mutex> lock(mutex_);
    // A simulated/ROS clock rollback invalidates cross-cycle continuity just
    // like a new global plan; never carry a future route into the reset task.
    if (previous_selected_trajectory_ &&
      rclcpp::Time(previous_selected_trajectory_->header.stamp) > rclcpp::Time(guide.header.stamp)) {
      previous_selected_trajectory_.reset();
    }
    previous = previous_selected_trajectory_;
  }
  const auto selection = CandidateSelector::select(candidate_set, scope_present ? &scores : nullptr,
    previous ? &*previous : nullptr, selector_config_, fresh, causal, frame_valid || !scope_present);
  const auto selected_it = std::find_if(candidate_set.candidates.begin(), candidate_set.candidates.end(),
    [&selection](const auto & c) {return c.candidate_index == selection.selected_candidate_index;});
  const auto & selected = selected_it == candidate_set.candidates.end() ? candidate_set.candidates.front() : *selected_it;
  const auto score_it = std::find_if(scores.scores.begin(), scores.scores.end(), [&selected](const auto & s) {return s.candidate_index == selected.candidate_index;});
  m1_local_fast2d::msg::SelectedLocalTrajectory selected_msg;
  selected_msg.header = guide.header; selected_msg.planning_result_id = candidate_set.planning_result_id; selected_msg.input_generation = input.generation;
  selected_msg.selected_candidate_index = selected.candidate_index; selected_msg.scope_prediction_id = scores.scope_prediction_id;
  selected_msg.prediction_used = selection.prediction_used; selected_msg.switched = selection.switched; selected_msg.selection_valid = selection.selection_valid;
  selected_msg.selection_cost = selection.selection_cost; selected_msg.selection_reason = selection.selection_reason; selected_msg.trajectory = selected.trajectory;
  if (score_it != scores.scores.end()) {selected_msg.selected_mean_risk = score_it->mean_risk; selected_msg.selected_max_risk = score_it->max_risk; selected_msg.selected_coverage = score_it->coverage_ratio;}
  if (selected_trajectory_pub_) {selected_trajectory_pub_->publish(selected_msg);}
  if (selection_diagnostics_pub_) {m1_local_fast2d::msg::SelectionDiagnosticArray d; d.header = guide.header; d.planning_result_id = candidate_set.planning_result_id; d.selected_candidate_index = selected.candidate_index; d.previous_matched_candidate_index = selection.previous_matched_candidate.value_or(std::numeric_limits<uint32_t>::max()); d.prediction_used = selection.prediction_used; d.switch_margin = selector_config_.switch_margin; d.selection_reason = selection.selection_reason; for (const auto & item : selection.diagnostics) {m1_local_fast2d::msg::SelectionCandidateDiagnostic x; x.candidate_index=item.candidate_index; x.usable=item.usable; x.matches_previous_route=item.matches_previous_route; x.raw_cost=item.raw_cost; x.switch_penalty=item.switch_penalty; x.final_cost=item.final_cost; d.candidates.push_back(x);} selection_diagnostics_pub_->publish(d);}
  // Candidate 0 retains the byte-for-byte Phase 4 forwarding path.  An
  // alternate only substitutes the local segment before the unchanged stitch.
  if (selected.candidate_index != 0) {
    guide.poses.clear(); guide.poses.reserve(selected.trajectory.points.size());
    for (const auto & p : selected.trajectory.points) {geometry_msgs::msg::PoseStamped pose; pose.header = guide.header; pose.pose.position.x = p.x; pose.pose.position.y = p.y; pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(p.yaw / 2.0), std::cos(p.yaw / 2.0))); guide.poses.push_back(pose);}
    mppi_guide = guide; mppi_guide.header.stamp = node->now(); for (auto & p : mppi_guide.poses) {p.header = mppi_guide.header;}
    stitched = mppi_guide; if (target != plan.poses.end()) {stitched.poses.insert(stitched.poses.end(), std::next(target), plan.poses.end());}
  }
  timed_trajectory = selected.trajectory;
  if (candidates_pub_) {candidates_pub_->publish(candidate_set);}
  nav_msgs::msg::Path goal_msg; goal_msg.header = guide.header; goal_msg.poses.push_back(result.selected_local_goal); path_pub_->publish(guide); goal_pub_->publish(goal_msg);
  {std::lock_guard<std::mutex> l(mutex_);
    previous_selected_trajectory_ = selected.trajectory;
    latest_guide_ = Guide{guide, stitched, std::move(timed_trajectory), std::move(snapshot), ++next_guide_id_, input.generation,
      node->now(), selected.candidate_index, true};
    ++guide_generated_count_;
  }
  // A generated primitive means the kinodynamic graph was expanded. The
  // generated==0 success is the A* start-node / collision-free connector
  // shortcut responsible for the very small latency mode in prior runs.
  publishDiagnostic(true, false, result.expanded_nodes,
    result.generated_nodes, result.latency_ms, guide.poses.size(), result.cycle_class);
}

void HybridController::publishAcceptedDiagnostic(
  const Guide & guide, const std::shared_ptr<const RestrictedTimedReference> & reference,
  bool switched, uint64_t previous_planning_result_id,
  const TimedReferenceRetimingDiagnostics & retiming)
{
  auto node = node_.lock(); if (!node || !diagnostics_pub_) {return;}
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "local_fast2d_accepted_snapshot";
  status.hardware_id = "m1";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "accepted guide, timed trajectory, and exact planning snapshot published";
  const auto add = [&status](const char * key, const auto & value) {
      diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = std::to_string(value);
      status.values.push_back(pair);
    };
  const auto add_text = [&status](const char * key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = value;
      status.values.push_back(pair);
    };
  add("accepted_guide_id", guide.id);
  add("accepted_trajectory_id", guide.timed_trajectory.planning_result_id);
  add("planning_result_id", guide.id);
  add("input_generation", guide.generation);
  const auto stamp_ns = static_cast<int64_t>(guide.snapshot.header.stamp.sec) * 1000000000LL +
    guide.snapshot.header.stamp.nanosec;
  add("snapshot_stamp_ns", stamp_ns);
  add("snapshot_size_x", guide.snapshot.metadata.size_x);
  add("snapshot_size_y", guide.snapshot.metadata.size_y);
  add("retiming_valid", retiming.valid ? 1 : 0);
  add("retiming_time_scale", retiming.time_scale);
  add("retiming_before_duration_s", retiming.before_duration);
  add("retiming_after_duration_s", retiming.after_duration);
  add("retiming_before_max_vx", retiming.before_max_vx);
  add("retiming_before_max_vy", retiming.before_max_vy);
  add("retiming_before_max_wz", retiming.before_max_wz);
  add("retiming_after_max_vx", retiming.after_max_vx);
  add("retiming_after_max_vy", retiming.after_max_vy);
  add("retiming_after_max_wz", retiming.after_max_wz);
  add("retiming_before_max_ax", retiming.before_max_ax);
  add("retiming_before_max_ay", retiming.before_max_ay);
  add("retiming_before_max_awz", retiming.before_max_awz);
  add("retiming_after_max_ax", retiming.after_max_ax);
  add("retiming_after_max_ay", retiming.after_max_ay);
  add("retiming_after_max_awz", retiming.after_max_awz);
  add("retiming_geometry_error_m", retiming.geometry_error);
  add("terminal_heading_split", retiming.terminal_heading_split ? 1 : 0);
  add("terminal_approach_distance_m", retiming.terminal_approach_distance);
  add("terminal_reference_speed_mps", retiming.terminal_approach_speed);
  add("terminal_rotation_duration_s", retiming.terminal_rotation_duration);
  add("terminal_time_scale", retiming.terminal_time_scale);
  if (reference) {
    const auto reference_diagnostics = reference->diagnostics(
      node->now().nanoseconds(), 2.0, 40, switched, previous_planning_result_id);
    add_text("reference_frame_id", reference_diagnostics.metadata.frame_id);
    add_text("velocity_frame", reference_diagnostics.velocity_frame);
    add_text("terminal_handling_mode", reference_diagnostics.terminal_handling_mode);
    add("reference_id", reference_diagnostics.metadata.reference_id);
    add("reference_generation", reference_generation_);
    add("reference_planning_result_id", reference_diagnostics.metadata.planning_result_id);
    add("reference_input_generation", reference_diagnostics.metadata.input_generation);
    add("selected_candidate_index", reference_diagnostics.metadata.selected_candidate_index);
    add("reference_activation_stamp_ns", reference_diagnostics.metadata.activation_stamp_ns);
    add("reference_source_stamp_ns", reference_diagnostics.metadata.source_stamp_ns);
    add("reference_age_ms", reference_diagnostics.reference_age_sec * 1000.0);
    add("reference_duration_sec", reference_diagnostics.reference_duration_sec);
    add("mppi_horizon_duration_sec", reference_diagnostics.mppi_horizon_duration_sec);
    add("mppi_horizon_sample_count", reference_diagnostics.horizon_sample_count);
    add("reference_x", reference_diagnostics.first_reference.x_ref);
    add("reference_y", reference_diagnostics.first_reference.y_ref);
    add("reference_yaw", reference_diagnostics.first_reference.yaw_ref);
    add("reference_vx_world", reference_diagnostics.first_reference.vx_world_ref);
    add("reference_vy_world", reference_diagnostics.first_reference.vy_world_ref);
    add("reference_vx_body", reference_diagnostics.first_reference.vx_body_ref);
    add("reference_vy_body", reference_diagnostics.first_reference.vy_body_ref);
    add("reference_omega_raw", reference_diagnostics.first_reference.omega_raw);
    add("reference_omega", reference_diagnostics.first_reference.omega_ref);
    add("reference_terminal_hold", reference_diagnostics.first_reference.terminal_hold ? 1 : 0);
    add("reference_switched", reference_diagnostics.reference_switched ? 1 : 0);
    add("previous_planning_result_id", reference_diagnostics.previous_planning_result_id);
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  // This header is intentionally the pairing stamp, rather than publication
  // time, so all three diagnostic artifacts identify one planning result.
  array.header = guide.snapshot.header;
  array.status.push_back(std::move(status)); diagnostics_pub_->publish(array);
}

void HybridController::publishRestrictedDiagnostic(const RestrictedMPPIDiagnostics & diagnostics)
{
  auto node = node_.lock();
  if (!node || !diagnostics_pub_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "restricted_mppi";
  status.hardware_id = "m1";
  status.level = diagnostics.status == "TRACKING" || diagnostics.status == "REFERENCE_SWITCH" ?
    diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = diagnostics.status;
  const auto add = [&status](const char * key, const auto & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = std::to_string(value);
      status.values.push_back(std::move(pair));
    };
  const auto addText = [&status](const char * key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(std::move(pair));
    };
  add("enabled", diagnostics.enabled ? 1 : 0);
  add("version_guard_ok", diagnostics.version_guard_ok ? 1 : 0);
  addText("upstream_version", diagnostics.upstream_version);
  add("planning_result_id", diagnostics.planning_result_id);
  add("reference_generation", diagnostics.reference_generation);
  add("selected_candidate_index", diagnostics.selected_candidate_index);
  add("reference_id", diagnostics.reference_id);
  add("reference_age_ms", diagnostics.reference_age_sec * 1000.0);
  add("previous_tracking_phase_ms", diagnostics.previous_tracking_phase_s * 1000.0);
  add("projected_phase_ms", diagnostics.projected_phase_s * 1000.0);
  add("tracking_phase_ms", diagnostics.tracking_phase_s * 1000.0);
  add("phase_lead_ms", diagnostics.phase_lead_s * 1000.0);
  add("robot_x", diagnostics.robot_x);
  add("robot_y", diagnostics.robot_y);
  add("reference_source_stamp_ns", diagnostics.reference_source_stamp_ns);
  add("reference_activation_stamp_ns", diagnostics.reference_activation_stamp_ns);
  add("reference_length_m", diagnostics.reference_length_m);
  add("reference_start_x", diagnostics.reference_start_x);
  add("reference_start_y", diagnostics.reference_start_y);
  add("reference_last_x", diagnostics.reference_last_x);
  add("reference_last_y", diagnostics.reference_last_y);
  add("initial_reference_error_m", diagnostics.initial_reference_error_m);
  add("reference_dt_s", diagnostics.reference_dt_s);
  add("reference0_yaw", diagnostics.reference0_yaw);
  add("reference0_vx_world", diagnostics.reference0_vx_world);
  add("reference0_vy_world", diagnostics.reference0_vy_world);
  add("reference0_wz", diagnostics.reference0_wz);
  add("robot_yaw", diagnostics.robot_yaw);
  add("robot_vx", diagnostics.robot_vx);
  add("robot_vy", diagnostics.robot_vy);
  add("robot_wz", diagnostics.robot_wz);
  add("rollout0_x", diagnostics.rollout0_x);
  add("rollout0_y", diagnostics.rollout0_y);
  add("rollout0_yaw", diagnostics.rollout0_yaw);
  add("tube0_reference_x", diagnostics.tube0_reference_x);
  add("tube0_reference_y", diagnostics.tube0_reference_y);
  add("tube0_deviation_m", diagnostics.tube0_deviation_m);
  addText("original_horizon_state_json", diagnostics.original_horizon_state_json);
  addText("conditioned_horizon_state_json", diagnostics.conditioned_horizon_state_json);
  addText("conditioning_input_check_json", diagnostics.conditioning_input_check_json);
  add("original_max_kinematic_residual_m", diagnostics.original_max_kinematic_residual_m);
  add("conditioned_max_kinematic_residual_m", diagnostics.conditioned_max_kinematic_residual_m);
  add("original_max_yaw_residual_rad", diagnostics.original_max_yaw_residual_rad);
  add("conditioned_max_yaw_residual_rad", diagnostics.conditioned_max_yaw_residual_rad);
  addText("horizon_state_json", diagnostics.horizon_state_json);
  addText("mppi_limits_json", diagnostics.mppi_limits_json);
  addText("first_rollout_json", diagnostics.first_rollout_json);
  addText("deviation_inputs_json", diagnostics.deviation_inputs_json);
  add("rollout_count", diagnostics.rollout_count);
  add("feasible_sample_count", diagnostics.feasible_sample_count);
  add("feasible_sample_ratio", diagnostics.feasible_sample_ratio);
  add("tube_rejected_count", diagnostics.tube_rejected_count);
  add("collision_rejected_count", diagnostics.collision_rejected_count);
  add("collision_full_mask", diagnostics.collision_full_mask ? 1 : 0);
  add("collision_full_mask_stamp_ns", diagnostics.collision_full_mask_stamp_ns);
  addText("collision_rollout_evidence_json", diagnostics.collision_rollout_evidence_json);
  addText("collision_costmap_json", diagnostics.collision_costmap_json);
  add("first_tube_breach_captured", diagnostics.first_tube_breach_captured ? 1 : 0);
  add("first_tube_breach_batch", diagnostics.first_tube_breach_batch);
  add("first_tube_breach_step", diagnostics.first_tube_breach_step);
  add("first_tube_breach_deviation_m", diagnostics.first_tube_breach_deviation_m);
  add("first_tube_breach_rollout_x", diagnostics.first_tube_breach_rollout_x);
  add("first_tube_breach_rollout_y", diagnostics.first_tube_breach_rollout_y);
  add("first_tube_breach_reference_x", diagnostics.first_tube_breach_reference_x);
  add("first_tube_breach_reference_y", diagnostics.first_tube_breach_reference_y);
  add("control_bound_clamped_count", diagnostics.control_bound_clamped_count);
  add("proposed_scalar_control_count", diagnostics.proposed_scalar_control_count);
  add("clamped_scalar_control_count", diagnostics.clamped_scalar_control_count);
  add("proposed_sample_step_count", diagnostics.proposed_sample_step_count);
  add("clamped_sample_step_count", diagnostics.clamped_sample_step_count);
  add("proposed_sample_count", diagnostics.proposed_sample_count);
  add("clamped_sample_count", diagnostics.clamped_sample_count);
  add("vx_clamp_count", diagnostics.vx_clamp_count);
  add("vy_clamp_count", diagnostics.vy_clamp_count);
  add("wz_clamp_count", diagnostics.wz_clamp_count);
  add("lower_bound_clamp_count", diagnostics.lower_bound_clamp_count);
  add("upper_bound_clamp_count", diagnostics.upper_bound_clamp_count);
  add("velocity_bound_clamp_count", diagnostics.velocity_bound_clamp_count);
  add("acceleration_bound_clamp_count", diagnostics.acceleration_bound_clamp_count);
  add("controller_total_ms", diagnostics.controller_total_ms);
  add("restricted_mppi_total_ms", diagnostics.restricted_mppi_total_ms);
  add("phase_tracking_ms", diagnostics.phase_tracking_ms);
  add("sampling_ms", diagnostics.sampling_ms);
  add("control_bounds_ms", diagnostics.control_bounds_ms);
  add("rollout_ms", diagnostics.rollout_ms);
  add("tube_filter_ms", diagnostics.tube_filter_ms);
  add("collision_filter_ms", diagnostics.collision_filter_ms);
  add("critic_ms", diagnostics.critic_ms);
  add("costmap_access_ms", diagnostics.costmap_access_ms);
  add("command_finalization_ms", diagnostics.command_finalization_ms);
  add("batch_size", diagnostics.batch_size);
  add("time_steps", diagnostics.time_steps);
  add("model_dt", diagnostics.model_dt);
  add("maximum_accepted_tube_deviation", diagnostics.maximum_accepted_tube_deviation);
  add("maximum_generated_tube_deviation", diagnostics.maximum_generated_tube_deviation);
  add("nominal_vx_ref", diagnostics.nominal_command.vx);
  add("nominal_vy_ref", diagnostics.nominal_command.vy);
  add("nominal_wz_ref", diagnostics.nominal_command.wz);
  add("delta_vx", diagnostics.selected_correction.vx);
  add("delta_vy", diagnostics.selected_correction.vy);
  add("delta_wz", diagnostics.selected_correction.wz);
  add("final_vx", diagnostics.final_command.vx);
  add("final_vy", diagnostics.final_command.vy);
  add("final_wz", diagnostics.final_command.wz);
  add("reference_switched", diagnostics.reference_switched ? 1 : 0);
  add("reference_switch_reset", diagnostics.reset_performed ? 1 : 0);
  addText("comparison_frame", diagnostics.comparison_frame);
  addText("status", diagnostics.status);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node->now();
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
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
