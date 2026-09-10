#ifndef M1_LOCAL_FAST2D__HYBRID_CONTROLLER_HPP_
#define M1_LOCAL_FAST2D__HYBRID_CONTROLLER_HPP_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav2_core/controller.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace m1_local_fast2d
{

class HybridController : public nav2_core::Controller
{
public:
  HybridController() = default;
  ~HybridController() override;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;
  void setSpeedLimit(const double &, const bool &) override;

private:
  struct Input {nav_msgs::msg::Path plan; geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Twist velocity; uint64_t generation{0};};
  // snapshot is the immutable master-map copy given to planLocalFast2D.  It is
  // retained solely so an accepted guide can be observed against the exact
  // map it was planned in; MPPI never receives or consults it.
  struct Guide {nav_msgs::msg::Path path; nav_msgs::msg::Path mppi_plan;
    nav2_msgs::msg::Costmap snapshot; uint64_t id{0}; uint64_t generation{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME}; bool valid{false};};
  void workerLoop();
  void planOnce(const Input & input);
  void publishDiagnostic(bool success, bool timeout, std::size_t expanded,
    std::size_t generated, double latency_ms, std::size_t path_points,
    const std::string & cycle_class);
  void publishAcceptedDiagnostic(const Guide & guide);
  struct LatencyStats {
    std::size_t count{0};
    double sum_ms{0.0};
    double max_ms{0.0};
    std::deque<double> samples_ms;
  };
  void recordLatency(LatencyStats & stats, double latency_ms);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_mppi_controller::MPPIController> mppi_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, goal_pub_, accepted_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr accepted_costmap_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::string name_, mode_{"shadow"}, local_frame_;
  double update_rate_{8.0}, stale_timeout_{0.5}, lookahead_{2.0}, cost_weight_{2.0};
  int max_planning_time_ms_{80};
  bool allow_unknown_{true}, active_{false}, stop_worker_{false};
  // Test-only, default-off switch used to verify the production fallback
  // branch without changing any planner or safety behavior.
  bool test_force_fallback_{false};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  Input latest_input_;
  Guide latest_guide_;
  uint64_t input_generation_{0}, forwarded_generation_{0}, next_guide_id_{0};
  uint64_t fallback_count_{0}, stale_count_{0}, no_path_count_{0}, timeout_count_{0};
  uint64_t guide_generated_count_{0}, guide_accepted_count_{0};
  std::string reference_source_{"global_fallback"};
  // These are written by the sole local-planning worker and exposed in its
  // diagnostics; keeping a bounded sample history permits p50/p95 without
  // changing planning behavior.
  LatencyStats full_search_stats_, shortcut_stats_, other_stats_;
  uint64_t path_reuse_count_{0}, no_op_count_{0};
};

}  // namespace m1_local_fast2d

#endif
