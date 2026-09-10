#include <cmath>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "m1_local_fast2d/local_fast2d_core.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"

namespace
{
class CausalityHarness : public rclcpp::Node
{
public:
  CausalityHarness() : Node("m1_local_fast2d_causality_harness")
  {
    declare_parameter("costmap_service", "/local_costmap/get_costmap");
    declare_parameter("scope_enabled", false); declare_parameter("scope_diagnostics_topic", "/scope/diagnostics");
    declare_parameter("wait_for_recorder", false);
    declare_parameter("planning_rate", 8.0); declare_parameter("reference_length", 2.0); declare_parameter("reference_spacing", 0.05);
    declare_parameter("pre_event_seconds", 4.0); declare_parameter("post_event_seconds", 4.0);
    scope_enabled_ = get_parameter("scope_enabled").as_bool();
    service_ = create_client<nav2_msgs::srv::GetCostmap>(get_parameter("costmap_service").as_string());
    const auto static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    reference_pub_ = create_publisher<nav_msgs::msg::Path>("/local_fast2d_test/reference_path", static_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_fast2d_test/path", 1);
    goal_pub_ = create_publisher<nav_msgs::msg::Path>("/local_fast2d_test/local_goal", static_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/local_fast2d_test/diagnostics", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/local_fast2d_test/state", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr msg) {last_odom_ = msg;});
    scope_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(get_parameter("scope_diagnostics_topic").as_string(), 10, [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr) {scope_ready_ = true;});
    recorder_ready_sub_ = create_subscription<std_msgs::msg::String>("/local_fast2d_test/recorder_ready", 1, [this](std_msgs::msg::String::SharedPtr) {recorder_ready_ = true;});
    const auto period = std::chrono::duration<double>(1.0 / get_parameter("planning_rate").as_double());
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), [this] {tick();});
  }
private:
  enum class State {WAIT_SYSTEM, WAIT_COSTMAP, WAIT_SCOPE_READY, TEST_READY, PRE_EVENT, CROSSING, POST_EVENT, TEST_COMPLETE};
  const char * stateName() const {static const char * names[] = {"WAIT_SYSTEM", "WAIT_COSTMAP", "WAIT_SCOPE_READY", "TEST_READY", "PRE_EVENT", "CROSSING", "POST_EVENT", "TEST_COMPLETE"}; return names[static_cast<int>(state_)];}
  void publishState()
  {
    std_msgs::msg::String message; message.data = std::string(stateName()) + (epoch_.nanoseconds() ? ";T0=" + std::to_string(epoch_.nanoseconds()) : ""); state_pub_->publish(message);
  }
  void requestCostmap()
  {
    if (pending_) {return;} pending_ = true;
    service_->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>(), [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future) {
      pending_ = false; try {costmap_ = future.get()->map; have_costmap_ = !costmap_.data.empty();} catch (...) {have_costmap_ = false;}
    });
  }
  void freezeAndCreateReference()
  {
    frozen_ = last_odom_->pose.pose; frozen_yaw_ = tf2::getYaw(frozen_.orientation); frozen_frame_ = last_odom_->header.frame_id;
    reference_.header.frame_id = frozen_frame_; reference_.header.stamp = now();
    const double length = get_parameter("reference_length").as_double(), spacing = get_parameter("reference_spacing").as_double();
    for (double d = 0.0; d <= length + 1e-9; d += spacing) {
      geometry_msgs::msg::PoseStamped p; p.header = reference_.header; p.pose = frozen_;
      p.pose.position.x += -std::sin(frozen_yaw_) * d; p.pose.position.y += std::cos(frozen_yaw_) * d; reference_.poses.push_back(p);
    }
    goal_.header = reference_.header; goal_.poses.push_back(reference_.poses.back());
    publishStaticArtifacts();
  }
  void publishStaticArtifacts()
  {
    reference_pub_->publish(reference_); goal_pub_->publish(goal_); last_static_publish_ = now();
  }
  void plan()
  {
    if (!have_costmap_ || reference_.poses.empty()) {return;}
    auto planning_reference = reference_; planning_reference.header.stamp = now(); for (auto & p : planning_reference.poses) {p.header = planning_reference.header;}
    m1_local_fast2d::LocalFast2DRequest request; request.costmap = costmap_; request.reference_path = planning_reference; request.start_yaw = frozen_yaw_;
    request.start = {frozen_.position.x, frozen_.position.y, 0.0, 0.0};
    const auto result = m1_local_fast2d::planLocalFast2D(request);
    if (result.success) {auto path = result.local_path; path.header.stamp = now(); path_pub_->publish(path);}
    diagnostic_msgs::msg::DiagnosticStatus status; status.name = "local_fast2d_causality"; status.level = result.success ? 0 : 1; status.message = result.failure_reason;
    const auto add = [&status](const std::string & k, const auto v) {diagnostic_msgs::msg::KeyValue value; value.key = k; value.value = std::to_string(v); status.values.push_back(value);};
    add("latency_ms", result.latency_ms); add("expanded_nodes", result.expanded_nodes); add("generated_nodes", result.generated_nodes);
    if (last_odom_) {add("robot_displacement_m", std::hypot(last_odom_->pose.pose.position.x - frozen_.position.x, last_odom_->pose.pose.position.y - frozen_.position.y));}
    diagnostic_msgs::msg::DiagnosticArray diagnostics; diagnostics.header.stamp = now(); diagnostics.status.push_back(status); diagnostics_pub_->publish(diagnostics);
  }
  void tick()
  {
    publishState();
    if (state_ == State::WAIT_SYSTEM) {if (last_odom_) {state_ = State::WAIT_COSTMAP;} return;}
    if (state_ == State::WAIT_COSTMAP) {if (!service_->service_is_ready()) {return;} requestCostmap(); if (have_costmap_) {state_ = scope_enabled_ ? State::WAIT_SCOPE_READY : State::TEST_READY;} return;}
    if (state_ == State::WAIT_SCOPE_READY) {requestCostmap(); if (scope_ready_ && have_costmap_) {state_ = State::TEST_READY;} return;}
    if (state_ == State::TEST_READY) {
      if (get_parameter("wait_for_recorder").as_bool() && !recorder_ready_) {return;}
      freezeAndCreateReference(); epoch_ = now(); state_ = State::PRE_EVENT; publishState(); return;
    }
    requestCostmap(); plan(); const double elapsed = (now() - epoch_).seconds();
    if ((now() - last_static_publish_).seconds() >= 1.0) {publishStaticArtifacts();}
    if (state_ == State::PRE_EVENT && elapsed >= get_parameter("pre_event_seconds").as_double()) {state_ = State::CROSSING;}
    if (state_ == State::CROSSING && elapsed >= get_parameter("pre_event_seconds").as_double() + get_parameter("post_event_seconds").as_double() / 2.0) {state_ = State::POST_EVENT;}
    if (elapsed >= get_parameter("pre_event_seconds").as_double() + get_parameter("post_event_seconds").as_double()) {state_ = State::TEST_COMPLETE; publishState(); timer_->cancel(); rclcpp::shutdown();}
  }
  State state_{State::WAIT_SYSTEM}; bool scope_enabled_{false}, scope_ready_{false}, recorder_ready_{false}, pending_{false}, have_costmap_{false};
  rclcpp::Time epoch_{0, 0, RCL_ROS_TIME}, last_static_publish_{0, 0, RCL_ROS_TIME}; nav2_msgs::msg::Costmap costmap_; nav_msgs::msg::Path reference_, goal_; geometry_msgs::msg::Pose frozen_; double frozen_yaw_{0.0}; std::string frozen_frame_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_; rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, path_pub_, goal_pub_; rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr scope_sub_; rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recorder_ready_sub_; rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace
int main(int argc, char ** argv) {rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CausalityHarness>()); if (rclcpp::ok()) {rclcpp::shutdown();} return 0;}
