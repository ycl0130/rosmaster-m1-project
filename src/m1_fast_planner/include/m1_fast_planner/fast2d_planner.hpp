#ifndef M1_FAST_PLANNER__FAST2D_PLANNER_HPP_
#define M1_FAST_PLANNER__FAST2D_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "m1_fast_planner/kinodynamic_astar.hpp"

namespace m1_fast_planner
{

class Fast2DPlanner : public nav2_core::GlobalPlanner
{
public:
  Fast2DPlanner() = default;
  ~Fast2DPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  static PlanarState rotateBodyVelocityToPlanningFrame(
    double body_vx, double body_vy, double yaw);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("m1_fast_planner")};
  std::string name_;
  std::string global_frame_;
  double path_resolution_{0.05};
  bool allow_unknown_{true};
  bool active_{false};
  KinodynamicAstarConfig search_config_;
  std::string odom_topic_{"/odom"};
  double odom_timeout_{0.5};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Time latest_odom_received_{0, 0, RCL_ROS_TIME};
  std::mutex odom_mutex_;
};

}  // namespace m1_fast_planner

#endif  // M1_FAST_PLANNER__FAST2D_PLANNER_HPP_
