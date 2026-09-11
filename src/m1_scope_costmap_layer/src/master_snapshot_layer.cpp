#include "m1_scope_costmap_layer/master_snapshot_layer.hpp"

#include <algorithm>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace m1_scope_costmap_layer
{

void MasterSnapshotLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("snapshot layer lifecycle node expired");}
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/local_costmap/debug/snapshot")));
  declareParameter("publish_rate", rclcpp::ParameterValue(20.0));
  std::string topic;
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic);
  node->get_parameter(getFullName("publish_rate"), publish_rate_);
  if (publish_rate_ <= 0.0) {throw std::runtime_error("snapshot publish_rate must be positive");}
  publisher_ = node->create_publisher<nav2_msgs::msg::Costmap>(topic, rclcpp::QoS(2).reliable());
  current_ = true;
  RCLCPP_INFO(logger_, "Read-only master snapshot layer publishes '%s'", topic.c_str());
}

void MasterSnapshotLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
  int, int, int, int)
{
  if (!enabled_ || !publisher_) {return;}
  auto node = node_.lock();
  if (!node) {return;}
  const auto now = node->now();
  if (last_publish_.nanoseconds() != 0 && (now - last_publish_).seconds() < 1.0 / publish_rate_) {return;}
  nav2_msgs::msg::Costmap message;
  message.header.stamp = now;
  message.header.frame_id = layered_costmap_->getGlobalFrameID();
  message.metadata.resolution = master_grid.getResolution();
  message.metadata.size_x = master_grid.getSizeInCellsX();
  message.metadata.size_y = master_grid.getSizeInCellsY();
  message.metadata.origin.position.x = master_grid.getOriginX();
  message.metadata.origin.position.y = master_grid.getOriginY();
  message.metadata.origin.orientation.w = 1.0;
  const auto count = static_cast<std::size_t>(message.metadata.size_x) * message.metadata.size_y;
  message.data.resize(count);
  std::copy_n(master_grid.getCharMap(), count, message.data.begin());
  publisher_->publish(std::move(message));
  last_publish_ = now;
}

}  // namespace m1_scope_costmap_layer

PLUGINLIB_EXPORT_CLASS(m1_scope_costmap_layer::MasterSnapshotLayer, nav2_costmap_2d::Layer)
