#ifndef M1_SCOPE_COSTMAP_LAYER__MASTER_SNAPSHOT_LAYER_HPP_
#define M1_SCOPE_COSTMAP_LAYER__MASTER_SNAPSHOT_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace m1_scope_costmap_layer
{

// A strictly read-only Layer used to capture the composed master grid at a
// precise position in the configured layer order.  It never changes bounds or
// costs, so it cannot affect planning or safety behavior.
class MasterSnapshotLayer : public nav2_costmap_2d::Layer
{
public:
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
    int, int, int, int) override;
  void reset() override {}
  bool isClearable() override {return false;}

protected:
  void onInitialize() override;

private:
  bool enabled_{true};
  double publish_rate_{20.0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr publisher_;
};

}  // namespace m1_scope_costmap_layer

#endif
