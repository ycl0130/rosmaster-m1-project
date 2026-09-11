#ifndef M1_SCOPE_COSTMAP_LAYER__SCOPE_LAYER_HPP_
#define M1_SCOPE_COSTMAP_LAYER__SCOPE_LAYER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace m1_scope_costmap_layer
{

struct RasterStats
{
  // Matched source cells for which both inputs are known (-1 is unknown).
  size_t valid_source_cells{0};
  // Valid source cells not fully contained by the current master update window.
  size_t clipped_source_cells{0};
  // Master cells whose value was actually raised to the corresponding SCOPE cost.
  size_t medium_master_cells{0};
  size_t lethal_master_cells{0};
};

constexpr size_t kRasterStatsDiagnosticCapacity = 96;

std::array<char, kRasterStatsDiagnosticCapacity> formatRasterStats(
  const RasterStats & stats);

class ScopeLayerTestPeer;

class ScopeLayer : public nav2_costmap_2d::Layer
{
public:
  ScopeLayer();
  ~ScopeLayer() override = default;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

  RasterStats getLastStats() const;

protected:
  void onInitialize() override;

private:
  friend class ScopeLayerTestPeer;

  struct Config
  {
    bool enabled{true};
    std::string prediction_topic{"/scope/prediction"};
    std::string uncertainty_topic{"/scope/uncertainty"};
    double low_threshold{0.35};
    double lethal_threshold{0.60};
    double uncertainty_gain{1.0};
    double uncertainty_encoding_scale{0.5};
    int medium_cost{200};
    bool temporal_prediction_hard_obstacle{true};
    double stale_timeout{0.5};
  };

  struct Bounds
  {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
  };

  struct PendingGrid
  {
    nav_msgs::msg::OccupancyGrid message;
    rclcpp::Time received_at;
    Bounds bounds;
  };

  struct Snapshot
  {
    nav_msgs::msg::OccupancyGrid prediction;
    nav_msgs::msg::OccupancyGrid uncertainty;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
    Bounds bounds;
    uint64_t generation{0};
  };

  struct SubscriptionPair
  {
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prediction;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr uncertainty;
  };

  static constexpr size_t kMaximumPendingPerTopic = 4;

  void predictionCallback(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message, uint64_t generation);
  void uncertaintyCallback(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message, uint64_t generation);
  void bufferPrediction(
    const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at);
  void bufferUncertainty(
    const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at);
  void bufferGrid(
    const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at,
    bool prediction, uint64_t generation);
  SubscriptionPair makeSubscriptions(const Config & config, uint64_t generation);
  rcl_interfaces::msg::SetParametersResult onParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters);

  bool isValidGrid(
    const nav_msgs::msg::OccupancyGrid & message, Bounds & bounds) const;
  static bool matchingKey(
    const nav_msgs::msg::OccupancyGrid & first,
    const nav_msgs::msg::OccupancyGrid & second);
  static bool gridBounds(
    const nav_msgs::msg::OccupancyGrid & message, Bounds & bounds);
  bool validConfig(const Config & config, std::string & reason) const;
  static void includeBounds(
    const Bounds & bounds, double * min_x, double * min_y,
    double * max_x, double * max_y);

  mutable std::mutex mutex_;
  std::mutex parameter_update_mutex_;
  Config config_;
  uint64_t subscription_generation_{0};
  std::deque<PendingGrid> pending_predictions_;
  std::deque<PendingGrid> pending_uncertainties_;
  std::optional<rclcpp::Time> last_receive_time_;
  std::shared_ptr<Snapshot> latest_snapshot_;
  std::shared_ptr<const Snapshot> cycle_snapshot_;
  Config cycle_config_;
  std::optional<Bounds> rendered_bounds_;
  RasterStats last_stats_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prediction_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr uncertainty_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace m1_scope_costmap_layer

#endif  // M1_SCOPE_COSTMAP_LAYER__SCOPE_LAYER_HPP_
