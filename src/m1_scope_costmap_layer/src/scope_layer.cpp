#include "m1_scope_costmap_layer/scope_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

namespace m1_scope_costmap_layer
{
namespace
{

using Point = std::array<double, 2>;

struct Polygon
{
  std::array<Point, 12> points{};
  size_t size{0};
};

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool samePose(const geometry_msgs::msg::Pose & first, const geometry_msgs::msg::Pose & second)
{
  return first.position.x == second.position.x &&
         first.position.y == second.position.y &&
         first.position.z == second.position.z &&
         first.orientation.x == second.orientation.x &&
         first.orientation.y == second.orientation.y &&
         first.orientation.z == second.orientation.z &&
         first.orientation.w == second.orientation.w;
}

Point transformPoint(
  const nav_msgs::msg::OccupancyGrid & grid, double local_x, double local_y)
{
  const auto & quaternion = grid.info.origin.orientation;
  const double yaw = std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return Point{
    grid.info.origin.position.x + cosine * local_x - sine * local_y,
    grid.info.origin.position.y + sine * local_x + cosine * local_y};
}

Polygon clipPolygon(
  const Polygon & input, size_t axis, double boundary, bool keep_greater)
{
  Polygon output;
  if (input.size == 0) {
    return output;
  }
  auto signed_distance = [axis, boundary](const Point & point) {
      return point[axis] - boundary;
    };
  auto inside = [keep_greater](double distance) {
      return keep_greater ? distance >= 0.0 : distance <= 0.0;
    };

  auto append = [&output](const Point & point) {
      output.points[output.size++] = point;
    };

  Point start = input.points[input.size - 1];
  double start_distance = signed_distance(start);
  bool start_inside = inside(start_distance);
  for (size_t index = 0; index < input.size; ++index) {
    const auto & end = input.points[index];
    const double end_distance = signed_distance(end);
    const bool end_inside = inside(end_distance);
    if (start_inside != end_inside) {
      const double denominator = start_distance - end_distance;
      const double fraction = denominator == 0.0 ? 0.0 : start_distance / denominator;
      append(
        Point{
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1])});
    }
    if (end_inside) {
      append(end);
    }
    start = end;
    start_distance = end_distance;
    start_inside = end_inside;
  }
  return output;
}

long double polygonArea(const Polygon & polygon)
{
  if (polygon.size < 3) {
    return 0.0L;
  }
  const auto & origin = polygon.points[0];
  long double twice_area = 0.0L;
  for (size_t index = 1; index + 1 < polygon.size; ++index) {
    const long double first_x = polygon.points[index][0] - origin[0];
    const long double first_y = polygon.points[index][1] - origin[1];
    const long double second_x = polygon.points[index + 1][0] - origin[0];
    const long double second_y = polygon.points[index + 1][1] - origin[1];
    twice_area += first_x * second_y - first_y * second_x;
  }
  return std::abs(twice_area) * 0.5L;
}

bool positiveAreaIntersection(
  const Polygon & polygon, double min_x, double min_y,
  double max_x, double max_y)
{
  auto clipped = clipPolygon(polygon, 0, min_x, true);
  clipped = clipPolygon(clipped, 0, max_x, false);
  clipped = clipPolygon(clipped, 1, min_y, true);
  clipped = clipPolygon(clipped, 1, max_y, false);
  return polygonArea(clipped) > 0.0L;
}

bool safeFloorToInt(double value, int & result)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double floored = std::floor(value);
  if (floored < static_cast<double>(std::numeric_limits<int>::min()) ||
    floored > static_cast<double>(std::numeric_limits<int>::max()))
  {
    return false;
  }
  result = static_cast<int>(floored);
  return true;
}

}  // namespace

std::array<char, kRasterStatsDiagnosticCapacity> formatRasterStats(
  const RasterStats & stats)
{
  std::array<char, kRasterStatsDiagnosticCapacity> diagnostic{};
  std::snprintf(
    diagnostic.data(), diagnostic.size(),
    "SCOPE raster stats valid=%zu clipped=%zu medium=%zu lethal=%zu",
    stats.valid_source_cells, stats.clipped_source_cells,
    stats.medium_master_cells, stats.lethal_master_cells);
  return diagnostic;
}

ScopeLayer::ScopeLayer()
{
  current_ = true;
  enabled_ = true;
}

void ScopeLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SCOPE layer lifecycle node expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("prediction_topic", rclcpp::ParameterValue(std::string("/scope/prediction")));
  declareParameter("uncertainty_topic", rclcpp::ParameterValue(std::string("/scope/uncertainty")));
  declareParameter("low_threshold", rclcpp::ParameterValue(0.35));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(0.60));
  declareParameter("uncertainty_gain", rclcpp::ParameterValue(1.0));
  declareParameter("uncertainty_encoding_scale", rclcpp::ParameterValue(0.5));
  declareParameter("medium_cost", rclcpp::ParameterValue(200));
  declareParameter("temporal_prediction_hard_obstacle", rclcpp::ParameterValue(true));
  declareParameter("stale_timeout", rclcpp::ParameterValue(0.5));

  Config initial;
  node->get_parameter(getFullName("enabled"), initial.enabled);
  node->get_parameter(getFullName("prediction_topic"), initial.prediction_topic);
  node->get_parameter(getFullName("uncertainty_topic"), initial.uncertainty_topic);
  node->get_parameter(getFullName("low_threshold"), initial.low_threshold);
  node->get_parameter(getFullName("lethal_threshold"), initial.lethal_threshold);
  node->get_parameter(getFullName("uncertainty_gain"), initial.uncertainty_gain);
  node->get_parameter(
    getFullName("uncertainty_encoding_scale"), initial.uncertainty_encoding_scale);
  node->get_parameter(getFullName("medium_cost"), initial.medium_cost);
  node->get_parameter(
    getFullName("temporal_prediction_hard_obstacle"), initial.temporal_prediction_hard_obstacle);
  node->get_parameter(getFullName("stale_timeout"), initial.stale_timeout);
  std::string reason;
  if (!validConfig(initial, reason)) {
    throw std::runtime_error("Invalid SCOPE layer parameters: " + reason);
  }
  constexpr uint64_t initial_generation = 1;
  auto subscriptions = makeSubscriptions(initial, initial_generation);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = initial;
    cycle_config_ = initial;
    enabled_ = initial.enabled;
    subscription_generation_ = initial_generation;
    prediction_subscription_ = std::move(subscriptions.prediction);
    uncertainty_subscription_ = std::move(subscriptions.uncertainty);
  }

  parameter_callback_ = node->add_on_set_parameters_callback(
    std::bind(&ScopeLayer::onParametersChanged, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "SCOPE costmap layer listens to '%s' and '%s' in frame '%s'",
    initial.prediction_topic.c_str(), initial.uncertainty_topic.c_str(),
    layered_costmap_->getGlobalFrameID().c_str());
}

ScopeLayer::SubscriptionPair ScopeLayer::makeSubscriptions(
  const Config & config, uint64_t generation)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SCOPE layer lifecycle node expired while creating subscriptions");
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(kMaximumPendingPerTopic)).reliable();
  SubscriptionPair subscriptions;
  subscriptions.prediction = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    config.prediction_topic, qos,
    [this, generation](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
      predictionCallback(std::move(message), generation);
    }, options);
  subscriptions.uncertainty = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    config.uncertainty_topic, qos,
    [this, generation](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
      uncertaintyCallback(std::move(message), generation);
    }, options);
  return subscriptions;
}

rcl_interfaces::msg::SetParametersResult ScopeLayer::onParametersChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> parameter_lock(parameter_update_mutex_);
  Config next;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    next = config_;
  }
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == getFullName("enabled")) {
      next.enabled = parameter.as_bool();
    } else if (name == getFullName("prediction_topic")) {
      next.prediction_topic = parameter.as_string();
    } else if (name == getFullName("uncertainty_topic")) {
      next.uncertainty_topic = parameter.as_string();
    } else if (name == getFullName("low_threshold")) {
      next.low_threshold = parameter.as_double();
    } else if (name == getFullName("lethal_threshold")) {
      next.lethal_threshold = parameter.as_double();
    } else if (name == getFullName("uncertainty_gain")) {
      next.uncertainty_gain = parameter.as_double();
    } else if (name == getFullName("uncertainty_encoding_scale")) {
      next.uncertainty_encoding_scale = parameter.as_double();
    } else if (name == getFullName("medium_cost")) {
      next.medium_cost = static_cast<int>(parameter.as_int());
    } else if (name == getFullName("temporal_prediction_hard_obstacle")) {
      next.temporal_prediction_hard_obstacle = parameter.as_bool();
    } else if (name == getFullName("stale_timeout")) {
      next.stale_timeout = parameter.as_double();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = validConfig(next, result.reason);
  if (!result.successful) {
    return result;
  }
  Config previous;
  uint64_t previous_generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous = config_;
    previous_generation = subscription_generation_;
  }
  const bool topic_changed =
    next.prediction_topic != previous.prediction_topic ||
    next.uncertainty_topic != previous.uncertainty_topic;
  if (topic_changed) {
    const uint64_t next_generation = previous_generation + 1;
    SubscriptionPair subscriptions;
    try {
      subscriptions = makeSubscriptions(next, next_generation);
    } catch (const std::exception & exception) {
      result.successful = false;
      result.reason = std::string("failed to create SCOPE subscriptions: ") + exception.what();
      return result;
    }
    SubscriptionPair previous_subscriptions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_subscriptions.prediction = std::move(prediction_subscription_);
      previous_subscriptions.uncertainty = std::move(uncertainty_subscription_);
      prediction_subscription_ = std::move(subscriptions.prediction);
      uncertainty_subscription_ = std::move(subscriptions.uncertainty);
      config_ = next;
      enabled_ = next.enabled;
      subscription_generation_ = next_generation;
      pending_predictions_.clear();
      pending_uncertainties_.clear();
      last_receive_time_.reset();
      latest_snapshot_.reset();
      cycle_snapshot_.reset();
    }
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = next;
    enabled_ = next.enabled;
  }
  return result;
}

bool ScopeLayer::validConfig(const Config & config, std::string & reason) const
{
  auto node = node_.lock();
  if (!node) {
    reason = "lifecycle node expired";
    return false;
  }
  try {
    rclcpp::expand_topic_or_service_name(
      config.prediction_topic, node->get_name(), node->get_namespace());
    rclcpp::expand_topic_or_service_name(
      config.uncertainty_topic, node->get_name(), node->get_namespace());
  } catch (const std::exception & exception) {
    reason = std::string("invalid input topic name: ") + exception.what();
    return false;
  }
  if (!std::isfinite(config.low_threshold) || !std::isfinite(config.lethal_threshold) ||
    config.low_threshold < 0.0 || config.lethal_threshold > 1.0 ||
    config.low_threshold > config.lethal_threshold)
  {
    reason = "thresholds must satisfy 0 <= low_threshold <= lethal_threshold <= 1";
    return false;
  }
  if (!std::isfinite(config.uncertainty_gain) || config.uncertainty_gain < 0.0 ||
    !std::isfinite(config.uncertainty_encoding_scale) ||
    config.uncertainty_encoding_scale < 0.0)
  {
    reason = "uncertainty gain and encoding scale must be finite and nonnegative";
    return false;
  }
  if (config.medium_cost <= nav2_costmap_2d::FREE_SPACE ||
    config.medium_cost > nav2_costmap_2d::MAX_NON_OBSTACLE)
  {
    reason = "medium_cost must be in [1, 252]";
    return false;
  }
  constexpr int64_t nanoseconds_per_second = 1000000000LL;
  constexpr double max_safe_timeout_seconds = static_cast<double>(
    std::numeric_limits<int64_t>::max() / nanoseconds_per_second);
  if (!std::isfinite(config.stale_timeout) || config.stale_timeout <= 0.0 ||
    config.stale_timeout > max_safe_timeout_seconds)
  {
    reason = "stale_timeout must be finite, positive, and safely representable in nanoseconds";
    return false;
  }
  reason.clear();
  return true;
}

void ScopeLayer::predictionCallback(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr message, uint64_t generation)
{
  bufferGrid(*message, clock_->now(), true, generation);
}

void ScopeLayer::uncertaintyCallback(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr message, uint64_t generation)
{
  bufferGrid(*message, clock_->now(), false, generation);
}

void ScopeLayer::bufferPrediction(
  const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at)
{
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = subscription_generation_;
  }
  bufferGrid(message, received_at, true, generation);
}

void ScopeLayer::bufferUncertainty(
  const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at)
{
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = subscription_generation_;
  }
  bufferGrid(message, received_at, false, generation);
}

void ScopeLayer::bufferGrid(
  const nav_msgs::msg::OccupancyGrid & message, const rclcpp::Time & received_at,
  bool prediction, uint64_t generation)
{
  Bounds message_bounds;
  if (!isValidGrid(message, message_bounds)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Rejected invalid or wrong-frame SCOPE OccupancyGrid");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != subscription_generation_) {
    return;
  }
  if (last_receive_time_ && received_at < *last_receive_time_) {
    pending_predictions_.clear();
    pending_uncertainties_.clear();
    latest_snapshot_.reset();
  }
  last_receive_time_ = received_at;
  auto & own_queue = prediction ? pending_predictions_ : pending_uncertainties_;
  auto & other_queue = prediction ? pending_uncertainties_ : pending_predictions_;
  own_queue.push_back(PendingGrid{message, received_at, message_bounds});
  while (own_queue.size() > kMaximumPendingPerTopic) {
    own_queue.pop_front();
  }

  auto own = std::prev(own_queue.end());
  auto other = std::find_if(
    other_queue.begin(), other_queue.end(),
    [&message](const PendingGrid & candidate) {
      return matchingKey(message, candidate.message);
    });
  if (other == other_queue.end()) {
    return;
  }

  const auto & prediction_grid = prediction ? own->message : other->message;
  const auto & uncertainty_grid = prediction ? other->message : own->message;
  const rclcpp::Time stamp(prediction_grid.header.stamp);
  const rclcpp::Time completed_at = own->received_at > other->received_at ?
    own->received_at : other->received_at;
  if (!latest_snapshot_ || stamp >= latest_snapshot_->stamp) {
    auto snapshot = std::make_shared<Snapshot>();
    snapshot->prediction = prediction_grid;
    snapshot->uncertainty = uncertainty_grid;
    snapshot->stamp = stamp;
    snapshot->received_at = completed_at;
    snapshot->bounds = prediction ? own->bounds : other->bounds;
    snapshot->generation = generation;
    latest_snapshot_ = std::move(snapshot);
  }
  other_queue.erase(other);
  own_queue.erase(own);
}

bool ScopeLayer::isValidGrid(
  const nav_msgs::msg::OccupancyGrid & message, Bounds & bounds) const
{
  constexpr uint32_t nanoseconds_per_second = 1000000000u;
  if (!layered_costmap_ || message.header.stamp.sec < 0 ||
    message.header.stamp.nanosec >= nanoseconds_per_second ||
    message.header.frame_id.empty() ||
    message.header.frame_id != layered_costmap_->getGlobalFrameID() ||
    message.info.width == 0 || message.info.height == 0 ||
    !std::isfinite(message.info.resolution) || message.info.resolution <= 0.0 ||
    !finitePose(message.info.origin))
  {
    return false;
  }
  const auto & orientation = message.info.origin.orientation;
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 1e-3 ||
    std::abs(orientation.x) > 1e-6 || std::abs(orientation.y) > 1e-6)
  {
    return false;
  }
  const size_t expected_size =
    static_cast<size_t>(message.info.width) * static_cast<size_t>(message.info.height);
  if (message.data.size() != expected_size) {
    return false;
  }
  if (!std::all_of(
      message.data.begin(), message.data.end(),
      [](int8_t value) {return value >= -1 && value <= 100;}))
  {
    return false;
  }
  const double width =
    static_cast<double>(message.info.width) * message.info.resolution;
  const double height =
    static_cast<double>(message.info.height) * message.info.resolution;
  return std::isfinite(width) && width > 0.0 &&
         std::isfinite(height) && height > 0.0 &&
         gridBounds(message, bounds);
}

bool ScopeLayer::matchingKey(
  const nav_msgs::msg::OccupancyGrid & first,
  const nav_msgs::msg::OccupancyGrid & second)
{
  return first.header.stamp.sec == second.header.stamp.sec &&
         first.header.stamp.nanosec == second.header.stamp.nanosec &&
         first.header.frame_id == second.header.frame_id &&
         first.info.width == second.info.width &&
         first.info.height == second.info.height &&
         first.info.resolution == second.info.resolution &&
         samePose(first.info.origin, second.info.origin);
}

bool ScopeLayer::gridBounds(
  const nav_msgs::msg::OccupancyGrid & message, Bounds & bounds)
{
  const double width = static_cast<double>(message.info.width) * message.info.resolution;
  const double height = static_cast<double>(message.info.height) * message.info.resolution;
  const std::array<Point, 4> corners{
    transformPoint(message, 0.0, 0.0), transformPoint(message, width, 0.0),
    transformPoint(message, width, height), transformPoint(message, 0.0, height)};
  for (const auto & corner : corners) {
    if (!std::isfinite(corner[0]) || !std::isfinite(corner[1])) {
      return false;
    }
  }
  bounds.min_x = bounds.max_x = corners.front()[0];
  bounds.min_y = bounds.max_y = corners.front()[1];
  for (const auto & corner : corners) {
    bounds.min_x = std::min(bounds.min_x, corner[0]);
    bounds.min_y = std::min(bounds.min_y, corner[1]);
    bounds.max_x = std::max(bounds.max_x, corner[0]);
    bounds.max_y = std::max(bounds.max_y, corner[1]);
  }
  return bounds.max_x > bounds.min_x && bounds.max_y > bounds.min_y;
}

void ScopeLayer::includeBounds(
  const Bounds & bounds, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, bounds.min_x);
  *min_y = std::min(*min_y, bounds.min_y);
  *max_x = std::max(*max_x, bounds.max_x);
  *max_y = std::max(*max_y, bounds.max_y);
}

void ScopeLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cycle_config_ = config_;
  cycle_snapshot_.reset();
  if (rendered_bounds_) {
    includeBounds(*rendered_bounds_, min_x, min_y, max_x, max_y);
  }
  if (!config_.enabled) {
    current_ = true;
    return;
  }

  const auto now = clock_->now();
  if (latest_snapshot_) {
    const int64_t age_ns = (now - latest_snapshot_->received_at).nanoseconds();
    const int64_t timeout_ns = rclcpp::Duration::from_seconds(config_.stale_timeout).nanoseconds();
    if (age_ns < 0) {
      latest_snapshot_.reset();
      pending_predictions_.clear();
      pending_uncertainties_.clear();
      last_receive_time_ = now;
    } else if (age_ns <= timeout_ns) {
      cycle_snapshot_ = latest_snapshot_;
      includeBounds(latest_snapshot_->bounds, min_x, min_y, max_x, max_y);
    }
  }
  current_ = true;
}

void ScopeLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::shared_ptr<const Snapshot> snapshot;
  Config config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = cycle_snapshot_;
    config = cycle_config_;
    last_stats_ = RasterStats{};
    if (!snapshot) {
      rendered_bounds_.reset();
      return;
    }
  }

  RasterStats stats;
  const auto & prediction = snapshot->prediction;
  const auto & uncertainty = snapshot->uncertainty;
  const double master_resolution = master_grid.getResolution();
  const double source_resolution = static_cast<double>(prediction.info.resolution);
  const double master_min_x = master_grid.getOriginX();
  const double master_min_y = master_grid.getOriginY();
  const unsigned int master_size_x = master_grid.getSizeInCellsX();
  const unsigned int master_size_y = master_grid.getSizeInCellsY();
  if (!std::isfinite(master_resolution) || master_resolution <= 0.0 ||
    !std::isfinite(master_min_x) || !std::isfinite(master_min_y) ||
    master_size_x > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
    master_size_y > static_cast<unsigned int>(std::numeric_limits<int>::max()))
  {
    return;
  }
  const int clipped_min_i = std::max(0, min_i);
  const int clipped_min_j = std::max(0, min_j);
  const int clipped_max_i = std::min(static_cast<int>(master_size_x), max_i);
  const int clipped_max_j = std::min(static_cast<int>(master_size_y), max_j);
  if (clipped_max_i <= clipped_min_i || clipped_max_j <= clipped_min_j) {
    return;
  }
  const double update_min_x = master_min_x + clipped_min_i * master_resolution;
  const double update_min_y = master_min_y + clipped_min_j * master_resolution;
  const double update_max_x = master_min_x + clipped_max_i * master_resolution;
  const double update_max_y = master_min_y + clipped_max_j * master_resolution;
  if (!std::isfinite(update_min_x) || !std::isfinite(update_min_y) ||
    !std::isfinite(update_max_x) || !std::isfinite(update_max_y) ||
    update_max_x <= update_min_x || update_max_y <= update_min_y)
  {
    return;
  }
  const size_t scratch_width = static_cast<size_t>(clipped_max_i - clipped_min_i);
  const size_t scratch_height = static_cast<size_t>(clipped_max_j - clipped_min_j);
  if (scratch_width > std::numeric_limits<size_t>::max() / scratch_height) {
    return;
  }
  std::vector<unsigned char> rasterized(
    scratch_width * scratch_height, nav2_costmap_2d::FREE_SPACE);

  for (unsigned int source_y = 0; source_y < prediction.info.height; ++source_y) {
    for (unsigned int source_x = 0; source_x < prediction.info.width; ++source_x) {
      const size_t source_index =
        static_cast<size_t>(source_y) * prediction.info.width + source_x;
      const int prediction_value = prediction.data[source_index];
      const int uncertainty_value = uncertainty.data[source_index];
      if (prediction_value < 0 || uncertainty_value < 0) {
        continue;
      }
      ++stats.valid_source_cells;

      const double local_x = static_cast<double>(source_x) * source_resolution;
      const double local_y = static_cast<double>(source_y) * source_resolution;
      Polygon polygon;
      polygon.size = 4;
      polygon.points[0] = transformPoint(prediction, local_x, local_y);
      polygon.points[1] = transformPoint(
        prediction, local_x + source_resolution, local_y);
      polygon.points[2] = transformPoint(
        prediction, local_x + source_resolution, local_y + source_resolution);
      polygon.points[3] = transformPoint(
        prediction, local_x, local_y + source_resolution);
      double polygon_min_x = polygon.points[0][0];
      double polygon_max_x = polygon.points[0][0];
      double polygon_min_y = polygon.points[0][1];
      double polygon_max_y = polygon.points[0][1];
      for (size_t index = 0; index < polygon.size; ++index) {
        const auto & point = polygon.points[index];
        if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
          polygon_min_x = std::numeric_limits<double>::quiet_NaN();
          break;
        }
        polygon_min_x = std::min(polygon_min_x, point[0]);
        polygon_max_x = std::max(polygon_max_x, point[0]);
        polygon_min_y = std::min(polygon_min_y, point[1]);
        polygon_max_y = std::max(polygon_max_y, point[1]);
      }
      if (!std::isfinite(polygon_min_x)) {
        continue;
      }
      const bool extends_left = polygon_min_x < update_min_x;
      const bool extends_down = polygon_min_y < update_min_y;
      const bool extends_right = polygon_max_x > update_max_x;
      const bool extends_up = polygon_max_y > update_max_y;
      const bool clipped = extends_left || extends_down || extends_right || extends_up;
      if (clipped) {
        ++stats.clipped_source_cells;
      }

      const double risk = std::min(
        1.0, static_cast<double>(prediction_value) / 100.0 +
        config.uncertainty_gain * config.uncertainty_encoding_scale *
        static_cast<double>(uncertainty_value) / 100.0);
      if (risk < config.low_threshold || polygon_max_x <= update_min_x ||
        polygon_min_x >= update_max_x || polygon_max_y <= update_min_y ||
        polygon_min_y >= update_max_y)
      {
        continue;
      }
      // A t+0.2 s forecast cannot safely be treated as a timeless hard wall
      // throughout the 1.943 s MPPI rollout.  In temporal-soft mode it still
      // raises local MPPI cost, while scan-observed obstacles remain lethal.
      const unsigned char cost = risk >= config.lethal_threshold &&
        config.temporal_prediction_hard_obstacle ? nav2_costmap_2d::LETHAL_OBSTACLE :
        static_cast<unsigned char>(config.medium_cost);

      const double cropped_min_x = std::max(polygon_min_x, update_min_x);
      const double cropped_min_y = std::max(polygon_min_y, update_min_y);
      const double cropped_max_x = std::min(polygon_max_x, update_max_x);
      const double cropped_max_y = std::min(polygon_max_y, update_max_y);
      int candidate_min_i = 0;
      int candidate_max_i = 0;
      int candidate_min_j = 0;
      int candidate_max_j = 0;
      if (!safeFloorToInt(
          (cropped_min_x - master_min_x) / master_resolution, candidate_min_i) ||
        !safeFloorToInt(
          (cropped_max_x - master_min_x) / master_resolution, candidate_max_i) ||
        !safeFloorToInt(
          (cropped_min_y - master_min_y) / master_resolution, candidate_min_j) ||
        !safeFloorToInt(
          (cropped_max_y - master_min_y) / master_resolution, candidate_max_j))
      {
        continue;
      }
      candidate_min_i = std::max(candidate_min_i, clipped_min_i);
      candidate_min_j = std::max(candidate_min_j, clipped_min_j);
      candidate_max_i = std::min(candidate_max_i, clipped_max_i - 1);
      candidate_max_j = std::min(candidate_max_j, clipped_max_j - 1);
      for (int master_y = candidate_min_j; master_y <= candidate_max_j; ++master_y) {
        for (int master_x = candidate_min_i; master_x <= candidate_max_i; ++master_x) {
          const double cell_min_x = master_min_x + master_x * master_resolution;
          const double cell_min_y = master_min_y + master_y * master_resolution;
          if (!positiveAreaIntersection(
              polygon, cell_min_x, cell_min_y,
              cell_min_x + master_resolution, cell_min_y + master_resolution))
          {
            continue;
          }
          const size_t scratch_index =
            static_cast<size_t>(master_y - clipped_min_j) * scratch_width +
            static_cast<size_t>(master_x - clipped_min_i);
          rasterized[scratch_index] = std::max(rasterized[scratch_index], cost);
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot->generation != subscription_generation_ || cycle_snapshot_ != snapshot) {
      return;
    }
    for (size_t scratch_y = 0; scratch_y < scratch_height; ++scratch_y) {
      for (size_t scratch_x = 0; scratch_x < scratch_width; ++scratch_x) {
        const unsigned char cost = rasterized[scratch_y * scratch_width + scratch_x];
        if (cost == nav2_costmap_2d::FREE_SPACE) {
          continue;
        }
        const auto master_x = static_cast<unsigned int>(
          clipped_min_i + static_cast<int>(scratch_x));
        const auto master_y = static_cast<unsigned int>(
          clipped_min_j + static_cast<int>(scratch_y));
        const unsigned char master_cost = master_grid.getCost(master_x, master_y);
        if (master_cost == nav2_costmap_2d::NO_INFORMATION || cost > master_cost) {
          master_grid.setCost(master_x, master_y, cost);
          if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
            ++stats.lethal_master_cells;
          } else {
            ++stats.medium_master_cells;
          }
        }
      }
    }
    last_stats_ = stats;
    rendered_bounds_ = snapshot->bounds;
  }
  const auto diagnostic = formatRasterStats(stats);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", diagnostic.data());
}

void ScopeLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_predictions_.clear();
  pending_uncertainties_.clear();
  last_receive_time_.reset();
  latest_snapshot_.reset();
  cycle_snapshot_.reset();
  last_stats_ = RasterStats{};
  current_ = true;
}

bool ScopeLayer::isClearable()
{
  return true;
}

RasterStats ScopeLayer::getLastStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_stats_;
}

}  // namespace m1_scope_costmap_layer

PLUGINLIB_EXPORT_CLASS(m1_scope_costmap_layer::ScopeLayer, nav2_costmap_2d::Layer)
