/*
Copyright 2026 Mert Guler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/debug/debug_analyzer.hpp"
#include "frontier_exploration_ros2/debug/debug_markers.hpp"
#include "frontier_exploration_ros2/qos_utils.hpp"

namespace frontier_exploration_ros2::debug
{

namespace
{

FrontierStrategy parse_strategy(std::string value)
{
  // The observer accepts the same strategy names as the explorer. Unknown values
  // fall back to nearest analysis so the debug node still starts with a baseline.
  for (auto & ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (value == "mrtsp") {
    return FrontierStrategy::MRTSP;
  }
  return FrontierStrategy::NEAREST;
}

std::size_t positive_size_parameter(
  const rclcpp::Node & node,
  const std::string & name,
  int fallback)
{
  // Label limits and DP settings are clamped to at least one item. This prevents
  // invalid launch values from disabling arrays in ways that look like topic loss.
  const int value = node.get_parameter(name).as_int();
  if (value > 0) {
    return static_cast<std::size_t>(value);
  }
  return static_cast<std::size_t>(std::max(1, fallback));
}

}  // namespace

class FrontierDebugObserverNode : public rclcpp::Node
{
public:
  explicit FrontierDebugObserverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("frontier_debug_observer", options)
  {
    // This node is intentionally passive: it subscribes, analyzes, and publishes
    // RViz overlays, but it never sends Nav2 goals or mutates explorer state.
    declare_parameters();
    read_parameters();
    create_ros_interfaces();
    RCLCPP_INFO(
      get_logger(),
      "Frontier debug observer ready: map='%s', costmap='%s', local_costmap='%s', strategy='%s'",
      map_topic_.c_str(),
      costmap_topic_.c_str(),
      local_costmap_topic_.c_str(),
      analyzer_config_.strategy == FrontierStrategy::MRTSP ? "mrtsp" : "nearest");
  }

private:
  void declare_parameters()
  {
    // Input topics and frames mirror the explorer node so the debug observer can
    // analyze the same map/costmap/TF context from a separate process.
    declare_parameter<std::string>("map_topic", "map");
    declare_parameter<std::string>("costmap_topic", "global_costmap/costmap");
    declare_parameter<std::string>("local_costmap_topic", "local_costmap/costmap");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_footprint");
    declare_parameter<std::string>("strategy", "mrtsp");
    declare_parameter<std::string>("map_qos_durability", "transient_local");
    declare_parameter<std::string>("map_qos_reliability", "reliable");
    declare_parameter<int>("map_qos_depth", 1);
    declare_parameter<std::string>("costmap_qos_reliability", "reliable");
    declare_parameter<int>("costmap_qos_depth", 10);
    declare_parameter<std::string>("local_costmap_qos_reliability", "inherit");
    declare_parameter<int>("local_costmap_qos_depth", -1);

    // Explorer scoring parameters are declared here as read-only inputs for the
    // analyzer. They are not used to control exploration from this node.
    declare_parameter<bool>("frontier_map_optimization_enabled", true);
    declare_parameter<double>("sigma_s", 2.0);
    declare_parameter<double>("sigma_r", 30.0);
    declare_parameter<int>("dilation_kernel_radius_cells", 1);
    declare_parameter<double>("sensor_effective_range_m", 1.5);
    declare_parameter<double>("weight_distance_wd", 1.0);
    declare_parameter<double>("weight_gain_ws", 1.0);
    declare_parameter<double>("max_linear_speed_vmax", 0.5);
    declare_parameter<double>("max_angular_speed_wmax", 1.0);
    declare_parameter<std::string>("mrtsp_solver", "dp");
    declare_parameter<int>("dp_solver_candidate_limit", 15);
    declare_parameter<int>("dp_planning_horizon", 10);
    declare_parameter<int>("occ_threshold", OCC_THRESHOLD);
    declare_parameter<int>("min_frontier_size_cells", MIN_FRONTIER_SIZE);
    declare_parameter<double>("frontier_candidate_min_goal_distance_m", 0.0);
    declare_parameter<double>("frontier_selection_min_distance", 0.5);
    declare_parameter<double>("frontier_visit_tolerance", 0.30);
    declare_parameter<bool>("escape_enabled", true);

    // RViz output parameters keep visual density adjustable without changing the
    // score analysis. Top-N limits protect RViz from very dense frontier sets.
    declare_parameter<double>("debug_update_rate_hz", 1.0);
    declare_parameter<bool>("debug_labels_enabled", true);
    declare_parameter<int>("debug_label_top_n", 30);
    declare_parameter<int>("debug_edge_top_n", 15);
    declare_parameter<double>("debug_marker_scale", 0.15);
    declare_parameter<double>("debug_selected_marker_scale", 0.30);
    declare_parameter<double>("debug_line_width", 0.04);
    declare_parameter<double>("debug_text_scale", 0.22);
    declare_parameter<bool>("debug_show_raw_frontiers", true);
    declare_parameter<bool>("debug_show_optimized_frontiers", true);
    declare_parameter<bool>("debug_show_nearest_scores", true);
    declare_parameter<bool>("debug_show_mrtsp_scores", true);
    declare_parameter<bool>("debug_show_mrtsp_order", true);
    declare_parameter<bool>("debug_show_dp_pruning", true);
    declare_parameter<bool>("debug_show_decision_map", true);
    declare_parameter<std::string>("debug_raw_frontiers_topic", "explore/debug/raw_frontiers");
    declare_parameter<std::string>("debug_optimized_frontiers_topic", "explore/debug/optimized_frontiers");
    declare_parameter<std::string>("debug_nearest_scores_topic", "explore/debug/nearest_scores");
    declare_parameter<std::string>("debug_mrtsp_scores_topic", "explore/debug/mrtsp_scores");
    declare_parameter<std::string>("debug_mrtsp_order_topic", "explore/debug/mrtsp_order");
    declare_parameter<std::string>("debug_dp_pruning_topic", "explore/debug/dp_pruning");
    declare_parameter<std::string>("debug_decision_map_topic", "explore/debug/decision_map");
  }

  void read_parameters()
  {
    // Parameters are copied into plain structs once at startup. The observer is
    // designed for stable debug sessions rather than dynamic reconfiguration.
    map_topic_ = get_parameter("map_topic").as_string();
    costmap_topic_ = get_parameter("costmap_topic").as_string();
    local_costmap_topic_ = get_parameter("local_costmap_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();

    analyzer_config_.strategy = parse_strategy(get_parameter("strategy").as_string());
    analyzer_config_.frontier_map_optimization_enabled =
      get_parameter("frontier_map_optimization_enabled").as_bool();
    analyzer_config_.sigma_s = get_parameter("sigma_s").as_double();
    analyzer_config_.sigma_r = get_parameter("sigma_r").as_double();
    analyzer_config_.dilation_kernel_radius_cells =
      get_parameter("dilation_kernel_radius_cells").as_int();
    analyzer_config_.sensor_effective_range_m =
      get_parameter("sensor_effective_range_m").as_double();
    analyzer_config_.weight_distance_wd = get_parameter("weight_distance_wd").as_double();
    analyzer_config_.weight_gain_ws = get_parameter("weight_gain_ws").as_double();
    analyzer_config_.max_linear_speed_vmax = get_parameter("max_linear_speed_vmax").as_double();
    analyzer_config_.max_angular_speed_wmax = get_parameter("max_angular_speed_wmax").as_double();
    analyzer_config_.mrtsp_solver = get_parameter("mrtsp_solver").as_string();
    analyzer_config_.dp_solver_candidate_limit =
      positive_size_parameter(*this, "dp_solver_candidate_limit", 15);
    analyzer_config_.dp_planning_horizon =
      positive_size_parameter(*this, "dp_planning_horizon", 10);
    analyzer_config_.occ_threshold = get_parameter("occ_threshold").as_int();
    analyzer_config_.min_frontier_size_cells = get_parameter("min_frontier_size_cells").as_int();
    analyzer_config_.frontier_candidate_min_goal_distance_m =
      get_parameter("frontier_candidate_min_goal_distance_m").as_double();
    analyzer_config_.frontier_selection_min_distance =
      get_parameter("frontier_selection_min_distance").as_double();
    analyzer_config_.frontier_visit_tolerance =
      get_parameter("frontier_visit_tolerance").as_double();
    analyzer_config_.escape_enabled = get_parameter("escape_enabled").as_bool();

    // Clamp visual scales and update rate to useful ranges. Extreme values can
    // make RViz overlays invisible or unnecessarily expensive to refresh.
    update_rate_hz_ = std::clamp(get_parameter("debug_update_rate_hz").as_double(), 0.1, 10.0);
    marker_config_.frame_id = global_frame_;
    marker_config_.point_scale = std::max(0.01, get_parameter("debug_marker_scale").as_double());
    marker_config_.selected_scale =
      std::max(0.02, get_parameter("debug_selected_marker_scale").as_double());
    marker_config_.line_width = std::max(0.005, get_parameter("debug_line_width").as_double());
    marker_config_.text_scale = std::max(0.05, get_parameter("debug_text_scale").as_double());
    marker_config_.labels_enabled = get_parameter("debug_labels_enabled").as_bool();
    marker_config_.label_top_n = positive_size_parameter(*this, "debug_label_top_n", 30);
    marker_config_.edge_top_n = positive_size_parameter(*this, "debug_edge_top_n", 15);

    show_raw_frontiers_ = get_parameter("debug_show_raw_frontiers").as_bool();
    show_optimized_frontiers_ = get_parameter("debug_show_optimized_frontiers").as_bool();
    show_nearest_scores_ = get_parameter("debug_show_nearest_scores").as_bool();
    show_mrtsp_scores_ = get_parameter("debug_show_mrtsp_scores").as_bool();
    show_mrtsp_order_ = get_parameter("debug_show_mrtsp_order").as_bool();
    show_dp_pruning_ = get_parameter("debug_show_dp_pruning").as_bool();
    show_decision_map_ = get_parameter("debug_show_decision_map").as_bool();

    // Only compute optional score families when their overlays need them. MRTSP
    // order still needs MRTSP scores because it depends on the same cost matrix.
    analyzer_config_.analyze_nearest_scores = show_nearest_scores_;
    analyzer_config_.analyze_mrtsp_scores = show_mrtsp_scores_ || show_mrtsp_order_;
    analyzer_config_.analyze_dp_pruning = show_dp_pruning_;

    raw_frontiers_topic_ = get_parameter("debug_raw_frontiers_topic").as_string();
    optimized_frontiers_topic_ = get_parameter("debug_optimized_frontiers_topic").as_string();
    nearest_scores_topic_ = get_parameter("debug_nearest_scores_topic").as_string();
    mrtsp_scores_topic_ = get_parameter("debug_mrtsp_scores_topic").as_string();
    mrtsp_order_topic_ = get_parameter("debug_mrtsp_order_topic").as_string();
    dp_pruning_topic_ = get_parameter("debug_dp_pruning_topic").as_string();
    decision_map_topic_ = get_parameter("debug_decision_map_topic").as_string();

    topic_qos_profiles_ = resolve_topic_qos_profiles(
      get_parameter("map_qos_durability").as_string(),
      get_parameter("map_qos_reliability").as_string(),
      get_parameter("map_qos_depth").as_int(),
      get_parameter("costmap_qos_reliability").as_string(),
      get_parameter("costmap_qos_depth").as_int(),
      get_parameter("local_costmap_qos_reliability").as_string(),
      get_parameter("local_costmap_qos_depth").as_int());
  }

  void create_ros_interfaces()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions only cache the latest grids. The timer copies the cached
    // values under a mutex, then performs analysis without holding the lock.
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      topic_qos_profiles_.make_map_qos(topic_qos_profiles_.map_durability),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_ = OccupancyGrid2d(msg);
      });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      topic_qos_profiles_.make_costmap_qos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        costmap_ = OccupancyGrid2d(msg);
      });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_,
      topic_qos_profiles_.make_local_costmap_qos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_costmap_ = OccupancyGrid2d(msg);
      });

    raw_frontiers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      raw_frontiers_topic_,
      10);
    optimized_frontiers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      optimized_frontiers_topic_,
      10);
    nearest_scores_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      nearest_scores_topic_,
      10);
    mrtsp_scores_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      mrtsp_scores_topic_,
      10);
    mrtsp_order_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      mrtsp_order_topic_,
      10);
    dp_pruning_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      dp_pruning_topic_,
      10);
    decision_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      decision_map_topic_,
      10);

    // The periodic timer makes debug output predictable and avoids running heavy
    // analysis directly inside map or costmap subscription callbacks.
    analysis_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      std::bind(&FrontierDebugObserverNode::analysis_timer_callback, this));
  }

  std::optional<geometry_msgs::msg::Pose> get_current_pose()
  {
    try {
      // The observer uses the latest available transform to match RViz and map
      // overlays. Missing TF simply skips this tick instead of blocking startup.
      const auto transform = tf_buffer_->lookupTransform(
        global_frame_,
        robot_base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & exc) {
      throttled_warn(
        "frontier_debug_observer_tf",
        "Could not transform " + robot_base_frame_ + " -> " + global_frame_ + ": " + exc.what());
      return std::nullopt;
    }
  }

  void throttled_warn(const std::string & key, const std::string & message)
  {
    // Missing maps, TF, or analysis inputs can happen during startup. Throttling
    // keeps logs useful while still showing persistent configuration problems.
    const int64_t now_ns = get_clock()->now().nanoseconds();
    auto & last_time = warning_times_[key];
    const int64_t throttle_ns = static_cast<int64_t>(5.0 * 1e9);
    if (!last_time.has_value() || now_ns - *last_time >= throttle_ns) {
      last_time = now_ns;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  void analysis_timer_callback()
  {
    std::optional<OccupancyGrid2d> map;
    std::optional<OccupancyGrid2d> costmap;
    std::optional<OccupancyGrid2d> local_costmap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map = map_;
      costmap = costmap_;
      local_costmap = local_costmap_;
    }

    // A global costmap is required because frontier eligibility and blocked-goal
    // checks depend on it. The local costmap remains optional for debug analysis.
    if (!map.has_value() || !costmap.has_value()) {
      throttled_warn(
        "frontier_debug_observer_maps",
        "Waiting for map and global costmap before publishing frontier debug overlays");
      return;
    }

    const auto current_pose = get_current_pose();
    if (!current_pose.has_value()) {
      return;
    }

    try {
      // The analyzer returns a complete, immutable snapshot for this tick. Each
      // publisher then converts the same snapshot into a separate RViz layer.
      const FrontierDebugSnapshot snapshot = analyze_frontier_debug_snapshot(
        *current_pose,
        *map,
        *costmap,
        local_costmap,
        analyzer_config_);

      if (show_raw_frontiers_) {
        raw_frontiers_pub_->publish(make_raw_frontier_markers(snapshot, marker_config_));
      }
      if (show_optimized_frontiers_) {
        optimized_frontiers_pub_->publish(make_optimized_frontier_markers(snapshot, marker_config_));
      }
      if (show_nearest_scores_) {
        nearest_scores_pub_->publish(make_nearest_score_markers(snapshot, marker_config_));
      }
      if (show_mrtsp_scores_) {
        mrtsp_scores_pub_->publish(make_mrtsp_score_markers(snapshot, marker_config_));
      }
      if (show_mrtsp_order_) {
        mrtsp_order_pub_->publish(make_mrtsp_order_markers(snapshot, *current_pose, marker_config_));
      }
      if (show_dp_pruning_) {
        dp_pruning_pub_->publish(make_dp_pruning_markers(snapshot, marker_config_));
      }
      if (show_decision_map_) {
        decision_map_pub_->publish(snapshot.decision_map_msg);
      }
      if (!first_successful_publish_logged_) {
        // A one-time success message confirms that all required inputs were
        // received and at least one complete overlay set reached the publishers.
        first_successful_publish_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Frontier debug overlays published successfully: raw_frontiers=%zu, optimized_frontiers=%zu, candidates=%zu, active_mode='%s'",
          snapshot.raw_frontiers.size(),
          snapshot.optimized_frontiers.size(),
          snapshot.candidates.size(),
          snapshot.active_selection_mode.c_str());
      }
    } catch (const std::exception & exc) {
      throttled_warn(
        "frontier_debug_observer_analysis",
        std::string("Could not build frontier debug snapshot: ") + exc.what());
    }
  }

  DebugAnalyzerConfig analyzer_config_;
  DebugMarkerConfig marker_config_;
  TopicQosProfiles topic_qos_profiles_;

  std::string map_topic_;
  std::string costmap_topic_;
  std::string local_costmap_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string raw_frontiers_topic_;
  std::string optimized_frontiers_topic_;
  std::string nearest_scores_topic_;
  std::string mrtsp_scores_topic_;
  std::string mrtsp_order_topic_;
  std::string dp_pruning_topic_;
  std::string decision_map_topic_;

  double update_rate_hz_{1.0};
  bool show_raw_frontiers_{true};
  bool show_optimized_frontiers_{true};
  bool show_nearest_scores_{true};
  bool show_mrtsp_scores_{true};
  bool show_mrtsp_order_{true};
  bool show_dp_pruning_{true};
  bool show_decision_map_{true};
  bool first_successful_publish_logged_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nearest_scores_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrtsp_scores_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrtsp_order_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dp_pruning_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr decision_map_pub_;
  rclcpp::TimerBase::SharedPtr analysis_timer_;

  std::mutex data_mutex_;
  std::optional<OccupancyGrid2d> map_;
  std::optional<OccupancyGrid2d> costmap_;
  std::optional<OccupancyGrid2d> local_costmap_;
  std::unordered_map<std::string, std::optional<int64_t>> warning_times_;
};

}  // namespace frontier_exploration_ros2::debug

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frontier_exploration_ros2::debug::FrontierDebugObserverNode>());
  rclcpp::shutdown();
  return 0;
}
