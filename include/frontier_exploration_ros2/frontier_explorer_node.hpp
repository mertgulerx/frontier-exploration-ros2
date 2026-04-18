/*
Copyright 2026 Mert Güler

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

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/qos_utils.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{

// ROS 2 wiring layer for FrontierExplorerCore (subscriptions, actions, worker queue, diagnostics).
class FrontierExplorerNode : public rclcpp::Node
{
public:
  FrontierExplorerNode();
  ~FrontierExplorerNode() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void publishCompletionEvent();

  std::optional<geometry_msgs::msg::Pose> getCurrentPose();
  void publishFrontierMarkers(const FrontierSequence & frontiers);
  void publishSelectedFrontierPose(const geometry_msgs::msg::PoseStamped & pose);
  void publishOptimizedMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  bool frontierMapOptimizationEnabled() const;
  bool debugOutputsEnabled() const;

  void dispatchGoalRequest(const GoalDispatchRequest & request);
  void createMapSubscription(rclcpp::DurabilityPolicy map_durability);
  void mapAutodetectTimeoutCallback();
  void logMapAutodetectStart(rclcpp::DurabilityPolicy selected_durability);
  void logMapAutodetectSwitch(rclcpp::DurabilityPolicy selected_durability);
  void logMapAutodetectComplete(
    const std::string & result,
    rclcpp::DurabilityPolicy selected_durability);
  double mapAutodetectElapsedSeconds() const;
  void suppressionWatchdogCallback();
  static int mapResultCodeToGoalStatus(rclcpp_action::ResultCode code);

  FrontierExplorerCoreParams params_;
  std::unique_ptr<FrontierExplorerCore> core_;
  struct CompletionEventConfig
  {
    bool enabled{false};
    // A transient-local Empty message keeps the event transport generic and cheap.
    std::string topic{"exploration_complete"};
  } completion_event_config_;

  // Parsed QoS policy and startup autodetect configuration.
  TopicQosProfiles topic_qos_profiles_;
  bool map_qos_autodetect_on_startup_{false};
  double map_qos_autodetect_timeout_s_{2.0};

  // ROS interfaces.
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr completion_event_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_frontier_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr optimized_map_pub_;

  // Subscriptions and timers.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::TimerBase::SharedPtr map_autodetect_timer_;
  rclcpp::TimerBase::SharedPtr suppression_watchdog_timer_;
  bool suppression_activation_logged_{false};
  std::optional<std::chrono::steady_clock::time_point> suppression_activation_at_;

  // Startup-only map QoS autodetect state.
  std::mutex map_autodetect_mutex_;
  bool map_received_once_{false};
  bool map_autodetect_complete_logged_{false};
  std::chrono::steady_clock::time_point map_autodetect_started_at_;
  std::optional<MapQosStartupAutodetect> map_qos_autodetect_;

  // Completion can be observed multiple times while the frontier set stays empty.
  bool completion_event_published_{false};

  // TF warning throttling state to avoid log spam.
  std::optional<int64_t> last_tf_warning_time_ns_;
  double tf_warning_throttle_seconds_{5.0};
};

}  // namespace frontier_exploration_ros2
