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

#pragma once

#include <cstddef>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/debug/debug_analyzer.hpp"

namespace frontier_exploration_ros2::debug
{

// Shared RViz marker styling. The observer keeps these values separate from the
// analyzer so score computation is independent from how overlays are displayed.
struct DebugMarkerConfig
{
  std::string frame_id{"map"};
  double point_scale{0.15};
  double selected_scale{0.30};
  double line_width{0.04};
  double text_scale{0.22};
  double z_offset{0.05};
  bool labels_enabled{true};
  std::size_t label_top_n{30};
  std::size_t edge_top_n{15};
};

// Frontier set before decision-map optimization. This overlay is useful for
// checking what the map contributes before filtering and smoothing change it.
visualization_msgs::msg::MarkerArray make_raw_frontier_markers(
  const FrontierDebugSnapshot & snapshot,
  const DebugMarkerConfig & config);

// Frontier set after decision-map optimization, plus the active first target.
visualization_msgs::msg::MarkerArray make_optimized_frontier_markers(
  const FrontierDebugSnapshot & snapshot,
  const DebugMarkerConfig & config);

// Nearest strategy explanation: candidate pool, rank, mode, and distances.
visualization_msgs::msg::MarkerArray make_nearest_score_markers(
  const FrontierDebugSnapshot & snapshot,
  const DebugMarkerConfig & config);

// MRTSP start-row score explanation: rank, gain, path cost, and time cost.
visualization_msgs::msg::MarkerArray make_mrtsp_score_markers(
  const FrontierDebugSnapshot & snapshot,
  const DebugMarkerConfig & config);

// MRTSP or DP route visualization. The line shows the analyzed sequence, while
// labels identify the order in which candidates appear in that sequence.
visualization_msgs::msg::MarkerArray make_mrtsp_order_markers(
  const FrontierDebugSnapshot & snapshot,
  const geometry_msgs::msg::Pose & current_pose,
  const DebugMarkerConfig & config);

// DP pruning visualization. Orange points are passed into bounded-horizon DP;
// grey points are outside the pruned candidate pool.
visualization_msgs::msg::MarkerArray make_dp_pruning_markers(
  const FrontierDebugSnapshot & snapshot,
  const DebugMarkerConfig & config);

}  // namespace frontier_exploration_ros2::debug
