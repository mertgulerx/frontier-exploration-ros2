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

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include "frontier_explorer_core_detail.hpp"

#include <algorithm>
#include <utility>

namespace frontier_exploration_ros2
{

FrontierExplorerCore::FrontierExplorerCore(
  FrontierExplorerCoreParams params_in,
  FrontierExplorerCoreCallbacks callbacks_in)
: params(std::move(params_in)), callbacks(std::move(callbacks_in))
{
  params.all_frontiers_suppressed_behavior = detail::normalize_suppressed_behavior(
    params.all_frontiers_suppressed_behavior);
  // Clamp settle configuration to safe minimums only when the settle gate is enabled.
  if (params.post_goal_settle_enabled) {
    params.post_goal_min_settle = std::max(0.0, params.post_goal_min_settle);
    params.post_goal_required_map_updates = std::max(1, params.post_goal_required_map_updates);
    params.post_goal_stable_updates = std::max(1, params.post_goal_stable_updates);
    params.post_goal_required_map_updates = std::max(
      params.post_goal_required_map_updates,
      params.post_goal_stable_updates);
  }
  params.frontier_suppression_attempt_threshold = std::max(
    1,
    params.frontier_suppression_attempt_threshold);
  params.frontier_suppression_base_size_m = std::max(0.1, params.frontier_suppression_base_size_m);
  params.frontier_suppression_expansion_size_m = std::max(0.0, params.frontier_suppression_expansion_size_m);
  params.frontier_suppression_timeout_s = std::max(0.1, params.frontier_suppression_timeout_s);
  params.frontier_suppression_no_progress_timeout_s = std::max(
    0.1,
    params.frontier_suppression_no_progress_timeout_s);
  params.frontier_suppression_progress_epsilon_m = std::max(
    0.0,
    params.frontier_suppression_progress_epsilon_m);
  params.frontier_suppression_startup_grace_period_s = std::max(
    0.0,
    params.frontier_suppression_startup_grace_period_s);
  params.frontier_suppression_max_attempt_records = std::max(
    1,
    params.frontier_suppression_max_attempt_records);
  params.frontier_suppression_max_regions = std::max(1, params.frontier_suppression_max_regions);
  params.sigma_s = std::max(params.sigma_s, 1e-6);
  params.sigma_r = std::max(params.sigma_r, 1e-6);
  params.dilation_kernel_radius_cells = std::max(0, params.dilation_kernel_radius_cells);
  params.sensor_effective_range_m = std::max(0.0, params.sensor_effective_range_m);
  params.weight_distance_wd = std::max(0.0, params.weight_distance_wd);
  params.weight_gain_ws = std::max(0.0, params.weight_gain_ws);
  params.max_linear_speed_vmax = std::max(1e-6, params.max_linear_speed_vmax);
  params.max_angular_speed_wmax = std::max(1e-6, params.max_angular_speed_wmax);
  params.occ_threshold = std::clamp(params.occ_threshold, 0, 100);
  params.min_frontier_size_cells = std::max(1, params.min_frontier_size_cells);
  params.frontier_candidate_min_goal_distance_m = std::max(
    0.0,
    params.frontier_candidate_min_goal_distance_m);
  // Visible-gain geometry is clamped here so later preemption checks can assume valid ranges.
  params.goal_preemption_lidar_range_m = std::max(
    0.1,
    params.goal_preemption_lidar_range_m);
  params.goal_preemption_lidar_fov_deg = std::clamp(
    params.goal_preemption_lidar_fov_deg,
    1.0,
    360.0);
  params.goal_preemption_lidar_ray_step_deg = std::clamp(
    params.goal_preemption_lidar_ray_step_deg,
    0.25,
    45.0);
  params.goal_preemption_complete_if_within_m = std::max(
    0.0,
    params.goal_preemption_complete_if_within_m);
  params.goal_preemption_lidar_min_reveal_length_m = std::max(
    0.0,
    params.goal_preemption_lidar_min_reveal_length_m);

  escape_active = params.escape_enabled;

  // Defensive defaults keep unit tests and partial hosts from crashing.
  if (!callbacks.now_ns) {
    callbacks.now_ns = []() {return int64_t{0};};
  }
  if (!callbacks.get_current_pose) {
    callbacks.get_current_pose = []() -> std::optional<geometry_msgs::msg::Pose> {return std::nullopt;};
  }
  if (!callbacks.wait_for_action_server) {
    callbacks.wait_for_action_server = [](double) {return true;};
  }
  if (!callbacks.dispatch_goal_request) {
    callbacks.dispatch_goal_request = [](const GoalDispatchRequest &) {};
  }
  if (!callbacks.publish_frontier_markers) {
    callbacks.publish_frontier_markers = [](const FrontierSequence &) {};
  }
  if (!callbacks.publish_selected_frontier_pose) {
    callbacks.publish_selected_frontier_pose = [](const geometry_msgs::msg::PoseStamped &) {};
  }
  if (!callbacks.publish_optimized_map) {
    callbacks.publish_optimized_map = [](const nav_msgs::msg::OccupancyGrid &) {};
  }
  if (!callbacks.on_exploration_complete) {
    callbacks.on_exploration_complete = []() {};
  }
  if (!callbacks.debug_outputs_enabled) {
    callbacks.debug_outputs_enabled = []() {return false;};
  }
  if (!callbacks.log_debug) {
    callbacks.log_debug = [](const std::string &) {};
  }
  if (!callbacks.log_info) {
    callbacks.log_info = [](const std::string &) {};
  }
  if (!callbacks.log_warn) {
    callbacks.log_warn = [](const std::string &) {};
  }
  if (!callbacks.log_error) {
    callbacks.log_error = [](const std::string &) {};
  }
  if (!callbacks.frontier_search) {
    // Fallback path delegates to package-level frontier extraction implementation.
    callbacks.frontier_search = [this](
      const geometry_msgs::msg::Pose & pose,
      const OccupancyGrid2d & occupancy,
      const OccupancyGrid2d & global_costmap,
      const std::optional<OccupancyGrid2d> & local,
      double min_goal_distance,
      bool return_robot_cell) {
      return get_frontier(
        pose,
        occupancy,
        global_costmap,
        local,
        min_goal_distance,
        return_robot_cell,
        frontier_search_options());
    };
  }

  if (params.frontier_suppression_enabled) {
    frontier_suppression_activation_ns_ =
      callbacks.now_ns() +
      static_cast<int64_t>(params.frontier_suppression_startup_grace_period_s * 1e9);
  }
}

bool FrontierExplorerCore::debug_outputs_enabled() const
{
  return callbacks.debug_outputs_enabled();
}

bool FrontierExplorerCore::mrtsp_enabled() const
{
  return params.strategy == FrontierStrategy::MRTSP;
}

bool FrontierExplorerCore::frontier_map_optimization_enabled() const
{
  return mrtsp_enabled() || params.frontier_map_optimization_enabled;
}

FrontierSearchOptions FrontierExplorerCore::frontier_search_options() const
{
  FrontierSearchOptions options;
  options.occ_threshold = params.occ_threshold;
  options.min_frontier_size_cells = params.min_frontier_size_cells;
  options.candidate_min_goal_distance_m = params.frontier_candidate_min_goal_distance_m;
  options.use_local_costmap_for_frontier_eligibility = !mrtsp_enabled();
  options.out_of_bounds_costmap_is_blocked = mrtsp_enabled();
  options.build_navigation_goal_point = !mrtsp_enabled();
  return options;
}

DecisionMapConfig FrontierExplorerCore::decision_map_config() const
{
  DecisionMapConfig config;
  config.optimization_enabled = frontier_map_optimization_enabled();
  config.occ_threshold = params.occ_threshold;
  config.sigma_s = params.sigma_s;
  config.sigma_r = params.sigma_r;
  config.dilation_kernel_radius_cells = params.dilation_kernel_radius_cells;
  return config;
}

FrontierSequence FrontierExplorerCore::to_frontier_sequence(
  const std::vector<FrontierCandidate> & frontiers)
{
  FrontierSequence sequence;
  // Preserve extraction order for deterministic policy behavior.
  sequence.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    sequence.push_back(frontier);
  }
  return sequence;
}

}  // namespace frontier_exploration_ros2
