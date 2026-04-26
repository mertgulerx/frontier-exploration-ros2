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
  params.mrtsp_direction_bias_weight = std::max(0.0, params.mrtsp_direction_bias_weight);
  params.mrtsp_direction_reverse_penalty_weight = std::max(
    0.0,
    params.mrtsp_direction_reverse_penalty_weight);
  params.mrtsp_direction_memory_mode = normalize_direction_memory_mode(
    params.mrtsp_direction_memory_mode);
  params.mrtsp_direction_min_motion_m = std::max(0.0, params.mrtsp_direction_min_motion_m);
  params.mrtsp_visited_radius_m = std::max(0.0, params.mrtsp_visited_radius_m);
  params.mrtsp_visited_penalty_weight = std::max(0.0, params.mrtsp_visited_penalty_weight);
  params.mrtsp_visited_history_max_size = std::max(1, params.mrtsp_visited_history_max_size);
  params.mrtsp_visited_history_timeout_s = std::max(0.0, params.mrtsp_visited_history_timeout_s);
  params.mrtsp_low_gain_distance_decay_weight = std::max(
    0.0,
    params.mrtsp_low_gain_distance_decay_weight);
  params.mrtsp_low_gain_distance_decay_lambda = std::max(
    1e-6,
    params.mrtsp_low_gain_distance_decay_lambda);
  params.mrtsp_low_gain_min_frontier_size_cells = std::max(
    1e-6,
    params.mrtsp_low_gain_min_frontier_size_cells);
  params.soft_obstacle_penalty_weight = std::max(0.0, params.soft_obstacle_penalty_weight);
  params.soft_obstacle_cost_start = std::clamp(params.soft_obstacle_cost_start, 0, 100);
  params.soft_obstacle_cost_max = std::clamp(params.soft_obstacle_cost_max, 0, 100);
  if (params.soft_obstacle_cost_max <= params.soft_obstacle_cost_start) {
    if (params.soft_obstacle_cost_start >= 100) {
      params.soft_obstacle_cost_start = 99;
    }
    params.soft_obstacle_cost_max = params.soft_obstacle_cost_start + 1;
  }
  params.candidate_visible_gain_weight = std::max(0.0, params.candidate_visible_gain_weight);
  params.candidate_visible_gain_max_candidates = std::max(
    1,
    params.candidate_visible_gain_max_candidates);
  params.candidate_visible_gain_ray_step_deg = std::clamp(
    params.candidate_visible_gain_ray_step_deg,
    0.25,
    45.0);
  params.candidate_visible_gain_mode = normalize_visible_gain_mode(
    params.candidate_visible_gain_mode);
  params.frontier_score_debug_top_n = std::max(1, params.frontier_score_debug_top_n);
  params.advanced_viewpoint_sampling_method = normalize_advanced_viewpoint_method(
    params.advanced_viewpoint_sampling_method);
  params.advanced_viewpoint_max_frontiers = std::clamp(
    params.advanced_viewpoint_max_frontiers,
    1,
    100);
  params.advanced_viewpoint_max_samples_per_frontier = std::clamp(
    params.advanced_viewpoint_max_samples_per_frontier,
    1,
    500);
  params.advanced_viewpoint_max_total_samples = std::clamp(
    params.advanced_viewpoint_max_total_samples,
    1,
    5000);
  params.advanced_viewpoint_runtime_budget_ms = std::clamp(
    params.advanced_viewpoint_runtime_budget_ms,
    0.0,
    1000.0);
  params.advanced_viewpoint_min_goal_distance_m = std::max(
    0.0,
    params.advanced_viewpoint_min_goal_distance_m);
  params.advanced_viewpoint_max_goal_distance_m = std::max(
    0.0,
    params.advanced_viewpoint_max_goal_distance_m);
  if (
    params.advanced_viewpoint_max_goal_distance_m > 0.0 &&
    params.advanced_viewpoint_max_goal_distance_m < params.advanced_viewpoint_min_goal_distance_m)
  {
    params.advanced_viewpoint_max_goal_distance_m = params.advanced_viewpoint_min_goal_distance_m;
  }
  params.rrt_max_nodes = std::clamp(params.rrt_max_nodes, 1, 500);
  params.rrt_max_iterations = std::clamp(params.rrt_max_iterations, 1, 2000);
  params.rrt_step_size_m = std::clamp(params.rrt_step_size_m, 0.01, 10.0);
  params.rrt_goal_bias_probability = std::clamp(params.rrt_goal_bias_probability, 0.0, 1.0);
  params.rrt_frontier_bias_probability = std::clamp(params.rrt_frontier_bias_probability, 0.0, 1.0);
  if (params.rrt_goal_bias_probability + params.rrt_frontier_bias_probability > 1.0) {
    params.rrt_frontier_bias_probability = 1.0 - params.rrt_goal_bias_probability;
  }
  params.rrt_min_sample_radius_m = std::max(0.0, params.rrt_min_sample_radius_m);
  params.rrt_sampling_radius_m = std::max(params.rrt_sampling_radius_m, params.rrt_min_sample_radius_m);
  params.rrt_sampling_radius_m = std::max(params.rrt_sampling_radius_m, params.rrt_step_size_m);
  params.rrt_collision_check_step_m = std::clamp(params.rrt_collision_check_step_m, 0.005, 1.0);
  params.rrt_polar_angle_bins = std::max(0, params.rrt_polar_angle_bins);
  params.monte_carlo_gain_mode = normalize_monte_carlo_gain_mode(params.monte_carlo_gain_mode);
  params.monte_carlo_gain_weight = std::max(0.0, params.monte_carlo_gain_weight);
  params.monte_carlo_gain_sample_count = std::clamp(params.monte_carlo_gain_sample_count, 1, 5000);
  params.monte_carlo_gain_max_candidates = std::clamp(params.monte_carlo_gain_max_candidates, 1, 100);
  params.monte_carlo_gain_runtime_budget_ms = std::clamp(
    params.monte_carlo_gain_runtime_budget_ms,
    0.0,
    1000.0);
  params.monte_carlo_gain_sensor_range_m = std::max(0.1, params.monte_carlo_gain_sensor_range_m);
  params.monte_carlo_gain_sensor_fov_deg = std::clamp(
    params.monte_carlo_gain_sensor_fov_deg,
    1.0,
    360.0);
  params.monte_carlo_gain_min_range_m = std::clamp(
    params.monte_carlo_gain_min_range_m,
    0.0,
    params.monte_carlo_gain_sensor_range_m);
  params.monte_carlo_gain_unknown_value = std::max(0.0, params.monte_carlo_gain_unknown_value);
  params.monte_carlo_gain_free_value = std::max(0.0, params.monte_carlo_gain_free_value);
  params.monte_carlo_gain_occupied_value = std::max(0.0, params.monte_carlo_gain_occupied_value);
  params.monte_carlo_gain_near_field_radius_fraction = std::clamp(
    params.monte_carlo_gain_near_field_radius_fraction,
    0.0,
    1.0);
  params.monte_carlo_gain_far_field_sample_stride = std::clamp(
    params.monte_carlo_gain_far_field_sample_stride,
    1,
    100);
  params.monte_carlo_gain_resolution_decay_lambda = std::max(
    0.0,
    params.monte_carlo_gain_resolution_decay_lambda);
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

MrtspScoringOptions FrontierExplorerCore::mrtsp_scoring_options() const
{
  MrtspScoringOptions options;
  options.direction_bias_enabled = params.mrtsp_direction_bias_enabled;
  options.direction_bias_weight = params.mrtsp_direction_bias_weight;
  options.direction_reverse_penalty_weight = params.mrtsp_direction_reverse_penalty_weight;
  options.direction_memory_mode = params.mrtsp_direction_memory_mode;
  options.direction_min_motion_m = params.mrtsp_direction_min_motion_m;
  options.visited_penalty_enabled = params.mrtsp_visited_penalty_enabled;
  options.visited_radius_m = params.mrtsp_visited_radius_m;
  options.visited_penalty_weight = params.mrtsp_visited_penalty_weight;
  options.visited_history_max_size = params.mrtsp_visited_history_max_size;
  options.visited_history_timeout_s = params.mrtsp_visited_history_timeout_s;
  options.low_gain_distance_decay_enabled = params.mrtsp_low_gain_distance_decay_enabled;
  options.low_gain_distance_decay_weight = params.mrtsp_low_gain_distance_decay_weight;
  options.low_gain_distance_decay_lambda = params.mrtsp_low_gain_distance_decay_lambda;
  options.low_gain_min_frontier_size_cells = params.mrtsp_low_gain_min_frontier_size_cells;
  options.soft_obstacle_penalty_enabled = params.soft_obstacle_penalty_enabled;
  options.soft_obstacle_penalty_weight = params.soft_obstacle_penalty_weight;
  options.soft_obstacle_cost_start = params.soft_obstacle_cost_start;
  options.soft_obstacle_cost_max = params.soft_obstacle_cost_max;
  return options;
}

MrtspScoringContext FrontierExplorerCore::mrtsp_scoring_context(
  const std::vector<FrontierCandidate> & candidates,
  const geometry_msgs::msg::Pose & current_pose) const
{
  MrtspScoringContext context;
  context.now_ns = callbacks.now_ns();
  context.visited_frontiers.assign(
    visited_frontier_history_.begin(),
    visited_frontier_history_.end());

  const double yaw = detail::yaw_from_quaternion(current_pose.orientation);
  const std::array<double, 2> yaw_direction{std::cos(yaw), std::sin(yaw)};
  const std::pair<double, double> current_position{
    current_pose.position.x,
    current_pose.position.y,
  };
  const auto use_delta_direction =
    [&current_position](
      const std::optional<std::pair<double, double>> & previous,
      double min_motion_m) -> std::optional<std::array<double, 2>>
    {
      if (!previous.has_value()) {
        return std::nullopt;
      }
      const std::array<double, 2> direction{
        current_position.first - previous->first,
        current_position.second - previous->second,
      };
      if (std::hypot(direction[0], direction[1]) < min_motion_m) {
        return std::nullopt;
      }
      return direction;
    };

  if (params.mrtsp_direction_memory_mode == "robot_yaw") {
    context.desired_direction = yaw_direction;
  } else if (params.mrtsp_direction_memory_mode == "last_goal") {
    context.desired_direction =
      use_delta_direction(last_frontier_goal_position_, params.mrtsp_direction_min_motion_m)
      .value_or(yaw_direction);
  } else {
    context.desired_direction =
      use_delta_direction(previous_robot_position_, params.mrtsp_direction_min_motion_m)
      .value_or(yaw_direction);
  }

  context.obstacle_costs.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    const auto point = candidate.goal_point.value_or(candidate.center_point);
    const auto local_cost = world_point_cost(local_costmap, point);
    const auto global_cost = world_point_cost(costmap, point);
    if (local_cost.has_value() && global_cost.has_value()) {
      context.obstacle_costs.push_back(std::max(*local_cost, *global_cost));
    } else if (local_cost.has_value()) {
      context.obstacle_costs.push_back(*local_cost);
    } else {
      context.obstacle_costs.push_back(global_cost);
    }
  }

  return context;
}

AdvancedViewpointSamplingConfig FrontierExplorerCore::advanced_viewpoint_sampling_config() const
{
  AdvancedViewpointSamplingConfig config;
  config.enabled = params.advanced_viewpoint_sampling_enabled;
  config.method = params.advanced_viewpoint_sampling_method;
  config.max_frontiers = params.advanced_viewpoint_max_frontiers;
  config.max_samples_per_frontier = params.advanced_viewpoint_max_samples_per_frontier;
  config.max_total_samples = params.advanced_viewpoint_max_total_samples;
  config.runtime_budget_ms = params.advanced_viewpoint_runtime_budget_ms;
  config.keep_original_goal_fallback = params.advanced_viewpoint_keep_original_goal_fallback;
  config.min_goal_distance_m = params.advanced_viewpoint_min_goal_distance_m;
  config.max_goal_distance_m = params.advanced_viewpoint_max_goal_distance_m;
  config.require_line_of_sight_to_frontier =
    params.advanced_viewpoint_require_line_of_sight_to_frontier;
  config.require_visible_unknown = params.advanced_viewpoint_require_visible_unknown;
  return config;
}

RrtViewpointConfig FrontierExplorerCore::rrt_viewpoint_config() const
{
  RrtViewpointConfig config;
  config.enabled = params.rrt_enabled;
  config.max_nodes = params.rrt_max_nodes;
  config.max_iterations = params.rrt_max_iterations;
  config.step_size_m = params.rrt_step_size_m;
  config.goal_bias_probability = params.rrt_goal_bias_probability;
  config.frontier_bias_probability = params.rrt_frontier_bias_probability;
  config.sampling_radius_m = params.rrt_sampling_radius_m;
  config.min_sample_radius_m = params.rrt_min_sample_radius_m;
  config.collision_check_step_m = params.rrt_collision_check_step_m;
  config.use_costmap_validation = params.rrt_use_costmap_validation;
  config.use_occupancy_validation = params.rrt_use_occupancy_validation;
  config.deterministic_seed = params.rrt_deterministic_seed;
  config.reset_seed_each_cycle = params.rrt_reset_seed_each_cycle;
  config.polar_sampling_enabled = params.rrt_polar_sampling_enabled;
  config.polar_area_uniform_radius = params.rrt_polar_area_uniform_radius;
  config.polar_angle_bins = params.rrt_polar_angle_bins;
  return config;
}

MonteCarloGainConfig FrontierExplorerCore::monte_carlo_gain_config() const
{
  MonteCarloGainConfig config;
  config.enabled = params.monte_carlo_gain_enabled;
  config.mode = params.monte_carlo_gain_mode;
  config.weight = params.monte_carlo_gain_weight;
  config.sample_count = params.monte_carlo_gain_sample_count;
  config.max_candidates = params.monte_carlo_gain_max_candidates;
  config.runtime_budget_ms = params.monte_carlo_gain_runtime_budget_ms;
  config.deterministic_seed = params.monte_carlo_gain_deterministic_seed;
  config.reset_seed_each_cycle = params.monte_carlo_gain_reset_seed_each_cycle;
  config.sensor_range_m = params.monte_carlo_gain_sensor_range_m;
  config.sensor_fov_deg = params.monte_carlo_gain_sensor_fov_deg;
  config.yaw_offset_deg = params.monte_carlo_gain_yaw_offset_deg;
  config.min_range_m = params.monte_carlo_gain_min_range_m;
  config.occupied_blocks_visibility = params.monte_carlo_gain_occupied_blocks_visibility;
  config.costmap_blocks_visibility = params.monte_carlo_gain_costmap_blocks_visibility;
  config.unknown_value = params.monte_carlo_gain_unknown_value;
  config.free_value = params.monte_carlo_gain_free_value;
  config.occupied_value = params.monte_carlo_gain_occupied_value;
  config.normalize_by_sample_count = params.monte_carlo_gain_normalize_by_sample_count;
  config.use_mixed_resolution = params.monte_carlo_gain_use_mixed_resolution;
  config.near_field_radius_fraction = params.monte_carlo_gain_near_field_radius_fraction;
  config.far_field_sample_stride = params.monte_carlo_gain_far_field_sample_stride;
  config.resolution_decay_lambda = params.monte_carlo_gain_resolution_decay_lambda;
  config.fail_open = params.monte_carlo_gain_fail_open;
  return config;
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
