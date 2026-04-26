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

#include <cstddef>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace frontier_exploration_ros2
{

struct ViewpointCandidate
{
  geometry_msgs::msg::PoseStamped pose;
  std::size_t frontier_index{0};
  int parent_node_index{-1};
  bool valid{false};
  bool fallback_original_goal{false};
  std::string source{"base"};
  double robot_distance_m{0.0};
  double frontier_distance_m{0.0};
  double visible_reveal_gain{0.0};
  double monte_carlo_gain{0.0};
  double soft_obstacle_penalty{0.0};
  double final_score{0.0};
};

struct RrtNode2D
{
  double x{0.0};
  double y{0.0};
  int parent{-1};
  double accumulated_length_m{0.0};
};

struct AdvancedViewpointSamplingConfig
{
  bool enabled{false};
  std::string method{"frontier_local_rrt"};
  int max_frontiers{8};
  int max_samples_per_frontier{32};
  int max_total_samples{256};
  double runtime_budget_ms{20.0};
  bool keep_original_goal_fallback{true};
  double min_goal_distance_m{0.0};
  double max_goal_distance_m{3.0};
  bool require_line_of_sight_to_frontier{true};
  bool require_visible_unknown{false};
};

struct RrtViewpointConfig
{
  bool enabled{false};
  int max_nodes{64};
  int max_iterations{128};
  double step_size_m{0.35};
  double goal_bias_probability{0.20};
  double frontier_bias_probability{0.50};
  double sampling_radius_m{2.0};
  double min_sample_radius_m{0.25};
  double collision_check_step_m{0.05};
  bool use_costmap_validation{true};
  bool use_occupancy_validation{true};
  int deterministic_seed{0};
  bool reset_seed_each_cycle{false};
  bool polar_sampling_enabled{true};
  bool polar_area_uniform_radius{true};
  int polar_angle_bins{0};
};

}  // namespace frontier_exploration_ros2
