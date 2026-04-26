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

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

struct VisitedFrontierRecord
{
  std::pair<double, double> position{0.0, 0.0};
  int64_t timestamp_ns{0};
  std::string reason{"success"};
};

struct MrtspScoringOptions
{
  bool direction_bias_enabled{false};
  double direction_bias_weight{0.15};
  double direction_reverse_penalty_weight{0.25};
  std::string direction_memory_mode{"last_motion"};
  double direction_min_motion_m{0.10};

  bool visited_penalty_enabled{false};
  double visited_radius_m{0.75};
  double visited_penalty_weight{0.50};
  int visited_history_max_size{128};
  double visited_history_timeout_s{300.0};

  bool low_gain_distance_decay_enabled{false};
  double low_gain_distance_decay_weight{0.25};
  double low_gain_distance_decay_lambda{0.8};
  double low_gain_min_frontier_size_cells{8.0};

  bool soft_obstacle_penalty_enabled{false};
  double soft_obstacle_penalty_weight{0.30};
  int soft_obstacle_cost_start{40};
  int soft_obstacle_cost_max{90};
};

struct MrtspScoringContext
{
  std::optional<std::array<double, 2>> desired_direction;
  std::vector<VisitedFrontierRecord> visited_frontiers;
  std::vector<std::optional<int>> obstacle_costs;
  int64_t now_ns{0};
};

struct MrtspScoreBreakdown
{
  double base_cost{0.0};
  double direction_similarity{0.0};
  double direction_bonus{0.0};
  double reverse_penalty{0.0};
  double visited_penalty{0.0};
  double low_gain_penalty{0.0};
  double soft_obstacle_penalty{0.0};
  double final_cost{0.0};
};

bool mrtsp_scoring_enabled(const MrtspScoringOptions & options);

std::string normalize_direction_memory_mode(std::string mode);
std::string normalize_visible_gain_mode(std::string mode);

double direction_similarity(
  const std::array<double, 2> & desired_direction,
  const std::array<double, 2> & candidate_direction);

double visited_frontier_penalty(
  const std::pair<double, double> & candidate_point,
  const std::vector<VisitedFrontierRecord> & visited_frontiers,
  int64_t now_ns,
  double radius_m,
  double timeout_s,
  double weight);

double low_gain_distance_decay_penalty(
  double distance_m,
  double frontier_gain,
  double min_frontier_size_cells,
  double lambda,
  double weight);

double soft_obstacle_penalty(
  std::optional<int> cost,
  int cost_start,
  int cost_max,
  double weight);

MrtspScoreBreakdown evaluate_mrtsp_score(
  double base_cost,
  const FrontierCandidate & target_frontier,
  std::size_t frontier_index,
  const std::array<double, 2> & robot_position,
  double transition_distance_m,
  bool apply_direction_bias,
  const MrtspScoringOptions & options,
  const MrtspScoringContext & context);

}  // namespace frontier_exploration_ros2
