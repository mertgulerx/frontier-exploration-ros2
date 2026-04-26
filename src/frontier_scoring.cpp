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

#include "frontier_exploration_ros2/frontier_scoring.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kEpsilon = 1e-9;

double distance_between(
  const std::pair<double, double> & first,
  const std::pair<double, double> & second)
{
  return std::hypot(first.first - second.first, first.second - second.second);
}

std::pair<double, double> scoring_point(const FrontierCandidate & frontier)
{
  return frontier.goal_point.value_or(frontier.center_point);
}

}  // namespace

bool mrtsp_scoring_enabled(const MrtspScoringOptions & options)
{
  return options.direction_bias_enabled ||
         options.visited_penalty_enabled ||
         options.low_gain_distance_decay_enabled ||
         options.soft_obstacle_penalty_enabled;
}

std::string normalize_direction_memory_mode(std::string mode)
{
  std::transform(
    mode.begin(),
    mode.end(),
    mode.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (mode == "robot_yaw" || mode == "last_goal") {
    return mode;
  }
  return "last_motion";
}

std::string normalize_visible_gain_mode(std::string mode)
{
  std::transform(
    mode.begin(),
    mode.end(),
    mode.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (mode == "rerank") {
    return mode;
  }
  return "bonus";
}

double direction_similarity(
  const std::array<double, 2> & desired_direction,
  const std::array<double, 2> & candidate_direction)
{
  const double desired_norm = std::hypot(desired_direction[0], desired_direction[1]);
  const double candidate_norm = std::hypot(candidate_direction[0], candidate_direction[1]);
  if (desired_norm <= kEpsilon || candidate_norm <= kEpsilon) {
    return 0.0;
  }

  const double similarity =
    ((desired_direction[0] / desired_norm) * (candidate_direction[0] / candidate_norm)) +
    ((desired_direction[1] / desired_norm) * (candidate_direction[1] / candidate_norm));
  return std::clamp(similarity, -1.0, 1.0);
}

double visited_frontier_penalty(
  const std::pair<double, double> & candidate_point,
  const std::vector<VisitedFrontierRecord> & visited_frontiers,
  int64_t now_ns,
  double radius_m,
  double timeout_s,
  double weight)
{
  if (visited_frontiers.empty() || radius_m <= 0.0 || weight <= 0.0) {
    return 0.0;
  }

  const int64_t timeout_ns = timeout_s > 0.0 ?
    static_cast<int64_t>(timeout_s * 1e9) :
    std::numeric_limits<int64_t>::max();
  for (const auto & visited : visited_frontiers) {
    if (
      timeout_s > 0.0 &&
      now_ns >= visited.timestamp_ns &&
      now_ns - visited.timestamp_ns > timeout_ns)
    {
      continue;
    }

    if (distance_between(candidate_point, visited.position) <= radius_m) {
      return weight;
    }
  }

  return 0.0;
}

double low_gain_distance_decay_penalty(
  double distance_m,
  double frontier_gain,
  double min_frontier_size_cells,
  double lambda,
  double weight)
{
  if (weight <= 0.0 || lambda <= 0.0 || min_frontier_size_cells <= 0.0) {
    return 0.0;
  }

  const double low_gain_factor = std::max(
    0.0,
    (min_frontier_size_cells - frontier_gain) / std::max(min_frontier_size_cells, kEpsilon));
  if (low_gain_factor <= 0.0) {
    return 0.0;
  }

  const double distance_factor = 1.0 - std::exp(-lambda * std::max(0.0, distance_m));
  return weight * low_gain_factor * distance_factor;
}

double soft_obstacle_penalty(
  std::optional<int> cost,
  int cost_start,
  int cost_max,
  double weight)
{
  if (!cost.has_value() || weight <= 0.0) {
    return 0.0;
  }

  const int safe_start = std::clamp(cost_start, 0, 100);
  const int safe_max = std::clamp(std::max(cost_max, safe_start + 1), 0, 100);
  if (*cost <= safe_start) {
    return 0.0;
  }

  const double normalized = std::clamp(
    (static_cast<double>(*cost) - static_cast<double>(safe_start)) /
    std::max(1.0, static_cast<double>(safe_max - safe_start)),
    0.0,
    1.0);
  return weight * normalized;
}

MrtspScoreBreakdown evaluate_mrtsp_score(
  double base_cost,
  const FrontierCandidate & target_frontier,
  std::size_t frontier_index,
  const std::array<double, 2> & robot_position,
  double transition_distance_m,
  bool apply_direction_bias,
  const MrtspScoringOptions & options,
  const MrtspScoringContext & context)
{
  MrtspScoreBreakdown breakdown;
  breakdown.base_cost = base_cost;
  breakdown.final_cost = base_cost;

  // Optional direction-aware scoring inspired by the 2026 Scientific Reports
  // multi-resolution field exploration paper. This does not replace the base
  // MRTSP cost model; it only nudges the current robot-to-frontier choice.
  if (
    apply_direction_bias &&
    options.direction_bias_enabled &&
    context.desired_direction.has_value())
  {
    const auto point = scoring_point(target_frontier);
    const std::array<double, 2> candidate_direction{
      point.first - robot_position[0],
      point.second - robot_position[1],
    };
    breakdown.direction_similarity =
      direction_similarity(*context.desired_direction, candidate_direction);
    breakdown.direction_bonus =
      options.direction_bias_weight * std::max(0.0, breakdown.direction_similarity);
    breakdown.reverse_penalty =
      options.direction_reverse_penalty_weight * std::max(0.0, -breakdown.direction_similarity);
    breakdown.final_cost =
      (breakdown.final_cost * (1.0 + breakdown.reverse_penalty)) - breakdown.direction_bonus;
  }

  // Recent-visit penalty is intentionally separate from suppression. Suppression
  // tracks repeated failures; this only discourages immediate successful revisits.
  if (options.visited_penalty_enabled) {
    breakdown.visited_penalty = visited_frontier_penalty(
      scoring_point(target_frontier),
      context.visited_frontiers,
      context.now_ns,
      options.visited_radius_m,
      options.visited_history_timeout_s,
      options.visited_penalty_weight);
    breakdown.final_cost += breakdown.visited_penalty;
  }

  // The base MRTSP model already includes distance/gain, so this add-on uses a
  // bounded decay term rather than another raw distance/gain ratio.
  if (options.low_gain_distance_decay_enabled) {
    breakdown.low_gain_penalty = low_gain_distance_decay_penalty(
      transition_distance_m,
      static_cast<double>(target_frontier.size),
      options.low_gain_min_frontier_size_cells,
      options.low_gain_distance_decay_lambda,
      options.low_gain_distance_decay_weight);
    breakdown.final_cost += breakdown.low_gain_penalty;
  }

  // Soft obstacle penalty is only a preference signal among already-valid candidates.
  if (options.soft_obstacle_penalty_enabled) {
    const std::optional<int> cost =
      frontier_index < context.obstacle_costs.size() ?
      context.obstacle_costs[frontier_index] :
      std::nullopt;
    breakdown.soft_obstacle_penalty = soft_obstacle_penalty(
      cost,
      options.soft_obstacle_cost_start,
      options.soft_obstacle_cost_max,
      options.soft_obstacle_penalty_weight);
    breakdown.final_cost += breakdown.soft_obstacle_penalty;
  }

  breakdown.final_cost = std::max(breakdown.final_cost, kEpsilon);
  return breakdown;
}

}  // namespace frontier_exploration_ros2
