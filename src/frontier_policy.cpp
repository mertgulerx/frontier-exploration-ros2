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

#include "frontier_exploration_ros2/frontier_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace frontier_exploration_ros2
{

namespace
{

// Unified helper for variant access in policy functions.
const FrontierCandidate * as_candidate(const FrontierLike & frontier)
{
  return std::get_if<FrontierCandidate>(&frontier);
}

}  // namespace

std::pair<double, double> frontier_position(const FrontierLike & frontier)
{
  // Candidate frontiers dispatch the nearest-specific goal point when available;
  // faithful MRTSP candidates fall back to center_point because no goal_point is materialized.
  if (const auto * candidate = as_candidate(frontier)) {
    return candidate->goal_point.value_or(candidate->center_point);
  }
  // Primitive frontiers encode one point that is both reference and goal.
  return std::get<PrimitiveFrontier>(frontier);
}

std::pair<double, double> frontier_reference_point(const FrontierLike & frontier)
{
  // Candidate frontiers separate "centroid for ranking" from "goal for navigation".
  if (const auto * candidate = as_candidate(frontier)) {
    return candidate->centroid;
  }
  // Primitive frontier has no centroid/goal distinction.
  return std::get<PrimitiveFrontier>(frontier);
}

int frontier_size(const FrontierLike & frontier)
{
  // Candidate keeps measured cluster size from extraction stage.
  if (const auto * candidate = as_candidate(frontier)) {
    return candidate->size;
  }
  // Primitive single-point representation is treated as size 1.
  return 1;
}

std::string describe_frontier(const FrontierLike & frontier)
{
  const auto [reference_x, reference_y] = frontier_reference_point(frontier);
  const auto [frontier_x, frontier_y] = frontier_position(frontier);
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);
  oss << "frontier reference=(" << reference_x << ", " << reference_y << ")"
      << ", dispatch=(" << frontier_x << ", " << frontier_y << ")"
      << ", size=" << frontier_size(frontier);
  return oss.str();
}

FrontierSignature frontier_signature(
  const FrontierSequence & frontiers,
  double frontier_visit_tolerance)
{
  // Quantization keeps signatures stable under small floating-point jitter.
  const double quantum = std::max(frontier_visit_tolerance, 0.1);
  FrontierSignature signature;
  signature.reserve(frontiers.size());

  for (const auto & frontier : frontiers) {
    // Signature captures both ranking reference and dispatched goal to
    // distinguish cases where centroid stays similar but selected goal changes.
    const auto [reference_x, reference_y] = frontier_reference_point(frontier);
    const auto [goal_x, goal_y] = frontier_position(frontier);
    signature.push_back({
      // Quantized integer bins keep signatures stable across tiny float jitter.
      static_cast<int64_t>(std::llround(reference_x / quantum)),
      static_cast<int64_t>(std::llround(reference_y / quantum)),
      static_cast<int64_t>(std::llround(goal_x / quantum)),
      static_cast<int64_t>(std::llround(goal_y / quantum)),
    });
  }

  // Sorted signature allows order-independent "same frontier set" comparison.
  std::sort(signature.begin(), signature.end());
  return signature;
}

FrontierSelectionResult select_primitive_frontier(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose,
  double frontier_selection_min_distance,
  double frontier_visit_tolerance,
  bool escape_active)
{
  // Preferred frontier = closest candidate that still respects min distance.
  // Fallback frontier = farthest candidate to escape local traps/congestion.
  std::optional<FrontierLike> preferred_frontier;
  double closest_distance = std::numeric_limits<double>::infinity();
  std::optional<FrontierLike> close_range_fallback;
  double farthest_distance = -std::numeric_limits<double>::infinity();

  for (const auto & frontier : frontiers) {
    // Selection uses reference point for distance ranking.
    const auto [frontier_x, frontier_y] = frontier_reference_point(frontier);
    // Goal point check avoids re-dispatching a target already within visit tolerance.
    const auto [goal_x, goal_y] = frontier_position(frontier);
    const double distance = std::hypot(
      frontier_x - current_pose.position.x,
      frontier_y - current_pose.position.y);
    const double goal_distance = std::hypot(
      goal_x - current_pose.position.x,
      goal_y - current_pose.position.y);

    // Skip any frontier whose actual goal point is essentially already visited.
    if (goal_distance < frontier_visit_tolerance) {
      continue;
    }

    // Preferred candidate: nearest frontier that still satisfies min distance.
    if (distance >= frontier_selection_min_distance && distance < closest_distance) {
      closest_distance = distance;
      preferred_frontier = frontier;
    }

    // Keep farthest fallback for escape and close-range fallback modes.
    if (distance > farthest_distance) {
      farthest_distance = distance;
      close_range_fallback = frontier;
    }
  }

  // Normal path: preferred frontier available.
  if (preferred_frontier.has_value()) {
    return {preferred_frontier, "preferred"};
  }

  // Escape path: move away from local minima when no preferred frontier exists.
  if (escape_active && close_range_fallback.has_value()) {
    return {close_range_fallback, "escape"};
  }

  // Last-resort path: still pick a frontier to keep exploration moving.
  if (close_range_fallback.has_value()) {
    return {close_range_fallback, "close-range-fallback"};
  }

  // No valid frontier survived filtering.
  return {std::nullopt, ""};
}

bool are_frontiers_equivalent(
  const std::optional<FrontierLike> & first_frontier,
  const std::optional<FrontierLike> & second_frontier,
  double frontier_visit_tolerance)
{
  // Equivalence requires both reference and selected goal points to match under tolerance.
  if (!first_frontier.has_value() || !second_frontier.has_value()) {
    return false;
  }

  const auto first_reference = frontier_reference_point(*first_frontier);
  const auto second_reference = frontier_reference_point(*second_frontier);
  const auto first_goal = frontier_position(*first_frontier);
  const auto second_goal = frontier_position(*second_frontier);

  const double reference_distance = std::hypot(
    first_reference.first - second_reference.first,
    first_reference.second - second_reference.second);
  const double goal_distance = std::hypot(
    first_goal.first - second_goal.first,
    first_goal.second - second_goal.second);

  return (
    // Both reference and goal must be close to avoid false positives.
    reference_distance < frontier_visit_tolerance &&
    goal_distance < frontier_visit_tolerance);
}

bool are_frontier_sequences_equivalent(
  const FrontierSequence & first_frontier_sequence,
  const FrontierSequence & second_frontier_sequence,
  double frontier_visit_tolerance)
{
  // Sequence comparison is ordered by design; caller controls ordering policy.
  if (first_frontier_sequence.size() != second_frontier_sequence.size()) {
    return false;
  }

  for (std::size_t i = 0; i < first_frontier_sequence.size(); ++i) {
    // Pairwise tolerance check per index.
    if (!are_frontiers_equivalent(
        first_frontier_sequence[i],
        second_frontier_sequence[i],
        frontier_visit_tolerance))
    {
      return false;
    }
  }

  return true;
}

}  // namespace frontier_exploration_ros2
