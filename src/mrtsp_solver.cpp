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

#include "frontier_exploration_ros2/mrtsp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>

namespace frontier_exploration_ros2
{

namespace
{

using MrtspMask = std::uint64_t;

// Costs that differ only by floating-point noise should fall through to stable tie-breakers.
constexpr double kScoreTieEpsilon = 1e-12;

void ensure_cost_state(std::vector<double> & costs, std::size_t frontier_count)
{
  // Each mask stores one cost per possible route end. Missing entries are initialized
  // lazily because only masks reachable within the requested horizon are materialized.
  if (costs.empty()) {
    costs.assign(frontier_count, std::numeric_limits<double>::infinity());
  }
}

void ensure_parent_state(std::vector<int> & parents, std::size_t frontier_count)
{
  // Parent entries mirror the cost vector layout. A value of -1 marks a route that
  // starts directly at the robot node and therefore has no previous frontier.
  if (parents.empty()) {
    parents.assign(frontier_count, -1);
  }
}

}  // namespace

std::vector<MrtspPrunedCandidate> prune_mrtsp_candidates(
  const std::vector<FrontierCandidate> & candidates,
  const RobotState & robot_state,
  const CostWeights & weights,
  double sensor_effective_range_m,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax,
  const MrtspSolverConfig & config)
{
  if (candidates.empty()) {
    return {};
  }

  std::vector<MrtspPrunedCandidate> scored_candidates;
  scored_candidates.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    // Pruning uses the matrix start row cost so candidate selection follows the
    // same distance, gain, sensor range, and first-motion terms as final ordering.
    double score = compute_mrtsp_start_cost(
      candidates[index],
      robot_state,
      weights,
      sensor_effective_range_m,
      max_linear_speed_vmax,
      max_angular_speed_wmax);
    // Keep invalid candidates at the end of the sorted pool without changing vector shape.
    if (std::isnan(score)) {
      score = std::numeric_limits<double>::infinity();
    }
    scored_candidates.push_back(MrtspPrunedCandidate{index, candidates[index], score});
  }

  // Stable pruning is important for repeatable goal selection when frontiers have
  // nearly equal costs. The comparator keeps all finite scores ahead of invalid ones,
  // prefers lower MRTSP start cost, then uses gain and extraction order as tie-breakers.
  std::sort(
    scored_candidates.begin(),
    scored_candidates.end(),
    [](const MrtspPrunedCandidate & lhs, const MrtspPrunedCandidate & rhs) {
      const bool lhs_finite = std::isfinite(lhs.score);
      const bool rhs_finite = std::isfinite(rhs.score);
      if (lhs_finite != rhs_finite) {
        return lhs_finite;
      }
      if (lhs_finite && std::abs(lhs.score - rhs.score) > kScoreTieEpsilon) {
        return lhs.score < rhs.score;
      }
      if (!lhs_finite && lhs.score != rhs.score) {
        return lhs.score < rhs.score;
      }
      // Larger frontier clusters win score ties, then extraction order makes the result stable.
      if (lhs.candidate.size != rhs.candidate.size) {
        return lhs.candidate.size > rhs.candidate.size;
      }
      return lhs.original_index < rhs.original_index;
    });

  const std::size_t normalized_limit = std::clamp<std::size_t>(
    config.candidate_limit,
    std::size_t{1U},
    kMaxDpSolverCandidateLimit);
  const std::size_t actual_limit = std::min(normalized_limit, scored_candidates.size());
  // Truncation happens after sorting so the solver receives only the most promising
  // candidate pool while each item still remembers its original frontier index.
  scored_candidates.resize(actual_limit);
  return scored_candidates;
}

std::vector<std::size_t> solve_bounded_horizon_mrtsp_order(
  const MrtspCostMatrix & cost_matrix,
  std::size_t planning_horizon)
{
  if (cost_matrix.dimension <= 1U || cost_matrix.values.empty() || planning_horizon == 0U) {
    return {};
  }

  const std::size_t frontier_count = cost_matrix.dimension - 1U;
  // The bitmask representation stores one bit per candidate; larger pools must be pruned first.
  if (frontier_count > std::numeric_limits<MrtspMask>::digits ||
    cost_matrix.values.size() < cost_matrix.dimension * cost_matrix.dimension)
  {
    return {};
  }

  const std::size_t effective_horizon = std::min(planning_horizon, frontier_count);
  std::map<MrtspMask, std::vector<double>> current_layer;
  std::map<MrtspMask, std::vector<double>> next_layer;
  std::map<MrtspMask, std::vector<int>> parent;

  // Initialize one-step routes directly from the synthetic robot start node.
  // State interpretation after this loop:
  //   current_layer[1 << j][j] = cost(robot -> frontier_j)
  for (std::size_t frontier = 0; frontier < frontier_count; ++frontier) {
    const double start_cost = cost_matrix.at(0U, frontier + 1U);
    if (!std::isfinite(start_cost)) {
      continue;
    }
    const MrtspMask mask = MrtspMask{1U} << frontier;
    auto & costs = current_layer[mask];
    ensure_cost_state(costs, frontier_count);
    costs[frontier] = start_cost;
    auto & parents = parent[mask];
    ensure_parent_state(parents, frontier_count);
  }

  if (current_layer.empty()) {
    return {};
  }

  // Expand only one path-length layer at a time so masks deeper than the requested
  // planning horizon are never materialized.
  for (std::size_t layer_size = 1U; layer_size < effective_horizon; ++layer_size) {
    next_layer.clear();
    for (const auto & [mask, costs] : current_layer) {
      for (std::size_t current = 0; current < frontier_count; ++current) {
        const MrtspMask current_bit = MrtspMask{1U} << current;
        // Ignore endpoints that are not part of the visited set or cannot be reached
        // with finite cost from the previous layer.
        if ((mask & current_bit) == 0U || !std::isfinite(costs[current])) {
          continue;
        }

        for (std::size_t next = 0; next < frontier_count; ++next) {
          const MrtspMask next_bit = MrtspMask{1U} << next;
          // Frontier visits are unique inside a route, so an already-set bit blocks
          // the transition and keeps the sequence from revisiting the same candidate.
          if ((mask & next_bit) != 0U) {
            continue;
          }

          const double edge_cost = cost_matrix.at(current + 1U, next + 1U);
          if (!std::isfinite(edge_cost)) {
            continue;
          }

          const double candidate_cost = costs[current] + edge_cost;
          if (!std::isfinite(candidate_cost)) {
            continue;
          }

          const MrtspMask next_mask = mask | next_bit;
          auto & next_costs = next_layer[next_mask];
          ensure_cost_state(next_costs, frontier_count);
          // Standard shortest-path relaxation for the bounded TSP state:
          // dp[next_mask][next] = min(dp[next_mask][next], dp[mask][current] + edge).
          if (candidate_cost < next_costs[next]) {
            next_costs[next] = candidate_cost;
            auto & parents = parent[next_mask];
            ensure_parent_state(parents, frontier_count);
            parents[next] = static_cast<int>(current);
          }
        }
      }
    }

    if (next_layer.empty()) {
      return {};
    }
    // Move the just-built states into the active layer. This discards shorter states
    // once they are no longer needed and keeps memory proportional to one horizon depth.
    current_layer = std::move(next_layer);
  }

  // Select the cheapest route that reaches the requested effective horizon.
  double best_cost = std::numeric_limits<double>::infinity();
  MrtspMask best_mask = 0U;
  int best_end = -1;
  for (const auto & [mask, costs] : current_layer) {
    for (std::size_t frontier = 0; frontier < frontier_count; ++frontier) {
      // At this point every state in current_layer has exactly effective_horizon bits,
      // so comparing end costs is enough to choose the best K-step route.
      if (std::isfinite(costs[frontier]) && costs[frontier] < best_cost) {
        best_cost = costs[frontier];
        best_mask = mask;
        best_end = static_cast<int>(frontier);
      }
    }
  }

  if (best_end < 0) {
    return {};
  }

  std::vector<std::size_t> reversed_order;
  MrtspMask mask = best_mask;
  int current = best_end;
  // Parent pointers store the previous frontier for each improved transition.
  // Walking backward recovers the selected route without keeping full paths in DP states.
  while (current >= 0) {
    reversed_order.push_back(static_cast<std::size_t>(current));
    int previous = -1;
    const auto parent_it = parent.find(mask);
    if (parent_it != parent.end() &&
      static_cast<std::size_t>(current) < parent_it->second.size())
    {
      previous = parent_it->second[static_cast<std::size_t>(current)];
    }
    // Clear the current frontier bit before following its parent. The next lookup
    // uses the mask that existed before the current frontier was appended.
    mask &= ~(MrtspMask{1U} << static_cast<std::size_t>(current));
    current = previous;
  }

  if (reversed_order.size() != effective_horizon) {
    return {};
  }

  // Reconstruction walks from the route end to the start, so reverse it before
  // returning the dispatch order expected by the core.
  std::reverse(reversed_order.begin(), reversed_order.end());
  return reversed_order;
}

}  // namespace frontier_exploration_ros2
