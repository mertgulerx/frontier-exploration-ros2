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
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"

namespace frontier_exploration_ros2
{

constexpr std::size_t kMaxDpSolverCandidateLimit{60U};

struct MrtspSolverConfig
{
  // Upper bound for the candidate pool passed to the bounded solver.
  std::size_t candidate_limit{15};
  // Number of distinct frontier visits evaluated in each receding-horizon plan.
  std::size_t planning_horizon{10};
};

struct MrtspPrunedCandidate
{
  // Index in the frontier vector received from the core. The DP solver works on the
  // pruned vector, then callers use this field to map the result back to dispatch data.
  std::size_t original_index{};
  FrontierCandidate candidate{};
  // MRTSP start-row cost used for deterministic top-N candidate pruning.
  double score{0.0};
};

// Scores candidates with the same robot-to-frontier cost used by the MRTSP matrix,
// sorts them deterministically, and returns the best bounded candidate pool.
std::vector<MrtspPrunedCandidate> prune_mrtsp_candidates(
  const std::vector<FrontierCandidate> & candidates,
  const RobotState & robot_state,
  const CostWeights & weights,
  double sensor_effective_range_m,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax,
  const MrtspSolverConfig & config);

// Searches the lowest-cost K-frontier sequence in the given matrix.
// Matrix node 0 is the robot start node, and nodes 1..M are pruned frontiers.
// Returned indices are relative to the pruned candidate list.
std::vector<std::size_t> solve_bounded_horizon_mrtsp_order(
  const MrtspCostMatrix & cost_matrix,
  std::size_t planning_horizon);

}  // namespace frontier_exploration_ros2
