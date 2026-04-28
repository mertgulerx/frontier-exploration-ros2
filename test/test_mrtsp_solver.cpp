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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"
#include "frontier_exploration_ros2/mrtsp_solver.hpp"

namespace frontier_exploration_ros2
{
namespace
{

FrontierCandidate make_candidate(double x, int size = 1)
{
  // Keep synthetic frontiers collinear so the start-row MRTSP score is easy to reason
  // about in pruning tests: lower x means lower distance when size is equal.
  return FrontierCandidate{{x, 0.0}, {x, 0.0}, size};
}

RobotState make_robot_state()
{
  RobotState robot_state;
  robot_state.position = {0.0, 0.0};
  robot_state.yaw = 0.0;
  return robot_state;
}

CostWeights make_weights()
{
  CostWeights weights;
  weights.distance_wd = 1.0;
  weights.gain_ws = 1.0;
  return weights;
}

geometry_msgs::msg::Pose make_pose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

MrtspCostMatrix make_matrix(std::size_t frontier_count)
{
  MrtspCostMatrix matrix;
  matrix.dimension = frontier_count + 1U;
  // Tests build only the edges relevant to each scenario. Infinity keeps unspecified
  // transitions unavailable, matching the solver's runtime treatment of invalid edges.
  matrix.values.assign(
    matrix.dimension * matrix.dimension,
    std::numeric_limits<double>::infinity());
  for (std::size_t row = 1U; row < matrix.dimension; ++row) {
    matrix.values[row * matrix.dimension] = 0.0;
  }
  return matrix;
}

void set_cost(MrtspCostMatrix & matrix, std::size_t row, std::size_t column, double cost)
{
  matrix.values[row * matrix.dimension + column] = cost;
}

TEST(MrtspSolverTests, CandidatePruningKeepsTopScoreCandidates)
{
  // Candidate scores are monotonic with x in this setup, so the two closest frontiers
  // should survive pruning while their original indices remain available for remapping.
  const std::vector<FrontierCandidate> candidates{
    make_candidate(5.0),
    make_candidate(2.0),
    make_candidate(3.0),
    make_candidate(1.0),
  };

  const auto pruned = prune_mrtsp_candidates(
    candidates,
    make_robot_state(),
    make_weights(),
    0.0,
    1.0,
    1.0,
    MrtspSolverConfig{2U, 4U});

  ASSERT_EQ(pruned.size(), 2U);
  EXPECT_EQ(pruned[0].original_index, 3U);
  EXPECT_EQ(pruned[1].original_index, 1U);
}

TEST(MrtspSolverTests, CandidatePruningIsDeterministic)
{
  // All candidates have equal start-row score because size and distance increase
  // together. The expected order verifies the size-descending tie-breaker first,
  // then original index ordering for stable repeated runs.
  const std::vector<FrontierCandidate> candidates{
    make_candidate(2.0, 2),
    make_candidate(3.0, 3),
    make_candidate(1.0, 1),
    make_candidate(4.0, 4),
  };

  const auto pruned = prune_mrtsp_candidates(
    candidates,
    make_robot_state(),
    make_weights(),
    0.0,
    1.0,
    1.0,
    MrtspSolverConfig{4U, 4U});

  ASSERT_EQ(pruned.size(), 4U);
  EXPECT_EQ(pruned[0].original_index, 3U);
  EXPECT_EQ(pruned[1].original_index, 1U);
  EXPECT_EQ(pruned[2].original_index, 0U);
  EXPECT_EQ(pruned[3].original_index, 2U);
}

TEST(MrtspSolverTests, CandidateLimitIsRespected)
{
  std::vector<FrontierCandidate> candidates;
  candidates.reserve(20U);
  // More candidates than the configured limit should be truncated after scoring.
  for (std::size_t index = 0; index < 20U; ++index) {
    candidates.push_back(make_candidate(static_cast<double>(index + 1U)));
  }

  const auto limited = prune_mrtsp_candidates(
    candidates,
    make_robot_state(),
    make_weights(),
    0.0,
    1.0,
    1.0,
    MrtspSolverConfig{15U, 4U});
  EXPECT_EQ(limited.size(), 15U);

  // Fewer candidates than the limit should pass through without padding or duplication.
  candidates.resize(10U);
  const auto under_limit = prune_mrtsp_candidates(
    candidates,
    make_robot_state(),
    make_weights(),
    0.0,
    1.0,
    1.0,
    MrtspSolverConfig{15U, 4U});
  EXPECT_EQ(under_limit.size(), 10U);
}

TEST(MrtspSolverTests, CandidateLimitIsClampedToHardMaximum)
{
  std::vector<FrontierCandidate> candidates;
  candidates.reserve(kMaxDpSolverCandidateLimit + 20U);
  for (std::size_t index = 0; index < kMaxDpSolverCandidateLimit + 20U; ++index) {
    candidates.push_back(make_candidate(static_cast<double>(index + 1U)));
  }

  // The pruning helper enforces the package-level DP cap even if a direct caller
  // provides a larger solver configuration.
  const auto pruned = prune_mrtsp_candidates(
    candidates,
    make_robot_state(),
    make_weights(),
    0.0,
    1.0,
    1.0,
    MrtspSolverConfig{200U, 4U});
  EXPECT_EQ(pruned.size(), kMaxDpSolverCandidateLimit);

  FrontierExplorerCoreParams params;
  params.dp_solver_candidate_limit = 200U;
  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});

  EXPECT_EQ(core.params.dp_solver_candidate_limit, kMaxDpSolverCandidateLimit);
}

TEST(MrtspSolverTests, BoundedHorizonCanBeatGreedy)
{
  // The start row makes A the cheapest immediate choice, which is exactly what greedy
  // selects. The B -> C -> D -> E corridor is much cheaper over four steps, so bounded
  // DP should choose B as the first dispatch target.
  auto matrix = make_matrix(5U);
  set_cost(matrix, 0U, 1U, 1.0);   // A
  set_cost(matrix, 0U, 2U, 2.0);   // B
  set_cost(matrix, 0U, 3U, 50.0);  // C
  set_cost(matrix, 0U, 4U, 50.0);  // D
  set_cost(matrix, 0U, 5U, 50.0);  // E

  for (std::size_t row = 1U; row < matrix.dimension; ++row) {
    for (std::size_t column = 1U; column < matrix.dimension; ++column) {
      if (row != column) {
        // Make all unspecified frontier-to-frontier transitions valid but unattractive.
        set_cost(matrix, row, column, 100.0);
      }
    }
  }
  set_cost(matrix, 2U, 3U, 1.0);  // B -> C
  set_cost(matrix, 3U, 4U, 1.0);  // C -> D
  set_cost(matrix, 4U, 5U, 1.0);  // D -> E

  const auto greedy_order = greedy_mrtsp_order(matrix);
  ASSERT_FALSE(greedy_order.empty());
  EXPECT_EQ(greedy_order.front(), 0U);

  const auto dp_order = solve_bounded_horizon_mrtsp_order(matrix, 4U);
  ASSERT_EQ(dp_order.size(), 4U);
  EXPECT_EQ(dp_order.front(), 1U);
}

TEST(MrtspSolverTests, HorizonOneChoosesBestStartEdge)
{
  // With a one-frontier horizon, the DP recurrence has no pairwise transitions to
  // evaluate and should reduce to the best finite edge from the robot start node.
  auto matrix = make_matrix(3U);
  set_cost(matrix, 0U, 1U, 5.0);
  set_cost(matrix, 0U, 2U, 2.0);
  set_cost(matrix, 0U, 3U, 7.0);

  const auto order = solve_bounded_horizon_mrtsp_order(matrix, 1U);

  ASSERT_EQ(order.size(), 1U);
  EXPECT_EQ(order[0], 1U);
}

TEST(MrtspSolverTests, HorizonClampsToCandidateCount)
{
  // Requesting a horizon longer than the candidate pool should evaluate every candidate
  // exactly once rather than failing or trying to allocate impossible route lengths.
  auto matrix = make_matrix(3U);
  set_cost(matrix, 0U, 1U, 1.0);
  set_cost(matrix, 0U, 2U, 2.0);
  set_cost(matrix, 0U, 3U, 3.0);
  for (std::size_t row = 1U; row < matrix.dimension; ++row) {
    for (std::size_t column = 1U; column < matrix.dimension; ++column) {
      if (row != column) {
        set_cost(matrix, row, column, 1.0);
      }
    }
  }

  const auto order = solve_bounded_horizon_mrtsp_order(matrix, 4U);

  EXPECT_EQ(order.size(), 3U);
}

TEST(MrtspSolverTests, InvalidMatrixReturnsEmptyOrderForFallback)
{
  // No finite start edge means no valid initial DP state. The core is responsible for
  // falling back to greedy when this empty result is returned.
  const auto matrix = make_matrix(2U);

  EXPECT_TRUE(solve_bounded_horizon_mrtsp_order(matrix, 2U).empty());
}

TEST(MrtspSolverTests, GreedyModePreservesFullCandidateOrderingAndBypassesCacheReuse)
{
  // Greedy mode ignores DP pruning limits and keeps the full MRTSP candidate order.
  // Changing the solver mode must invalidate the cache because the same frontier set
  // can produce a shorter receding-horizon sequence in DP mode.
  FrontierExplorerCoreParams params;
  params.strategy = FrontierStrategy::MRTSP;
  params.mrtsp_solver = "greedy";
  params.dp_solver_candidate_limit = 1U;
  params.dp_planning_horizon = 1U;
  FrontierExplorerCoreCallbacks callbacks;
  FrontierExplorerCore core(params, callbacks);

  const FrontierSequence frontiers = FrontierExplorerCore::to_frontier_sequence(
    {make_candidate(1.0), make_candidate(2.0), make_candidate(3.0)});

  const auto greedy_sequence = core.build_mrtsp_frontier_sequence(
    frontiers,
    make_pose(0.0, 0.0));
  EXPECT_EQ(greedy_sequence.size(), 3U);

  core.params.mrtsp_solver = "dp";
  const auto dp_sequence = core.build_mrtsp_frontier_sequence(
    frontiers,
    make_pose(0.0, 0.0));

  EXPECT_EQ(dp_sequence.size(), 1U);
  EXPECT_EQ(core.mrtsp_order_cache_misses, 2);
}

}  // namespace
}  // namespace frontier_exploration_ros2
