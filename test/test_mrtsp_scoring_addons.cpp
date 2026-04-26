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

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "frontier_exploration_ros2/frontier_scoring.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"

namespace frontier_exploration_ros2
{
namespace
{

FrontierCandidate make_candidate(double x, double y, int size)
{
  return FrontierCandidate{
    {x, y},
    {x, y},
    {static_cast<int>(std::llround(x)), static_cast<int>(std::llround(y))},
    {static_cast<int>(std::llround(x)), static_cast<int>(std::llround(y))},
    {x, y},
    std::nullopt,
    size};
}

TEST(MrtspScoringAddons, DirectionSimilarityHandlesCardinalCases)
{
  EXPECT_DOUBLE_EQ(direction_similarity({1.0, 0.0}, {1.0, 0.0}), 1.0);
  EXPECT_DOUBLE_EQ(direction_similarity({1.0, 0.0}, {-1.0, 0.0}), -1.0);
  EXPECT_NEAR(direction_similarity({1.0, 0.0}, {0.0, 1.0}), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(direction_similarity({0.0, 0.0}, {1.0, 0.0}), 0.0);
}

TEST(MrtspScoringAddons, DirectionBiasOnlyNudgesStartEdgeCost)
{
  MrtspScoringOptions options;
  options.direction_bias_enabled = true;
  options.direction_bias_weight = 0.20;
  options.direction_reverse_penalty_weight = 0.50;

  MrtspScoringContext context;
  context.desired_direction = std::array<double, 2>{1.0, 0.0};

  const std::array<double, 2> robot_position{0.0, 0.0};
  const auto forward = evaluate_mrtsp_score(
    10.0,
    make_candidate(2.0, 0.0, 10),
    0U,
    robot_position,
    2.0,
    true,
    options,
    context);
  const auto reverse = evaluate_mrtsp_score(
    10.0,
    make_candidate(-2.0, 0.0, 10),
    0U,
    robot_position,
    2.0,
    true,
    options,
    context);
  const auto future_edge = evaluate_mrtsp_score(
    10.0,
    make_candidate(-2.0, 0.0, 10),
    0U,
    robot_position,
    2.0,
    false,
    options,
    context);

  EXPECT_LT(forward.final_cost, 10.0);
  EXPECT_GT(reverse.final_cost, 10.0);
  EXPECT_DOUBLE_EQ(future_edge.final_cost, 10.0);
}

TEST(MrtspScoringAddons, DisabledOptionsLeaveMatrixUnchanged)
{
  const std::vector<FrontierCandidate> candidates{
    make_candidate(3.0, 0.0, 8),
    make_candidate(0.0, 3.0, 12),
  };
  RobotState robot_state;
  robot_state.position = {0.0, 0.0};
  robot_state.yaw = 0.0;
  CostWeights weights;

  const auto base_matrix = build_cost_matrix(
    candidates,
    robot_state,
    weights,
    1.5,
    0.5,
    1.0);
  const auto scored_matrix = build_cost_matrix(
    candidates,
    robot_state,
    weights,
    1.5,
    0.5,
    1.0,
    MrtspScoringOptions{},
    MrtspScoringContext{});

  ASSERT_EQ(base_matrix.values.size(), scored_matrix.values.size());
  EXPECT_EQ(base_matrix.values, scored_matrix.values);
}

TEST(MrtspScoringAddons, VisitedPenaltyUsesRadiusAndTimeout)
{
  const std::vector<VisitedFrontierRecord> history{
    {{1.0, 1.0}, 1'000'000'000, "success"},
  };

  EXPECT_DOUBLE_EQ(
    visited_frontier_penalty({1.2, 1.1}, history, 2'000'000'000, 0.5, 10.0, 0.75),
    0.75);
  EXPECT_DOUBLE_EQ(
    visited_frontier_penalty({3.0, 3.0}, history, 2'000'000'000, 0.5, 10.0, 0.75),
    0.0);
  EXPECT_DOUBLE_EQ(
    visited_frontier_penalty({1.2, 1.1}, history, 20'000'000'000, 0.5, 1.0, 0.75),
    0.0);
}

TEST(MrtspScoringAddons, LowGainDistanceDecayIsBoundedAndMonotonic)
{
  const double close_penalty = low_gain_distance_decay_penalty(0.25, 2.0, 8.0, 0.8, 0.5);
  const double far_penalty = low_gain_distance_decay_penalty(5.0, 2.0, 8.0, 0.8, 0.5);
  const double large_frontier_penalty = low_gain_distance_decay_penalty(5.0, 12.0, 8.0, 0.8, 0.5);

  EXPECT_GT(far_penalty, close_penalty);
  EXPECT_GT(close_penalty, 0.0);
  EXPECT_DOUBLE_EQ(large_frontier_penalty, 0.0);
}

TEST(MrtspScoringAddons, SoftObstaclePenaltyNormalizesAndSaturates)
{
  EXPECT_DOUBLE_EQ(soft_obstacle_penalty(20, 40, 90, 0.30), 0.0);
  EXPECT_NEAR(soft_obstacle_penalty(65, 40, 90, 0.30), 0.15, 1e-12);
  EXPECT_DOUBLE_EQ(soft_obstacle_penalty(100, 40, 90, 0.30), 0.30);
  EXPECT_DOUBLE_EQ(soft_obstacle_penalty(std::nullopt, 40, 90, 0.30), 0.0);
}

TEST(MrtspScoringAddons, BuildMatrixAppliesDirectionOnlyToStartRow)
{
  const std::vector<FrontierCandidate> candidates{
    make_candidate(3.0, 0.0, 8),
    make_candidate(-3.0, 0.0, 8),
  };
  RobotState robot_state;
  robot_state.position = {0.0, 0.0};
  robot_state.yaw = 0.0;
  CostWeights weights;

  MrtspScoringOptions options;
  options.direction_bias_enabled = true;
  options.direction_bias_weight = 0.25;
  options.direction_reverse_penalty_weight = 0.25;
  MrtspScoringContext context;
  context.desired_direction = std::array<double, 2>{1.0, 0.0};

  const auto base_matrix = build_cost_matrix(
    candidates,
    robot_state,
    weights,
    1.5,
    0.5,
    1.0);
  const auto scored_matrix = build_cost_matrix(
    candidates,
    robot_state,
    weights,
    1.5,
    0.5,
    1.0,
    options,
    context);

  EXPECT_NE(base_matrix.at(0U, 1U), scored_matrix.at(0U, 1U));
  EXPECT_NE(base_matrix.at(0U, 2U), scored_matrix.at(0U, 2U));
  EXPECT_DOUBLE_EQ(base_matrix.at(1U, 2U), scored_matrix.at(1U, 2U));
}

}  // namespace
}  // namespace frontier_exploration_ros2
