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
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/decision_map.hpp"
#include "frontier_exploration_ros2/frontier_policy.hpp"
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"

namespace frontier_exploration_ros2
{
namespace
{

geometry_msgs::msg::Pose make_pose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

void set_cell(nav_msgs::msg::OccupancyGrid & msg, int x, int y, int value)
{
  const int width = static_cast<int>(msg.info.width);
  msg.data[static_cast<std::size_t>(y * width + x)] = static_cast<int8_t>(value);
}

void set_rect(nav_msgs::msg::OccupancyGrid & msg, int x0, int y0, int x1, int y1, int value)
{
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      set_cell(msg, x, y, value);
    }
  }
}

std::vector<float> reference_bilateral_filter(
  const PaperImage & image,
  double sigma_s,
  double sigma_r)
{
  const double safe_sigma_s = std::max(sigma_s, 1e-6);
  const double safe_sigma_r = std::max(sigma_r, 1e-6);
  const int radius = std::max(1, static_cast<int>(std::ceil(2.0 * safe_sigma_s)));
  const double spatial_denominator = 2.0 * safe_sigma_s * safe_sigma_s;
  const double range_denominator = 2.0 * safe_sigma_r * safe_sigma_r;
  std::vector<float> filtered(image.data.size(), 0.0F);

  for (int y = 0; y < image.height; ++y) {
    const std::size_t row_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width);
    for (int x = 0; x < image.width; ++x) {
      const std::size_t idx = row_offset + static_cast<std::size_t>(x);
      const uint8_t center_value = image.data[idx];
      float weighted_sum = 0.0F;
      float normalization = 0.0F;

      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int sample_x = std::clamp(x + dx, 0, image.width - 1);
          const int sample_y = std::clamp(y + dy, 0, image.height - 1);
          const uint8_t shifted_value = image.at(sample_x, sample_y);
          const double distance_sq = static_cast<double>((dx * dx) + (dy * dy));
          const double diff =
            static_cast<double>(center_value) - static_cast<double>(shifted_value);
          const float weight = static_cast<float>(
            std::exp(-(distance_sq / spatial_denominator)) *
            std::exp(-((diff * diff) / range_denominator)));
          weighted_sum += weight * static_cast<float>(shifted_value);
          normalization += weight;
        }
      }

      filtered[idx] =
        normalization > std::numeric_limits<float>::epsilon() ?
        (weighted_sum / normalization) :
        static_cast<float>(center_value);
    }
  }

  return filtered;
}

PaperImage reference_threshold_image(
  const PaperImage & raw_image,
  const std::vector<float> & filtered_image)
{
  PaperImage threshold_image;
  threshold_image.width = raw_image.width;
  threshold_image.height = raw_image.height;
  threshold_image.data.resize(raw_image.data.size(), PAPER_UNKNOWN);
  const float free_threshold =
    static_cast<float>((static_cast<int>(PAPER_FREE) + static_cast<int>(PAPER_UNKNOWN)) / 2.0);

  for (std::size_t i = 0; i < raw_image.data.size(); ++i) {
    threshold_image.data[i] =
      filtered_image[i] >= free_threshold ? PAPER_FREE : PAPER_UNKNOWN;
    if (raw_image.data[i] == PAPER_OCCUPIED) {
      threshold_image.data[i] = PAPER_OCCUPIED;
    }
  }

  return threshold_image;
}

PaperImage reference_dilated_image(
  const PaperImage & raw_image,
  const PaperImage & threshold_image,
  int dilation_radius)
{
  PaperImage optimized_image;
  optimized_image.width = raw_image.width;
  optimized_image.height = raw_image.height;
  optimized_image.data.resize(raw_image.data.size(), PAPER_UNKNOWN);

  const int safe_radius = std::max(0, dilation_radius);
  for (int y = 0; y < raw_image.height; ++y) {
    const std::size_t row_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(raw_image.width);
    for (int x = 0; x < raw_image.width; ++x) {
      const std::size_t idx = row_offset + static_cast<std::size_t>(x);
      if (raw_image.data[idx] == PAPER_OCCUPIED) {
        optimized_image.data[idx] = PAPER_OCCUPIED;
        continue;
      }

      uint8_t optimized_value = PAPER_UNKNOWN;
      for (int dy = -safe_radius; dy <= safe_radius; ++dy) {
        for (int dx = -safe_radius; dx <= safe_radius; ++dx) {
          if ((dx * dx) + (dy * dy) > (safe_radius * safe_radius)) {
            continue;
          }

          const int sample_x = x + dx;
          const int sample_y = y + dy;
          if (
            sample_x < 0 || sample_y < 0 ||
            sample_x >= raw_image.width || sample_y >= raw_image.height)
          {
            continue;
          }

          if (threshold_image.at(sample_x, sample_y) == PAPER_FREE) {
            optimized_value = PAPER_FREE;
            break;
          }
        }
        if (optimized_value == PAPER_FREE) {
          break;
        }
      }

      optimized_image.data[idx] = optimized_value;
    }
  }

  return optimized_image;
}

std::vector<int8_t> paper_image_to_occupancy_costs(const PaperImage & image)
{
  std::vector<int8_t> data(image.data.size(), -1);
  for (std::size_t i = 0; i < image.data.size(); ++i) {
    if (image.data[i] == PAPER_FREE) {
      data[i] = 0;
    } else if (image.data[i] == PAPER_OCCUPIED) {
      data[i] = 100;
    }
  }
  return data;
}

TEST(DecisionMapTests, OccupancyToPaperMappingPreservesThresholdSemantics)
{
  auto map_msg = build_grid(6, 1, -1);
  set_cell(map_msg, 0, 0, -1);
  set_cell(map_msg, 1, 0, 0);
  set_cell(map_msg, 2, 0, 49);
  set_cell(map_msg, 3, 0, 50);
  set_cell(map_msg, 4, 0, 100);
  set_cell(map_msg, 5, 0, -42);

  const auto paper_image = occupancy_grid_to_paper_image(OccupancyGrid2d(map_msg), 50);

  const std::vector<uint8_t> expected{
    PAPER_UNKNOWN,
    PAPER_FREE,
    PAPER_FREE,
    PAPER_OCCUPIED,
    PAPER_OCCUPIED,
    PAPER_UNKNOWN,
  };
  EXPECT_EQ(paper_image.data, expected);
}

TEST(DecisionMapTests, OptimizationReducesFrontierCountWhilePreservingOccupiedCells)
{
  auto map_msg = build_grid(12, 12, -1);
  set_rect(map_msg, 2, 2, 9, 9, 0);
  set_cell(map_msg, 4, 4, -1);
  set_cell(map_msg, 7, 4, -1);
  set_cell(map_msg, 4, 7, -1);
  set_cell(map_msg, 7, 7, -1);
  set_cell(map_msg, 6, 6, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d raw_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.build_navigation_goal_point = false;

  const auto raw_frontiers = get_frontier(
    make_pose(3.0, 3.0),
    raw_map,
    costmap,
    std::nullopt,
    0.0,
    false,
    options);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;
  const auto decision_map_result = build_decision_map(raw_map, config);

  const auto decision_frontiers = get_frontier(
    make_pose(3.0, 3.0),
    decision_map_result.decision_map,
    costmap,
    std::nullopt,
    0.0,
    false,
    options);

  EXPECT_GT(raw_frontiers.frontiers.size(), decision_frontiers.frontiers.size());
  EXPECT_EQ(decision_map_result.decision_map.getCost(6, 6), 100);
}

TEST(DecisionMapTests, OptimizationKeepsNarrowDoorwayTraversable)
{
  auto map_msg = build_grid(12, 12, -1);
  set_rect(map_msg, 2, 2, 9, 5, 0);
  for (int x = 2; x <= 9; ++x) {
    if (x == 6) {
      continue;
    }
    set_cell(map_msg, x, 6, 100);
  }
  set_cell(map_msg, 6, 6, 0);
  set_cell(map_msg, 6, 7, 0);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d raw_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;
  const auto decision_map_result = build_decision_map(raw_map, config);

  EXPECT_EQ(decision_map_result.decision_map.getCost(6, 6), 0);
  EXPECT_EQ(decision_map_result.decision_map.getCost(5, 6), 100);

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.build_navigation_goal_point = false;
  const auto decision_frontiers = get_frontier(
    make_pose(6.0, 3.0),
    decision_map_result.decision_map,
    costmap,
    std::nullopt,
    0.0,
    false,
    options);
  EXPECT_FALSE(decision_frontiers.frontiers.empty());
}

TEST(DecisionMapTests, WorkspaceReusesExistingOutputForIdenticalMapContent)
{
  auto map_msg = build_grid(10, 10, -1);
  set_rect(map_msg, 2, 2, 7, 7, 0);

  const OccupancyGrid2d raw_map(map_msg);
  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;

  DecisionMapWorkspace workspace;
  const auto first = build_decision_map(raw_map, config, workspace);
  const auto second = build_decision_map(raw_map, config, workspace);

  EXPECT_TRUE(first.output_changed);
  EXPECT_FALSE(first.reused_existing_output);
  EXPECT_FALSE(second.output_changed);
  EXPECT_TRUE(second.reused_existing_output);
}

TEST(DecisionMapTests, FilterThresholdAndDilationMatchReferenceMathAtInteriorAndBorders)
{
  auto map_msg = build_grid(14, 14, -1);
  set_rect(map_msg, 2, 2, 11, 11, 0);
  set_rect(map_msg, 0, 0, 1, 3, 0);
  set_rect(map_msg, 11, 0, 13, 2, 0);
  set_cell(map_msg, 4, 4, -1);
  set_cell(map_msg, 7, 5, -1);
  set_cell(map_msg, 10, 10, -1);
  set_cell(map_msg, 0, 0, 100);
  set_cell(map_msg, 5, 5, 100);
  set_cell(map_msg, 12, 1, 100);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;

  const OccupancyGrid2d raw_map(map_msg);
  const auto raw_image = occupancy_grid_to_paper_image(raw_map, config.occ_threshold);
  const auto expected_filtered = reference_bilateral_filter(raw_image, config.sigma_s, config.sigma_r);
  const auto expected_threshold = reference_threshold_image(raw_image, expected_filtered);
  const auto expected_optimized =
    reference_dilated_image(raw_image, expected_threshold, config.dilation_kernel_radius_cells);
  const auto expected_costs = paper_image_to_occupancy_costs(expected_optimized);

  const auto result = build_decision_map(raw_map, config);

  EXPECT_EQ(result.raw_image.data, raw_image.data);
  ASSERT_EQ(result.filtered_image.size(), expected_filtered.size());
  for (std::size_t i = 0; i < result.filtered_image.size(); ++i) {
    EXPECT_NEAR(result.filtered_image[i], expected_filtered[i], 1e-4F);
  }
  EXPECT_EQ(result.threshold_image.data, expected_threshold.data);
  EXPECT_EQ(result.optimized_image.data, expected_optimized.data);
  EXPECT_EQ(result.optimized_map_msg.data, expected_costs);
}

TEST(DecisionMapTests, DirtyRegionWorkspaceRecomputeMatchesFullRecompute)
{
  auto first_map_msg = build_grid(12, 12, -1);
  set_rect(first_map_msg, 2, 2, 9, 9, 0);
  set_cell(first_map_msg, 4, 4, -1);
  set_cell(first_map_msg, 6, 6, 100);

  auto second_map_msg = first_map_msg;
  set_cell(second_map_msg, 4, 4, 0);
  set_cell(second_map_msg, 5, 4, 0);
  set_cell(second_map_msg, 5, 5, -1);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;

  DecisionMapWorkspace workspace;
  (void)build_decision_map(OccupancyGrid2d(first_map_msg), config, workspace);
  const auto incremental_status = build_decision_map(OccupancyGrid2d(second_map_msg), config, workspace);
  const auto full_result = build_decision_map(OccupancyGrid2d(second_map_msg), config);

  EXPECT_FALSE(incremental_status.geometry_changed);
  EXPECT_EQ(workspace.raw_image.data, full_result.raw_image.data);
  EXPECT_EQ(workspace.filtered_image, full_result.filtered_image);
  EXPECT_EQ(workspace.threshold_image.data, full_result.threshold_image.data);
  EXPECT_EQ(workspace.optimized_image.data, full_result.optimized_image.data);
  EXPECT_EQ(workspace.optimized_map_msg->data, full_result.optimized_map_msg.data);
}

TEST(DecisionMapTests, DisjointDirtyRegionsRecomputeMatchesFullRecompute)
{
  auto first_map_msg = build_grid(20, 20, -1);
  set_rect(first_map_msg, 2, 2, 17, 17, 0);
  set_cell(first_map_msg, 4, 4, -1);
  set_cell(first_map_msg, 15, 15, -1);
  set_cell(first_map_msg, 10, 10, 100);

  auto second_map_msg = first_map_msg;
  set_cell(second_map_msg, 3, 4, -1);
  set_cell(second_map_msg, 4, 4, 0);
  set_cell(second_map_msg, 16, 15, 100);
  set_cell(second_map_msg, 15, 14, -1);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;

  DecisionMapWorkspace workspace;
  (void)build_decision_map(OccupancyGrid2d(first_map_msg), config, workspace);
  const auto incremental_status = build_decision_map(OccupancyGrid2d(second_map_msg), config, workspace);
  const auto full_result = build_decision_map(OccupancyGrid2d(second_map_msg), config);

  EXPECT_FALSE(incremental_status.geometry_changed);
  EXPECT_EQ(workspace.raw_image.data, full_result.raw_image.data);
  EXPECT_EQ(workspace.filtered_image, full_result.filtered_image);
  EXPECT_EQ(workspace.threshold_image.data, full_result.threshold_image.data);
  EXPECT_EQ(workspace.optimized_image.data, full_result.optimized_image.data);
  EXPECT_EQ(workspace.optimized_map_msg->data, full_result.optimized_map_msg.data);
}

TEST(DecisionMapTests, BorderDirtySpansRecomputeMatchesFullRecompute)
{
  auto first_map_msg = build_grid(16, 16, -1);
  set_rect(first_map_msg, 1, 1, 14, 14, 0);
  set_cell(first_map_msg, 0, 0, 100);
  set_cell(first_map_msg, 15, 15, 100);

  auto second_map_msg = first_map_msg;
  set_cell(second_map_msg, 0, 1, 0);
  set_cell(second_map_msg, 1, 0, -1);
  set_cell(second_map_msg, 15, 14, 0);
  set_cell(second_map_msg, 14, 15, -1);

  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 50;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;

  DecisionMapWorkspace workspace;
  (void)build_decision_map(OccupancyGrid2d(first_map_msg), config, workspace);
  const auto incremental_status = build_decision_map(OccupancyGrid2d(second_map_msg), config, workspace);
  const auto full_result = build_decision_map(OccupancyGrid2d(second_map_msg), config);

  EXPECT_FALSE(incremental_status.geometry_changed);
  EXPECT_EQ(workspace.raw_image.data, full_result.raw_image.data);
  EXPECT_EQ(workspace.filtered_image, full_result.filtered_image);
  EXPECT_EQ(workspace.threshold_image.data, full_result.threshold_image.data);
  EXPECT_EQ(workspace.optimized_image.data, full_result.optimized_image.data);
  EXPECT_EQ(workspace.optimized_map_msg->data, full_result.optimized_map_msg.data);
}

TEST(MrtspOrderingTests, GreedyOrderingCanPreferHighGainFrontierOverNearest)
{
  const FrontierCandidate near_small_gain{
    {1.0, 0.0},
    {1.0, 0.0},
    {1, 0},
    {1, 0},
    {1.0, 0.0},
    std::nullopt,
    1};
  const FrontierCandidate farther_high_gain{
    {2.0, 0.0},
    {2.0, 0.0},
    {2, 0},
    {2, 0},
    {2.0, 0.0},
    std::nullopt,
    10};

  RobotState robot_state;
  robot_state.position = {0.0, 0.0};
  robot_state.yaw = 0.0;

  CostWeights weights;
  weights.distance_wd = 1.0;
  weights.gain_ws = 1.0;

  const auto matrix = build_cost_matrix(
    {near_small_gain, farther_high_gain},
    robot_state,
    weights,
    0.0,
    1.0,
    1.0);
  const auto order_first = greedy_mrtsp_order(matrix);
  const auto order_second = greedy_mrtsp_order(matrix);

  ASSERT_EQ(order_first.size(), 2U);
  EXPECT_EQ(order_first, order_second);
  EXPECT_EQ(order_first.front(), 1U);
}

TEST(MrtspOrderingTests, FrontierDispatchPointFallsBackToCenterPointWithoutGoalPoint)
{
  const FrontierCandidate nearest_candidate{
    {4.0, 4.0},
    {3.0, 4.0},
    {3, 4},
    {3, 4},
    {3.0, 4.0},
    std::make_optional(std::pair<double, double>{2.0, 4.0}),
    8};
  const FrontierCandidate mrtsp_candidate{
    {4.0, 4.0},
    {3.0, 4.0},
    {3, 4},
    {3, 4},
    {3.0, 4.0},
    std::nullopt,
    8};

  EXPECT_EQ(frontier_position(FrontierLike{nearest_candidate}), (std::pair<double, double>{2.0, 4.0}));
  EXPECT_EQ(frontier_position(FrontierLike{mrtsp_candidate}), (std::pair<double, double>{3.0, 4.0}));
}

TEST(MrtspOrderingTests, MinGoalDistanceFiltersMrtspCandidateByCenterPointDistance)
{
  auto map_msg = build_grid(8, 8, 0);
  auto costmap_msg = build_grid(8, 8, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  FrontierCache frontier_cache(8, 8);
  std::vector<FrontierPoint *> frontier_points{
    frontier_cache.getPoint(1, 1),
    frontier_cache.getPoint(1, 2),
    frontier_cache.getPoint(2, 1),
    frontier_cache.getPoint(2, 2),
  };

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.candidate_min_goal_distance_m = 2.0;
  options.build_navigation_goal_point = false;

  const auto rejected_candidate = build_frontier_candidate(
    frontier_points,
    {1, 1},
    occupancy_map,
    costmap,
    std::nullopt,
    frontier_cache,
    make_pose(2.0, 2.0),
    0.0,
    options);
  EXPECT_FALSE(rejected_candidate.has_value());

  const auto accepted_candidate = build_frontier_candidate(
    frontier_points,
    {1, 1},
    occupancy_map,
    costmap,
    std::nullopt,
    frontier_cache,
    make_pose(5.0, 5.0),
    0.0,
    options);
  ASSERT_TRUE(accepted_candidate.has_value());
}

TEST(MrtspOrderingTests, CenterPointTieBreakMatchesSortedCellOrder)
{
  auto map_msg = build_grid(8, 8, 0);
  auto costmap_msg = build_grid(8, 8, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  FrontierCache frontier_cache(8, 8);
  std::vector<FrontierPoint *> frontier_points{
    frontier_cache.getPoint(3, 2),
    frontier_cache.getPoint(2, 3),
    frontier_cache.getPoint(1, 2),
    frontier_cache.getPoint(2, 1),
  };

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.build_navigation_goal_point = false;

  const auto candidate = build_frontier_candidate(
    frontier_points,
    {3, 2},
    occupancy_map,
    costmap,
    std::nullopt,
    frontier_cache,
    make_pose(0.0, 0.0),
    0.0,
    options);

  ASSERT_TRUE(candidate.has_value());
  EXPECT_EQ(candidate->center_cell, (std::pair<int, int>{2, 1}));
  EXPECT_EQ(candidate->center_point, (std::pair<double, double>{2.5, 1.5}));
}

}  // namespace
}  // namespace frontier_exploration_ros2
