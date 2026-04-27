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

#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Validates frontier extraction primitives plus snapshot/marker cache behavior in core.

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = std::cos(yaw * 0.5);
  pose.orientation.z = std::sin(yaw * 0.5);
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  // Builds compact synthetic occupancy/cost grids for deterministic unit scenarios.
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

void set_cells(
  nav_msgs::msg::OccupancyGrid & msg,
  const std::vector<std::pair<int, int>> & cells,
  int value)
{
  const int width = static_cast<int>(msg.info.width);
  for (const auto & [x, y] : cells) {
    msg.data[static_cast<std::size_t>(y * width + x)] = static_cast<int8_t>(value);
  }
}

std::pair<OccupancyGrid2d, OccupancyGrid2d> simple_frontier_world(
  const std::vector<std::pair<int, int>> & free_cells,
  const std::vector<std::pair<int, int>> & blocked_cells = {})
{
  // Occupancy starts unknown, then carve free islands and optional blocked cost cells.
  auto map_msg = build_grid(10, 10, -1);
  set_cells(map_msg, free_cells, 0);

  auto costmap_msg = build_grid(10, 10, 0);
  set_cells(costmap_msg, blocked_cells, 100);

  return {OccupancyGrid2d(map_msg), OccupancyGrid2d(costmap_msg)};
}

FrontierCandidate dummy_frontier(double x = 1.0, double y = 1.0)
{
  return FrontierCandidate{{x, y}, {x, y}, 8};
}

void expect_frontier_results_equal(
  const FrontierSearchResult & expected,
  const FrontierSearchResult & actual)
{
  ASSERT_EQ(expected.frontiers.size(), actual.frontiers.size());
  ASSERT_EQ(expected.robot_map_cell, actual.robot_map_cell);
  for (std::size_t i = 0; i < expected.frontiers.size(); ++i) {
    EXPECT_EQ(expected.frontiers[i].centroid, actual.frontiers[i].centroid);
    EXPECT_EQ(expected.frontiers[i].goal_point, actual.frontiers[i].goal_point);
    EXPECT_EQ(expected.frontiers[i].size, actual.frontiers[i].size);
  }
}

std::unique_ptr<FrontierExplorerCore> make_snapshot_core()
{
  // Creates a core with stable clocks/maps so cache invalidation is fully controlled by tests.
  FrontierExplorerCoreParams params;
  FrontierExplorerCoreCallbacks callbacks;

  auto now_ns = std::make_shared<int64_t>(10'000'000'000);
  callbacks.now_ns = [now_ns]() {
      *now_ns += 1'000'000;
      return *now_ns;
    };
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);

  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);

  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->local_costmap = OccupancyGrid2d(costmap_msg);
  core->map_generation = 1;
  core->costmap_generation = 2;
  core->local_costmap_generation = 3;
  core->frontier_stats_log_throttle_seconds = 0.0;
  core->params.frontier_visit_tolerance = 0.3;

  return core;
}

// Frontier extraction behavior.
TEST(FrontierSearchTests, FrontierExtractionWhenRobotIsOnFreeCell)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, FrontierExtractionWhenRobotStartsUnknownUsesFindFree)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(0.0, 0.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, GlobalCostmapBlockingEliminatesFrontiers)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells, free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_TRUE(result.frontiers.empty());
}

TEST(FrontierSearchTests, LocalCostmapBlockingEliminatesFrontierGoals)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto local_costmap_msg = build_grid(10, 10, 0);
  set_cells(local_costmap_msg, free_cells, 100);
  auto local_costmap = OccupancyGrid2d(local_costmap_msg);

  auto result = get_frontier(
    make_pose(4.0, 4.0),
    occupancy_map,
    costmap,
    local_costmap);

  EXPECT_TRUE(result.frontiers.empty());
}

TEST(FrontierSearchTests, FrontierChoiceDeterministicAcrossIdenticalRuns)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 1; x < 4; ++x) {
    for (int y = 1; y < 4; ++y) {
      free_cells.emplace_back(x, y);
    }
  }
  for (int x = 6; x < 9; ++x) {
    for (int y = 6; y < 9; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto first = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);
  auto second = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);

  ASSERT_EQ(first.frontiers.size(), second.frontiers.size());
  for (std::size_t i = 0; i < first.frontiers.size(); ++i) {
    EXPECT_EQ(first.frontiers[i].centroid, second.frontiers[i].centroid);
    EXPECT_EQ(first.frontiers[i].goal_point, second.frontiers[i].goal_point);
    EXPECT_EQ(first.frontiers[i].size, second.frontiers[i].size);
  }
}

TEST(FrontierSearchTests, ConcurrentSearchProducesDeterministicEquivalentResults)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 2; x < 8; ++x) {
    for (int y = 2; y < 8; ++y) {
      if (!(x == 4 && y == 4)) {
        free_cells.emplace_back(x, y);
      }
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  const auto baseline = get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);

  auto future_one = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });
  auto future_two = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });

  const auto result_one = future_one.get();
  const auto result_two = future_two.get();
  expect_frontier_results_equal(baseline, result_one);
  expect_frontier_results_equal(baseline, result_two);
}

TEST(FrontierSearchTests, VisibleRevealGainStopsAtOccupiedWall)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 5; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{5, 6}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto visible_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(visible_gain.has_value());
  EXPECT_EQ(visible_gain->visible_reveal_cell_count, 0);
  EXPECT_DOUBLE_EQ(visible_gain->visible_reveal_length_m, 0.0);
}

TEST(FrontierSearchTests, VisibleRevealGainRespectsYawAndFov)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 4; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{3, 5}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto east_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);
  const auto north_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, -1.57079632679),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(east_gain.has_value());
  ASSERT_TRUE(north_gain.has_value());
  EXPECT_GT(east_gain->visible_reveal_cell_count, 0);
  EXPECT_EQ(north_gain->visible_reveal_cell_count, 0);
}

TEST(FrontierSearchTests, ExplorationCompleteCallbackRunsWhenNoFrontiersRemain)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;

  int completion_calls = 0;
  int frontier_search_calls = 0;
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  core->try_send_next_goal();
  core->try_send_next_goal();

  EXPECT_EQ(completion_calls, 1);
  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

// Frontier snapshot cache behavior.
TEST(FrontierSnapshotTests, SnapshotCacheHitsOnSameGenerationsAndRobotCell)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  const auto first = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  const auto second = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(first.signature, second.signature);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, SnapshotReusesCacheWhenOnlyRawMapGenerationChanges)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, RefreshDecisionMapSkipsGenerationBumpWhenOutputIsUnchanged)
{
  auto core = make_snapshot_core();
  core->params.frontier_map_optimization_enabled = true;

  auto map_msg = build_grid(20, 20, -1);
  set_cells(map_msg, {{4, 4}, {4, 5}, {5, 4}, {5, 5}}, 0);
  core->map = OccupancyGrid2d(map_msg);

  core->refresh_decision_map();
  const int generation_after_first_refresh = core->decision_map_generation;
  const int misses_after_first_refresh = core->decision_map_cache_misses;

  core->refresh_decision_map();

  EXPECT_EQ(core->decision_map_generation, generation_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_misses, misses_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_hits, 1);
}

TEST(FrontierSnapshotTests, DebugOutputsDoNotChangeSnapshotOrSelection)
{
  auto debug_core = make_snapshot_core();
  auto quiet_core = make_snapshot_core();
  debug_core->params.frontier_map_optimization_enabled = true;
  quiet_core->params.frontier_map_optimization_enabled = true;
  debug_core->callbacks.debug_outputs_enabled = []() {return true;};
  quiet_core->callbacks.debug_outputs_enabled = []() {return false;};
  debug_core->params.strategy = FrontierStrategy::MRTSP;
  quiet_core->params.strategy = FrontierStrategy::MRTSP;

  auto map_msg = build_grid(20, 20, -1);
  set_cells(
    map_msg,
    {
      {3, 3}, {3, 4}, {4, 3}, {4, 4},
      {10, 10}, {10, 11}, {11, 10}, {11, 11},
    },
    0);
  auto costmap_msg = build_grid(20, 20, 0);

  debug_core->map = OccupancyGrid2d(map_msg);
  quiet_core->map = OccupancyGrid2d(map_msg);
  debug_core->costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->costmap = OccupancyGrid2d(costmap_msg);
  debug_core->local_costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->local_costmap = OccupancyGrid2d(costmap_msg);
  debug_core->map_generation = 10;
  quiet_core->map_generation = 10;
  debug_core->costmap_generation = 11;
  quiet_core->costmap_generation = 11;
  debug_core->local_costmap_generation = 12;
  quiet_core->local_costmap_generation = 12;

  auto frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {3, 3};
      result.frontiers.push_back(FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 4});
      result.frontiers.push_back(FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 9});
      return result;
    };
  debug_core->callbacks.frontier_search = frontier_search;
  quiet_core->callbacks.frontier_search = frontier_search;

  const auto debug_snapshot = debug_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);
  const auto quiet_snapshot = quiet_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);

  ASSERT_EQ(debug_snapshot.frontiers.size(), quiet_snapshot.frontiers.size());
  EXPECT_EQ(debug_snapshot.signature, quiet_snapshot.signature);
  for (std::size_t i = 0; i < debug_snapshot.frontiers.size(); ++i) {
    const auto & debug_candidate = std::get<FrontierCandidate>(debug_snapshot.frontiers[i]);
    const auto & quiet_candidate = std::get<FrontierCandidate>(quiet_snapshot.frontiers[i]);
    EXPECT_EQ(debug_candidate.centroid, quiet_candidate.centroid);
    EXPECT_EQ(debug_candidate.goal_point, quiet_candidate.goal_point);
    EXPECT_EQ(debug_candidate.size, quiet_candidate.size);
  }

  const auto debug_selection = debug_core->select_frontier(debug_snapshot.frontiers, make_pose(3.0, 3.0));
  const auto quiet_selection = quiet_core->select_frontier(quiet_snapshot.frontiers, make_pose(3.0, 3.0));

  ASSERT_TRUE(debug_selection.frontier.has_value());
  ASSERT_TRUE(quiet_selection.frontier.has_value());
  const auto & debug_selected = std::get<FrontierCandidate>(*debug_selection.frontier);
  const auto & quiet_selected = std::get<FrontierCandidate>(*quiet_selection.frontier);
  EXPECT_EQ(debug_selected.centroid, quiet_selected.centroid);
  EXPECT_EQ(debug_selected.goal_point, quiet_selected.goal_point);
  EXPECT_EQ(debug_selected.size, quiet_selected.size);
}

TEST(FrontierSnapshotTests, MrtspForcesDecisionMapOptimizationRegardlessOfParameter)
{
  auto core = make_snapshot_core();
  core->params.strategy = FrontierStrategy::MRTSP;
  core->params.frontier_map_optimization_enabled = false;

  const auto config = core->decision_map_config();

  EXPECT_TRUE(core->mrtsp_enabled());
  EXPECT_TRUE(core->frontier_map_optimization_enabled());
  EXPECT_TRUE(config.optimization_enabled);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnDecisionMapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->decision_map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnCostmapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->costmap_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, CostmapCallbacksOnlyBumpGenerationWhenSearchInputChanges)
{
  auto core = make_snapshot_core();
  const int initial_costmap_generation = core->costmap_generation;
  const int initial_local_costmap_generation = core->local_costmap_generation;

  auto unchanged_costmap = build_grid(20, 20, 0);
  core->costmapCallback(OccupancyGrid2d(unchanged_costmap));
  core->localCostmapCallback(OccupancyGrid2d(unchanged_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation);

  auto changed_costmap = unchanged_costmap;
  set_cells(changed_costmap, {{3, 3}}, 100);
  core->costmapCallback(OccupancyGrid2d(changed_costmap));
  core->localCostmapCallback(OccupancyGrid2d(changed_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation + 1);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation + 1);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnRobotCellChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose & pose,
    const OccupancyGrid2d & occupancy,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = occupancy.worldToMap(pose.position.x, pose.position.y);
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(2.0, 2.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnMinGoalDistanceChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.4);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, ObservePostGoalSettleReusesSnapshotSignatureWithoutRecompute)
{
  auto core = make_snapshot_core();
  core->awaiting_map_refresh = true;
  core->post_goal_settle_active = true;
  core->post_goal_map_updates_seen = 0;
  core->post_goal_stable_update_count = 0;
  core->post_goal_last_frontier_signature.reset();
  core->params.frontier_candidate_min_goal_distance_m = 0.3;
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };

  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->observe_post_goal_settle_update();

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(core->post_goal_map_updates_seen, 1);
  EXPECT_EQ(core->post_goal_stable_update_count, 1);
}

TEST(FrontierSnapshotTests, ObservePostGoalSettleCanAdvanceFromCostmapTicksWithoutRecompute)
{
  auto core = make_snapshot_core();
  core->awaiting_map_refresh = true;
  core->post_goal_settle_active = true;
  core->map_updated = false;
  core->post_goal_map_updates_seen = 0;
  core->post_goal_stable_update_count = 0;
  core->post_goal_last_frontier_signature.reset();

  const auto seed = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->frontier_snapshot = seed;

  int refresh_calls = 0;
  core->callbacks.frontier_search = [&refresh_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      refresh_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->observe_post_goal_settle_update(false);
  core->observe_post_goal_settle_update(false);

  EXPECT_TRUE(core->map_updated);
  EXPECT_EQ(core->post_goal_map_updates_seen, 2);
  EXPECT_EQ(core->post_goal_stable_update_count, 2);
  EXPECT_EQ(refresh_calls, 0);
}

TEST(FrontierSelectionTests, SelectFrontierSequenceAddsLookAheadFrontier)
{
  auto core = make_snapshot_core();
  const FrontierSequence frontiers{
    PrimitiveFrontier{1.0, 0.0},
    PrimitiveFrontier{2.0, 0.0},
    PrimitiveFrontier{5.0, 0.0},
  };

  const auto frontier_sequence = core->select_frontier_sequence(
    frontiers,
    make_pose(0.0, 0.0),
    frontiers.front());

  ASSERT_EQ(frontier_sequence.size(), 2U);
  EXPECT_TRUE(core->are_frontiers_equivalent(frontier_sequence[0], frontiers[0]));
  EXPECT_TRUE(core->are_frontiers_equivalent(frontier_sequence[1], frontiers[1]));
}

TEST(FrontierSelectionTests, BuildGoalPoseFacesTargetWhenLookAheadUnavailable)
{
  auto core = make_snapshot_core();
  const auto goal_pose = core->build_goal_pose(
    PrimitiveFrontier{1.0, 1.0},
    make_pose(0.0, 0.0, 1.2));

  EXPECT_NEAR(goal_pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.w, 0.9238795325, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.z, 0.3826834324, 1e-9);
}

// Marker publish deduplication behavior.
TEST(FrontierMarkerTests, MarkerPublishDeduplicatedWhenSignatureUnchanged)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  FrontierSequence frontiers = FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)});
  core->publish_frontier_markers(frontiers);
  core->publish_frontier_markers(frontiers);

  EXPECT_EQ(publish_calls, 1);
}

TEST(FrontierMarkerTests, MarkerPublishRunsWhenSignatureChanges)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)}));
  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(3.0, 3.0)}));

  EXPECT_EQ(publish_calls, 2);
}

}  // namespace
}  // namespace frontier_exploration_ros2
