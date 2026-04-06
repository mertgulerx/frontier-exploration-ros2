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

TEST(FrontierSearchTests, ExplorationCompleteCallbackRunsWhenNoFrontiersRemain)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;

  int completion_calls = 0;
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  core->try_send_next_goal();

  EXPECT_EQ(completion_calls, 1);
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

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnMapGenerationChange)
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
