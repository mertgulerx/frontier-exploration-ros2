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

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Covers goal-update/cancel/reselection/result lifecycle edges in FrontierExplorerCore.

class FakeGoalHandle : public GoalHandleInterface
{
public:
  void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) override
  {
    cancel_calls += 1;
    pending_callback = std::move(callback);
  }

  void resolve_cancel(bool accepted = true, const std::string & error_message = "")
  {
    if (pending_callback) {
      pending_callback(accepted, error_message);
      pending_callback = nullptr;
    }
  }

  int cancel_calls{0};

private:
  std::function<void(bool accepted, const std::string & error_message)> pending_callback;
};

geometry_msgs::msg::Pose make_pose(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  // Shared synthetic grid builder for map/costmap-only unit scenarios.
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

std::unique_ptr<FrontierExplorerCore> make_preemption_core(std::vector<std::string> * info_logs = nullptr)
{
  // Creates an ACTIVE frontier-goal core so tests can exercise goal-update entrypoints directly.
  FrontierExplorerCoreParams params;
  params.goal_preemption_enabled = true;
  params.goal_skip_on_blocked_goal = true;
  params.goal_preemption_min_interval_s = 0.0;
  params.post_goal_settle_enabled = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose());
    };
  callbacks.log_info = [info_logs](const std::string & message) {
      if (info_logs != nullptr) {
        info_logs->push_back(message);
      }
    };
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);

  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);

  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->local_costmap.reset();
  core->map_generation = 1;
  core->costmap_generation = 1;
  core->local_costmap_generation = 1;

  core->set_goal_state(GoalLifecycleState::ACTIVE);
  core->active_goal_frontier = PrimitiveFrontier{1.0, 1.0};
  core->active_goal_frontiers = {PrimitiveFrontier{1.0, 1.0}};
  core->active_goal_kind = "frontier";
  core->active_goal_sent_time_ns = 0;
  core->current_dispatch_id = 1;

  return core;
}

// Replacement and cancellation policy behavior.
TEST(PreemptionFlowTests, ReselectionReplacementDispatchesWithoutCancel)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };

  FrontierSequence frontier_sequence{PrimitiveFrontier{2.0, 2.0}};
  core->request_frontier_reselection(frontier_sequence, make_pose(), "preferred", "replacement");
  core->request_frontier_reselection(frontier_sequence, make_pose(), "preferred", "replacement");

  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, ReplacementDebounceTracksSelectedFrontierOnly)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };

  const FrontierCandidate selected_frontier{
    {2.0, 2.0},
    {2.0, 2.0},
    {2, 2},
    {2, 2},
    {2.0, 2.0},
    std::nullopt,
    6};
  const FrontierCandidate look_ahead_a{
    {4.0, 4.0},
    {4.0, 4.0},
    {4, 4},
    {4, 4},
    {4.0, 4.0},
    std::nullopt,
    6};
  const FrontierCandidate look_ahead_b{
    {6.0, 6.0},
    {6.0, 6.0},
    {6, 6},
    {6, 6},
    {6.0, 6.0},
    std::nullopt,
    6};

  const FrontierSequence first_sequence{selected_frontier, look_ahead_a};
  const FrontierSequence second_sequence{selected_frontier, look_ahead_b};

  core->request_frontier_reselection(first_sequence, make_pose(), "preferred", "replacement");
  core->request_frontier_reselection(second_sequence, make_pose(), "preferred", "replacement");

  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, BlockedGoalWithoutReplacementUsesExplicitCancel)
{
  std::vector<std::string> info_logs;
  std::vector<std::string> debug_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->callbacks.log_debug = [&debug_logs](const std::string & message) {
      debug_logs.push_back(message);
    };

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers.clear();
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 1);
  ASSERT_FALSE(debug_logs.empty());
  EXPECT_NE(debug_logs.back().find("no replacement frontier is available"), std::string::npos);
}

TEST(PreemptionFlowTests, BlockedGoalReplacementLogsSkipVerb)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->replacement_required_hits = 1;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(dispatch_calls, 1);
  ASSERT_FALSE(info_logs.empty());
  bool found_skip_log = false;
  for (const auto & message : info_logs) {
    if (message.find("Skipping blocked frontier goal:") != std::string::npos) {
      found_skip_log = true;
      break;
    }
  }
  EXPECT_TRUE(found_skip_log);
}

TEST(PreemptionFlowTests, PreemptionReplacementDispatchesImmediatelyWhenSettleEnabled)
{
  std::vector<std::string> info_logs;
  FrontierExplorerCoreParams params;
  params.goal_preemption_enabled = true;
  params.goal_skip_on_blocked_goal = true;
  params.goal_preemption_min_interval_s = 0.0;
  params.post_goal_settle_enabled = true;
  params.post_goal_min_settle = 0.0;
  params.post_goal_required_map_updates = 1;
  params.post_goal_stable_updates = 1;

  int64_t now_ns = 5'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      now_ns += 100'000'000;
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose());
    };
  callbacks.log_info = [&info_logs](const std::string & message) {
      info_logs.push_back(message);
    };
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.local_costmap.reset();
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 1;
  core.set_goal_state(GoalLifecycleState::ACTIVE);
  core.active_goal_frontier = PrimitiveFrontier{1.0, 1.0};
  core.active_goal_frontiers = {PrimitiveFrontier{1.0, 1.0}};
  core.active_goal_kind = "frontier";
  core.active_goal_sent_time_ns = 0;
  core.current_dispatch_id = 1;

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_handle = fake_handle;

  int dispatch_calls = 0;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core.callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };
  core.replacement_required_hits = 1;

  core.consider_preempt_active_goal("map");

  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_TRUE(core.pending_frontier_sequence.empty());
  EXPECT_FALSE(core.awaiting_map_refresh);
  EXPECT_FALSE(core.post_goal_settle_active);
}

TEST(PreemptionFlowTests, SupersededResultCallbackDoesNotClearActiveState)
{
  auto core = make_preemption_core();
  core->current_dispatch_id = 2;
  core->set_goal_state(GoalLifecycleState::ACTIVE);

  core->get_result_callback(
    1,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_TRUE(core->goal_in_progress);
}

TEST(PreemptionFlowTests, CostmapTriggerOnlyChecksBlockingAndSkipsReselection)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  int frontier_search_calls = 0;
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
      return result;
    };

  core->consider_preempt_active_goal("costmap");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_TRUE(core->active_goal_blocked_reason.has_value());
}

TEST(PreemptionFlowTests, BlockedGoalSkipCanBeDisabledViaParameter)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_enabled = false;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_FALSE(core->active_goal_blocked_reason.has_value());
}

TEST(PreemptionFlowTests, VisibleGainPreemptionCanBeDisabledViaParameter)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;

  int frontier_search_calls = 0;
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
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(PreemptionFlowTests, VisibleGainGateSkipsSnapshotSearchWhenGoalStillHasVisibleReveal)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
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
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_TRUE(core->pending_frontier_sequence.empty());
}

TEST(PreemptionFlowTests, VisibleGainGateFallsBackToSnapshotReselectionWhenGoalGainIsExhausted)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 4.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;
  const FrontierCandidate active_frontier{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontier = active_frontier;
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, 0);
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_EQ(dispatch_calls, 1);
}

TEST(PreemptionFlowTests, LowGainSnapshotReselectionIsThrottledByPreemptionInterval)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_min_interval_s = 0.5;
  core->params.goal_preemption_lidar_range_m = 4.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;
  const FrontierCandidate active_frontier{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontier = active_frontier;
  core->active_goal_frontiers = {*core->active_goal_frontier};

  int64_t now_ns = 5'000'000'000;
  core->callbacks.now_ns = [&now_ns]() {
      return now_ns;
    };

  auto map_msg = build_grid(12, 12, 0);
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls, active_frontier](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {active_frontier};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");
  EXPECT_EQ(frontier_search_calls, 1);

  core->frontier_snapshot.reset();
  core->map_generation += 1;
  core->decision_map_generation += 1;
  now_ns += 100'000'000;
  core->consider_preempt_active_goal("map");
  EXPECT_EQ(frontier_search_calls, 1);

  core->frontier_snapshot.reset();
  core->map_generation += 1;
  core->decision_map_generation += 1;
  now_ns += 500'000'000;
  core->consider_preempt_active_goal("map");
  EXPECT_EQ(frontier_search_calls, 2);
}

TEST(PreemptionFlowTests, CompletionDistanceTreatsNearFrontierAsCompleteWithVisibleGainGate)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.9, 5.0));
    };

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {
        FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8},
        FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceIsDisabledAtZero)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_complete_if_within_m = 0.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.9, 5.0));
    };

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
}

TEST(PreemptionFlowTests, CompletionDistanceTriggersWhenVisibleGainPreemptionDisabled)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.9, 5.0));
    };

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {
        FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8},
        FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceDoesNotReselectWhenOutsideThresholdAndPreemptionDisabled)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 5.0));
    };

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
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
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, CompletionDistanceCancelsWithoutReplacementAndAvoidsRedispatch)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.9, 5.0));
    };

  int completion_calls = 0;
  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double min_goal_distance,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      if (min_goal_distance >= 0.25) {
        result.robot_map_cell = {0, 0};
        return result;
      }
      result.frontiers = {FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_EQ(dispatch_calls, 0);
  ASSERT_TRUE(core->distance_completed_frontier.has_value());

  core->get_result_callback(
    core->current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_CANCELED,
    0,
    "");
  core->map_updated = true;
  core->try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(completion_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

// Queue/lifecycle and completion gating behavior.
TEST(PreemptionFlowTests, ReplacementQueueStaysSingleWithLatestSequence)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  core->replacement_required_hits = 1;
  core->callbacks.wait_for_action_server = [](double) {return false;};

  FrontierSequence first{PrimitiveFrontier{2.0, 2.0}};
  FrontierSequence second{PrimitiveFrontier{3.0, 3.0}};

  core->request_frontier_reselection(first, make_pose(), "preferred", "first");
  core->request_frontier_reselection(second, make_pose(), "preferred", "second");

  EXPECT_TRUE(core->are_frontier_sequences_equivalent(core->pending_frontier_sequence, second));
}

TEST(PreemptionFlowTests, ReturnToStartCompletedStopsFurtherFrontierSearch)
{
  auto core = make_preemption_core();
  core->set_goal_state(GoalLifecycleState::IDLE);
  core->return_to_start_completed = true;

  int frontier_search_calls = 0;
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
      return result;
    };

  core->try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(PreemptionFlowTests, GoalSucceededCanProgressWithCostmapOnlyUpdates)
{
  FrontierExplorerCoreParams params;
  params.post_goal_settle_enabled = true;
  params.post_goal_min_settle = 0.0;
  params.post_goal_required_map_updates = 2;
  params.post_goal_stable_updates = 1;
  params.return_to_start_on_complete = false;

  int64_t now_ns = 1'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      now_ns += 100'000'000;  // 0.1s
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.wait_for_action_server = [](double) {return true;};

  int dispatch_calls = 0;
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 10}};
      result.robot_map_cell = {1, 1};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 0;

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_EQ(core.current_dispatch_id, 1);

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_response_callback(core.current_dispatch_id, fake_handle, true, "");
  core.get_result_callback(
    core.current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_TRUE(core.awaiting_map_refresh);
  EXPECT_TRUE(core.post_goal_settle_active);

  // Only costmap ticks arrive: core should still leave settle state and dispatch next goal.
  core.costmapCallback(OccupancyGrid2d(costmap_msg));
  EXPECT_EQ(dispatch_calls, 1);

  core.occupancyGridCallback(OccupancyGrid2d(map_msg));
  EXPECT_EQ(dispatch_calls, 2);
}

TEST(PreemptionFlowTests, GoalSucceededUsesSingleRefreshWaitWhenPostGoalSettleDisabled)
{
  FrontierExplorerCoreParams params;
  params.post_goal_settle_enabled = false;
  params.post_goal_min_settle = 99.0;
  params.post_goal_required_map_updates = 99;
  params.post_goal_stable_updates = 99;
  params.return_to_start_on_complete = false;

  int64_t now_ns = 1'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      now_ns += 100'000'000;
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.wait_for_action_server = [](double) {return true;};

  int dispatch_calls = 0;
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 10}};
      result.robot_map_cell = {1, 1};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 0;

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_EQ(core.current_dispatch_id, 1);

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_response_callback(core.current_dispatch_id, fake_handle, true, "");
  core.get_result_callback(
    core.current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_TRUE(core.awaiting_map_refresh);
  EXPECT_FALSE(core.post_goal_settle_active);

  core.occupancyGridCallback(OccupancyGrid2d(map_msg));
  EXPECT_EQ(dispatch_calls, 2);
}

}  // namespace
}  // namespace frontier_exploration_ros2
