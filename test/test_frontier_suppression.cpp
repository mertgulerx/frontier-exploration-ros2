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

#include "frontier_exploration_ros2/frontier_suppression.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

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

FrontierCandidate make_candidate(double x, double y)
{
  return FrontierCandidate{{x, y}, {x, y}, 8};
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

std::unique_ptr<FrontierExplorerCore> make_suppression_core(
  int64_t * now_ns,
  int * dispatch_calls,
  std::vector<std::string> * info_logs = nullptr,
  std::vector<std::string> * warn_logs = nullptr)
{
  FrontierExplorerCoreParams params;
  params.frontier_suppression_enabled = true;
  params.frontier_suppression_attempt_threshold = 1;
  params.frontier_suppression_timeout_s = 90.0;
  params.frontier_suppression_no_progress_timeout_s = 5.0;
  params.frontier_suppression_progress_epsilon_m = 0.5;
  params.frontier_suppression_startup_grace_period_s = 0.0;
  params.return_to_start_on_complete = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [now_ns]() {return *now_ns;};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [dispatch_calls](const GoalDispatchRequest &) {
      *dispatch_calls += 1;
    };
  callbacks.log_info = [info_logs](const std::string & message) {
      if (info_logs) {
        info_logs->push_back(message);
      }
    };
  callbacks.log_warn = [warn_logs](const std::string & message) {
      if (warn_logs) {
        warn_logs->push_back(message);
      }
    };
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {make_candidate(4.0, 4.0)};
      result.robot_map_cell = {1, 1};
      return result;
    };

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->map_generation = 1;
  core->costmap_generation = 1;
  core->local_costmap_generation = 0;
  return core;
}

TEST(FrontierSuppressionTests, ThresholdCreatesRegionAndClearsAttempt)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    2, 2.0, 1.0, 90.0, 20.0, 0.25, 8, 4, 0.3});

  suppression.record_failed_attempt(make_candidate(2.0, 2.0), 1'000'000'000);
  EXPECT_EQ(suppression.attempt_count(), 1U);
  EXPECT_EQ(suppression.region_count(), 0U);

  suppression.record_failed_attempt(make_candidate(2.0, 2.0), 2'000'000'000);
  EXPECT_EQ(suppression.attempt_count(), 0U);
  ASSERT_EQ(suppression.region_count(), 1U);
  EXPECT_DOUBLE_EQ(suppression.regions().front().center.first, 2.0);
  EXPECT_DOUBLE_EQ(suppression.regions().front().side_length_m, 2.0);
}

TEST(FrontierSuppressionTests, ExpansionRingDoublesRegionAndMovesCenter)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    1, 2.0, 2.0, 90.0, 20.0, 0.25, 8, 4, 0.3});

  suppression.record_failed_attempt(make_candidate(0.0, 0.0), 1'000'000'000);
  suppression.record_failed_attempt(make_candidate(1.5, 0.0), 2'000'000'000);

  ASSERT_EQ(suppression.region_count(), 1U);
  EXPECT_DOUBLE_EQ(suppression.regions().front().center.first, 0.75);
  EXPECT_DOUBLE_EQ(suppression.regions().front().center.second, 0.0);
  EXPECT_DOUBLE_EQ(suppression.regions().front().side_length_m, 4.0);
}

TEST(FrontierSuppressionTests, PruneExpiredRemovesAttemptsAndRegions)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    2, 2.0, 1.0, 5.0, 20.0, 0.25, 8, 4, 0.3});

  suppression.record_failed_attempt(make_candidate(2.0, 2.0), 0);
  EXPECT_EQ(suppression.attempt_count(), 1U);
  suppression.prune_expired(6'000'000'000);
  EXPECT_EQ(suppression.attempt_count(), 0U);

  suppression.record_failed_attempt(make_candidate(3.0, 3.0), 7'000'000'000);
  suppression.record_failed_attempt(make_candidate(3.0, 3.0), 8'000'000'000);
  ASSERT_EQ(suppression.region_count(), 1U);
  suppression.prune_expired(14'000'000'000);
  EXPECT_EQ(suppression.region_count(), 0U);
}

TEST(FrontierSuppressionTests, AttemptCapacityEvictsOldestRecord)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    3, 2.0, 1.0, 90.0, 20.0, 0.25, 2, 4, 0.3});
  std::vector<std::string> warn_logs;
  const auto log_warn = [&warn_logs](const std::string & message) {warn_logs.push_back(message);};

  suppression.record_failed_attempt(make_candidate(1.0, 1.0), 1'000'000'000, log_warn);
  suppression.record_failed_attempt(make_candidate(2.0, 2.0), 2'000'000'000, log_warn);
  suppression.record_failed_attempt(make_candidate(3.0, 3.0), 3'000'000'000, log_warn);

  EXPECT_EQ(suppression.attempt_count(), 2U);
  EXPECT_FALSE(warn_logs.empty());
}

TEST(FrontierSuppressionTests, NoProgressTimeoutTriggersCancelRequest)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    3, 2.0, 1.0, 90.0, 5.0, 0.5, 8, 4, 0.3});

  suppression.start_goal_progress_tracking(3, 0);
  suppression.note_goal_progress(3, 10.0, 0);
  EXPECT_FALSE(suppression.mark_timeout_cancel_if_needed(3, 4'000'000'000));
  EXPECT_TRUE(suppression.mark_timeout_cancel_if_needed(3, 6'000'000'000));
  EXPECT_TRUE(suppression.progress_timeout_cancel_requested());
}

TEST(FrontierSuppressionTests, MeaningfulProgressResetsTimeout)
{
  FrontierSuppression suppression(FrontierSuppressionConfig{
    3, 2.0, 1.0, 90.0, 5.0, 0.5, 8, 4, 0.3});

  suppression.start_goal_progress_tracking(7, 0);
  suppression.note_goal_progress(7, 10.0, 0);
  suppression.note_goal_progress(7, 9.4, 3'000'000'000);
  EXPECT_FALSE(suppression.mark_timeout_cancel_if_needed(7, 7'000'000'000));
  EXPECT_TRUE(suppression.mark_timeout_cancel_if_needed(7, 9'000'000'000));
}

TEST(FrontierSuppressionCoreTests, RejectedFrontierBecomesSuppressedAndStopsRedispatch)
{
  int64_t now_ns = 1'000'000'000;
  int dispatch_calls = 0;
  std::vector<std::string> info_logs;
  auto core = make_suppression_core(&now_ns, &dispatch_calls, &info_logs, nullptr);

  core->try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  EXPECT_TRUE(core->suppression_state_allocated());

  core->goal_response_callback(core->current_dispatch_id, nullptr, false, "rejected");
  EXPECT_TRUE(core->suppression_state_allocated());
  EXPECT_EQ(core->suppressed_region_count(), 1U);

  core->try_send_next_goal();
  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_FALSE(info_logs.empty());
}

TEST(FrontierSuppressionCoreTests, NoProgressTimeoutCanceledGoalCreatesSuppressedRegion)
{
  int64_t now_ns = 0;
  int dispatch_calls = 0;
  auto core = make_suppression_core(&now_ns, &dispatch_calls, nullptr, nullptr);

  core->try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(core->current_dispatch_id, fake_handle, true, "");
  core->feedback_callback(10.0, core->current_dispatch_id);

  now_ns = 6'000'000'000;
  EXPECT_TRUE(core->evaluate_active_goal_progress_timeout());
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  fake_handle->resolve_cancel(true, "");
  core->get_result_callback(
    core->current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_CANCELED,
    0,
    "");

  EXPECT_EQ(core->suppressed_region_count(), 1U);
}

TEST(FrontierSuppressionCoreTests, AllSuppressedCanDispatchTemporaryReturnToStart)
{
  int64_t now_ns = 1'000'000'000;
  int dispatch_calls = 0;
  geometry_msgs::msg::Pose current_pose = make_pose(1.0, 1.0);
  std::vector<std::string> dispatched_goal_kinds;

  FrontierExplorerCoreParams params;
  params.frontier_suppression_enabled = true;
  params.frontier_suppression_attempt_threshold = 1;
  params.frontier_suppression_startup_grace_period_s = 0.0;
  params.all_frontiers_suppressed_behavior = "return_to_start";
  params.return_to_start_on_complete = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {return now_ns;};
  callbacks.get_current_pose = [&current_pose]() {
      return std::optional<geometry_msgs::msg::Pose>(current_pose);
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_goal_kinds](const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_goal_kinds.push_back(request.goal_kind);
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {make_candidate(4.0, 4.0)};
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

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  EXPECT_EQ(dispatched_goal_kinds.back(), "frontier");

  core.goal_response_callback(core.current_dispatch_id, nullptr, false, "rejected");
  current_pose = make_pose(3.0, 3.0);
  core.try_send_next_goal();

  ASSERT_EQ(dispatch_calls, 2);
  EXPECT_EQ(dispatched_goal_kinds.back(), "suppressed_return_to_start");
  EXPECT_FALSE(core.return_to_start_completed);
}

TEST(FrontierSuppressionCoreTests, TemporaryReturnToStartCancelsWhenFrontiersBecomeAvailableAgain)
{
  int64_t now_ns = 1'000'000'000;
  int dispatch_calls = 0;
  geometry_msgs::msg::Pose current_pose = make_pose(1.0, 1.0);
  std::vector<std::string> dispatched_goal_kinds;
  bool use_suppressed_frontier = true;

  FrontierExplorerCoreParams params;
  params.frontier_suppression_enabled = true;
  params.frontier_suppression_attempt_threshold = 1;
  params.frontier_suppression_startup_grace_period_s = 0.0;
  params.all_frontiers_suppressed_behavior = "return_to_start";
  params.return_to_start_on_complete = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {return now_ns;};
  callbacks.get_current_pose = [&current_pose]() {
      return std::optional<geometry_msgs::msg::Pose>(current_pose);
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_goal_kinds](const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_goal_kinds.push_back(request.goal_kind);
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.frontier_search = [&use_suppressed_frontier](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {
        use_suppressed_frontier ? make_candidate(4.0, 4.0) : make_candidate(8.0, 8.0)};
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

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  core.goal_response_callback(core.current_dispatch_id, nullptr, false, "rejected");

  current_pose = make_pose(3.0, 3.0);
  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 2);
  ASSERT_EQ(dispatched_goal_kinds.back(), "suppressed_return_to_start");

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_response_callback(core.current_dispatch_id, fake_handle, true, "");

  use_suppressed_frontier = false;
  core.occupancyGridCallback(OccupancyGrid2d(map_msg));
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_FALSE(core.pending_frontier_sequence.empty());

  fake_handle->resolve_cancel(true, "");
  core.get_result_callback(
    core.current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_CANCELED,
    0,
    "");

  ASSERT_EQ(dispatch_calls, 3);
  EXPECT_EQ(dispatched_goal_kinds.back(), "frontier");
}

TEST(FrontierSuppressionCoreTests, StartupGracePeriodDefersSuppressionFailures)
{
  int64_t now_ns = 0;
  int dispatch_calls = 0;
  FrontierExplorerCoreParams params;
  params.frontier_suppression_enabled = true;
  params.frontier_suppression_attempt_threshold = 1;
  params.frontier_suppression_startup_grace_period_s = 5.0;
  params.return_to_start_on_complete = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {return now_ns;};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {make_candidate(4.0, 4.0)};
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

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  core.goal_response_callback(core.current_dispatch_id, nullptr, false, "rejected");
  EXPECT_FALSE(core.suppression_state_allocated());
  EXPECT_EQ(core.suppressed_region_count(), 0U);

  now_ns = 6'000'000'000;
  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 2);
  core.goal_response_callback(core.current_dispatch_id, nullptr, false, "rejected");
  EXPECT_TRUE(core.suppression_state_allocated());
  EXPECT_EQ(core.suppressed_region_count(), 1U);
}

}  // namespace
}  // namespace frontier_exploration_ros2
