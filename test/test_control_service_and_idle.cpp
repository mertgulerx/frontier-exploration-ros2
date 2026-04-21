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

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
#include "frontier_exploration_ros2/srv/control_exploration.hpp"
#include "frontier_exploration_ctl_detail.hpp"

namespace frontier_exploration_ros2
{
namespace
{

using ControlExploration = frontier_exploration_ros2::srv::ControlExploration;

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
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

TEST(ControlCoreSessionTests, StopExplorationSessionDisablesScheduling)
{
  int frontier_search_calls = 0;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{1'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.frontier_search =
    [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      return FrontierSearchResult{};
    };

  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);
  core.map = OccupancyGrid2d(build_grid(10, 10, 0));
  core.costmap = OccupancyGrid2d(build_grid(10, 10, 0));
  core.stop_exploration_session("test stop");
  core.try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(ControlCoreSessionTests, StartExplorationSessionResetsSessionState)
{
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);

  geometry_msgs::msg::PoseStamped persistent_start_pose;
  persistent_start_pose.pose = make_pose(2.0, 3.0);
  core.start_pose = persistent_start_pose;
  core.pending_frontier_sequence = {PrimitiveFrontier{1.0, 1.0}};
  core.pending_frontier_selection_mode = "preferred";
  core.return_to_start_completed = true;
  core.no_frontiers_reported = true;
  core.frontier_suppression_ = std::make_unique<FrontierSuppression>(FrontierSuppressionConfig{});
  core.map = OccupancyGrid2d(build_grid(10, 10, 0));
  core.costmap = OccupancyGrid2d(build_grid(10, 10, 0));
  core.map_generation = 1;
  core.costmap_generation = 2;

  core.start_exploration_session();

  EXPECT_TRUE(core.exploration_enabled);
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);
  EXPECT_TRUE(core.pending_frontier_sequence.empty());
  EXPECT_TRUE(core.pending_frontier_selection_mode.empty());
  EXPECT_FALSE(core.return_to_start_completed);
  EXPECT_FALSE(core.no_frontiers_reported);
  EXPECT_FALSE(core.map.has_value());
  EXPECT_FALSE(core.costmap.has_value());
  EXPECT_EQ(core.map_generation, 0);
  EXPECT_EQ(core.costmap_generation, 0);
  EXPECT_FALSE(core.frontier_suppression_);
}

TEST(ControlCoreSessionTests, StopAndStartSessionsPreserveOriginalStartPose)
{
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);

  geometry_msgs::msg::PoseStamped persistent_start_pose;
  persistent_start_pose.pose = make_pose(4.0, 6.0);
  core.start_pose = persistent_start_pose;

  core.stop_exploration_session("test stop");
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);

  core.start_exploration_session();
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);
}

class FrontierControlNodeTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    helper_node_ = std::make_shared<rclcpp::Node>("frontier_control_test_helper");
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_->remove_node(node_);
      node_.reset();
    }
    if (helper_node_) {
      executor_->remove_node(helper_node_);
      helper_node_.reset();
    }
    executor_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void create_node(bool autostart, bool control_service_enabled = true)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("autostart", autostart),
      rclcpp::Parameter("control_service_enabled", control_service_enabled),
      rclcpp::Parameter("completion_event_enabled", false),
      rclcpp::Parameter("frontier_suppression_enabled", false),
    });
    node_ = std::make_shared<FrontierExplorerNode>(options);
    executor_->add_node(node_);
  }

  std::shared_ptr<ControlExploration::Response> call_control_service(
    uint8_t action,
    double delay_seconds = 0.0,
    bool quit_after_stop = false)
  {
    auto client = helper_node_->create_client<ControlExploration>("/control_exploration");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      ADD_FAILURE() << "control_exploration service did not become ready";
      return nullptr;
    }

    auto request = std::make_shared<ControlExploration::Request>();
    request->action = action;
    request->delay_seconds = static_cast<float>(delay_seconds);
    request->quit_after_stop = quit_after_stop;

    auto future = client->async_send_request(request);
    if (
      executor_->spin_until_future_complete(future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "control_exploration service call timed out";
      return nullptr;
    }
    return future.get();
  }

  bool wait_for_condition(
    const std::function<bool()> & predicate,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    return predicate();
  }

  bool wait_for_control_service_availability(
    bool expected_available,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    auto client = helper_node_->create_client<ControlExploration>("/control_exploration");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      const bool available = client->wait_for_service(std::chrono::milliseconds(0));
      if (available == expected_available) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    return client->wait_for_service(std::chrono::milliseconds(0)) == expected_available;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr helper_node_;
  std::shared_ptr<FrontierExplorerNode> node_;
};

TEST(ControlCliParserTests, ParsesImmediateStart)
{
  const auto parsed = parse_control_command_args({"start"});
  ASSERT_TRUE(parsed.ok);
  EXPECT_EQ(parsed.command.action, ControlExploration::Request::ACTION_START);
  EXPECT_DOUBLE_EQ(parsed.command.delay_seconds, 0.0);
  EXPECT_FALSE(parsed.command.quit_after_stop);
  EXPECT_EQ(parsed.command.service_name, "control_exploration");
}

TEST(ControlCliParserTests, ParsesDelayedStopWithQuit)
{
  const auto parsed = parse_control_command_args({"stop", "-t", "10", "-q"});
  ASSERT_TRUE(parsed.ok);
  EXPECT_EQ(parsed.command.action, ControlExploration::Request::ACTION_STOP);
  EXPECT_DOUBLE_EQ(parsed.command.delay_seconds, 10.0);
  EXPECT_TRUE(parsed.command.quit_after_stop);
}

TEST(ControlCliParserTests, RejectsQuitOnStart)
{
  const auto parsed = parse_control_command_args({"start", "-q"});
  EXPECT_FALSE(parsed.ok);
}

TEST_F(FrontierControlNodeTests, AutostartFalseKeepsSubscriptionsInactive)
{
  create_node(false);

  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));
}

TEST_F(FrontierControlNodeTests, AutostartTrueCreatesSubscriptions)
{
  create_node(true);

  ASSERT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(2000)));
}

TEST_F(FrontierControlNodeTests, ControlServiceCanBeDisabledWhenAutostartIsTrue)
{
  create_node(true, false);

  EXPECT_FALSE(node_->hasControlService());
  EXPECT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(2000)));
  EXPECT_TRUE(wait_for_control_service_availability(false));
}

TEST_F(FrontierControlNodeTests, ColdIdleForcesControlServiceOn)
{
  create_node(false, false);

  EXPECT_TRUE(node_->hasControlService());
  EXPECT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));
  EXPECT_TRUE(wait_for_control_service_availability(true));
}

TEST_F(FrontierControlNodeTests, StartServiceActivatesSubscriptions)
{
  create_node(false);

  const auto response = call_control_service(ControlExploration::Request::ACTION_START);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->accepted);
  EXPECT_FALSE(response->scheduled);
  EXPECT_EQ(response->state, ControlExploration::Request::STATE_RUNNING);

  ASSERT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));
}

TEST_F(FrontierControlNodeTests, DelayedStartServiceActivatesSubscriptionsLater)
{
  create_node(false);

  const auto response = call_control_service(ControlExploration::Request::ACTION_START, 0.1);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->accepted);
  EXPECT_TRUE(response->scheduled);
  EXPECT_EQ(response->state, ControlExploration::Request::STATE_START_SCHEDULED);

  ASSERT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(2000)));
}

TEST_F(FrontierControlNodeTests, StopServiceReturnsNodeToColdIdle)
{
  create_node(true);
  ASSERT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));

  const auto response = call_control_service(ControlExploration::Request::ACTION_STOP);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->accepted);
  EXPECT_FALSE(response->scheduled);
  EXPECT_EQ(response->state, ControlExploration::Request::STATE_IDLE);

  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));
}

TEST_F(FrontierControlNodeTests, DelayedStopIsRejectedWhileColdIdle)
{
  create_node(false);
  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));

  const auto response = call_control_service(ControlExploration::Request::ACTION_STOP, 0.1);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->accepted);
  EXPECT_FALSE(response->scheduled);
  EXPECT_EQ(response->state, ControlExploration::Request::STATE_IDLE);
}

TEST_F(FrontierControlNodeTests, ImmediateStartClearsPendingScheduledStop)
{
  create_node(true);
  ASSERT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));

  const auto scheduled_stop = call_control_service(ControlExploration::Request::ACTION_STOP, 0.2);
  ASSERT_NE(scheduled_stop, nullptr);
  ASSERT_TRUE(scheduled_stop->accepted);
  EXPECT_TRUE(scheduled_stop->scheduled);
  EXPECT_EQ(scheduled_stop->state, ControlExploration::Request::STATE_STOP_SCHEDULED);

  const auto start_response = call_control_service(ControlExploration::Request::ACTION_START);
  ASSERT_NE(start_response, nullptr);
  EXPECT_TRUE(start_response->accepted);
  EXPECT_FALSE(start_response->scheduled);
  EXPECT_EQ(start_response->state, ControlExploration::Request::STATE_RUNNING);

  ASSERT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(500)));
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  executor_->spin_some();
  EXPECT_TRUE(node_->hasActiveExplorationSubscriptions());
}

TEST_F(FrontierControlNodeTests, StopWithQuitRequestsOnlyExplorerExit)
{
  create_node(false);
  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));

  const auto response = call_control_service(
    ControlExploration::Request::ACTION_STOP,
    0.0,
    true);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->accepted);
  EXPECT_FALSE(response->scheduled);
  EXPECT_EQ(response->state, ControlExploration::Request::STATE_SHUTDOWN_PENDING);

  ASSERT_TRUE(wait_for_condition([this]() { return node_->quitRequested(); }));
  EXPECT_TRUE(rclcpp::ok());
}

}  // namespace
}  // namespace frontier_exploration_ros2
