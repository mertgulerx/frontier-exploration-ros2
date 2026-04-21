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

#include <chrono>

#include "frontier_exploration_ros2_rviz/exploration_panel_logic.hpp"

namespace frontier_exploration_ros2_rviz
{
namespace
{

TEST(ExplorationPanelLogicTests, BuildsStopRequestWithQuitFlag)
{
  const auto request = build_control_request(
    ControlExploration::Request::ACTION_STOP,
    5.0,
    true);

  ASSERT_TRUE(request.valid);
  EXPECT_EQ(request.action, ControlExploration::Request::ACTION_STOP);
  EXPECT_FLOAT_EQ(request.delay_seconds, 5.0F);
  EXPECT_TRUE(request.quit_after_stop);
  EXPECT_NE(request.command_text.find("-q"), std::string::npos);
}

TEST(ExplorationPanelLogicTests, RejectsQuitFlagOnStart)
{
  const auto request = build_control_request(
    ControlExploration::Request::ACTION_START,
    0.0,
    true);

  EXPECT_FALSE(request.valid);
  EXPECT_NE(request.error_message.find("only valid"), std::string::npos);
}

TEST(ExplorationPanelLogicTests, ClassifiesReturnToStartLogsAsBlue)
{
  rcl_interfaces::msg::Log log;
  log.level = rcl_interfaces::msg::Log::INFO;
  log.msg = "Returned to start pose";

  const auto event = classify_log_event(log);

  EXPECT_TRUE(event.matched);
  EXPECT_EQ(event.color, PanelEventColor::BLUE);
  EXPECT_EQ(event.label, "Return to start");
}

TEST(ExplorationPanelLogicTests, ClassifiesPreemptionLogsAsOrange)
{
  rcl_interfaces::msg::Log log;
  log.level = rcl_interfaces::msg::Log::INFO;
  log.msg = "Preempting active frontier goal: active frontier no longer appears";

  const auto event = classify_log_event(log);

  EXPECT_TRUE(event.matched);
  EXPECT_EQ(event.color, PanelEventColor::ORANGE);
  EXPECT_EQ(event.label, "Preemption");
}

TEST(ExplorationPanelLogicTests, IgnoresFollowupPreemptionCancelDebugLog)
{
  rcl_interfaces::msg::Log log;
  log.level = rcl_interfaces::msg::Log::DEBUG;
  log.msg = "Canceling active frontier goal before applying the updated frontier selection";

  const auto event = classify_log_event(log);

  EXPECT_FALSE(event.matched);
}

TEST(ExplorationPanelLogicTests, ClassifiesGoalAcceptedLogs)
{
  rcl_interfaces::msg::Log log;
  log.level = rcl_interfaces::msg::Log::INFO;
  log.msg = "Frontier goal accepted";

  const auto event = classify_log_event(log);

  EXPECT_TRUE(event.matched);
  EXPECT_EQ(event.color, PanelEventColor::GRAY);
  EXPECT_EQ(event.label, "Goal accepted");
}

TEST(ExplorationPanelLogicTests, CountdownTrackerShadowsScheduledState)
{
  CommandCountdownTracker tracker;
  const auto now = CommandCountdownTracker::Clock::now();

  tracker.arm(
    ControlExploration::Request::ACTION_START,
    3.0,
    false,
    "start in 3.0s",
    now);

  EXPECT_TRUE(tracker.active(now + std::chrono::milliseconds(500)));
  EXPECT_EQ(
    tracker.shadow_state(ControlExploration::Request::STATE_IDLE, now + std::chrono::milliseconds(500)),
    ControlExploration::Request::STATE_START_SCHEDULED);

  const auto remaining = tracker.remaining_seconds(now + std::chrono::milliseconds(500));
  ASSERT_TRUE(remaining.has_value());
  EXPECT_GT(*remaining, 2.0);

  EXPECT_FALSE(tracker.active(now + std::chrono::seconds(4)));
  EXPECT_EQ(
    tracker.shadow_state(ControlExploration::Request::STATE_RUNNING, now + std::chrono::seconds(4)),
    ControlExploration::Request::STATE_RUNNING);
}

}  // namespace
}  // namespace frontier_exploration_ros2_rviz
