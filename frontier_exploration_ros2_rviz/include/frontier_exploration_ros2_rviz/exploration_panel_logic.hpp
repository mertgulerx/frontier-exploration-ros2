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

#include <chrono>
#include <optional>
#include <string>

#include <rcl_interfaces/msg/log.hpp>

#include "frontier_exploration_ros2/srv/control_exploration.hpp"

namespace frontier_exploration_ros2_rviz
{

using ControlExploration = frontier_exploration_ros2::srv::ControlExploration;

enum class PanelEventColor
{
  GREEN,
  BLUE,
  ORANGE,
  GRAY,
  RED,
};

struct BuiltControlRequest
{
  bool valid{false};
  uint8_t action{ControlExploration::Request::ACTION_START};
  float delay_seconds{0.0F};
  bool quit_after_stop{false};
  std::string command_text;
  std::string error_message;
};

struct ClassifiedLogEvent
{
  bool matched{false};
  PanelEventColor color{PanelEventColor::GRAY};
  std::string label{"Waiting"};
  std::string detail;
};

BuiltControlRequest build_control_request(
  uint8_t action,
  double delay_seconds,
  bool quit_after_stop);

std::string control_state_to_string(uint8_t state);
std::string panel_event_color_to_hex(PanelEventColor color);
ClassifiedLogEvent classify_log_event(const rcl_interfaces::msg::Log & log);

class CommandCountdownTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct ScheduledCommand
  {
    uint8_t action{ControlExploration::Request::ACTION_START};
    bool quit_after_stop{false};
    TimePoint deadline{};
    std::string command_text;
  };

  void arm(
    uint8_t action,
    double delay_seconds,
    bool quit_after_stop,
    const std::string & command_text,
    TimePoint now = Clock::now());

  void clear();
  [[nodiscard]] bool active(TimePoint now = Clock::now()) const;
  [[nodiscard]] std::optional<double> remaining_seconds(TimePoint now = Clock::now()) const;
  [[nodiscard]] uint8_t shadow_state(uint8_t fallback_state, TimePoint now = Clock::now()) const;
  [[nodiscard]] std::string status_text(TimePoint now = Clock::now()) const;

private:
  std::optional<ScheduledCommand> scheduled_command_;
};

}  // namespace frontier_exploration_ros2_rviz
