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

#include "frontier_exploration_ros2_rviz/exploration_panel_logic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace frontier_exploration_ros2_rviz
{
namespace
{

std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

std::string command_name(uint8_t action)
{
  return action == ControlExploration::Request::ACTION_STOP ? "stop" : "start";
}

std::string format_delay(double delay_seconds)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(delay_seconds < 10.0 ? 1 : 0) << delay_seconds;
  return stream.str();
}

bool contains_any(const std::string & haystack, std::initializer_list<const char *> needles)
{
  for (const char * needle : needles) {
    if (haystack.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

BuiltControlRequest build_control_request(
  uint8_t action,
  double delay_seconds,
  bool quit_after_stop)
{
  BuiltControlRequest request;
  request.action = action;
  request.delay_seconds = static_cast<float>(delay_seconds);
  request.quit_after_stop = quit_after_stop;

  if (
    action != ControlExploration::Request::ACTION_START &&
    action != ControlExploration::Request::ACTION_STOP)
  {
    request.error_message = "Unsupported control action";
    return request;
  }

  if (delay_seconds < 0.0 || !std::isfinite(delay_seconds)) {
    request.error_message = "Delay must be a finite non-negative value";
    return request;
  }

  if (
    quit_after_stop &&
    action != ControlExploration::Request::ACTION_STOP)
  {
    request.error_message = "Quit after stop is only valid for stop commands";
    return request;
  }

  request.valid = true;
  request.command_text = command_name(action);
  if (delay_seconds > 0.0) {
    request.command_text += " in " + format_delay(delay_seconds) + "s";
  } else {
    request.command_text += " now";
  }
  if (quit_after_stop) {
    request.command_text += " with -q";
  }
  return request;
}

std::string control_state_to_string(uint8_t state)
{
  switch (state) {
    case ControlExploration::Request::STATE_RUNNING:
      return "running";
    case ControlExploration::Request::STATE_START_SCHEDULED:
      return "start scheduled";
    case ControlExploration::Request::STATE_STOP_SCHEDULED:
      return "stop scheduled";
    case ControlExploration::Request::STATE_STOPPING:
      return "stopping";
    case ControlExploration::Request::STATE_SHUTDOWN_PENDING:
      return "shutdown pending";
    case ControlExploration::Request::STATE_IDLE:
    default:
      return "idle";
  }
}

std::string panel_event_color_to_hex(PanelEventColor color)
{
  switch (color) {
    case PanelEventColor::GREEN:
      return "#2e9d55";
    case PanelEventColor::BLUE:
      return "#2f6db5";
    case PanelEventColor::ORANGE:
      return "#7ecf6b";
    case PanelEventColor::RED:
      return "#b73a3a";
    case PanelEventColor::GRAY:
    default:
      return "#6d7680";
  }
}

ClassifiedLogEvent classify_log_event(const rcl_interfaces::msg::Log & log)
{
  const std::string message_lower = to_lower(log.msg);

  if (
    log.level >= rcl_interfaces::msg::Log::ERROR ||
    contains_any(
      message_lower,
      {" failed", "was rejected", "finished with status aborted", "error", "could not"}))
  {
    return {true, PanelEventColor::RED, "Error", log.msg};
  }

  if (
    contains_any(
      message_lower,
      {"returned to start pose", "return-to-start goal accepted", "reached start pose while frontiers remain temporarily suppressed",
        "exploration finished, returning to start pose"}))
  {
    return {true, PanelEventColor::BLUE, "Return to start", log.msg};
  }

  if (
    contains_any(
      message_lower,
      {"preempting active frontier goal", "skipping blocked frontier goal"}))
  {
    return {true, PanelEventColor::ORANGE, "Preemption", log.msg};
  }

  if (
    contains_any(
      message_lower,
      {"frontier goal accepted", "temporary return-to-start goal accepted while frontiers are suppressed"}))
  {
    return {true, PanelEventColor::GRAY, "Goal accepted", log.msg};
  }

  if (
    contains_any(
      message_lower,
      {"frontier goal reached", "no more frontiers found", "exploration finished at the start pose"}))
  {
    return {true, PanelEventColor::GREEN, "Goal reached", log.msg};
  }

  if (
    contains_any(
      message_lower,
      {"temporarily suppressed", "waiting for a map update", "cold idle", "explorer is in cold idle",
        "not ready yet, waiting", "staying at the start pose while waiting"}))
  {
    return {true, PanelEventColor::GRAY, "Waiting", log.msg};
  }

  return {false, PanelEventColor::GRAY, "Info", log.msg};
}

void CommandCountdownTracker::arm(
  uint8_t action,
  double delay_seconds,
  bool quit_after_stop,
  const std::string & command_text,
  TimePoint now)
{
  if (delay_seconds <= 0.0) {
    scheduled_command_.reset();
    return;
  }

  scheduled_command_ = ScheduledCommand{
    action,
    quit_after_stop,
    now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(delay_seconds)),
    command_text,
  };
}

void CommandCountdownTracker::clear()
{
  scheduled_command_.reset();
}

bool CommandCountdownTracker::active(TimePoint now) const
{
  return scheduled_command_.has_value() && now < scheduled_command_->deadline;
}

std::optional<double> CommandCountdownTracker::remaining_seconds(TimePoint now) const
{
  if (!active(now)) {
    return std::nullopt;
  }

  const auto remaining = scheduled_command_->deadline - now;
  return std::chrono::duration<double>(remaining).count();
}

uint8_t CommandCountdownTracker::shadow_state(uint8_t fallback_state, TimePoint now) const
{
  if (!active(now)) {
    return fallback_state;
  }

  return scheduled_command_->action == ControlExploration::Request::ACTION_STOP ?
         ControlExploration::Request::STATE_STOP_SCHEDULED :
         ControlExploration::Request::STATE_START_SCHEDULED;
}

std::string CommandCountdownTracker::status_text(TimePoint now) const
{
  const auto remaining = remaining_seconds(now);
  if (!remaining.has_value()) {
    return "";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1)
         << "Scheduled " << command_name(scheduled_command_->action)
         << " in " << *remaining << "s";
  if (scheduled_command_->quit_after_stop) {
    stream << " (-q)";
  }
  return stream.str();
}

}  // namespace frontier_exploration_ros2_rviz
