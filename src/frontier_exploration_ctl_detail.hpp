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

#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "frontier_exploration_ros2/srv/control_exploration.hpp"

namespace frontier_exploration_ros2
{

struct ParsedControlCommand
{
  uint8_t action{srv::ControlExploration::Request::ACTION_START};
  double delay_seconds{0.0};
  bool quit_after_stop{false};
  std::string service_name{"control_exploration"};
};

struct ParsedControlCommandResult
{
  bool ok{false};
  bool show_help{false};
  std::string error_message;
  ParsedControlCommand command;
};

inline ParsedControlCommandResult parse_control_command_args(
  const std::vector<std::string> & args)
{
  ParsedControlCommandResult result;
  bool command_seen = false;
  bool delay_seen = false;
  bool service_seen = false;

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & arg = args[i];
    if (arg == "-h" || arg == "--help") {
      result.show_help = true;
      result.ok = true;
      return result;
    }

    if (arg == "--service") {
      if (service_seen || i + 1 >= args.size()) {
        result.error_message = "Expected exactly one service name after --service";
        return result;
      }
      service_seen = true;
      result.command.service_name = args[++i];
      if (result.command.service_name.empty()) {
        result.error_message = "Service name cannot be empty";
        return result;
      }
      continue;
    }

    if (arg == "-t") {
      if (delay_seen || i + 1 >= args.size()) {
        result.error_message = "Expected exactly one numeric value after -t";
        return result;
      }
      char * parse_end = nullptr;
      const double parsed_value = std::strtod(args[++i].c_str(), &parse_end);
      if (parse_end == nullptr || *parse_end != '\0') {
        result.error_message = "Delay after -t must be numeric";
        return result;
      }
      if (parsed_value < 0.0) {
        result.error_message = "Delay must be non-negative";
        return result;
      }
      delay_seen = true;
      result.command.delay_seconds = parsed_value;
      continue;
    }

    if (arg == "-q") {
      result.command.quit_after_stop = true;
      continue;
    }

    if (!command_seen && (arg == "start" || arg == "stop")) {
      command_seen = true;
      result.command.action = arg == "start" ?
        srv::ControlExploration::Request::ACTION_START :
        srv::ControlExploration::Request::ACTION_STOP;
      continue;
    }

    result.error_message = "Unknown argument: " + arg;
    return result;
  }

  if (!command_seen) {
    result.error_message = "Expected one command: start or stop";
    return result;
  }

  if (
    result.command.quit_after_stop &&
    result.command.action != srv::ControlExploration::Request::ACTION_STOP)
  {
    result.error_message = "-q is only valid with stop";
    return result;
  }

  result.ok = true;
  return result;
}

inline std::string control_state_to_string(uint8_t state)
{
  switch (state) {
    case srv::ControlExploration::Request::STATE_RUNNING:
      return "running";
    case srv::ControlExploration::Request::STATE_START_SCHEDULED:
      return "start scheduled";
    case srv::ControlExploration::Request::STATE_STOP_SCHEDULED:
      return "stop scheduled";
    case srv::ControlExploration::Request::STATE_STOPPING:
      return "stopping";
    case srv::ControlExploration::Request::STATE_SHUTDOWN_PENDING:
      return "shutdown pending";
    case srv::ControlExploration::Request::STATE_IDLE:
    default:
      return "idle";
  }
}

}  // namespace frontier_exploration_ros2
