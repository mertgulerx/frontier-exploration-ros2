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

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "frontier_exploration_ros2/srv/control_exploration.hpp"
#include "frontier_exploration_ctl_detail.hpp"

namespace frontier_exploration_ros2
{
namespace
{

bool wait_for_control_service(
  const rclcpp::Client<frontier_exploration_ros2::srv::ControlExploration>::SharedPtr & client,
  const std::string & service_name)
{
  using namespace std::chrono_literals;

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  bool waiting_message_printed = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (client->wait_for_service(500ms)) {
      return true;
    }
    if (!waiting_message_printed) {
      std::cerr
        << "Waiting for control service discovery on '" << service_name << "'...\n";
      waiting_message_printed = true;
    }
  }
  return false;
}

}  // namespace

void print_usage()
{
  std::cout
    << "Usage:\n"
    << "  frontier_exploration_ctl start [-t seconds] [--service service_name]\n"
    << "  frontier_exploration_ctl stop [-t seconds] [-q] [--service service_name]\n";
}
}  // namespace frontier_exploration_ros2

int main(int argc, char ** argv)
{
  const std::vector<std::string> args(argv + 1, argv + argc);
  const auto parsed = frontier_exploration_ros2::parse_control_command_args(args);
  if (!parsed.ok) {
    std::cerr << parsed.error_message << "\n";
    frontier_exploration_ros2::print_usage();
    return 1;
  }
  if (parsed.show_help) {
    frontier_exploration_ros2::print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("frontier_exploration_ctl");
  auto client = node->create_client<frontier_exploration_ros2::srv::ControlExploration>(
    parsed.command.service_name);

  if (!frontier_exploration_ros2::wait_for_control_service(client, parsed.command.service_name)) {
    std::cerr
      << "Service '" << parsed.command.service_name
      << "' is not available after waiting for discovery.\n";
    rclcpp::shutdown();
    return 1;
  }

  auto request =
    std::make_shared<frontier_exploration_ros2::srv::ControlExploration::Request>();
  request->action = parsed.command.action;
  request->delay_seconds = static_cast<float>(parsed.command.delay_seconds);
  request->quit_after_stop = parsed.command.quit_after_stop;

  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "Timed out while waiting for control service response.\n";
    rclcpp::shutdown();
    return 1;
  }

  const auto response = future.get();
  std::cout
    << response->message
    << " [state=" << frontier_exploration_ros2::control_state_to_string(response->state)
    << ", accepted=" << (response->accepted ? "true" : "false")
    << ", scheduled=" << (response->scheduled ? "true" : "false")
    << "]\n";

  rclcpp::shutdown();
  return response->accepted ? 0 : 1;
}
