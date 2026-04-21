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

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<frontier_exploration_ros2::FrontierExplorerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->quitRequested()) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
