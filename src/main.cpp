#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<frontier_exploration_ros2::FrontierExplorerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
