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
#include <memory>
#include <string>
#include <thread>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include "frontier_exploration_ros2_rviz/exploration_panel_backend.hpp"

namespace frontier_exploration_ros2_rviz
{
namespace
{

class FakeFrontierControlServer : public rclcpp::Node
{
public:
  explicit FakeFrontierControlServer(const std::string & namespace_value = "")
  : Node("frontier_explorer", namespace_value)
  {
    service_ = create_service<ControlExploration>(
      "control_exploration",
      [this](
        const std::shared_ptr<ControlExploration::Request> request,
        std::shared_ptr<ControlExploration::Response> response)
      {
        last_request_ = *request;
        response->accepted = true;
        response->scheduled = request->delay_seconds > 0.0F;
        response->state = request->action == ControlExploration::Request::ACTION_STOP ?
          (response->scheduled ? ControlExploration::Request::STATE_STOP_SCHEDULED :
          ControlExploration::Request::STATE_STOPPING) :
          (response->scheduled ? ControlExploration::Request::STATE_START_SCHEDULED :
          ControlExploration::Request::STATE_RUNNING);
        response->message = request->action == ControlExploration::Request::ACTION_STOP ?
          "Scheduled exploration stop" :
          "Exploration started";
      });

    rosout_pub_ = create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
  }

  void publish_log(uint8_t level, const std::string & message)
  {
    rcl_interfaces::msg::Log log;
    log.level = level;
    log.name = "frontier_explorer";
    log.msg = message;
    rosout_pub_->publish(log);
  }

  ControlExploration::Request last_request() const
  {
    return last_request_;
  }

private:
  rclcpp::Service<ControlExploration>::SharedPtr service_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr rosout_pub_;
  ControlExploration::Request last_request_{};
};

class ExplorationPanelBackendTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }

    backend_node_ = std::make_shared<rclcpp::Node>("exploration_panel_backend_test");
    server_node_ = std::make_shared<FakeFrontierControlServer>();
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(backend_node_);
    executor_->add_node(server_node_);
    backend_ = std::make_unique<ExplorationPanelBackend>(backend_node_);
  }

  void TearDown() override
  {
    backend_.reset();
    if (executor_) {
      if (server_node_) {
        executor_->remove_node(server_node_);
      }
      if (backend_node_) {
        executor_->remove_node(backend_node_);
      }
    }
    server_node_.reset();
    backend_node_.reset();
    executor_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void pump_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      backend_->poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    backend_->poll();
  }

  std::shared_ptr<rclcpp::Node> backend_node_;
  std::shared_ptr<FakeFrontierControlServer> server_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<ExplorationPanelBackend> backend_;
};

TEST_F(ExplorationPanelBackendTests, SendsDelayedStopWithQuitFlag)
{
  pump_for(std::chrono::milliseconds(150));
  backend_->set_delay_seconds(4.0);
  backend_->set_quit_after_stop(true);

  ASSERT_TRUE(backend_->send_stop());
  pump_for(std::chrono::milliseconds(250));

  const auto request = server_node_->last_request();
  EXPECT_EQ(request.action, ControlExploration::Request::ACTION_STOP);
  EXPECT_FLOAT_EQ(request.delay_seconds, 4.0F);
  EXPECT_TRUE(request.quit_after_stop);

  const PanelSnapshot snapshot = backend_->snapshot();
  EXPECT_EQ(snapshot.control_state, ControlExploration::Request::STATE_STOP_SCHEDULED);
  EXPECT_NE(snapshot.status_event_text.find("Scheduled stop in"), std::string::npos);
}

TEST_F(ExplorationPanelBackendTests, ConsumesRosoutEventsForStatusBadge)
{
  pump_for(std::chrono::milliseconds(150));

  server_node_->publish_log(rcl_interfaces::msg::Log::INFO, "Frontier goal reached");
  pump_for(std::chrono::milliseconds(100));

  const PanelSnapshot snapshot = backend_->snapshot();
  EXPECT_EQ(snapshot.status_color, PanelEventColor::GREEN);
  EXPECT_NE(snapshot.status_event_text.find("Frontier goal reached"), std::string::npos);
}

TEST_F(ExplorationPanelBackendTests, KeepsPreemptionReasonVisibleOverFollowupCancelLog)
{
  pump_for(std::chrono::milliseconds(150));

  server_node_->publish_log(
    rcl_interfaces::msg::Log::INFO,
    "Preempting active frontier goal: active frontier revealed and visible reveal gain exhausted at target pose");
  pump_for(std::chrono::milliseconds(80));

  server_node_->publish_log(
    rcl_interfaces::msg::Log::DEBUG,
    "Canceling active frontier goal before applying the updated frontier selection");
  pump_for(std::chrono::milliseconds(80));

  const PanelSnapshot snapshot = backend_->snapshot();
  EXPECT_EQ(snapshot.status_color, PanelEventColor::ORANGE);
  EXPECT_NE(snapshot.status_event_text.find("Preempting active frontier goal"), std::string::npos);
}

}  // namespace
}  // namespace frontier_exploration_ros2_rviz
