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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include "frontier_exploration_ros2_rviz/exploration_panel_logic.hpp"

namespace frontier_exploration_ros2_rviz
{

struct PanelSnapshot
{
  std::vector<std::string> discovered_services;
  std::string selected_service;
  std::string node_override;
  std::string service_text{"No control service discovered"};
  uint8_t control_state{ControlExploration::Request::STATE_IDLE};
  std::string control_state_text{"idle"};
  std::string last_command_text{"No command sent"};
  std::string status_event_text{"Waiting for frontier explorer events"};
  PanelEventColor status_color{PanelEventColor::GRAY};
  std::string countdown_text;
  bool service_ready{false};
};

class ExplorationPanelBackend
{
public:
  explicit ExplorationPanelBackend(const rclcpp::Node::SharedPtr & node);

  void set_delay_seconds(double delay_seconds);
  void set_quit_after_stop(bool quit_after_stop);
  void set_selected_service(const std::string & service_name);
  void set_logger_or_node_override(const std::string & override_value);

  [[nodiscard]] double delay_seconds() const;
  [[nodiscard]] bool quit_after_stop() const;
  [[nodiscard]] std::string selected_service() const;
  [[nodiscard]] std::string logger_or_node_override() const;

  void poll();
  bool send_start();
  bool send_stop();

  [[nodiscard]] PanelSnapshot snapshot() const;

private:
  using ControlClient = rclcpp::Client<ControlExploration>;

  struct PendingRequest
  {
    BuiltControlRequest request;
    ControlClient::SharedFuture future;
  };

  struct LastResponse
  {
    bool accepted{false};
    bool scheduled{false};
    uint8_t state{ControlExploration::Request::STATE_IDLE};
    std::string message;
  };

  void refresh_discovered_services();
  void refresh_client();
  void process_pending_response();
  void update_observed_runtime_state();
  void handle_rosout_message(const rcl_interfaces::msg::Log::SharedPtr msg);
  bool send_command(uint8_t action);
  void apply_response(const BuiltControlRequest & request, const ControlExploration::Response & response);
  void set_status_event(const ClassifiedLogEvent & event);
  void set_error_status(const std::string & message);

  [[nodiscard]] bool matches_selected_logger(const std::string & logger_name) const;
  [[nodiscard]] std::pair<std::string, std::string> target_node_identity() const;
  [[nodiscard]] static std::string service_namespace(const std::string & service_name);
  [[nodiscard]] static std::string trim(const std::string & text);
  [[nodiscard]] static std::string normalize_name(const std::string & text);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription_;
  ControlClient::SharedPtr client_;

  double delay_seconds_{0.0};
  bool quit_after_stop_{false};
  std::string selected_service_;
  std::string logger_or_node_override_;
  std::vector<std::string> discovered_services_;
  std::optional<PendingRequest> pending_request_;
  std::optional<LastResponse> last_response_;
  std::optional<ClassifiedLogEvent> last_event_;
  CommandCountdownTracker countdown_tracker_;
  uint8_t observed_runtime_state_{ControlExploration::Request::STATE_IDLE};
  bool observed_runtime_state_valid_{false};
};

}  // namespace frontier_exploration_ros2_rviz
