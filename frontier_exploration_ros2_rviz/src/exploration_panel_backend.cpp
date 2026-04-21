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

#include "frontier_exploration_ros2_rviz/exploration_panel_backend.hpp"

#include <algorithm>
#include <exception>
#include <utility>

namespace frontier_exploration_ros2_rviz
{

ExplorationPanelBackend::ExplorationPanelBackend(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  rosout_subscription_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout",
    rclcpp::QoS(50),
    [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
      this->handle_rosout_message(msg);
    });
}

void ExplorationPanelBackend::set_delay_seconds(double delay_seconds)
{
  delay_seconds_ = delay_seconds;
}

void ExplorationPanelBackend::set_quit_after_stop(bool quit_after_stop)
{
  quit_after_stop_ = quit_after_stop;
}

void ExplorationPanelBackend::set_selected_service(const std::string & service_name)
{
  if (selected_service_ == service_name) {
    return;
  }

  selected_service_ = service_name;
  client_.reset();
}

void ExplorationPanelBackend::set_logger_or_node_override(const std::string & override_value)
{
  logger_or_node_override_ = trim(override_value);
}

double ExplorationPanelBackend::delay_seconds() const
{
  return delay_seconds_;
}

bool ExplorationPanelBackend::quit_after_stop() const
{
  return quit_after_stop_;
}

std::string ExplorationPanelBackend::selected_service() const
{
  return selected_service_;
}

std::string ExplorationPanelBackend::logger_or_node_override() const
{
  return logger_or_node_override_;
}

void ExplorationPanelBackend::poll()
{
  refresh_discovered_services();
  refresh_client();
  process_pending_response();
  update_observed_runtime_state();
}

bool ExplorationPanelBackend::send_start()
{
  return send_command(ControlExploration::Request::ACTION_START);
}

bool ExplorationPanelBackend::send_stop()
{
  return send_command(ControlExploration::Request::ACTION_STOP);
}

PanelSnapshot ExplorationPanelBackend::snapshot() const
{
  PanelSnapshot snapshot;
  snapshot.discovered_services = discovered_services_;
  snapshot.selected_service = selected_service_;
  snapshot.node_override = logger_or_node_override_;
  snapshot.service_ready = !selected_service_.empty();
  snapshot.service_text = selected_service_.empty() ? "No control service discovered" : selected_service_;

  uint8_t resolved_state = ControlExploration::Request::STATE_IDLE;
  if (last_response_.has_value()) {
    resolved_state = last_response_->state;
  } else if (observed_runtime_state_valid_) {
    resolved_state = observed_runtime_state_;
  }
  resolved_state = countdown_tracker_.shadow_state(resolved_state);
  snapshot.control_state = resolved_state;
  snapshot.control_state_text = control_state_to_string(resolved_state);

  if (last_response_.has_value()) {
    snapshot.last_command_text = last_response_->message;
  }
  if (pending_request_.has_value()) {
    snapshot.last_command_text = "Waiting for '" + pending_request_->request.command_text + "' response";
  }
  if (last_response_.has_value()) {
    snapshot.last_command_text +=
      " [accepted=" + std::string(last_response_->accepted ? "true" : "false") +
      ", scheduled=" + std::string(last_response_->scheduled ? "true" : "false") + "]";
  }

  snapshot.countdown_text = countdown_tracker_.status_text();

  if (!snapshot.countdown_text.empty()) {
    snapshot.status_event_text = snapshot.countdown_text;
    snapshot.status_color = PanelEventColor::GRAY;
  } else if (last_event_.has_value()) {
    snapshot.status_event_text = last_event_->detail;
    snapshot.status_color = last_event_->color;
  } else if (last_response_.has_value()) {
    snapshot.status_event_text = last_response_->message;
    snapshot.status_color = last_response_->accepted ? PanelEventColor::GRAY : PanelEventColor::RED;
  }

  return snapshot;
}

void ExplorationPanelBackend::refresh_discovered_services()
{
  std::vector<std::string> services;
  for (const auto & entry : node_->get_service_names_and_types()) {
    const auto & types = entry.second;
    if (
      std::find(
        types.begin(),
        types.end(),
        "frontier_exploration_ros2/srv/ControlExploration") != types.end())
    {
      services.push_back(entry.first);
    }
  }

  std::sort(services.begin(), services.end());
  services.erase(std::unique(services.begin(), services.end()), services.end());
  discovered_services_ = std::move(services);

  if (discovered_services_.empty()) {
    selected_service_.clear();
    client_.reset();
    return;
  }

  if (
    selected_service_.empty() ||
    std::find(discovered_services_.begin(), discovered_services_.end(), selected_service_) ==
    discovered_services_.end())
  {
    selected_service_ = discovered_services_.front();
    client_.reset();
  }
}

void ExplorationPanelBackend::refresh_client()
{
  if (selected_service_.empty()) {
    client_.reset();
    return;
  }

  if (!client_ || client_->get_service_name() != selected_service_) {
    client_ = node_->create_client<ControlExploration>(selected_service_);
  }
}

void ExplorationPanelBackend::process_pending_response()
{
  if (!pending_request_.has_value()) {
    return;
  }

  auto & pending = *pending_request_;
  if (pending.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  try {
    const auto response = pending.future.get();
    if (!response) {
      set_error_status("Control service returned an empty response");
      pending_request_.reset();
      return;
    }
    apply_response(pending.request, *response);
  } catch (const std::exception & exc) {
    set_error_status(std::string("Control service request failed: ") + exc.what());
  }

  pending_request_.reset();
}

void ExplorationPanelBackend::update_observed_runtime_state()
{
  if (selected_service_.empty()) {
    observed_runtime_state_valid_ = false;
    return;
  }

  const auto [node_name, node_namespace] = target_node_identity();
  try {
    const auto subscriptions =
      node_->get_node_graph_interface()->get_subscriber_names_and_types_by_node(
      node_name,
      node_namespace,
      false);

    std::size_t occupancy_grid_subscriptions = 0;
    for (const auto & subscription : subscriptions) {
      const auto & types = subscription.second;
      if (
        std::find(types.begin(), types.end(), "nav_msgs/msg/OccupancyGrid") != types.end())
      {
        occupancy_grid_subscriptions += 1;
      }
    }

    observed_runtime_state_valid_ = true;
    observed_runtime_state_ = occupancy_grid_subscriptions >= 2 ?
      ControlExploration::Request::STATE_RUNNING :
      ControlExploration::Request::STATE_IDLE;
  } catch (const std::exception &) {
    observed_runtime_state_valid_ = false;
  }
}

void ExplorationPanelBackend::handle_rosout_message(const rcl_interfaces::msg::Log::SharedPtr msg)
{
  if (!msg || !matches_selected_logger(msg->name)) {
    return;
  }

  ClassifiedLogEvent event = classify_log_event(*msg);
  if (!event.matched && msg->level < rcl_interfaces::msg::Log::WARN) {
    return;
  }

  if (!event.matched) {
    event = {true, PanelEventColor::RED, "Warning", msg->msg};
  }
  set_status_event(event);
}

bool ExplorationPanelBackend::send_command(uint8_t action)
{
  refresh_discovered_services();
  refresh_client();

  if (!client_) {
    set_error_status("No frontier control service is selected");
    return false;
  }

  const bool quit_after_stop =
    action == ControlExploration::Request::ACTION_STOP ? quit_after_stop_ : false;
  const BuiltControlRequest built_request =
    build_control_request(action, delay_seconds_, quit_after_stop);
  if (!built_request.valid) {
    set_error_status(built_request.error_message);
    return false;
  }

  if (!client_->wait_for_service(std::chrono::milliseconds(150))) {
    set_error_status("Selected control service is not ready");
    return false;
  }

  auto request = std::make_shared<ControlExploration::Request>();
  request->action = built_request.action;
  request->delay_seconds = built_request.delay_seconds;
  request->quit_after_stop = built_request.quit_after_stop;

  auto future = client_->async_send_request(request);
  pending_request_ = PendingRequest{
    built_request,
    future.future.share(),
  };
  return true;
}

void ExplorationPanelBackend::apply_response(
  const BuiltControlRequest & request,
  const ControlExploration::Response & response)
{
  last_response_ = LastResponse{
    response.accepted,
    response.scheduled,
    response.state,
    response.message,
  };

  if (response.accepted && response.scheduled) {
    countdown_tracker_.arm(
      request.action,
      static_cast<double>(request.delay_seconds),
      request.quit_after_stop,
      request.command_text);
  } else if (!response.scheduled) {
    countdown_tracker_.clear();
  }

  ClassifiedLogEvent event;
  event.matched = true;
  event.color = response.accepted ? PanelEventColor::GRAY : PanelEventColor::RED;
  event.label = response.accepted ? "Control" : "Rejected";
  event.detail = response.message;
  set_status_event(event);
}

void ExplorationPanelBackend::set_status_event(const ClassifiedLogEvent & event)
{
  last_event_ = event;
}

void ExplorationPanelBackend::set_error_status(const std::string & message)
{
  set_status_event(ClassifiedLogEvent{true, PanelEventColor::RED, "Error", message});
}

bool ExplorationPanelBackend::matches_selected_logger(const std::string & logger_name) const
{
  if (selected_service_.empty()) {
    return false;
  }

  const std::string normalized_logger = normalize_name(logger_name);
  if (normalized_logger.empty()) {
    return false;
  }

  const auto [node_name, node_namespace] = target_node_identity();
  const std::string normalized_node_name = normalize_name(node_name);
  const std::string normalized_namespaced_node = normalize_name(node_namespace + "/" + node_name);

  return normalized_logger == normalized_node_name ||
         normalized_logger == normalized_namespaced_node ||
         (
    normalized_logger.size() > normalized_node_name.size() &&
    normalized_logger.compare(
      normalized_logger.size() - normalized_node_name.size(),
      normalized_node_name.size(),
      normalized_node_name) == 0);
}

std::pair<std::string, std::string> ExplorationPanelBackend::target_node_identity() const
{
  std::string node_namespace = service_namespace(selected_service_);
  std::string node_name = "frontier_explorer";

  if (logger_or_node_override_.empty()) {
    return {node_name, node_namespace};
  }

  const std::string trimmed_override = trim(logger_or_node_override_);
  const std::size_t slash = trimmed_override.find_last_of('/');
  if (slash != std::string::npos) {
    node_namespace = slash == 0 ? "/" : trimmed_override.substr(0, slash);
    node_name = trimmed_override.substr(slash + 1);
    if (node_name.empty()) {
      node_name = "frontier_explorer";
    }
    return {node_name, node_namespace};
  }

  return {trimmed_override, node_namespace};
}

std::string ExplorationPanelBackend::service_namespace(const std::string & service_name)
{
  if (service_name.empty() || service_name == "/control_exploration") {
    return "/";
  }

  const std::size_t slash = service_name.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return "/";
  }
  return service_name.substr(0, slash);
}

std::string ExplorationPanelBackend::trim(const std::string & text)
{
  const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();

  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

std::string ExplorationPanelBackend::normalize_name(const std::string & text)
{
  std::string normalized = trim(text);
  std::replace(normalized.begin(), normalized.end(), '/', '.');
  normalized.erase(
    normalized.begin(),
    std::find_if(normalized.begin(), normalized.end(), [](unsigned char c) {
      return c != '.' && !std::isspace(c);
    }));
  normalized.erase(
    std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }),
    normalized.end());
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return normalized;
}

}  // namespace frontier_exploration_ros2_rviz
