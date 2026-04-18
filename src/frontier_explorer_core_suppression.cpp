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

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frontier_exploration_ros2
{

bool FrontierExplorerCore::suppression_enabled() const
{
  return params.frontier_suppression_enabled;
}

bool FrontierExplorerCore::suppression_runtime_active(int64_t now_ns) const
{
  return (
    suppression_enabled() &&
    (!frontier_suppression_activation_ns_.has_value() || now_ns >= *frontier_suppression_activation_ns_));
}

bool FrontierExplorerCore::should_return_to_start_when_all_frontiers_suppressed() const
{
  return params.all_frontiers_suppressed_behavior == "return_to_start";
}

FrontierSuppression * FrontierExplorerCore::ensure_frontier_suppression()
{
  if (!suppression_enabled()) {
    return nullptr;
  }
  if (!frontier_suppression_) {
    frontier_suppression_ = std::make_unique<FrontierSuppression>(FrontierSuppressionConfig{
      params.frontier_suppression_attempt_threshold,
      params.frontier_suppression_base_size_m,
      params.frontier_suppression_expansion_size_m,
      params.frontier_suppression_timeout_s,
      params.frontier_suppression_no_progress_timeout_s,
      params.frontier_suppression_progress_epsilon_m,
      params.frontier_suppression_max_attempt_records,
      params.frontier_suppression_max_regions,
      params.frontier_visit_tolerance,
    });
  }
  return frontier_suppression_.get();
}

FrontierSequence FrontierExplorerCore::filter_frontiers_for_suppression(
  const FrontierSequence & frontiers)
{
  const int64_t now_ns = callbacks.now_ns();
  if (!suppression_runtime_active(now_ns)) {
    return frontiers;
  }
  FrontierSuppression * suppression = ensure_frontier_suppression();
  if (!suppression) {
    return frontiers;
  }
  suppression->prune_expired(now_ns);
  return suppression->filter_frontiers(frontiers);
}

void FrontierExplorerCore::record_failed_frontier_attempt(
  const std::optional<FrontierLike> & frontier)
{
  const int64_t now_ns = callbacks.now_ns();
  if (!suppression_runtime_active(now_ns) || !frontier.has_value()) {
    return;
  }
  FrontierSuppression * suppression = ensure_frontier_suppression();
  if (suppression) {
    suppression->record_failed_attempt(
      *frontier,
      now_ns,
      callbacks.log_warn);
  }
}

void FrontierExplorerCore::clear_active_goal_progress_state()
{
  if (frontier_suppression_) {
    frontier_suppression_->clear_goal_progress_tracking();
  }
}

void FrontierExplorerCore::start_active_goal_progress_tracking()
{
  const int64_t now_ns = callbacks.now_ns();
  FrontierSuppression * suppression = ensure_frontier_suppression();
  if (suppression_runtime_active(now_ns) && suppression && active_goal_kind == "frontier") {
    suppression->start_goal_progress_tracking(current_dispatch_id, now_ns);
    return;
  }
  clear_active_goal_progress_state();
}

void FrontierExplorerCore::note_active_goal_progress(double distance_remaining)
{
  if (frontier_suppression_) {
    frontier_suppression_->note_goal_progress(
      current_dispatch_id,
      distance_remaining,
      callbacks.now_ns());
  }
}

bool FrontierExplorerCore::evaluate_active_goal_progress_timeout()
{
  const int64_t now_ns = callbacks.now_ns();
  if (!suppression_runtime_active(now_ns)) {
    return false;
  }
  FrontierSuppression * suppression = ensure_frontier_suppression();
  if (!suppression ||
    active_goal_kind != "frontier" ||
    goal_state != GoalLifecycleState::ACTIVE ||
    !goal_handle ||
    cancel_request_in_progress)
  {
    return false;
  }
  suppression->prune_expired(now_ns);
  if (!suppression->is_tracking_dispatch(current_dispatch_id)) {
    suppression->start_goal_progress_tracking(current_dispatch_id, now_ns);
    return false;
  }
  if (!suppression->mark_timeout_cancel_if_needed(current_dispatch_id, now_ns)) {
    return false;
  }
  request_active_goal_cancel(
    "Canceling frontier goal after no meaningful progress was observed within suppression timeout.");
  return true;
}

void FrontierExplorerCore::handle_all_frontiers_suppressed(
  const geometry_msgs::msg::Pose & current_pose)
{
  if (
    !should_return_to_start_when_all_frontiers_suppressed() ||
    !start_pose.has_value() ||
    suppressed_return_to_start_started)
  {
    return;
  }

  if (is_pose_within_xy_tolerance(current_pose, start_pose->pose)) {
    callbacks.log_info("All frontiers remain suppressed; staying at the start pose while waiting");
    suppressed_return_to_start_started = true;
    return;
  }

  suppressed_return_to_start_started = true;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "All frontiers are temporarily suppressed, returning to start pose while waiting: ("
      << start_pose->pose.position.x << ", "
      << start_pose->pose.position.y << ")";

  if (!send_pose_goal(
      *start_pose,
      "suppressed_return_to_start",
      std::nullopt,
      {},
      oss.str()))
  {
    suppressed_return_to_start_started = false;
  }
}

void FrontierExplorerCore::consider_cancel_suppressed_return_to_start()
{
  if (
    !goal_in_progress ||
    active_goal_kind != "suppressed_return_to_start" ||
    !map.has_value() ||
    !costmap.has_value())
  {
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    return;
  }

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_candidate_min_goal_distance_m);
  } catch (const std::out_of_range &) {
    return;
  }

  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(snapshot.frontiers);
  if (filtered_frontiers.empty()) {
    publish_frontier_markers(filtered_frontiers);
    return;
  }

  const auto selection = select_frontier(filtered_frontiers, *current_pose);
  publish_frontier_markers(filtered_frontiers);
  if (!selection.frontier.has_value()) {
    return;
  }

  const auto frontier_sequence = select_frontier_sequence(
    filtered_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    return;
  }

  pending_frontier_sequence = frontier_sequence;
  pending_frontier_selection_mode = selection.mode;
  pending_frontier_dispatch_context = "suppression_cleared";
  suppressed_return_to_start_started = false;
  request_active_goal_cancel(
    "Frontiers are available again; canceling temporary return-to-start goal");
}

}  // namespace frontier_exploration_ros2
