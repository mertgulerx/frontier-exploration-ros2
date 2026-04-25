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

#include "frontier_explorer_core_detail.hpp"

#include <action_msgs/msg/goal_status.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::try_send_next_goal()
{
  // Main scheduler entrypoint: exits early unless maps, pose, and lifecycle gates are ready.
  if (!exploration_enabled) {
    return;
  }

  if (return_to_start_completed) {
    // Exploration is fully finished.
    return;
  }

  if (goal_in_progress || !map.has_value() || !costmap.has_value()) {
    // Cannot dispatch a new goal while one is active, or before required maps arrive.
    return;
  }

  if (awaiting_map_refresh && !post_goal_settle_ready()) {
    // Settle window is still active after previous goal completion.
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    // Skip scheduling until TF provides a valid robot pose.
    return;
  }

  if (awaiting_map_refresh) {
    clear_post_goal_wait_state();
  }

  record_start_pose(*current_pose);

  if (!pending_frontier_sequence.empty()) {
    // Replacement/preemption path already computed a concrete next target.
    dispatch_pending_frontier_goal(current_pose);
    return;
  }

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(
      *current_pose,
      frontier_snapshot_min_goal_distance_for_pose(*current_pose));
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier update: ") + exc.what());
    return;
  }

  const FrontierSequence & frontiers = snapshot.frontiers;
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(frontiers);
  if (filtered_frontiers.empty() && !frontiers.empty()) {
    no_frontiers_reported = false;
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);
    if (!all_frontiers_suppressed_reported) {
      callbacks.log_info("All currently detected frontiers are temporarily suppressed");
      all_frontiers_suppressed_reported = true;
    }
    handle_all_frontiers_suppressed(*current_pose);
    return;
  }

  all_frontiers_suppressed_reported = false;
  suppressed_return_to_start_started = false;
  if (filtered_frontiers.empty()) {
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);

    if (!no_frontiers_reported) {
      // Log once per no-frontier streak to avoid repetitive output.
      callbacks.log_info("No more frontiers found");
      no_frontiers_reported = true;
    }

    handle_exploration_complete(*current_pose);
    return;
  }

  no_frontiers_reported = false;
  const auto selection = select_frontier(filtered_frontiers, *current_pose);
  publish_frontier_markers(filtered_frontiers);

  if (!selection.frontier.has_value()) {
    if (!no_reachable_frontier_reported) {
      callbacks.log_info(
        "No reachable frontier candidate is available right now; waiting for a map update");
      no_reachable_frontier_reported = true;
    }
    return;
  }

  no_reachable_frontier_reported = false;
  const auto frontier_sequence = select_frontier_sequence(
    filtered_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    return;
  }

  send_frontier_goal(
    frontier_sequence,
    *current_pose,
    "Sending frontier goal (" + selection.mode + "): " + describe_frontier(frontier_sequence.front()));
}

void FrontierExplorerCore::reset_exploration_runtime_state(bool clear_maps)
{
  clear_post_goal_wait_state();
  clear_active_goal_progress_state();

  no_frontiers_reported = false;
  no_reachable_frontier_reported = false;
  all_frontiers_suppressed_reported = false;
  escape_active = params.escape_enabled;
  return_to_start_started = false;
  return_to_start_completed = false;
  suppressed_return_to_start_started = false;
  cancel_request_in_progress = false;
  pending_cancel_reason.reset();
  pending_frontier_sequence.clear();
  pending_frontier_selection_mode.clear();
  pending_frontier_dispatch_context.clear();
  active_goal_blocked_reason.reset();
  distance_completed_frontier.reset();
  last_low_gain_reselection_time_ns.reset();
  reset_replacement_candidate_tracking();
  last_published_frontier_signature.reset();
  frontier_snapshot.reset();
  raw_frontier_debug_cache.reset();
  mrtsp_order_cache.reset();
  frontier_suppression_.reset();
  frontier_suppression_activation_ns_.reset();

  if (clear_maps) {
    map.reset();
    decision_map.reset();
    decision_map_msg.reset();
    decision_map_workspace = DecisionMapWorkspace{};
    costmap.reset();
    local_costmap.reset();
    map_generation = 0;
    decision_map_generation = 0;
    costmap_generation = 0;
    local_costmap_generation = 0;
    decision_map_cache_hits = 0;
    decision_map_cache_misses = 0;
    frontier_snapshot_cache_hits = 0;
    frontier_snapshot_cache_misses = 0;
    mrtsp_order_cache_hits = 0;
    mrtsp_order_cache_misses = 0;
  }
}

void FrontierExplorerCore::start_exploration_session()
{
  reset_exploration_runtime_state(true);
  exploration_enabled = true;
  if (params.frontier_suppression_enabled) {
    frontier_suppression_activation_ns_ =
      callbacks.now_ns() +
      static_cast<int64_t>(params.frontier_suppression_startup_grace_period_s * 1e9);
  }
}

void FrontierExplorerCore::stop_exploration_session(const std::string & reason)
{
  exploration_enabled = false;
  reset_exploration_runtime_state(true);
  if (goal_in_progress) {
    request_active_goal_cancel(reason);
  }
}

bool FrontierExplorerCore::ready_for_shutdown() const
{
  return !goal_in_progress && !cancel_request_in_progress;
}

void FrontierExplorerCore::set_goal_state(GoalLifecycleState state)
{
  // Keep enum state and convenience bool in sync from one write location.
  goal_state = state;
  // IDLE <-> non-IDLE mapping kept centralized to avoid inconsistent flags.
  goal_in_progress = state != GoalLifecycleState::IDLE;
}

void FrontierExplorerCore::mark_dispatch_state(int dispatch_id, GoalLifecycleState state)
{
  // Tracks lifecycle by dispatch id for stale callback filtering and diagnostics.
  dispatch_states[dispatch_id] = state;
}

void FrontierExplorerCore::reset_replacement_candidate_tracking()
{
  // Clear both candidate payload and debounce counter together to avoid stale "stable" hits.
  replacement_candidate_frontier.reset();
  replacement_candidate_hits = 0;
}

bool FrontierExplorerCore::has_stable_replacement_candidate(const FrontierSequence & frontier_sequence)
{
  if (frontier_sequence.empty()) {
    reset_replacement_candidate_tracking();
    return false;
  }

  const FrontierLike & selected_frontier = frontier_sequence.front();
  if (replacement_candidate_frontier.has_value() &&
    are_frontiers_equivalent(selected_frontier, replacement_candidate_frontier))
  {
    // Debounce only the selected replacement frontier. Later MRTSP order entries may change
    // without changing the actual replacement target, and Python reference behavior treats
    // that case as stable.
    replacement_candidate_hits += 1;
  } else {
    // New candidate observed; restart stability counter.
    replacement_candidate_frontier = selected_frontier;
    replacement_candidate_hits = 1;
  }

  // Candidate is considered stable only after configurable repeated hits.
  return replacement_candidate_hits >= replacement_required_hits;
}

std::optional<double> FrontierExplorerCore::active_goal_visible_reveal_length() const
{
  if (!map.has_value() || !costmap.has_value() || !active_goal_frontier.has_value()) {
    return std::nullopt;
  }

  // Model the sensor at the dispatched target pose, not at the robot's current pose.
  // This keeps the gain estimate aligned with "what would be revealed if we finish this goal?"
  const auto goal_point = frontier_position(*active_goal_frontier);
  const auto reference_point = frontier_reference_point(*active_goal_frontier);
  double heading = 0.0;
  const double dx = reference_point.first - goal_point.first;
  const double dy = reference_point.second - goal_point.second;
  if (std::hypot(dx, dy) > 1e-6) {
    // Preferred heading points from the dispatched goal toward the frontier reference geometry.
    heading = std::atan2(dy, dx);
  } else if (active_goal_pose.has_value()) {
    // Degenerate frontier geometry falls back to the yaw of the already-dispatched goal pose.
    heading = detail::yaw_from_quaternion(active_goal_pose->pose.orientation);
  }

  const double yaw_offset_rad = params.goal_preemption_lidar_yaw_offset_deg * (detail::kPi / 180.0);
  geometry_msgs::msg::Pose sensor_pose;
  sensor_pose.position.x = goal_point.first;
  sensor_pose.position.y = goal_point.second;
  sensor_pose.orientation = detail::quaternion_from_yaw(heading + yaw_offset_rad);

  // The helper returns a local, occlusion-aware reveal length estimate around the target pose.
  const auto visible_gain = compute_visible_reveal_gain(
    sensor_pose,
    *map,
    *costmap,
    local_costmap,
    params.goal_preemption_lidar_range_m,
    params.goal_preemption_lidar_fov_deg,
    params.goal_preemption_lidar_ray_step_deg);
  if (!visible_gain.has_value()) {
    return std::nullopt;
  }

  return visible_gain->visible_reveal_length_m;
}

double FrontierExplorerCore::frontier_snapshot_min_goal_distance_for_pose(
  const geometry_msgs::msg::Pose & current_pose)
{
  if (!distance_completed_frontier.has_value() ||
    params.goal_preemption_complete_if_within_m <= 0.0)
  {
    return params.frontier_candidate_min_goal_distance_m;
  }

  const auto completed_reference = frontier_reference_point(*distance_completed_frontier);
  const double completed_distance = std::hypot(
    completed_reference.first - current_pose.position.x,
    completed_reference.second - current_pose.position.y);
  if (completed_distance > params.goal_preemption_complete_if_within_m) {
    distance_completed_frontier.reset();
    return params.frontier_candidate_min_goal_distance_m;
  }

  return std::max(
    params.frontier_candidate_min_goal_distance_m,
    params.goal_preemption_complete_if_within_m);
}

void FrontierExplorerCore::consider_preempt_active_goal(const std::string & trigger_source)
{
  // Costmap-triggered calls only update blocked reason; map-triggered calls may reseat goals.
  const bool completion_distance_enabled = params.goal_preemption_complete_if_within_m > 0.0;
  const bool preemption_allowed = (
    params.goal_preemption_enabled ||
    params.goal_skip_on_blocked_goal ||
    completion_distance_enabled);
  if (!preemption_allowed) {
    return;
  }

  if (!map.has_value() || !costmap.has_value() || !goal_in_progress || !active_goal_frontier.has_value()) {
    // Preemption logic requires an active frontier goal plus current map context.
    return;
  }

  if (!goal_handle) {
    // Goal has not been accepted yet, so cancel/reselect actions cannot be issued.
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    // Pose uncertainty: defer preemption decision.
    return;
  }

  const auto active_goal_cost_status = params.goal_skip_on_blocked_goal ?
    frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};
  if (
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled &&
    !completion_distance_enabled)
  {
    // Nothing to do when all active-goal update triggers are effectively inactive.
    return;
  }

  if (trigger_source != "map") {
    // Costmap/local-costmap callbacks intentionally skip full frontier reselection work.
    if (active_goal_cost_status.has_value()) {
      active_goal_blocked_reason = *active_goal_cost_status;
    }
    return;
  }

  if (cancel_request_in_progress) {
    // Avoid duplicate cancel requests while previous cancel handshake is in flight.
    return;
  }

  if (!active_goal_cost_status.has_value()) {
    active_goal_blocked_reason.reset();
  }

  const auto active_goal_reference = frontier_reference_point(*active_goal_frontier);
  const double active_goal_distance = std::hypot(
    active_goal_reference.first - current_pose->position.x,
    active_goal_reference.second - current_pose->position.y);
  std::optional<double> visible_reveal_length;
  // Near-goal completion is independent from visible-gain preemption; it is a close-enough guard.
  const bool revealed_completion_distance_reached =
    !active_goal_cost_status.has_value() &&
    completion_distance_enabled &&
    active_goal_distance <= params.goal_preemption_complete_if_within_m;
  const std::string completion_distance_reason =
    "active frontier considered complete by goal_preemption_complete_if_within_m (distance=" +
    detail::format_meters(active_goal_distance) + ", threshold=" +
    detail::format_meters(params.goal_preemption_complete_if_within_m) + ")";

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled)
  {
    return;
  }

  if (revealed_completion_distance_reached) {
    distance_completed_frontier = active_goal_frontier;
    request_active_goal_cancel(completion_distance_reason);
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_sent_time_ns.has_value() &&
    !active_goal_cost_status.has_value())
  {
    // Without send timestamp we cannot evaluate time-based visible-gain preemption gate.
    return;
  }

  const int64_t now_ns = callbacks.now_ns();
  const double elapsed = active_goal_sent_time_ns.has_value() ?
    static_cast<double>(now_ns - *active_goal_sent_time_ns) / 1e9 : 0.0;

  // Time-based gate for visible-gain preemption to avoid immediate churn.
  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    elapsed < params.goal_preemption_min_interval_s)
  {
    return;
  }

  // Visible-gain gate is only meaningful on the map-triggered revealed-preemption path.
  const bool visible_gain_gate_active =
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled;
  bool visible_reveal_gain_exhausted = false;

  if (visible_gain_gate_active && !revealed_completion_distance_reached) {
    visible_reveal_length = active_goal_visible_reveal_length();
    if (!visible_reveal_length.has_value()) {
      callbacks.log_warn(
        "Skipping visible reveal gain gate for the active goal; falling back to frontier snapshot reselection");
    } else if (*visible_reveal_length >= params.goal_preemption_lidar_min_reveal_length_m) {
      // Keep current goal while target pose can still reveal enough unexplored boundary on arrival.
      reset_replacement_candidate_tracking();
      return;
    } else {
      // Low visible gain alone is not enough to preempt; the refreshed frontier snapshot still decides.
      visible_reveal_gain_exhausted = true;
    }
  }

  if (
    visible_reveal_gain_exhausted &&
    last_low_gain_reselection_time_ns.has_value() &&
    params.goal_preemption_min_interval_s > 0.0)
  {
    const double elapsed_since_low_gain_reselection =
      static_cast<double>(now_ns - *last_low_gain_reselection_time_ns) / 1e9;
    if (elapsed_since_low_gain_reselection < params.goal_preemption_min_interval_s) {
      return;
    }
  }
  if (visible_reveal_gain_exhausted) {
    last_low_gain_reselection_time_ns = now_ns;
  }

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_candidate_min_goal_distance_m);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier reselection: ") + exc.what());
    return;
  }

  const FrontierSequence & frontiers = snapshot.frontiers;
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(frontiers);
  if (filtered_frontiers.empty() && !frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    callbacks.log_info("All replacement frontiers are temporarily suppressed");
    if (active_goal_cost_status.has_value()) {
      // If current goal is blocked and no alternative frontier exists, cancel proactively.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no unsuppressed replacement frontier is available");
    }
    return;
  }

  if (filtered_frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    if (active_goal_cost_status.has_value()) {
      // Blocked-goal path cancels proactively when no replacement target exists at all.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no replacement frontier is available");
    }
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    frontier_exists_in_set(active_goal_frontier, filtered_frontiers))
  {
    // Snapshot membership remains the final authority for revealed-preemption.
    // If the frontier still exists in the refreshed set, keep the active goal.
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    return;
  }

  const auto selection = select_frontier(filtered_frontiers, *current_pose);
  publish_frontier_markers(filtered_frontiers);
  const auto frontier_sequence = select_frontier_sequence(
    filtered_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    // Selection produced no dispatchable target.
    reset_replacement_candidate_tracking();
    return;
  }

  if (are_frontier_sequences_equivalent(frontier_sequence, active_goal_frontiers)) {
    // Replacement candidate is effectively same as active target.
    reset_replacement_candidate_tracking();
    return;
  }

  // Materialize a single human-readable reason so logs and cancel/reselection flow stay aligned.
  const std::string revealed_preemption_reason = active_goal_cost_status.value_or(
    visible_reveal_gain_exhausted && visible_reveal_length.has_value() ?
    "active frontier revealed and visible reveal gain exhausted at target pose (visible=" +
    detail::format_meters(*visible_reveal_length) + ", required=" +
    detail::format_meters(params.goal_preemption_lidar_min_reveal_length_m) +
    "); preempting to the next frontier" :
    "active frontier no longer appears in the latest frontier snapshot after reveal updates; preempting to the next frontier");

  const std::string goal_update_log_prefix = active_goal_cost_status.has_value() ?
    "Skipping blocked frontier goal" :
    "Preempting active frontier goal";

  request_frontier_reselection(
    frontier_sequence,
    *current_pose,
    selection.mode,
    revealed_preemption_reason,
    goal_update_log_prefix);
}

void FrontierExplorerCore::request_frontier_reselection(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & selection_mode,
  const std::string & reselection_reason,
  const std::string & goal_update_log_prefix)
{
  if (!goal_handle || frontier_sequence.empty()) {
    // Reselection is meaningful only with an active accepted goal and a non-empty replacement.
    return;
  }

  if (!pending_frontier_sequence.empty() &&
    are_frontier_sequences_equivalent(frontier_sequence, pending_frontier_sequence))
  {
    // Keep queue compact by suppressing duplicate pending replacement requests.
    return;
  }

  if (!has_stable_replacement_candidate(frontier_sequence)) {
    // Require repeated observation before replacing active target.
    return;
  }

  pending_frontier_sequence = frontier_sequence;
  pending_frontier_selection_mode = selection_mode;
  pending_frontier_dispatch_context = "reselected";
  callbacks.log_info(
    goal_update_log_prefix + ": " + reselection_reason);
  // Use Nav2's native action preemption path by sending the replacement goal directly.
  // Post-goal settle remains reserved for terminal result paths, not live replacements.
  dispatch_pending_frontier_goal(current_pose);
}

void FrontierExplorerCore::request_active_goal_cancel(const std::string & reason)
{
  // Keep earliest cancel reason; subsequent requests should not erase context.
  if (!pending_cancel_reason.has_value()) {
    pending_cancel_reason = reason;
  }

  if (!goal_handle || cancel_request_in_progress) {
    // Cancel will be attempted later when handle exists and no request is in progress.
    return;
  }

  issue_active_goal_cancel();
}

void FrontierExplorerCore::issue_active_goal_cancel()
{
  if (!goal_handle || cancel_request_in_progress) {
    return;
  }

  const std::string reason = pending_cancel_reason.value_or("Canceling active goal");
  pending_cancel_reason.reset();
  cancel_request_in_progress = true;
  set_goal_state(GoalLifecycleState::CANCELING);
  callbacks.log_debug(reason);

  const int dispatch_id = current_dispatch_id;
  // Bind cancel response to current dispatch to ignore late/stale acknowledgements.
  goal_handle->cancel_goal_async(
    [this, dispatch_id](bool accepted, const std::string & error_message) {
      cancel_response_callback(dispatch_id, accepted, error_message);
    });
}

void FrontierExplorerCore::cancel_response_callback(
  int dispatch_id,
  bool cancel_accepted,
  const std::string & error_message)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore cancel responses from superseded goals.
    return;
  }

  if (!error_message.empty()) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Failed to cancel active goal: " + error_message);
    return;
  }

  if (!cancel_accepted) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Active goal cancel request was not accepted");
  }
}

bool FrontierExplorerCore::dispatch_pending_frontier_goal(
  const std::optional<geometry_msgs::msg::Pose> & current_pose)
{
  if (pending_frontier_sequence.empty()) {
    // Nothing queued by preemption/reselection logic.
    return false;
  }

  std::optional<geometry_msgs::msg::Pose> pose = current_pose;
  if (!pose.has_value()) {
    // Keep sequence pending if TF is transiently unavailable; caller retries later.
    pose = callbacks.get_current_pose();
    if (!pose.has_value()) {
      return true;
    }
  }

  const FrontierSequence frontier_sequence = pending_frontier_sequence;
  const std::string selection_mode = pending_frontier_selection_mode.empty() ?
    "preferred" : pending_frontier_selection_mode;

  const bool dispatched = send_frontier_goal(
    frontier_sequence,
    *pose,
    "Sending updated frontier goal (" + selection_mode + "): " +
    describe_frontier(frontier_sequence.front()));

  if (dispatched) {
    // Consume pending sequence only after successful dispatch to preserve at-least-once intent.
    pending_frontier_sequence.clear();
    pending_frontier_selection_mode.clear();
    pending_frontier_dispatch_context.clear();
    active_goal_blocked_reason.reset();
    reset_replacement_candidate_tracking();
    clear_post_goal_wait_state();
  }

  return true;
}

void FrontierExplorerCore::handle_exploration_complete(const geometry_msgs::msg::Pose & current_pose)
{
  callbacks.on_exploration_complete();

  if (return_to_start_completed) {
    return;
  }

  if (!params.return_to_start_on_complete || !start_pose.has_value())
  {
    // Mark completion when return-to-start is disabled (or unavailable) to stop repeated no-frontier scans.
    return_to_start_completed = true;
    return;
  }

  if (is_pose_within_xy_tolerance(current_pose, start_pose->pose)) {
    // Exploration already ended near start; avoid issuing a redundant return goal.
    return_to_start_completed = true;
    callbacks.log_info("Exploration finished at the start pose");
    return;
  }

  if (!return_to_start_started) {
    // Latch this flag before dispatch to prevent duplicate return goals on repeated callbacks.
    return_to_start_started = true;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Exploration finished, returning to start pose: ("
        << start_pose->pose.position.x << ", "
        << start_pose->pose.position.y << ")";

    send_pose_goal(
      *start_pose,
      "return_to_start",
      std::nullopt,
      {},
      oss.str());
  }
}

bool FrontierExplorerCore::is_pose_within_xy_tolerance(
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double tolerance) const
{
  // XY-only proximity check intentionally ignores yaw for exploration completion decisions.
  return std::hypot(
    goal_pose.position.x - current_pose.position.x,
    goal_pose.position.y - current_pose.position.y) <= tolerance;
}

bool FrontierExplorerCore::send_pose_goal(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  if (!callbacks.wait_for_action_server(1.0)) {
    // Keep core non-blocking: schedule retry on future updates instead of waiting indefinitely.
    callbacks.log_info(
      "Action '" + params.navigate_to_pose_action_name + "' not ready yet, waiting...");
    if (goal_kind == "return_to_start") {
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    return false;
  }

  dispatch_goal_request(
    params.navigate_to_pose_action_name,
    goal_pose,
    goal_kind,
    frontier,
    frontier_sequence,
    description);
  return true;
}

bool FrontierExplorerCore::send_frontier_goal(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & description)
{
  if (frontier_sequence.empty()) {
    return false;
  }

  // Only the first frontier is dispatched; the second frontier, when present, shapes arrival yaw.
  const std::optional<FrontierLike> look_ahead_frontier =
    frontier_sequence.size() > 1 ?
    std::optional<FrontierLike>(frontier_sequence[1]) :
    std::nullopt;
  const auto goal_pose = build_goal_pose(
    frontier_sequence.front(),
    current_pose,
    look_ahead_frontier);
  if (debug_outputs_enabled()) {
    callbacks.publish_selected_frontier_pose(goal_pose);
  }
  // Frontier mode dispatches only the first element from the selected sequence.
  return send_pose_goal(
    goal_pose,
    "frontier",
    frontier_sequence.front(),
    frontier_sequence,
    description);
}

void FrontierExplorerCore::dispatch_goal_request(
  const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  // Dispatch ids make out-of-order callbacks safe when old and new goals overlap.
  if (goal_in_progress && dispatch_states.find(current_dispatch_id) != dispatch_states.end()) {
    mark_dispatch_state(current_dispatch_id, GoalLifecycleState::SUPERSEDED);
  }

  current_dispatch_id += 1;
  const int dispatch_id = current_dispatch_id;

  active_goal_kind = goal_kind;
  active_goal_pose = goal_pose;
  active_goal_frontier = frontier;
  active_goal_frontiers = frontier_sequence;
  active_action_name = action_name;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::SENDING);
  set_goal_state(GoalLifecycleState::SENDING);
  goal_handle.reset();
  // Send timestamp anchors min-interval preemption gating.
  active_goal_sent_time_ns = callbacks.now_ns();
  last_low_gain_reselection_time_ns.reset();
  awaiting_map_refresh = false;
  map_updated = false;
  callbacks.log_info(description);

  dispatch_contexts[dispatch_id] = DispatchContext{
    goal_kind,
    frontier,
    frontier_sequence,
    action_name,
  };

  callbacks.dispatch_goal_request(GoalDispatchRequest{
    dispatch_id,
    action_name,
    goal_kind,
    goal_pose,
    frontier,
    frontier_sequence,
    description,
  });
}

void FrontierExplorerCore::clear_active_goal_state()
{
  // Clears all active lifecycle fields after terminal result/cancel.
  set_goal_state(GoalLifecycleState::IDLE);
  goal_handle.reset();
  active_goal_pose.reset();
  active_goal_frontier.reset();
  active_goal_frontiers.clear();
  active_goal_kind.clear();
  active_action_name.clear();
  active_goal_sent_time_ns.reset();
  active_goal_blocked_reason.reset();
  last_low_gain_reselection_time_ns.reset();
  clear_active_goal_progress_state();
  dispatch_states.clear();
  dispatch_contexts.clear();
}

std::optional<FrontierExplorerCore::DispatchContext> FrontierExplorerCore::dispatch_context_for(
  int dispatch_id) const
{
  // Dispatch context is best-effort metadata; stale callbacks may legitimately miss this entry.
  auto it = dispatch_contexts.find(dispatch_id);
  if (it == dispatch_contexts.end()) {
    return std::nullopt;
  }
  return it->second;
}

void FrontierExplorerCore::goal_response_callback(
  int dispatch_id,
  const std::shared_ptr<GoalHandleInterface> & received_goal_handle,
  bool accepted,
  const std::string & error_message)
{
  if (shutdown_requested) {
    // Ignore callbacks after teardown requested.
    return;
  }

  const auto context = dispatch_context_for(dispatch_id);

  if (!accepted) {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (context.has_value() && context->goal_kind == "return_to_start") {
      return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "frontier") {
      record_failed_frontier_attempt(context->frontier);
    }
    clear_active_goal_state();
    callbacks.log_error(error_message.empty() ? "Frontier goal was rejected" : error_message);
    return;
  }

  if (dispatch_id != current_dispatch_id) {
    // Late response from superseded dispatch is ignored; Nav2 handles active-goal preemption.
    return;
  }

  goal_handle = received_goal_handle;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::ACTIVE);
  set_goal_state(GoalLifecycleState::ACTIVE);
  if (context.has_value() && context->goal_kind == "return_to_start") {
    callbacks.log_info("Return-to-start goal accepted");
  } else if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
    callbacks.log_info("Temporary return-to-start goal accepted while frontiers are suppressed");
  } else {
    callbacks.log_info("Frontier goal accepted");
  }
  start_active_goal_progress_tracking();

  if (pending_cancel_reason.has_value()) {
    // If cancel was requested while sending, execute cancel immediately after accept.
    issue_active_goal_cancel();
  }
}

void FrontierExplorerCore::get_result_callback(
  int dispatch_id,
  int status,
  int error_code,
  const std::string & error_message,
  const std::string & exception_text)
{
  const auto context = dispatch_context_for(dispatch_id);
  // Cache derived values early so later cleanup cannot invalidate log/context decisions.
  const std::string goal_kind = context.has_value() ? context->goal_kind : "";
  const FrontierSequence frontier_sequence = context.has_value() ? context->frontier_sequence : FrontierSequence{};

  if (exception_text.empty()) {
    if (dispatch_id != current_dispatch_id) {
      // Result belongs to an older superseded dispatch.
      return;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && goal_kind == "return_to_start") {
      // Return path uses dedicated completion latch consumed by scheduler.
      return_to_start_completed = true;
      callbacks.log_info("Returned to start pose");
    } else if (
      status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
      goal_kind == "suppressed_return_to_start")
    {
      callbacks.log_info("Reached start pose while frontiers remain temporarily suppressed");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      if (goal_kind == "frontier" && escape_active) {
        escape_active = false;
        callbacks.log_info("Escape mode disabled after the first successful frontier");
      }

      if (frontier_sequence.size() > 1) {
        callbacks.log_info("Frontier goal reached with look-ahead orientation");
      } else {
        callbacks.log_info("Frontier goal reached");
      }
    } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      // Expected during explicit preemption/cancel paths.
      callbacks.log_debug(goal_kind + " was canceled before completion");
      if (
        goal_kind == "frontier" &&
        frontier_suppression_ &&
        frontier_suppression_->progress_timeout_cancel_requested())
      {
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
    } else {
      if (goal_kind == "return_to_start") {
        // Failed/aborted return-to-start should permit a future retry.
        return_to_start_started = false;
      } else if (goal_kind == "suppressed_return_to_start") {
        suppressed_return_to_start_started = false;
      }
      if (goal_kind == "frontier") {
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
      callbacks.log_warn(
        goal_kind + " finished with status " + detail::status_to_string(status) +
        ", error_code=" + std::to_string(error_code) +
        ", error_msg='" + error_message + "'");
    }
  } else {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (goal_kind == "return_to_start") {
      // Exceptions in result wait path should also reopen return-to-start retry path.
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    callbacks.log_error("Failed while waiting for result: " + exception_text);
  }

  if (dispatch_id != current_dispatch_id) {
    // Ignore stale results from superseded dispatches.
    return;
  }

  cancel_request_in_progress = false;
  // Cancel reason must not leak to the next goal lifecycle.
  pending_cancel_reason.reset();
  clear_active_goal_state();

  if (!exploration_enabled) {
    clear_post_goal_wait_state();
    return;
  }

  // Pending replacement goal may either dispatch immediately or enter the settle gate first.
  if (!pending_frontier_sequence.empty()) {
    const bool settle_before_reselection_dispatch =
      params.post_goal_settle_enabled &&
      pending_frontier_dispatch_context == "reselected";
    if (settle_before_reselection_dispatch) {
      start_post_goal_settle();
      return;
    }
    if (dispatch_pending_frontier_goal()) {
      return;
    }
  }

  if (goal_kind == "frontier" && status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    // Success path optionally waits for settle gates; disabled mode falls back to one fresh map edge.
    if (params.post_goal_settle_enabled) {
      start_post_goal_settle();
    } else {
      wait_for_next_map_refresh();
    }
  } else {
    wait_for_next_map_refresh();
  }
}

void FrontierExplorerCore::feedback_callback(double distance_remaining, int dispatch_id)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore feedback from stale dispatches.
    return;
  }

  note_active_goal_progress(distance_remaining);

  std::ostringstream oss;
  // Keep distance precision stable to ease visual comparison in debug logs.
  oss << std::fixed << std::setprecision(3)
      << "Distance remaining: " << distance_remaining;
  callbacks.log_debug(oss.str());
}

void FrontierExplorerCore::publish_frontier_markers(const FrontierSequence & frontiers)
{
  const auto signature = frontier_signature(frontiers);
  if (last_published_frontier_signature.has_value() &&
    *last_published_frontier_signature == signature)
  {
    // Skip duplicate publish to reduce RViz/topic noise.
    return;
  }

  callbacks.publish_frontier_markers(frontiers);
  last_published_frontier_signature = signature;
}

void FrontierExplorerCore::request_shutdown()
{
  // Prevent future callback-side state transitions and release current goal handle.
  shutdown_requested = true;
  goal_handle.reset();
}

}  // namespace frontier_exploration_ros2
