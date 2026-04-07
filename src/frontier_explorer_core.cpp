#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include <action_msgs/msg/goal_status.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace frontier_exploration_ros2
{

namespace
{

// Shared angle constant for the visible-gain sensor model and degree/radian conversions.
constexpr double kPi = 3.14159265358979323846;

std::string normalize_suppressed_behavior(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (value == "return_to_start") {
    return value;
  }
  return "stay";
}

std::string status_to_string(int status)
{
  // Keep status logging allocation/localized in one helper.
  std::ostringstream oss;
  // Keep numeric status visible for unknown/new action status values.
  oss << status;
  return oss.str();
}

// Formats numeric thresholds and measured distances for preemption logs so runtime
// decisions can be read back against parameter values without manual unit decoding.
std::string format_meters(double value)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << value << " m";
  return oss.str();
}

// Extracts planar yaw from a full quaternion because the visible-gain helper only
// needs a 2D heading for the target-pose sensor model.
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

// Rebuilds a quaternion from planar yaw so the target pose can be passed to the
// ray-cast helper using standard geometry message types.
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::cos(yaw * 0.5);
  orientation.z = std::sin(yaw * 0.5);
  return orientation;
}

}  // namespace

FrontierExplorerCore::FrontierExplorerCore(
  FrontierExplorerCoreParams params_in,
  FrontierExplorerCoreCallbacks callbacks_in)
: params(std::move(params_in)), callbacks(std::move(callbacks_in))
{
  params.all_frontiers_suppressed_behavior = normalize_suppressed_behavior(
    params.all_frontiers_suppressed_behavior);
  // Clamp settle configuration to safe minimums so state machine cannot deadlock on invalid params.
  params.post_goal_min_settle = std::max(0.0, params.post_goal_min_settle);
  params.post_goal_required_map_updates = std::max(1, params.post_goal_required_map_updates);
  params.post_goal_stable_updates = std::max(1, params.post_goal_stable_updates);
  params.post_goal_required_map_updates = std::max(
    params.post_goal_required_map_updates,
    params.post_goal_stable_updates);
  params.frontier_suppression_attempt_threshold = std::max(
    1,
    params.frontier_suppression_attempt_threshold);
  params.frontier_suppression_base_size_m = std::max(0.1, params.frontier_suppression_base_size_m);
  params.frontier_suppression_expansion_size_m = std::max(0.0, params.frontier_suppression_expansion_size_m);
  params.frontier_suppression_timeout_s = std::max(0.1, params.frontier_suppression_timeout_s);
  params.frontier_suppression_no_progress_timeout_s = std::max(
    0.1,
    params.frontier_suppression_no_progress_timeout_s);
  params.frontier_suppression_progress_epsilon_m = std::max(
    0.0,
    params.frontier_suppression_progress_epsilon_m);
  params.frontier_suppression_startup_grace_period_s = std::max(
    0.0,
    params.frontier_suppression_startup_grace_period_s);
  params.frontier_suppression_max_attempt_records = std::max(
    1,
    params.frontier_suppression_max_attempt_records);
  params.frontier_suppression_max_regions = std::max(1, params.frontier_suppression_max_regions);
  // Visible-gain geometry is clamped here so later preemption checks can assume valid ranges.
  params.goal_preemption_visible_gain_range_m = std::max(
    0.1,
    params.goal_preemption_visible_gain_range_m);
  params.goal_preemption_visible_gain_fov_deg = std::clamp(
    params.goal_preemption_visible_gain_fov_deg,
    1.0,
    360.0);
  params.goal_preemption_visible_gain_ray_step_deg = std::clamp(
    params.goal_preemption_visible_gain_ray_step_deg,
    0.25,
    45.0);
  params.goal_preemption_complete_if_within_m = std::max(
    0.0,
    params.goal_preemption_complete_if_within_m);
  params.goal_preemption_visible_gain_min_frontier_length_m = std::max(
    0.0,
    params.goal_preemption_visible_gain_min_frontier_length_m);

  escape_active = params.escape_enabled;

  // Defensive defaults keep unit tests and partial hosts from crashing.
  if (!callbacks.now_ns) {
    callbacks.now_ns = []() {return int64_t{0};};
  }
  if (!callbacks.get_current_pose) {
    callbacks.get_current_pose = []() -> std::optional<geometry_msgs::msg::Pose> {return std::nullopt;};
  }
  if (!callbacks.wait_for_action_server) {
    callbacks.wait_for_action_server = [](double) {return true;};
  }
  if (!callbacks.dispatch_goal_request) {
    callbacks.dispatch_goal_request = [](const GoalDispatchRequest &) {};
  }
  if (!callbacks.publish_frontier_markers) {
    callbacks.publish_frontier_markers = [](const FrontierSequence &) {};
  }
  if (!callbacks.on_exploration_complete) {
    callbacks.on_exploration_complete = []() {};
  }
  if (!callbacks.log_debug) {
    callbacks.log_debug = [](const std::string &) {};
  }
  if (!callbacks.log_info) {
    callbacks.log_info = [](const std::string &) {};
  }
  if (!callbacks.log_warn) {
    callbacks.log_warn = [](const std::string &) {};
  }
  if (!callbacks.log_error) {
    callbacks.log_error = [](const std::string &) {};
  }
  if (!callbacks.frontier_search) {
    // Fallback path delegates to package-level frontier extraction implementation.
    callbacks.frontier_search = [](
      const geometry_msgs::msg::Pose & pose,
      const OccupancyGrid2d & occupancy,
      const OccupancyGrid2d & global_costmap,
      const std::optional<OccupancyGrid2d> & local,
      double min_goal_distance,
      bool return_robot_cell) {
      return get_frontier(
        pose,
        occupancy,
        global_costmap,
        local,
        min_goal_distance,
        return_robot_cell);
    };
  }

  if (params.frontier_suppression_enabled) {
    frontier_suppression_activation_ns_ =
      callbacks.now_ns() +
      static_cast<int64_t>(params.frontier_suppression_startup_grace_period_s * 1e9);
  }
}

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

void FrontierExplorerCore::occupancyGridCallback(const OccupancyGrid2d & map_msg)
{
  // Map updates are authoritative for frontier discovery and settle progression.
  map = map_msg;
  // Generation is monotonic and used to invalidate frontier snapshots.
  map_generation += 1;
  // Any map update satisfies the first settle precondition.
  map_updated = true;
  if (goal_in_progress && active_goal_kind == "frontier") {
    // While navigating a frontier, map updates are routed through preemption policy first.
    consider_preempt_active_goal("map");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  observe_post_goal_settle_update();
  try_send_next_goal();
}

void FrontierExplorerCore::costmapCallback(const OccupancyGrid2d & map_msg)
{
  // Costmaps can trigger preemption while goal is active and advance settle while idle.
  costmap = map_msg;
  // Costmap generation participates in snapshot-cache key.
  costmap_generation += 1;
  if (goal_in_progress && active_goal_kind == "frontier") {
    consider_preempt_active_goal("costmap");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  observe_post_goal_settle_update(false);
  try_send_next_goal();
}

void FrontierExplorerCore::localCostmapCallback(const OccupancyGrid2d & map_msg)
{
  local_costmap = map_msg;
  // Local costmap generation also invalidates cached frontier snapshot.
  local_costmap_generation += 1;
  if (goal_in_progress && active_goal_kind == "frontier") {
    consider_preempt_active_goal("costmap");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  observe_post_goal_settle_update(false);
  try_send_next_goal();
}

void FrontierExplorerCore::try_send_next_goal()
{
  // Main scheduler entrypoint: exits early unless maps, pose, and lifecycle gates are ready.
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
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_visit_tolerance);
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

std::pair<double, double> FrontierExplorerCore::frontier_position(const FrontierLike & frontier) const
{
  return frontier_exploration_ros2::frontier_position(frontier);
}

std::pair<double, double> FrontierExplorerCore::frontier_reference_point(const FrontierLike & frontier) const
{
  return frontier_exploration_ros2::frontier_reference_point(frontier);
}

int FrontierExplorerCore::frontier_size(const FrontierLike & frontier) const
{
  return frontier_exploration_ros2::frontier_size(frontier);
}

std::string FrontierExplorerCore::describe_frontier(const FrontierLike & frontier) const
{
  return frontier_exploration_ros2::describe_frontier(frontier);
}

FrontierSignature FrontierExplorerCore::frontier_signature(const FrontierSequence & frontiers) const
{
  return frontier_exploration_ros2::frontier_signature(frontiers, params.frontier_visit_tolerance);
}

bool FrontierExplorerCore::frontier_snapshot_matches(
  const std::optional<FrontierSnapshot> & snapshot,
  const std::pair<int, int> & robot_map_cell,
  double min_goal_distance) const
{
  // Snapshot reuse is valid only when every cache-key field still matches exactly.
  return (
    snapshot.has_value() &&
    snapshot->map_generation == map_generation &&
    snapshot->costmap_generation == costmap_generation &&
    snapshot->local_costmap_generation == local_costmap_generation &&
    snapshot->robot_map_cell == robot_map_cell &&
    snapshot->min_goal_distance == min_goal_distance);
}

void FrontierExplorerCore::throttled_debug(const std::string & message)
{
  const int64_t now_ns = callbacks.now_ns();
  // Shared throttle avoids flooding debug logs in rapid map/costmap callback bursts.
  const int64_t throttle_ns = static_cast<int64_t>(frontier_stats_log_throttle_seconds * 1e9);
  if (!last_frontier_stats_log_time_ns.has_value() ||
    now_ns - *last_frontier_stats_log_time_ns >= throttle_ns)
  {
    // Move throttle window only when we actually emit a message.
    last_frontier_stats_log_time_ns = now_ns;
    callbacks.log_debug(message);
  }
}

void FrontierExplorerCore::log_frontier_snapshot_stats(
  const FrontierSequence & frontiers,
  double duration_ms,
  bool cache_hit)
{
  std::ostringstream oss;
  // Compact log line keeps p50/p95-style frontier timing inspection easy in runtime logs.
  oss << "frontier_snapshot: "
      << (cache_hit ? "hit" : "miss")
      << ", frontiers=" << frontiers.size()
      << ", duration_ms=" << std::fixed << std::setprecision(2) << duration_ms
      << ", hits=" << frontier_snapshot_cache_hits
      << ", misses=" << frontier_snapshot_cache_misses;
  throttled_debug(oss.str());
}

FrontierSnapshot FrontierExplorerCore::get_frontier_snapshot(
  const geometry_msgs::msg::Pose & current_pose,
  double min_goal_distance)
{
  // Snapshot cache is keyed by generations + robot cell + min_goal_distance.
  const auto robot_map_cell = map->worldToMap(current_pose.position.x, current_pose.position.y);
  if (frontier_snapshot_matches(frontier_snapshot, robot_map_cell, min_goal_distance)) {
    // Cache hit: avoid repeating expensive frontier extraction.
    frontier_snapshot_cache_hits += 1;
    log_frontier_snapshot_stats(frontier_snapshot->frontiers, 0.0, true);
    return *frontier_snapshot;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const auto search_result = callbacks.frontier_search(
    current_pose,
    *map,
    *costmap,
    local_costmap,
    min_goal_distance,
    true);
  const auto finished_at = std::chrono::steady_clock::now();
  const double duration_ms = std::chrono::duration<double, std::milli>(finished_at - started_at).count();

  FrontierSnapshot snapshot;
  // Convert low-level search output into policy-facing representation + cache key metadata.
  snapshot.frontiers = to_frontier_sequence(search_result.frontiers);
  snapshot.signature = frontier_signature(snapshot.frontiers);
  snapshot.map_generation = map_generation;
  snapshot.costmap_generation = costmap_generation;
  snapshot.local_costmap_generation = local_costmap_generation;
  snapshot.robot_map_cell = search_result.robot_map_cell;
  snapshot.min_goal_distance = min_goal_distance;

  frontier_snapshot = snapshot;
  frontier_snapshot_cache_misses += 1;
  log_frontier_snapshot_stats(snapshot.frontiers, duration_ms, false);
  return snapshot;
}

void FrontierExplorerCore::start_post_goal_settle()
{
  // After a succeeded frontier goal, wait for map/costmap stabilization before choosing next goal.
  awaiting_map_refresh = true;
  map_updated = false;
  post_goal_settle_active = true;
  post_goal_settle_started_at_ns = callbacks.now_ns();
  post_goal_map_updates_seen = 0;
  post_goal_stable_update_count = 0;
  post_goal_last_frontier_signature.reset();
}

void FrontierExplorerCore::wait_for_next_map_refresh()
{
  // Non-success terminal states use a simpler "wait for fresh map" gate.
  awaiting_map_refresh = true;
  map_updated = false;
  post_goal_settle_active = false;
  post_goal_settle_started_at_ns.reset();
  post_goal_map_updates_seen = 0;
  post_goal_stable_update_count = 0;
  post_goal_last_frontier_signature.reset();
}

void FrontierExplorerCore::clear_post_goal_wait_state()
{
  // Resets both simple wait-for-refresh and full settle state.
  awaiting_map_refresh = false;
  map_updated = false;
  post_goal_settle_active = false;
  post_goal_settle_started_at_ns.reset();
  post_goal_map_updates_seen = 0;
  post_goal_stable_update_count = 0;
  post_goal_last_frontier_signature.reset();
}

void FrontierExplorerCore::observe_post_goal_settle_update(bool refresh_frontier_signature)
{
  if (!awaiting_map_refresh || !post_goal_settle_active) {
    // Nothing to observe when settle is inactive.
    return;
  }

  map_updated = true;
  // Count every update event participating in settle decision.
  post_goal_map_updates_seen += 1;

  // Stable signature counting prevents immediate re-goaling on transient map changes.
  const auto update_stable_signature = [this](const FrontierSignature & signature) {
      if (post_goal_last_frontier_signature.has_value() &&
        signature == *post_goal_last_frontier_signature)
      {
        post_goal_stable_update_count += 1;
      } else {
        post_goal_last_frontier_signature = signature;
        post_goal_stable_update_count = 1;
      }
    };

  if (!refresh_frontier_signature) {
    // Costmap/local updates can still advance settle using the last known signature.
    if (frontier_snapshot.has_value()) {
      update_stable_signature(frontier_snapshot->signature);
    }
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value() || !map.has_value() || !costmap.has_value()) {
    // Signature refresh requires full search prerequisites.
    return;
  }

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_visit_tolerance);
  } catch (const std::out_of_range &) {
    return;
  }

  update_stable_signature(snapshot.signature);
}

bool FrontierExplorerCore::post_goal_settle_ready() const
{
  // Readiness predicate:
  //   map_updated
  //   AND elapsed >= post_goal_min_settle
  //   AND post_goal_map_updates_seen >= post_goal_required_map_updates
  //   AND post_goal_stable_update_count >= post_goal_stable_updates
  if (!map_updated) {
    return false;
  }

  if (!post_goal_settle_active) {
    // In simple refresh mode, one map update is enough.
    return true;
  }

  if (!post_goal_settle_started_at_ns.has_value()) {
    return false;
  }

  const double elapsed = static_cast<double>(callbacks.now_ns() - *post_goal_settle_started_at_ns) / 1e9;
  if (elapsed < params.post_goal_min_settle) {
    // Minimum dwell time not reached yet.
    return false;
  }

  if (post_goal_map_updates_seen < params.post_goal_required_map_updates) {
    return false;
  }

  if (post_goal_stable_update_count < params.post_goal_stable_updates) {
    return false;
  }

  return true;
}

FrontierSelectionResult FrontierExplorerCore::select_primitive_frontier(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  return frontier_exploration_ros2::select_primitive_frontier(
    frontiers,
    current_pose,
    params.frontier_min_distance,
    params.frontier_visit_tolerance,
    escape_active);
}

FrontierSelectionResult FrontierExplorerCore::select_frontier(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  return select_primitive_frontier(frontiers, current_pose);
}

void FrontierExplorerCore::record_start_pose(const geometry_msgs::msg::Pose & current_pose)
{
  if (start_pose.has_value()) {
    // Start pose is recorded once per exploration session.
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = params.global_frame;
  pose.pose = current_pose;
  start_pose = pose;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "Recorded exploration start pose: ("
      << current_pose.position.x << ", "
      << current_pose.position.y << ")";
  callbacks.log_info(oss.str());
}

bool FrontierExplorerCore::are_frontiers_equivalent(
  const std::optional<FrontierLike> & first_frontier,
  const std::optional<FrontierLike> & second_frontier) const
{
  return frontier_exploration_ros2::are_frontiers_equivalent(
    first_frontier,
    second_frontier,
    params.frontier_visit_tolerance);
}

bool FrontierExplorerCore::frontier_exists_in_set(
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontiers) const
{
  if (!frontier.has_value()) {
    return false;
  }

  for (const auto & candidate : frontiers) {
    // Tolerance-based check reuses shared frontier equivalence policy.
    if (are_frontiers_equivalent(frontier, candidate)) {
      return true;
    }
  }

  return false;
}

std::optional<std::string> FrontierExplorerCore::frontier_cost_status(
  const std::optional<FrontierLike> & frontier) const
{
  if (!frontier.has_value()) {
    return std::nullopt;
  }

  const auto goal_point = frontier_position(*frontier);

  const auto local_cost = world_point_cost(local_costmap, goal_point);
  if (local_cost.has_value() && *local_cost > OCC_THRESHOLD) {
    // Local map blocks have priority because they are most immediate for controller safety.
    return std::string(
      "Current frontier target is blocked in local costmap (cost=") +
      std::to_string(*local_cost) + ")";
  }

  const auto global_cost = world_point_cost(costmap, goal_point);
  if (global_cost.has_value() && *global_cost > OCC_THRESHOLD) {
    return std::string(
      "Current frontier target is blocked in global costmap (cost=") +
      std::to_string(*global_cost) + ")";
  }

  return std::nullopt;
}

geometry_msgs::msg::PoseStamped FrontierExplorerCore::build_goal_pose(
  const FrontierLike & target_frontier,
  const geometry_msgs::msg::Pose & current_pose,
  const std::optional<FrontierLike> & look_ahead_frontier) const
{
  const auto [target_x, target_y] = frontier_position(target_frontier);
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = params.global_frame;
  goal_pose.pose.position.x = target_x;
  goal_pose.pose.position.y = target_y;
  goal_pose.pose.orientation = current_pose.orientation;
  if (look_ahead_frontier.has_value()) {
    const auto [look_ahead_x, look_ahead_y] = frontier_position(*look_ahead_frontier);
    const double dx = look_ahead_x - target_x;
    const double dy = look_ahead_y - target_y;
    if (std::hypot(dx, dy) > 1e-6) {
      // When a second frontier is already known, bias arrival heading toward it.
      goal_pose.pose.orientation = quaternion_from_yaw(std::atan2(dy, dx));
    }
  }
  return goal_pose;
}

std::vector<geometry_msgs::msg::PoseStamped> FrontierExplorerCore::build_goal_pose_sequence(
  const FrontierSequence & target_frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  std::vector<geometry_msgs::msg::PoseStamped> goal_sequence;
  // Reserve once to keep goal sequence creation allocation-free for steady-state single-frontier mode.
  goal_sequence.reserve(target_frontiers.size());
  for (std::size_t i = 0; i < target_frontiers.size(); ++i) {
    const std::optional<FrontierLike> look_ahead_frontier =
      i + 1 < target_frontiers.size() ?
      std::optional<FrontierLike>(target_frontiers[i + 1]) :
      std::nullopt;
    goal_sequence.push_back(build_goal_pose(target_frontiers[i], current_pose, look_ahead_frontier));
  }
  return goal_sequence;
}

FrontierSequence FrontierExplorerCore::select_frontier_sequence(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose,
  const std::optional<FrontierLike> & initial_frontier) const
{
  if (!initial_frontier.has_value()) {
    return {};
  }

  FrontierSequence frontier_sequence{*initial_frontier};
  FrontierSequence remaining_frontiers;
  remaining_frontiers.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    if (!are_frontiers_equivalent(initial_frontier, frontier)) {
      remaining_frontiers.push_back(frontier);
    }
  }

  if (remaining_frontiers.empty()) {
    return frontier_sequence;
  }

  // Re-score the remaining frontiers as if the robot had already reached the current target.
  geometry_msgs::msg::Pose look_ahead_pose = current_pose;
  const auto [target_x, target_y] = frontier_position(*initial_frontier);
  look_ahead_pose.position.x = target_x;
  look_ahead_pose.position.y = target_y;

  const auto next_selection = select_frontier(remaining_frontiers, look_ahead_pose);
  if (next_selection.frontier.has_value()) {
    // The second entry is only a heading hint for the first dispatched frontier goal.
    frontier_sequence.push_back(*next_selection.frontier);
  }

  return frontier_sequence;
}

bool FrontierExplorerCore::are_frontier_sequences_equivalent(
  const FrontierSequence & first_frontier_sequence,
  const FrontierSequence & second_frontier_sequence) const
{
  return frontier_exploration_ros2::are_frontier_sequences_equivalent(
    first_frontier_sequence,
    second_frontier_sequence,
    params.frontier_visit_tolerance);
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
  replacement_candidate_sequence.clear();
  replacement_candidate_hits = 0;
}

bool FrontierExplorerCore::has_stable_replacement_candidate(const FrontierSequence & frontier_sequence)
{
  if (frontier_sequence.empty()) {
    reset_replacement_candidate_tracking();
    return false;
  }

  if (!replacement_candidate_sequence.empty() &&
    are_frontier_sequences_equivalent(frontier_sequence, replacement_candidate_sequence))
  {
    // Debounce reselection on stable observations instead of single-frame fluctuations.
    replacement_candidate_hits += 1;
  } else {
    // New candidate observed; restart stability counter.
    replacement_candidate_sequence = frontier_sequence;
    replacement_candidate_hits = 1;
  }

  // Candidate is considered stable only after configurable repeated hits.
  return replacement_candidate_hits >= replacement_required_hits;
}

std::optional<double> FrontierExplorerCore::active_goal_visible_frontier_length() const
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
    heading = yaw_from_quaternion(active_goal_pose->pose.orientation);
  }

  const double yaw_offset_rad = params.goal_preemption_visible_gain_yaw_offset_deg * (kPi / 180.0);
  geometry_msgs::msg::Pose sensor_pose;
  sensor_pose.position.x = goal_point.first;
  sensor_pose.position.y = goal_point.second;
  sensor_pose.orientation = quaternion_from_yaw(heading + yaw_offset_rad);

  // The helper returns a local, occlusion-aware frontier length estimate around the target pose.
  const auto visible_gain = compute_visible_frontier_gain(
    sensor_pose,
    *map,
    *costmap,
    local_costmap,
    params.goal_preemption_visible_gain_range_m,
    params.goal_preemption_visible_gain_fov_deg,
    params.goal_preemption_visible_gain_ray_step_deg);
  if (!visible_gain.has_value()) {
    return std::nullopt;
  }

  return visible_gain->visible_frontier_length_m;
}

void FrontierExplorerCore::consider_preempt_active_goal(const std::string & trigger_source)
{
  // Costmap-triggered calls only update blocked reason; map-triggered calls may reseat goals.
  const bool preemption_allowed = (
    params.goal_preemption_on_frontier_revealed ||
    params.goal_preemption_on_blocked_goal);
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

  const auto active_goal_cost_status = params.goal_preemption_on_blocked_goal ?
    frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};
  if (!active_goal_cost_status.has_value() && !params.goal_preemption_on_frontier_revealed) {
    // Nothing to do when both preemption triggers are effectively inactive.
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

  if (!active_goal_sent_time_ns.has_value() && !active_goal_cost_status.has_value()) {
    // Without send timestamp we cannot evaluate time-based frontier-revealed gate.
    return;
  }

  const double elapsed = active_goal_sent_time_ns.has_value() ?
    static_cast<double>(callbacks.now_ns() - *active_goal_sent_time_ns) / 1e9 : 0.0;

  // Time-based gate for frontier-revealed preemption to avoid immediate churn.
  if (
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_on_frontier_revealed &&
    elapsed < params.goal_preemption_min_interval_s)
  {
    return;
  }

  const auto active_goal_reference = frontier_reference_point(*active_goal_frontier);
  const double active_goal_distance = std::hypot(
    active_goal_reference.first - current_pose->position.x,
    active_goal_reference.second - current_pose->position.y);
  std::optional<double> visible_frontier_length;
  // Near-goal completion can force revealed-preemption even before visible-gain evaluation.
  const bool revealed_completion_distance_reached =
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_on_frontier_revealed &&
    params.goal_preemption_complete_if_within_m > 0.0 &&
    active_goal_distance <= params.goal_preemption_complete_if_within_m;
  // Visible-gain gate is only meaningful on the map-triggered revealed-preemption path.
  const bool visible_gain_gate_active =
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_on_frontier_revealed &&
    params.goal_preemption_visible_gain_gate_enabled;
  bool visible_frontier_gain_exhausted = false;

  // Distance-based gate: if robot is already close, finishing current goal is usually cheaper.
  // The completion-distance policy can override this with its own "close enough means complete" threshold.
  if (
    !revealed_completion_distance_reached &&
    !visible_gain_gate_active &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_on_frontier_revealed &&
    active_goal_distance <= params.goal_preemption_skip_if_within_m)
  {
    return;
  }

  if (visible_gain_gate_active && !revealed_completion_distance_reached) {
    visible_frontier_length = active_goal_visible_frontier_length();
    if (!visible_frontier_length.has_value()) {
      callbacks.log_warn(
        "Skipping visible frontier gain gate for the active goal; falling back to frontier snapshot reselection");
    } else if (*visible_frontier_length >= params.goal_preemption_visible_gain_min_frontier_length_m) {
      // Keep the current goal while the target pose can still reveal enough frontier on arrival.
      reset_replacement_candidate_tracking();
      return;
    } else {
      // Low visible gain alone is not enough to preempt; the refreshed frontier snapshot still decides.
      visible_frontier_gain_exhausted = true;
    }
  }

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_visit_tolerance);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier reselection: ") + exc.what());
    return;
  }

  const FrontierSequence & frontiers = snapshot.frontiers;
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(frontiers);
  if (revealed_completion_distance_reached) {
    // Completion-distance preemption treats the current frontier as finished, so exclude it
    // from replacement candidates in this pass to avoid immediately selecting it again.
    filtered_frontiers.erase(
      std::remove_if(
        filtered_frontiers.begin(),
        filtered_frontiers.end(),
        [this](const FrontierLike & frontier) {
          return are_frontiers_equivalent(active_goal_frontier, frontier);
        }),
      filtered_frontiers.end());
  }
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
    params.goal_preemption_on_frontier_revealed &&
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
    revealed_completion_distance_reached ?
    "active frontier considered complete by goal_preemption_complete_if_within_m (distance=" +
    format_meters(active_goal_distance) + ", threshold=" +
    format_meters(params.goal_preemption_complete_if_within_m) +
    "); preempting to the next frontier" :
    visible_frontier_gain_exhausted && visible_frontier_length.has_value() ?
    "active frontier revealed and visible frontier gain exhausted at target pose (visible=" +
    format_meters(*visible_frontier_length) + ", required=" +
    format_meters(params.goal_preemption_visible_gain_min_frontier_length_m) +
    "); preempting to the next frontier" :
    "active frontier no longer appears in the latest frontier snapshot after reveal updates; preempting to the next frontier");

  request_frontier_reselection(
    frontier_sequence,
    *current_pose,
    selection.mode,
    revealed_preemption_reason);
}

void FrontierExplorerCore::request_frontier_reselection(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & selection_mode,
  const std::string & reselection_reason)
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
    "Preempting active frontier goal: " + reselection_reason);
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
  callbacks.log_info(reason);

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

  if (!params.return_to_start_on_complete ||
    !start_pose.has_value() ||
    return_to_start_completed)
  {
    // Either feature is disabled, start pose is unavailable, or completion already handled.
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
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_visit_tolerance);
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
    // Late response from superseded dispatch: cancel it immediately.
    if (received_goal_handle) {
      received_goal_handle->cancel_goal_async(
        [](bool, const std::string &) {});
    }
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
        goal_kind + " finished with status " + status_to_string(status) +
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

  // Pending replacement goal has priority over settle/refresh waiting.
  if (dispatch_pending_frontier_goal()) {
    return;
  }

  if (goal_kind == "frontier" && status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    // Success path waits for settle gates; non-success paths wait for the next refresh edge.
    start_post_goal_settle();
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

FrontierSequence FrontierExplorerCore::to_frontier_sequence(
  const std::vector<FrontierCandidate> & frontiers)
{
  FrontierSequence sequence;
  // Preserve extraction order for deterministic policy behavior.
  sequence.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    sequence.push_back(frontier);
  }
  return sequence;
}

}  // namespace frontier_exploration_ros2
