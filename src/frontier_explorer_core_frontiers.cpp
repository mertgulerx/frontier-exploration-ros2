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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frontier_exploration_ros2
{

FrontierSequence FrontierExplorerCore::build_mrtsp_frontier_sequence(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (frontiers.empty()) {
    return {};
  }

  const double pose_quantum = std::max(params.frontier_visit_tolerance, 0.1);
  const int pose_x_bucket = detail::quantize_bucket(current_pose.position.x, pose_quantum);
  const int pose_y_bucket = detail::quantize_bucket(current_pose.position.y, pose_quantum);
  const int yaw_bucket = detail::quantize_bucket(
    detail::yaw_from_quaternion(current_pose.orientation),
    detail::kPi / 12.0);
  const FrontierSignature signature = frontier_signature(frontiers);

  if (mrtsp_order_cache.has_value() &&
    mrtsp_order_cache->frontier_signature == signature &&
    mrtsp_order_cache->pose_x_bucket == pose_x_bucket &&
    mrtsp_order_cache->pose_y_bucket == pose_y_bucket &&
    mrtsp_order_cache->yaw_bucket == yaw_bucket &&
    mrtsp_order_cache->sensor_effective_range_m == params.sensor_effective_range_m &&
    mrtsp_order_cache->weight_distance_wd == params.weight_distance_wd &&
    mrtsp_order_cache->weight_gain_ws == params.weight_gain_ws &&
    mrtsp_order_cache->max_linear_speed_vmax == params.max_linear_speed_vmax &&
    mrtsp_order_cache->max_angular_speed_wmax == params.max_angular_speed_wmax)
  {
    const_cast<FrontierExplorerCore *>(this)->mrtsp_order_cache_hits += 1;
    if (debug_outputs_enabled()) {
      callbacks.log_debug(
        "mrtsp_order_cache: hit, frontiers=" + std::to_string(frontiers.size()) +
        ", strategy=" + detail::strategy_to_string(params.strategy));
    }
    return mrtsp_order_cache->frontier_sequence;
  }

  std::vector<FrontierCandidate> candidates;
  candidates.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    if (const auto * candidate = std::get_if<FrontierCandidate>(&frontier)) {
      candidates.push_back(*candidate);
    }
  }

  if (candidates.empty()) {
    return {};
  }

  RobotState robot_state;
  robot_state.position = {current_pose.position.x, current_pose.position.y};
  robot_state.yaw = detail::yaw_from_quaternion(current_pose.orientation);

  CostWeights weights;
  weights.distance_wd = params.weight_distance_wd;
  weights.gain_ws = params.weight_gain_ws;

  const MrtspCostMatrix cost_matrix = build_cost_matrix(
    candidates,
    robot_state,
    weights,
    params.sensor_effective_range_m,
    params.max_linear_speed_vmax,
    params.max_angular_speed_wmax);
  const std::vector<std::size_t> order = greedy_mrtsp_order(cost_matrix);

  FrontierSequence ordered_frontiers;
  ordered_frontiers.reserve(order.size());
  for (const std::size_t index : order) {
    if (index < frontiers.size()) {
      ordered_frontiers.push_back(frontiers[index]);
    }
  }

  auto & mutable_self = const_cast<FrontierExplorerCore &>(*this);
  mutable_self.mrtsp_order_cache = MrtspOrderCacheEntry{
    signature,
    pose_x_bucket,
    pose_y_bucket,
    yaw_bucket,
    params.sensor_effective_range_m,
    params.weight_distance_wd,
    params.weight_gain_ws,
    params.max_linear_speed_vmax,
    params.max_angular_speed_wmax,
    ordered_frontiers,
  };
  mutable_self.mrtsp_order_cache_misses += 1;
  if (debug_outputs_enabled()) {
    callbacks.log_debug(
      "mrtsp_order_cache: miss, frontiers=" + std::to_string(frontiers.size()) +
      ", ordered=" + std::to_string(ordered_frontiers.size()));
  }
  return ordered_frontiers;
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
  // Snapshot reuse follows the actual frontier-search inputs. Raw map generation may advance
  // without changing decision_map output, so decision_map_generation is the map-side key.
  return (
    snapshot.has_value() &&
    snapshot->decision_map_generation == decision_map_generation &&
    snapshot->costmap_generation == costmap_generation &&
    snapshot->local_costmap_generation == local_costmap_generation &&
    snapshot->robot_map_cell == robot_map_cell &&
    snapshot->min_goal_distance == min_goal_distance &&
    snapshot->strategy == params.strategy);
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
  if (!decision_map.has_value()) {
    if (!map.has_value()) {
      throw std::logic_error("Decision map is not initialized");
    }
    refresh_decision_map();
  }
  // Snapshot cache is keyed by decision/costmap generations + robot cell + min_goal_distance.
  const auto robot_map_cell = decision_map->worldToMap(current_pose.position.x, current_pose.position.y);
  if (frontier_snapshot_matches(frontier_snapshot, robot_map_cell, min_goal_distance)) {
    // Cache hit: avoid repeating expensive frontier extraction.
    frontier_snapshot_cache_hits += 1;
    log_frontier_snapshot_stats(frontier_snapshot->frontiers, 0.0, true);
    return *frontier_snapshot;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const auto search_result = callbacks.frontier_search(
    current_pose,
    *decision_map,
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
  snapshot.decision_map_generation = decision_map_generation;
  snapshot.costmap_generation = costmap_generation;
  snapshot.local_costmap_generation = local_costmap_generation;
  snapshot.robot_map_cell = search_result.robot_map_cell;
  snapshot.min_goal_distance = min_goal_distance;
  snapshot.strategy = params.strategy;

  frontier_snapshot = snapshot;
  frontier_snapshot_cache_misses += 1;
  if (debug_outputs_enabled() && map.has_value()) {
    std::size_t raw_frontier_count = snapshot.frontiers.size();
    if (frontier_map_optimization_enabled()) {
      const FrontierSearchOptions options = frontier_search_options();
      if (
        raw_frontier_debug_cache.has_value() &&
        raw_frontier_debug_cache->map_generation == map_generation &&
        raw_frontier_debug_cache->costmap_generation == costmap_generation &&
        raw_frontier_debug_cache->local_costmap_generation == local_costmap_generation &&
        raw_frontier_debug_cache->robot_map_cell == search_result.robot_map_cell &&
        raw_frontier_debug_cache->min_goal_distance == min_goal_distance &&
        raw_frontier_debug_cache->strategy == params.strategy &&
        raw_frontier_debug_cache->search_options.occ_threshold == options.occ_threshold &&
        raw_frontier_debug_cache->search_options.min_frontier_size_cells == options.min_frontier_size_cells &&
        raw_frontier_debug_cache->search_options.candidate_min_goal_distance_m == options.candidate_min_goal_distance_m &&
        raw_frontier_debug_cache->search_options.use_local_costmap_for_frontier_eligibility ==
        options.use_local_costmap_for_frontier_eligibility &&
        raw_frontier_debug_cache->search_options.out_of_bounds_costmap_is_blocked ==
        options.out_of_bounds_costmap_is_blocked &&
        raw_frontier_debug_cache->search_options.build_navigation_goal_point ==
        options.build_navigation_goal_point)
      {
        raw_frontier_count = raw_frontier_debug_cache->frontier_count;
      } else {
        const auto raw_search_result = get_frontier(
          current_pose,
          *map,
          *costmap,
          local_costmap,
          min_goal_distance,
          false,
          options);
        raw_frontier_count = raw_search_result.frontiers.size();
        raw_frontier_debug_cache = RawFrontierDebugCacheEntry{
          map_generation,
          costmap_generation,
          local_costmap_generation,
          raw_search_result.robot_map_cell,
          min_goal_distance,
          params.strategy,
          options,
          raw_frontier_count,
        };
      }
    }
    callbacks.log_debug(
      "frontier_counts: raw=" + std::to_string(raw_frontier_count) +
      ", decision=" + std::to_string(snapshot.frontiers.size()) +
      ", strategy=" + detail::strategy_to_string(params.strategy));
  }
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
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_candidate_min_goal_distance_m);
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
    params.frontier_selection_min_distance,
    params.frontier_visit_tolerance,
    escape_active);
}

FrontierSelectionResult FrontierExplorerCore::select_frontier(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (mrtsp_enabled()) {
    const FrontierSequence ordered_frontiers = build_mrtsp_frontier_sequence(frontiers, current_pose);
    if (ordered_frontiers.empty()) {
      return {std::nullopt, ""};
    }
    return {ordered_frontiers.front(), "mrtsp"};
  }
  return select_primitive_frontier(frontiers, current_pose);
}

void FrontierExplorerCore::record_start_pose(const geometry_msgs::msg::Pose & current_pose)
{
  if (start_pose.has_value()) {
    // Start pose is recorded once per node lifetime and survives session stop/start cycles.
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
  // Default heading follows the travel vector toward the selected target frontier.
  // This avoids forcing a stale "current heading" orientation at goal completion.
  goal_pose.pose.orientation = current_pose.orientation;
  const double to_target_dx = target_x - current_pose.position.x;
  const double to_target_dy = target_y - current_pose.position.y;
  if (std::hypot(to_target_dx, to_target_dy) > 1e-6) {
    goal_pose.pose.orientation = detail::quaternion_from_yaw(std::atan2(to_target_dy, to_target_dx));
  }
  if (look_ahead_frontier.has_value()) {
    const auto [look_ahead_x, look_ahead_y] = frontier_position(*look_ahead_frontier);
    const double dx = look_ahead_x - target_x;
    const double dy = look_ahead_y - target_y;
    if (std::hypot(dx, dy) > 1e-6) {
      // When a second frontier is already known, bias arrival heading toward it.
      goal_pose.pose.orientation = detail::quaternion_from_yaw(std::atan2(dy, dx));
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
  if (mrtsp_enabled()) {
    (void)initial_frontier;
    return build_mrtsp_frontier_sequence(frontiers, current_pose);
  }

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

}  // namespace frontier_exploration_ros2
