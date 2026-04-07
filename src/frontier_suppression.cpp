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

#include "frontier_exploration_ros2/frontier_suppression.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace frontier_exploration_ros2
{

namespace
{

// Small helper kept local so distance comparisons stay cheap and allocation-free.
// The expansion policy only needs relative ordering between candidate regions,
// so squared distance avoids an unnecessary sqrt while keeping the logic clear.
double squared_distance(
  const std::pair<double, double> & first_point,
  const std::pair<double, double> & second_point)
{
  const double dx = first_point.first - second_point.first;
  const double dy = first_point.second - second_point.second;
  return (dx * dx) + (dy * dy);
}

}  // namespace

std::size_t SuppressionKeyHash::operator()(const SuppressionKey & key) const noexcept
{
  // Standard hash-combine pattern; good enough here because buckets are already quantized.
  // Since keys are coarse buckets rather than raw floating-point coordinates, we do not need
  // a more complicated spatial hash here.
  const std::size_t seed = std::hash<int64_t>{}(key.x_bucket);
  return seed ^ (std::hash<int64_t>{}(key.y_bucket) + 0x9e3779b97f4a7c15ULL + (seed << 6U) +
         (seed >> 2U));
}

FrontierSuppression::FrontierSuppression(FrontierSuppressionConfig config)
: config_(std::move(config))
{
  // Clamp to safe lower bounds so invalid external params cannot destabilize suppression behavior.
  // The policy intentionally self-heals bad config values instead of pushing validation burden
  // into every caller.
  config_.attempt_threshold = std::max(1, config_.attempt_threshold);
  config_.base_size_m = std::max(0.1, config_.base_size_m);
  config_.expansion_size_m = std::max(0.0, config_.expansion_size_m);
  config_.timeout_s = std::max(0.1, config_.timeout_s);
  config_.no_progress_timeout_s = std::max(0.1, config_.no_progress_timeout_s);
  config_.progress_epsilon_m = std::max(0.0, config_.progress_epsilon_m);
  config_.max_attempt_records = std::max(1, config_.max_attempt_records);
  config_.max_regions = std::max(1, config_.max_regions);
  config_.equivalence_tolerance = std::max(0.1, config_.equivalence_tolerance);

  // Reserve once up front so the feature has predictable bounded-memory behavior under load.
  state_.attempts.reserve(static_cast<std::size_t>(config_.max_attempt_records));
  state_.regions.reserve(static_cast<std::size_t>(config_.max_regions));
}

std::size_t FrontierSuppression::attempt_count() const noexcept
{
  return state_.attempts.size();
}

std::size_t FrontierSuppression::region_count() const noexcept
{
  return state_.regions.size();
}

const std::vector<SuppressedRegion> & FrontierSuppression::regions() const noexcept
{
  return state_.regions;
}

const std::unordered_map<SuppressionKey, SuppressionAttempt, SuppressionKeyHash> &
FrontierSuppression::attempts() const noexcept
{
  return state_.attempts;
}

double FrontierSuppression::quantum() const
{
  // Quantization follows the same tolerance family as frontier equivalence in the core.
  // This keeps "same place" semantics aligned between selection, revisit logic, and suppression.
  return std::max(config_.equivalence_tolerance, 0.1);
}

SuppressionKey FrontierSuppression::make_key(const std::pair<double, double> & goal_point) const
{
  // Quantization intentionally collapses near-identical failures into the same record so
  // tiny numerical drift or path planner jitter does not fragment the attempt history.
  const double q = quantum();
  return {
    static_cast<int64_t>(std::llround(goal_point.first / q)),
    static_cast<int64_t>(std::llround(goal_point.second / q)),
  };
}

bool FrontierSuppression::point_in_region(
  const std::pair<double, double> & point,
  const SuppressedRegion & region) const
{
  // Regions are axis-aligned squares to keep filtering and expansion checks simple and cheap.
  // The suppression policy is intended to be a robust coarse exclusion mechanism rather than
  // a precise geometric obstacle model.
  const double half_side = region.side_length_m * 0.5;
  return (
    std::abs(point.first - region.center.first) <= half_side &&
    std::abs(point.second - region.center.second) <= half_side);
}

bool FrontierSuppression::point_in_expansion_ring(
  const std::pair<double, double> & point,
  const SuppressedRegion & region) const
{
  // The ring is defined as the outer expanded square minus the currently suppressed inner square.
  // A failure in this band suggests the robot is struggling in the same neighborhood but just
  // outside the current square, so the square should grow rather than creating a second fragment.
  const SuppressedRegion outer_region{
    region.center,
    region.side_length_m + config_.expansion_size_m,
    region.last_updated_ns,
  };
  return point_in_region(point, outer_region) && !point_in_region(point, region);
}

bool FrontierSuppression::point_in_any_region(const std::pair<double, double> & point) const
{
  // Region counts are capped and expected to stay small, so a linear scan is simpler and
  // usually more cache-friendly than maintaining a heavier spatial index.
  return std::any_of(
    state_.regions.begin(),
    state_.regions.end(),
    [this, &point](const SuppressedRegion & region) {
      return point_in_region(point, region);
    });
}

void FrontierSuppression::prune_expired(int64_t now_ns)
{
  // A shared TTL keeps both transient attempts and active regions self-cleaning over time.
  // Attempt records represent "recently failing" memory, while regions represent "temporarily
  // avoid this area" memory; both are deliberately temporary and should decay away.
  const int64_t ttl_ns = static_cast<int64_t>(config_.timeout_s * 1e9);

  for (auto it = state_.attempts.begin(); it != state_.attempts.end();) {
    if (now_ns - it->second.last_updated_ns >= ttl_ns) {
      it = state_.attempts.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = state_.regions.begin(); it != state_.regions.end();) {
    if (now_ns - it->last_updated_ns >= ttl_ns) {
      it = state_.regions.erase(it);
    } else {
      ++it;
    }
  }
}

FrontierSequence FrontierSuppression::filter_frontiers(const FrontierSequence & frontiers) const
{
  if (state_.regions.empty()) {
    // Fast path avoids extra per-frontier checks when no regions are active.
    return frontiers;
  }

  FrontierSequence filtered;
  filtered.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    // Suppression operates on the actual selected goal point, not on the centroid.
    // This matters because the planner is asked to navigate to the frontier goal point, and
    // that specific point is what has demonstrated repeated failure.
    if (!point_in_any_region(frontier_position(frontier))) {
      filtered.push_back(frontier);
    }
  }
  return filtered;
}

void FrontierSuppression::remove_attempts_inside_region(const SuppressedRegion & region)
{
  // Once a square is suppressed, individual attempt records inside it no longer add value.
  // Keeping them would only waste bounded capacity and could later cause confusing re-promotion.
  for (auto it = state_.attempts.begin(); it != state_.attempts.end();) {
    if (point_in_region(it->second.goal_point, region)) {
      it = state_.attempts.erase(it);
    } else {
      ++it;
    }
  }
}

void FrontierSuppression::evict_oldest_attempt_record(
  const std::function<void(const std::string &)> & log_warn)
{
  if (state_.attempts.empty()) {
    return;
  }

  // Oldest-entry eviction enforces the hard memory cap when TTL cleanup is not enough.
  // This gives the structure a strict upper bound without constantly reallocating or shrinking.
  auto oldest_it = state_.attempts.begin();
  for (auto it = state_.attempts.begin(); it != state_.attempts.end(); ++it) {
    if (it->second.last_updated_ns < oldest_it->second.last_updated_ns) {
      oldest_it = it;
    }
  }
  if (log_warn) {
    log_warn("Frontier suppression attempt storage reached capacity; evicting oldest record.");
  }
  state_.attempts.erase(oldest_it);
}

void FrontierSuppression::evict_oldest_region(
  const std::function<void(const std::string &)> & log_warn)
{
  if (state_.regions.empty()) {
    return;
  }

  // Regions are also bounded so prolonged operation cannot accumulate infinite memory.
  // If this path is ever hot, the warning log should make it visible to operators so they can
  // revisit TTL or cap sizing.
  auto oldest_it = state_.regions.begin();
  for (auto it = state_.regions.begin(); it != state_.regions.end(); ++it) {
    if (it->last_updated_ns < oldest_it->last_updated_ns) {
      oldest_it = it;
    }
  }
  if (log_warn) {
    log_warn("Frontier suppression region storage reached capacity; evicting oldest region.");
  }
  state_.regions.erase(oldest_it);
}

void FrontierSuppression::promote_attempt_to_region(
  const std::pair<double, double> & goal_point,
  int64_t now_ns,
  const std::function<void(const std::string &)> & log_warn)
{
  // Prefer expanding the nearest compatible region so repeated failures coalesce spatially.
  // This avoids creating many overlapping squares for what is effectively the same hard area.
  auto chosen_it = state_.regions.end();
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (auto it = state_.regions.begin(); it != state_.regions.end(); ++it) {
    if (!point_in_expansion_ring(goal_point, *it)) {
      continue;
    }
    const double distance_sq = squared_distance(goal_point, it->center);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      chosen_it = it;
    }
  }

  if (chosen_it != state_.regions.end()) {
    // Expansion moves the center toward the new failure point and doubles the square size.
    // The midpoint update keeps the square representative of both the historical region and
    // the new failure location without introducing complex merge bookkeeping.
    chosen_it->center = {
      (chosen_it->center.first + goal_point.first) * 0.5,
      (chosen_it->center.second + goal_point.second) * 0.5,
    };
    chosen_it->side_length_m *= 2.0;
    chosen_it->last_updated_ns = now_ns;
    remove_attempts_inside_region(*chosen_it);
    return;
  }

  for (auto & region : state_.regions) {
    if (point_in_region(goal_point, region)) {
      // Hitting an already suppressed region only refreshes its TTL.
      // We intentionally do not expand here; the point is already inside the current square and
      // therefore does not provide new geometric information.
      region.last_updated_ns = now_ns;
      remove_attempts_inside_region(region);
      return;
    }
  }

  while (state_.regions.size() >= static_cast<std::size_t>(config_.max_regions)) {
    evict_oldest_region(log_warn);
  }

  // No compatible region exists, so start a fresh square centered on the failed goal point.
  // This is the smallest and safest suppression action we can take after a matured failure.
  state_.regions.push_back(SuppressedRegion{
    goal_point,
    config_.base_size_m,
    now_ns,
  });
  remove_attempts_inside_region(state_.regions.back());
}

void FrontierSuppression::record_failed_attempt(
  const FrontierLike & frontier,
  int64_t now_ns,
  const std::function<void(const std::string &)> & log_warn)
{
  // Keep the state clean before inserting anything new.
  // This also means capacity checks act on only live data rather than stale entries.
  prune_expired(now_ns);

  const auto goal_point = frontier_position(frontier);
  if (point_in_any_region(goal_point)) {
    // Failures inside an already suppressed region are intentionally ignored.
    // The area is already excluded, so additional counting would not change behavior.
    return;
  }

  const auto key = make_key(goal_point);
  auto it = state_.attempts.find(key);
  if (it == state_.attempts.end()) {
    // Attempt records are bounded independently from regions.
    // Separate attempt and region caps let us tune short-term failure memory without forcing
    // the same limit on long-lived suppressed areas.
    while (state_.attempts.size() >= static_cast<std::size_t>(config_.max_attempt_records)) {
      evict_oldest_attempt_record(log_warn);
    }
    it = state_.attempts.emplace(
      key,
      SuppressionAttempt{goal_point, 0, now_ns}).first;
  }

  it->second.goal_point = goal_point;
  it->second.failure_count += 1;
  it->second.last_updated_ns = now_ns;

  if (it->second.failure_count >= config_.attempt_threshold) {
    // Threshold reached: convert transient failure memory into spatial suppression.
    // Promotion happens immediately so the next frontier search can avoid the area.
    promote_attempt_to_region(goal_point, now_ns, log_warn);
    state_.attempts.erase(key);
  }
}

void FrontierSuppression::start_goal_progress_tracking(int dispatch_id, int64_t now_ns)
{
  // Progress tracking starts from an optimistic baseline and waits for feedback samples.
  // The watchdog can be armed before the first feedback arrives, but cancellation is still
  // measured from the last meaningful-progress timestamp which is seeded here.
  state_.active_goal_progress = ActiveGoalProgress{
    dispatch_id,
    false,
    0.0,
    0.0,
    now_ns,
    now_ns,
    false,
  };
}

void FrontierSuppression::clear_goal_progress_tracking()
{
  state_.active_goal_progress.reset();
}

void FrontierSuppression::note_goal_progress(int dispatch_id, double distance_remaining, int64_t now_ns)
{
  if (!state_.active_goal_progress.has_value() || state_.active_goal_progress->dispatch_id != dispatch_id) {
    // Ignore stale feedback from superseded dispatches.
    return;
  }

  auto & progress = *state_.active_goal_progress;
  progress.last_feedback_ns = now_ns;
  if (!progress.has_feedback) {
    // The first feedback sample seeds both "best seen" and "last meaningful progress" state.
    // From this point on, timeout decisions are based on actual navigation feedback.
    progress.has_feedback = true;
    progress.best_distance_remaining = distance_remaining;
    progress.distance_at_last_progress = distance_remaining;
    progress.last_meaningful_progress_ns = now_ns;
    return;
  }

  if (distance_remaining < progress.best_distance_remaining) {
    // Track the best observed distance monotonically so noisy feedback cannot fake regression.
    progress.best_distance_remaining = distance_remaining;
  }

  // Only a meaningful drop resets the watchdog; tiny oscillations should not.
  // This protects against controllers that wobble in place and emit small fluctuating values.
  if (progress.distance_at_last_progress - progress.best_distance_remaining >= config_.progress_epsilon_m) {
    progress.distance_at_last_progress = progress.best_distance_remaining;
    progress.last_meaningful_progress_ns = now_ns;
  }
}

bool FrontierSuppression::is_tracking_dispatch(int dispatch_id) const noexcept
{
  return state_.active_goal_progress.has_value() &&
         state_.active_goal_progress->dispatch_id == dispatch_id;
}

bool FrontierSuppression::mark_timeout_cancel_if_needed(int dispatch_id, int64_t now_ns)
{
  if (!state_.active_goal_progress.has_value()) {
    return false;
  }

  auto & progress = *state_.active_goal_progress;
  if (progress.dispatch_id != dispatch_id || progress.cancel_requested_by_timeout) {
    // Never re-arm a timeout cancel for the same dispatch once it has already been requested.
    return false;
  }

  // Timeout is measured from the last meaningful progress event, not from dispatch time.
  // That makes the watchdog tolerant of long routes as long as the robot is still getting closer.
  const int64_t timeout_ns = static_cast<int64_t>(config_.no_progress_timeout_s * 1e9);
  if (now_ns - progress.last_meaningful_progress_ns < timeout_ns) {
    return false;
  }

  // The core uses this sticky flag to attribute the later canceled result to suppression.
  progress.cancel_requested_by_timeout = true;
  return true;
}

bool FrontierSuppression::progress_timeout_cancel_requested() const noexcept
{
  return state_.active_goal_progress.has_value() &&
         state_.active_goal_progress->cancel_requested_by_timeout;
}

}  // namespace frontier_exploration_ros2
