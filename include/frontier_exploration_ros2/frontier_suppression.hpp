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

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_policy.hpp"

namespace frontier_exploration_ros2
{

// Runtime knobs for failure-memory based frontier suppression.
// These values are owned by the core, then copied into the suppression module
// when the feature is enabled and instantiated.
struct FrontierSuppressionConfig
{
  // Number of failed attempts required before an area is suppressed.
  int attempt_threshold{3};
  // Initial side length of a new square suppression region.
  double base_size_m{2.0};
  // Outer ring width used to detect nearby follow-up failures and expand a region.
  double expansion_size_m{1.0};
  // TTL shared by both transient attempt records and suppressed regions.
  double timeout_s{90.0};
  // Maximum allowed stall duration for an active frontier goal.
  double no_progress_timeout_s{20.0};
  // Minimum improvement needed before progress is treated as meaningful.
  double progress_epsilon_m{0.25};
  // Hard cap for bounded-memory attempt tracking.
  int max_attempt_records{256};
  // Hard cap for bounded-memory region tracking.
  int max_regions{64};
  // Quantization scale reused from frontier equivalence tolerance.
  double equivalence_tolerance{0.30};
};

// Quantized key used to collapse near-identical goal points into one attempt bucket.
struct SuppressionKey
{
  int64_t x_bucket{0};
  int64_t y_bucket{0};

  bool operator==(const SuppressionKey & other) const noexcept
  {
    return x_bucket == other.x_bucket && y_bucket == other.y_bucket;
  }
};

struct SuppressionKeyHash
{
  std::size_t operator()(const SuppressionKey & key) const noexcept;
};

// Temporary failure-memory record for one quantized goal area.
struct SuppressionAttempt
{
  std::pair<double, double> goal_point;
  int failure_count{0};
  int64_t last_updated_ns{0};
};

// Active square suppression region stored in world coordinates.
struct SuppressedRegion
{
  std::pair<double, double> center;
  double side_length_m{0.0};
  int64_t last_updated_ns{0};
};

// Per-dispatch progress bookkeeping used by the no-progress watchdog.
struct ActiveGoalProgress
{
  int dispatch_id{0};
  bool has_feedback{false};
  double best_distance_remaining{0.0};
  double distance_at_last_progress{0.0};
  int64_t last_feedback_ns{0};
  int64_t last_meaningful_progress_ns{0};
  bool cancel_requested_by_timeout{false};
};

// Internal bounded-memory state for suppression policy.
// The containers are reserved once and then kept capped to avoid unbounded growth.
struct FrontierSuppressionState
{
  std::unordered_map<SuppressionKey, SuppressionAttempt, SuppressionKeyHash> attempts;
  std::vector<SuppressedRegion> regions;
  std::optional<ActiveGoalProgress> active_goal_progress;
};

// Owns the failure-memory policy behind optional frontier suppression.
// The core delegates filtering, promotion, expiration, and no-progress tracking here
// so exploration orchestration can stay focused on goal lifecycle flow.
class FrontierSuppression
{
public:
  explicit FrontierSuppression(FrontierSuppressionConfig config);

  std::size_t attempt_count() const noexcept;
  std::size_t region_count() const noexcept;
  const std::vector<SuppressedRegion> & regions() const noexcept;
  const std::unordered_map<SuppressionKey, SuppressionAttempt, SuppressionKeyHash> & attempts() const noexcept;

  // Drop expired attempt and region entries using the shared suppression TTL.
  void prune_expired(int64_t now_ns);
  // Remove frontier candidates whose selected goal points fall inside a suppressed square region.
  FrontierSequence filter_frontiers(const FrontierSequence & frontiers) const;
  // Record one failed attempt and promote it into a suppression region once the threshold is reached.
  void record_failed_attempt(
    const FrontierLike & frontier,
    int64_t now_ns,
    const std::function<void(const std::string &)> & log_warn = {});

  // Begin tracking a frontier goal for no-progress timeout evaluation.
  void start_goal_progress_tracking(int dispatch_id, int64_t now_ns);
  void clear_goal_progress_tracking();
  // Feed distance_remaining samples into the progress tracker.
  void note_goal_progress(int dispatch_id, double distance_remaining, int64_t now_ns);
  // Helper for callers that need to know whether the current dispatch is already being watched.
  bool is_tracking_dispatch(int dispatch_id) const noexcept;
  // Mark the current dispatch for timeout-driven cancellation once the stall window expires.
  bool mark_timeout_cancel_if_needed(int dispatch_id, int64_t now_ns);
  bool progress_timeout_cancel_requested() const noexcept;

private:
  // Convert a goal point into a stable hashable bucket.
  SuppressionKey make_key(const std::pair<double, double> & goal_point) const;
  // Returns the effective quantization scale used by SuppressionKey.
  double quantum() const;
  // Inclusive square-membership test for a region's current side length.
  bool point_in_region(const std::pair<double, double> & point, const SuppressedRegion & region) const;
  // Tests whether a point lies in the outer expansion band but not the current inner square.
  bool point_in_expansion_ring(
    const std::pair<double, double> & point,
    const SuppressedRegion & region) const;
  // Checks whether a point is currently hidden by any live suppression region.
  bool point_in_any_region(const std::pair<double, double> & point) const;
  // Cleanup helper to drop now-redundant attempts after region creation or expansion.
  void remove_attempts_inside_region(const SuppressedRegion & region);
  // Capacity guard for the attempt hash map.
  void evict_oldest_attempt_record(const std::function<void(const std::string &)> & log_warn);
  // Capacity guard for the region store.
  void evict_oldest_region(const std::function<void(const std::string &)> & log_warn);
  // Promote a matured failure record into a new or expanded square suppression region.
  void promote_attempt_to_region(
    const std::pair<double, double> & goal_point,
    int64_t now_ns,
    const std::function<void(const std::string &)> & log_warn);

  FrontierSuppressionConfig config_;
  FrontierSuppressionState state_;
};

}  // namespace frontier_exploration_ros2
