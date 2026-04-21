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

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "frontier_exploration_ros2/frontier_suppression.hpp"
#include "frontier_exploration_ros2/frontier_policy.hpp"
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "frontier_exploration_ros2/decision_map.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"
#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

// Minimal adapter interface that allows core tests to mock action goal handles.
class GoalHandleInterface
{
public:
  virtual ~GoalHandleInterface() = default;
  virtual void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) = 0;
};

// Immutable payload sent from core to node when dispatching an action goal.
struct GoalDispatchRequest
{
  int dispatch_id{0};
  std::string action_name;
  std::string goal_kind;
  geometry_msgs::msg::PoseStamped goal_pose;
  std::optional<FrontierLike> frontier;
  FrontierSequence frontier_sequence;
  std::string description;
};

// Runtime behavior knobs for frontier selection, goal updates, and settle policy.
struct FrontierExplorerCoreParams
{
  std::string map_topic{"map"};
  std::string costmap_topic{"global_costmap/costmap"};
  std::string local_costmap_topic{"local_costmap/costmap"};
  std::string navigate_to_pose_action_name{"navigate_to_pose"};
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  std::string frontier_marker_topic{"explore/frontiers"};
  std::string selected_frontier_topic{"explore/selected_frontier"};
  std::string optimized_map_topic{"explore/optimized_map"};
  FrontierStrategy strategy{FrontierStrategy::NEAREST};
  double frontier_marker_scale{0.15};
  bool frontier_map_optimization_enabled{true};
  double sigma_s{2.0};
  double sigma_r{30.0};
  int dilation_kernel_radius_cells{1};
  double sensor_effective_range_m{1.5};
  double weight_distance_wd{1.0};
  double weight_gain_ws{1.0};
  double max_linear_speed_vmax{0.5};
  double max_angular_speed_wmax{1.0};
  int occ_threshold{OCC_THRESHOLD};
  int min_frontier_size_cells{MIN_FRONTIER_SIZE};
  double frontier_candidate_min_goal_distance_m{0.0};
  double frontier_selection_min_distance{0.5};
  double frontier_visit_tolerance{0.30};
  bool goal_preemption_enabled{false};
  bool goal_skip_on_blocked_goal{false};
  double goal_preemption_min_interval_s{2.0};
  // LiDAR-model geometry used only by the visible-gain helper at the target pose.
  // Maximum ray length for the target-pose visibility estimate.
  double goal_preemption_lidar_range_m{12.0};
  // Angular coverage for the target-pose visibility estimate.
  double goal_preemption_lidar_fov_deg{360.0};
  // Angular sampling density for the target-pose ray-cast estimate.
  double goal_preemption_lidar_ray_step_deg{1.0};
  // Independent near-goal completion shortcut for revealed-preemption paths.
  double goal_preemption_complete_if_within_m{0.0};
  // Minimum occlusion-aware reveal length required to keep the current goal.
  double goal_preemption_lidar_min_reveal_length_m{0.5};
  // Optional heading correction for the target-pose sensor model.
  double goal_preemption_lidar_yaw_offset_deg{0.0};
  bool escape_enabled{true};
  bool post_goal_settle_enabled{true};
  double post_goal_min_settle{0.80};
  int post_goal_required_map_updates{3};
  int post_goal_stable_updates{2};
  bool return_to_start_on_complete{true};
  std::string all_frontiers_suppressed_behavior{"stay"};
  bool frontier_suppression_enabled{false};
  int frontier_suppression_attempt_threshold{3};
  double frontier_suppression_base_size_m{1.0};
  double frontier_suppression_expansion_size_m{0.5};
  double frontier_suppression_timeout_s{90.0};
  double frontier_suppression_no_progress_timeout_s{20.0};
  double frontier_suppression_progress_epsilon_m{0.05};
  double frontier_suppression_startup_grace_period_s{15.0};
  int frontier_suppression_max_attempt_records{256};
  int frontier_suppression_max_regions{64};
};

// Host callbacks injected by the node wrapper (time, TF pose, action transport, logging).
struct FrontierExplorerCoreCallbacks
{
  std::function<int64_t()> now_ns;
  std::function<std::optional<geometry_msgs::msg::Pose>()> get_current_pose;
  std::function<bool(double timeout_sec)> wait_for_action_server;
  std::function<void(const GoalDispatchRequest &)> dispatch_goal_request;
  std::function<void(const FrontierSequence &)> publish_frontier_markers;
  std::function<void(const geometry_msgs::msg::PoseStamped &)> publish_selected_frontier_pose;
  std::function<void(const nav_msgs::msg::OccupancyGrid &)> publish_optimized_map;
  // Completion hook lets the ROS-facing node trigger optional post-completion side effects.
  std::function<void()> on_exploration_complete;
  std::function<bool()> debug_outputs_enabled;
  std::function<void(const std::string &)> log_debug;
  std::function<void(const std::string &)> log_info;
  std::function<void(const std::string &)> log_warn;
  std::function<void(const std::string &)> log_error;
  std::function<FrontierSearchResult(
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)> frontier_search;
};

// Stateful exploration controller independent from ROS transport details.
class FrontierExplorerCore
{
public:
  explicit FrontierExplorerCore(
    FrontierExplorerCoreParams params,
    FrontierExplorerCoreCallbacks callbacks);

  void occupancyGridCallback(const OccupancyGrid2d & map_msg);
  void costmapCallback(const OccupancyGrid2d & map_msg);
  void localCostmapCallback(const OccupancyGrid2d & map_msg);

  [[nodiscard]] bool mrtsp_enabled() const;
  [[nodiscard]] bool frontier_map_optimization_enabled() const;
  [[nodiscard]] FrontierSearchOptions frontier_search_options() const;
  [[nodiscard]] DecisionMapConfig decision_map_config() const;
  void refresh_decision_map();
  FrontierSequence build_mrtsp_frontier_sequence(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  void try_send_next_goal();

  std::pair<double, double> frontier_position(const FrontierLike & frontier) const;
  std::pair<double, double> frontier_reference_point(const FrontierLike & frontier) const;
  int frontier_size(const FrontierLike & frontier) const;
  std::string describe_frontier(const FrontierLike & frontier) const;
  FrontierSignature frontier_signature(const FrontierSequence & frontiers) const;

  bool frontier_snapshot_matches(
    const std::optional<FrontierSnapshot> & snapshot,
    const std::pair<int, int> & robot_map_cell,
    double min_goal_distance) const;

  FrontierSnapshot get_frontier_snapshot(
    const geometry_msgs::msg::Pose & current_pose,
    double min_goal_distance);

  void start_post_goal_settle();
  void wait_for_next_map_refresh();
  void clear_post_goal_wait_state();
  void observe_post_goal_settle_update(bool refresh_frontier_signature = true);
  bool post_goal_settle_ready() const;

  FrontierSelectionResult select_primitive_frontier(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  FrontierSelectionResult select_frontier(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  void record_start_pose(const geometry_msgs::msg::Pose & current_pose);

  bool are_frontiers_equivalent(
    const std::optional<FrontierLike> & first_frontier,
    const std::optional<FrontierLike> & second_frontier) const;

  bool frontier_exists_in_set(
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontiers) const;

  std::optional<std::string> frontier_cost_status(
    const std::optional<FrontierLike> & frontier) const;

  geometry_msgs::msg::PoseStamped build_goal_pose(
    const FrontierLike & target_frontier,
    const geometry_msgs::msg::Pose & current_pose,
    const std::optional<FrontierLike> & look_ahead_frontier = std::nullopt) const;

  std::vector<geometry_msgs::msg::PoseStamped> build_goal_pose_sequence(
    const FrontierSequence & target_frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  FrontierSequence select_frontier_sequence(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose,
    const std::optional<FrontierLike> & initial_frontier) const;

  bool are_frontier_sequences_equivalent(
    const FrontierSequence & first_frontier_sequence,
    const FrontierSequence & second_frontier_sequence) const;

  void set_goal_state(GoalLifecycleState state);
  void mark_dispatch_state(int dispatch_id, GoalLifecycleState state);

  void reset_replacement_candidate_tracking();
  bool has_stable_replacement_candidate(const FrontierSequence & frontier_sequence);
  // Reprojects the active goal into a hypothetical sensor pose and estimates the
  // still-visible frontier length from there; nullopt means "fall back to snapshot logic".
  std::optional<double> active_goal_visible_reveal_length() const;

  void consider_preempt_active_goal(const std::string & trigger_source = "map");

  void request_frontier_reselection(
    const FrontierSequence & frontier_sequence,
    const geometry_msgs::msg::Pose & current_pose,
    const std::string & selection_mode,
    const std::string & reselection_reason,
    const std::string & goal_update_log_prefix = "Preempting active frontier goal");

  void request_active_goal_cancel(const std::string & reason);
  void issue_active_goal_cancel();
  void cancel_response_callback(
    int dispatch_id,
    bool cancel_accepted,
    const std::string & error_message = "");

  bool dispatch_pending_frontier_goal(
    const std::optional<geometry_msgs::msg::Pose> & current_pose = std::nullopt);

  void start_exploration_session();
  void stop_exploration_session(const std::string & reason = "Stopping exploration session");
  [[nodiscard]] bool ready_for_shutdown() const;

  void handle_exploration_complete(const geometry_msgs::msg::Pose & current_pose);
  void handle_all_frontiers_suppressed(const geometry_msgs::msg::Pose & current_pose);
  void consider_cancel_suppressed_return_to_start();

  bool is_pose_within_xy_tolerance(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    double tolerance = 0.25) const;

  bool send_pose_goal(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & goal_kind,
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontier_sequence,
    const std::string & description);

  bool send_frontier_goal(
    const FrontierSequence & frontier_sequence,
    const geometry_msgs::msg::Pose & current_pose,
    const std::string & description);

  void dispatch_goal_request(
    const std::string & action_name,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & goal_kind,
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontier_sequence,
    const std::string & description);

  void clear_active_goal_state();

  void goal_response_callback(
    int dispatch_id,
    const std::shared_ptr<GoalHandleInterface> & received_goal_handle,
    bool accepted,
    const std::string & error_message = "");

  void get_result_callback(
    int dispatch_id,
    int status,
    int error_code,
    const std::string & error_message,
    const std::string & exception_text = "");

  void feedback_callback(double distance_remaining, int dispatch_id);
  bool evaluate_active_goal_progress_timeout();

  void publish_frontier_markers(const FrontierSequence & frontiers);

  void request_shutdown();

  static FrontierSequence to_frontier_sequence(
    const std::vector<FrontierCandidate> & frontiers);

  bool suppression_state_allocated() const
  {
    return static_cast<bool>(frontier_suppression_);
  }

  std::size_t suppression_attempt_count() const
  {
    return frontier_suppression_ ? frontier_suppression_->attempt_count() : 0U;
  }

  std::size_t suppressed_region_count() const
  {
    return frontier_suppression_ ? frontier_suppression_->region_count() : 0U;
  }

  // Configuration and host integration hooks.
  FrontierExplorerCoreParams params;
  FrontierExplorerCoreCallbacks callbacks;

  // Latest map/costmap snapshots and monotonic generation counters.
  std::optional<OccupancyGrid2d> map;
  std::optional<OccupancyGrid2d> decision_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr decision_map_msg;
  DecisionMapWorkspace decision_map_workspace;
  std::optional<OccupancyGrid2d> costmap;
  std::optional<OccupancyGrid2d> local_costmap;
  int map_generation{0};
  int decision_map_generation{0};
  int costmap_generation{0};
  int local_costmap_generation{0};
  int decision_map_cache_hits{0};
  int decision_map_cache_misses{0};

  // Frontier snapshot cache keyed by generation + robot cell + min distance.
  std::optional<FrontierSnapshot> frontier_snapshot;
  int frontier_snapshot_cache_hits{0};
  int frontier_snapshot_cache_misses{0};
  double frontier_stats_log_throttle_seconds{2.0};
  std::optional<int64_t> last_frontier_stats_log_time_ns;

  struct RawFrontierDebugCacheEntry
  {
    int map_generation{0};
    int costmap_generation{0};
    int local_costmap_generation{0};
    std::pair<int, int> robot_map_cell{0, 0};
    double min_goal_distance{0.0};
    FrontierStrategy strategy{FrontierStrategy::NEAREST};
    FrontierSearchOptions search_options;
    std::size_t frontier_count{0};
  };
  std::optional<RawFrontierDebugCacheEntry> raw_frontier_debug_cache;

  struct MrtspOrderCacheEntry
  {
    FrontierSignature frontier_signature;
    int pose_x_bucket{0};
    int pose_y_bucket{0};
    int yaw_bucket{0};
    double sensor_effective_range_m{0.0};
    double weight_distance_wd{0.0};
    double weight_gain_ws{0.0};
    double max_linear_speed_vmax{0.0};
    double max_angular_speed_wmax{0.0};
    FrontierSequence frontier_sequence;
  };
  std::optional<MrtspOrderCacheEntry> mrtsp_order_cache;
  int mrtsp_order_cache_hits{0};
  int mrtsp_order_cache_misses{0};

  // Active action/goal lifecycle state.
  std::shared_ptr<GoalHandleInterface> goal_handle;
  std::optional<geometry_msgs::msg::PoseStamped> start_pose;
  std::optional<geometry_msgs::msg::PoseStamped> active_goal_pose;
  std::optional<FrontierLike> active_goal_frontier;
  FrontierSequence active_goal_frontiers;
  std::string active_goal_kind;
  std::string active_action_name;
  std::optional<int64_t> active_goal_sent_time_ns;
  bool goal_in_progress{false};
  GoalLifecycleState goal_state{GoalLifecycleState::IDLE};
  bool exploration_enabled{true};
  int current_dispatch_id{0};
  std::unordered_map<int, GoalLifecycleState> dispatch_states;

  // Exploration completion/reporting flags.
  bool no_frontiers_reported{false};
  bool no_reachable_frontier_reported{false};
  bool all_frontiers_suppressed_reported{false};

  // Post-goal settle and map-refresh gating state.
  bool awaiting_map_refresh{false};
  bool map_updated{false};
  bool post_goal_settle_active{false};
  std::optional<int64_t> post_goal_settle_started_at_ns;
  int post_goal_map_updates_seen{0};
  int post_goal_stable_update_count{0};
  std::optional<FrontierSignature> post_goal_last_frontier_signature;

  // Escape and return-to-start behavior flags.
  bool escape_active{true};
  bool return_to_start_started{false};
  bool return_to_start_completed{false};
  bool suppressed_return_to_start_started{false};

  // Preemption/cancel pipeline and pending replacement goal state.
  bool cancel_request_in_progress{false};
  std::optional<std::string> pending_cancel_reason;
  FrontierSequence pending_frontier_sequence;
  std::string pending_frontier_selection_mode;
  std::string pending_frontier_dispatch_context;
  std::optional<std::string> active_goal_blocked_reason;

  // Replacement frontier debouncing and marker deduplication state.
  std::optional<FrontierLike> replacement_candidate_frontier;
  int replacement_candidate_hits{0};
  int replacement_required_hits{2};
  std::optional<FrontierSignature> last_published_frontier_signature;
  std::unique_ptr<FrontierSuppression> frontier_suppression_;
  std::optional<int64_t> frontier_suppression_activation_ns_;

  // Shutdown guard for callbacks racing during teardown.
  bool shutdown_requested{false};

private:
  struct DispatchContext
  {
    std::string goal_kind;
    std::optional<FrontierLike> frontier;
    FrontierSequence frontier_sequence;
    std::string action_name;
  };

  std::unordered_map<int, DispatchContext> dispatch_contexts;

  bool debug_outputs_enabled() const;
  void throttled_debug(const std::string & message);
  void log_frontier_snapshot_stats(
    const FrontierSequence & frontiers,
    double duration_ms,
    bool cache_hit);
  FrontierSuppression * ensure_frontier_suppression();
  bool suppression_enabled() const;
  bool suppression_runtime_active(int64_t now_ns) const;
  bool should_return_to_start_when_all_frontiers_suppressed() const;
  FrontierSequence filter_frontiers_for_suppression(const FrontierSequence & frontiers);
  void record_failed_frontier_attempt(const std::optional<FrontierLike> & frontier);
  void clear_active_goal_progress_state();
  void start_active_goal_progress_tracking();
  void note_active_goal_progress(double distance_remaining);
  void reset_exploration_runtime_state(bool clear_maps);

  std::optional<DispatchContext> dispatch_context_for(int dispatch_id) const;
};

}  // namespace frontier_exploration_ros2
