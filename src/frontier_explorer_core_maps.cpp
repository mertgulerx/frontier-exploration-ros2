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

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::refresh_decision_map()
{
  if (!map.has_value()) {
    return;
  }

  const auto previous_decision_map_msg = decision_map_msg;
  const DecisionMapBuildStatus status = build_decision_map(
    *map,
    decision_map_config(),
    decision_map_workspace);
  decision_map_msg = decision_map_workspace.optimized_map_msg;
  if (
    !decision_map.has_value() ||
    previous_decision_map_msg != decision_map_msg ||
    status.geometry_changed)
  {
    decision_map = OccupancyGrid2d(decision_map_msg);
  }

  if (status.output_changed || !decision_map.has_value()) {
    decision_map_generation += 1;
    decision_map_cache_misses += 1;
    frontier_snapshot.reset();
    if (debug_outputs_enabled() && decision_map_msg) {
      callbacks.publish_optimized_map(*decision_map_msg);
      callbacks.log_debug(
        "decision_map: miss, raw_generation=" + std::to_string(map_generation) +
        ", decision_generation=" + std::to_string(decision_map_generation) +
        ", optimization_enabled=" +
        std::string(frontier_map_optimization_enabled() ? "true" : "false"));
    }
    return;
  }

  decision_map_cache_hits += 1;
  if (debug_outputs_enabled()) {
    callbacks.log_debug(
      "decision_map: hit, raw_generation=" + std::to_string(map_generation) +
      ", decision_generation=" + std::to_string(decision_map_generation) +
      ", optimization_enabled=" +
      std::string(frontier_map_optimization_enabled() ? "true" : "false"));
  }
}

void FrontierExplorerCore::occupancyGridCallback(const OccupancyGrid2d & map_msg)
{
  // Map updates are authoritative for frontier discovery and settle progression.
  map = map_msg;
  // Generation is monotonic and used to invalidate frontier snapshots.
  map_generation += 1;
  refresh_decision_map();
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

}  // namespace frontier_exploration_ros2
