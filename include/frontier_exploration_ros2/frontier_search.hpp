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

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

// Output of a single frontier search pass.
struct FrontierSearchResult
{
  // Candidate frontiers ordered by discovery order in the BFS traversal.
  std::vector<FrontierCandidate> frontiers;
  // Robot cell on the occupancy map at search start.
  std::pair<int, int> robot_map_cell{0, 0};
};

// Visible reveal gain measured from a hypothetical sensor pose on the map.
struct VisibleRevealGain
{
  // First-hit reveal cells are deduplicated across rays, then converted to meters via map resolution.
  int visible_reveal_cell_count{0};
  // Convenience metric used directly by the preemption threshold.
  double visible_reveal_length_m{0.0};
};

struct FrontierSearchOptions
{
  int occ_threshold{OCC_THRESHOLD};
  int min_frontier_size_cells{MIN_FRONTIER_SIZE};
  double candidate_min_goal_distance_m{0.0};
  bool use_local_costmap_for_frontier_eligibility{true};
  bool out_of_bounds_costmap_is_blocked{false};
  bool build_navigation_goal_point{true};
};

// Scratch context for one search pass. Stores reusable caches keyed by map cell.
class FrontierSearchContext
{
public:
  FrontierSearchContext(
    const OccupancyGrid2d & occupancy_map,
    const OccupancyGrid2d & costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    FrontierSearchOptions search_options = {});

  // Lazily converts and caches occupancy cell to world coordinates.
  std::pair<double, double> world_point(int map_x, int map_y);
  // Returns whether the aligned or transformed global costmap cell is blocked.
  bool global_cost_blocked(int map_x, int map_y);
  // Returns whether the aligned or transformed local costmap cell is blocked.
  bool local_cost_blocked(int map_x, int map_y);
  // Starts a new dedup scope for candidate accessibility marking.
  void begin_candidate_accessible_scan();
  // Returns true only on the first mark in the current dedup scope.
  bool mark_accessible_cell_once(int map_x, int map_y);
  // Starts a new dedup scope for visible frontier accumulation.
  void begin_visible_frontier_scan();
  // Returns true only on the first visible frontier mark in the current scope.
  bool mark_visible_frontier_cell_once(int map_x, int map_y);

private:
  std::size_t cell_index(int map_x, int map_y) const;

  void advance_generation(std::vector<uint32_t> & stamps, uint32_t & generation);
  bool is_cost_blocked(
    const std::optional<OccupancyGrid2d> & costmap,
    bool aligned_with_occupancy,
    int map_x,
    int map_y);

  // Immutable map snapshots used during this search pass.
  const OccupancyGrid2d & occupancy_map_;
  std::optional<OccupancyGrid2d> costmap_;
  std::optional<OccupancyGrid2d> local_costmap_;
  int size_x_{0};
  int size_y_{0};
  bool global_costmap_aligned_{false};
  bool local_costmap_aligned_{false};

  // Cell-indexed caches with generation stamps to avoid per-pass clears.
  std::vector<std::pair<double, double>> map_to_world_cache_;
  std::vector<uint32_t> map_to_world_cache_stamp_;
  uint32_t map_to_world_generation_{1};

  std::vector<uint8_t> global_cost_blocked_cache_;
  std::vector<uint32_t> global_cost_blocked_cache_stamp_;
  uint32_t global_cost_blocked_generation_{1};

  std::vector<uint8_t> local_cost_blocked_cache_;
  std::vector<uint32_t> local_cost_blocked_cache_stamp_;
  uint32_t local_cost_blocked_generation_{1};

  std::vector<uint8_t> frontier_eligibility_cache_;
  std::vector<uint32_t> frontier_eligibility_cache_stamp_;
  uint32_t frontier_eligibility_generation_{1};

  std::vector<uint32_t> accessible_cell_stamp_;
  uint32_t accessible_cell_generation_{1};

  std::vector<uint32_t> visible_frontier_cell_stamp_;
  uint32_t visible_frontier_cell_generation_{1};
  FrontierSearchOptions search_options_;

  // Frontier eligibility helper needs direct access to stamp/cache internals.
  friend bool is_frontier_point(
    FrontierPoint * point,
    const OccupancyGrid2d & occupancy_map,
    const std::optional<OccupancyGrid2d> & costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    FrontierCache & frontier_cache,
    FrontierSearchContext * search_context);
};

// Computes centroid for a non-empty list of world points.
std::pair<double, double> centroid(const std::vector<std::pair<double, double>> & arr);

// Squared Euclidean distance helper for hot path comparisons.
double squared_distance(
  const std::pair<double, double> & first_point,
  const std::pair<double, double> & second_point);

// Looks up cost at world coordinates; returns nullopt when outside map bounds.
std::optional<int> world_point_cost(
  const std::optional<OccupancyGrid2d> & costmap,
  const std::pair<double, double> & world_point);

// Returns true if any provided costmap marks the world point as blocked.
bool is_world_point_blocked(
  const std::vector<std::optional<OccupancyGrid2d>> & costmaps,
  const std::pair<double, double> & world_point,
  int occ_threshold = OCC_THRESHOLD);

// Chooses the best reachable goal cell near centroid with optional min-robot-distance filter.
std::optional<std::pair<double, double>> choose_accessible_frontier_goal(
  const std::pair<double, double> & frontier_centroid,
  const std::vector<std::pair<int, int>> & accessible_cells,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<geometry_msgs::msg::Pose> & current_pose = std::nullopt,
  double min_robot_distance = 0.0);

// Materializes a candidate from raw frontier cells after accessibility/cost filtering.
std::optional<FrontierCandidate> build_frontier_candidate(
  const std::vector<FrontierPoint *> & new_frontier,
  const std::pair<int, int> & start_cell,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  const geometry_msgs::msg::Pose & current_pose,
  double min_goal_distance,
  const FrontierSearchOptions & options = {},
  FrontierSearchContext * search_context = nullptr);

// Finds nearest free occupancy cell from a seed cell.
std::pair<int, int> find_free(int mx, int my, const OccupancyGrid2d & occupancy_map);

// Cached variant of find_free that reuses a provided FrontierCache.
std::pair<int, int> find_free_with_cache(
  int mx,
  int my,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache);

// Runs full wavefront frontier extraction from the robot pose.
FrontierSearchResult get_frontier(
  const geometry_msgs::msg::Pose & current_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap = std::nullopt,
  double min_goal_distance = 0.0,
  bool return_robot_cell = false,
  const FrontierSearchOptions & options = {});

// Returns 8-neighborhood (plus center) using the shared cache storage.
std::vector<FrontierPoint *> get_neighbors(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache);

// Returns true if point is frontier-eligible under map and costmap constraints.
bool is_frontier_point(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  FrontierSearchContext * search_context = nullptr);

// Estimates how much frontier becomes directly visible from a target sensor pose.
// Rays stop at occupied map cells and only count frontier-eligible unknown cells, so
// wall-occluded unknown regions do not inflate the gain estimate.
std::optional<VisibleRevealGain> compute_visible_reveal_gain(
  const geometry_msgs::msg::Pose & sensor_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double range_m,
  double fov_deg,
  double ray_step_deg);

}  // namespace frontier_exploration_ros2
