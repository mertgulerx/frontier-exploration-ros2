#include "frontier_exploration_ros2/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdint>
#include <limits>

namespace frontier_exploration_ros2
{

namespace
{

// 8-connected neighborhood plus center cell for local scans.
constexpr int kNeighborOffsets[9][2] = {
  {-1, -1},
  {-1, 0},
  {-1, 1},
  {0, -1},
  {0, 0},
  {0, 1},
  {1, -1},
  {1, 0},
  {1, 1},
};

constexpr int classification_flag(PointClassification classification) noexcept
{
  // Enum values are bit-flags; convert once for branch-friendly bit operations.
  return static_cast<int>(classification);
}

inline bool has_classification(
  const FrontierPoint * point,
  PointClassification classification) noexcept
{
  // Standard mask test: true if the requested bit is set.
  return (point->classification & classification_flag(classification)) != 0;
}

inline void set_classification(
  FrontierPoint * point,
  PointClassification classification) noexcept
{
  // Monotonic set operation; flags are never toggled off inside one pass.
  point->classification |= classification_flag(classification);
}

template<typename Fn>
void for_each_neighbor(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache,
  Fn && fn)
{
  // Neighbor iteration stays allocation-free in hot paths.
  const int size_x = occupancy_map.getSizeX();
  const int size_y = occupancy_map.getSizeY();
  for (const auto & offset : kNeighborOffsets) {
    const int x = point->mapX + offset[0];
    const int y = point->mapY + offset[1];
    if (x < 0 || y < 0 || x >= size_x || y >= size_y) {
      continue;
    }
    fn(frontier_cache.getPoint(x, y));
  }
}

std::vector<FrontierPoint *> iter_neighbors(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  std::vector<FrontierPoint *> neighbors;
  neighbors.reserve(9);
  for_each_neighbor(point, occupancy_map, frontier_cache, [&neighbors](FrontierPoint * neighbor) {
    neighbors.push_back(neighbor);
  });
  return neighbors;
}

}  // namespace

FrontierSearchContext::FrontierSearchContext(
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap)
: occupancy_map_(occupancy_map),
  costmap_(costmap),
  local_costmap_(local_costmap),
  size_x_(occupancy_map.getSizeX()),
  size_y_(occupancy_map.getSizeY()),
  global_costmap_aligned_(occupancy_map.isGeometryAlignedWith(costmap)),
  local_costmap_aligned_(
    local_costmap.has_value() && occupancy_map.isGeometryAlignedWith(*local_costmap))
{
  // Caches are sized once per pass and reused via generation stamps.
  const std::size_t cell_count =
    static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_);
  map_to_world_cache_.assign(cell_count, {0.0, 0.0});
  map_to_world_cache_stamp_.assign(cell_count, 0);
  global_cost_blocked_cache_.assign(cell_count, 0);
  global_cost_blocked_cache_stamp_.assign(cell_count, 0);
  local_cost_blocked_cache_.assign(cell_count, 0);
  local_cost_blocked_cache_stamp_.assign(cell_count, 0);
  frontier_eligibility_cache_.assign(cell_count, 0);
  frontier_eligibility_cache_stamp_.assign(cell_count, 0);
  accessible_cell_stamp_.assign(cell_count, 0);
}

std::size_t FrontierSearchContext::cell_index(int map_x, int map_y) const
{
  return static_cast<std::size_t>(map_y) * static_cast<std::size_t>(size_x_) +
         static_cast<std::size_t>(map_x);
}

void FrontierSearchContext::advance_generation(
  std::vector<uint32_t> & stamps,
  uint32_t & generation)
{
  // Generation bump invalidates previous cache without touching all elements.
  if (generation == std::numeric_limits<uint32_t>::max()) {
    // Rare wrap-around path: explicit clear to restore stamp invariants.
    std::fill(stamps.begin(), stamps.end(), 0);
    generation = 1;
    return;
  }
  ++generation;
}

void FrontierSearchContext::begin_candidate_accessible_scan()
{
  advance_generation(accessible_cell_stamp_, accessible_cell_generation_);
}

bool FrontierSearchContext::mark_accessible_cell_once(int map_x, int map_y)
{
  const std::size_t idx = cell_index(map_x, map_y);
  // Duplicate suppression inside one candidate-materialization pass.
  if (accessible_cell_stamp_[idx] == accessible_cell_generation_) {
    return false;
  }
  accessible_cell_stamp_[idx] = accessible_cell_generation_;
  return true;
}

std::pair<double, double> FrontierSearchContext::world_point(int map_x, int map_y)
{
  // map_to_world_cache_[i] is valid iff stamp[i] == current generation.
  const std::size_t idx = cell_index(map_x, map_y);
  if (map_to_world_cache_stamp_[idx] == map_to_world_generation_) {
    return map_to_world_cache_[idx];
  }

  map_to_world_cache_stamp_[idx] = map_to_world_generation_;
  map_to_world_cache_[idx] = occupancy_map_.mapToWorld(map_x, map_y);
  return map_to_world_cache_[idx];
}

bool FrontierSearchContext::global_cost_blocked(int map_x, int map_y)
{
  // Cost-blocked cache stores bool-as-byte; stamp controls validity.
  const std::size_t idx = cell_index(map_x, map_y);
  if (global_cost_blocked_cache_stamp_[idx] == global_cost_blocked_generation_) {
    return global_cost_blocked_cache_[idx] != 0;
  }

  const bool blocked = is_cost_blocked(costmap_, global_costmap_aligned_, map_x, map_y);
  global_cost_blocked_cache_stamp_[idx] = global_cost_blocked_generation_;
  global_cost_blocked_cache_[idx] = blocked ? 1 : 0;
  return blocked;
}

bool FrontierSearchContext::local_cost_blocked(int map_x, int map_y)
{
  const std::size_t idx = cell_index(map_x, map_y);
  if (local_cost_blocked_cache_stamp_[idx] == local_cost_blocked_generation_) {
    return local_cost_blocked_cache_[idx] != 0;
  }

  const bool blocked = is_cost_blocked(local_costmap_, local_costmap_aligned_, map_x, map_y);
  local_cost_blocked_cache_stamp_[idx] = local_cost_blocked_generation_;
  local_cost_blocked_cache_[idx] = blocked ? 1 : 0;
  return blocked;
}

bool FrontierSearchContext::is_cost_blocked(
  const std::optional<OccupancyGrid2d> & costmap,
  bool aligned_with_occupancy,
  int map_x,
  int map_y)
{
  if (!costmap.has_value()) {
    // Missing costmap is treated as "no additional blocking information".
    return false;
  }

  if (aligned_with_occupancy) {
    // Fast path when occupancy and costmap share geometry.
    if (
      map_x < 0 || map_y < 0 ||
      map_x >= costmap->getSizeX() || map_y >= costmap->getSizeY())
    {
      // Out-of-range in aligned space means "unknown to this costmap", not blocked.
      return false;
    }
    return costmap->getCost(map_x, map_y) > OCC_THRESHOLD;
  }

  const auto world = world_point(map_x, map_y);
  // Fallback for mismatched map geometry: map cell -> world -> costmap cell.
  int cost_x = 0;
  int cost_y = 0;
  if (!costmap->worldToMapNoThrow(world.first, world.second, cost_x, cost_y)) {
    // If transformed point is outside costmap bounds, do not force-block it.
    return false;
  }
  return costmap->getCost(cost_x, cost_y) > OCC_THRESHOLD;
}

std::pair<double, double> centroid(const std::vector<std::pair<double, double>> & arr)
{
  // Centroid equation:
  //   cx = (1/N) * sum(x_i), cy = (1/N) * sum(y_i)
  const std::size_t length = arr.size();
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & point : arr) {
    sum_x += point.first;
    sum_y += point.second;
  }
  return {sum_x / static_cast<double>(length), sum_y / static_cast<double>(length)};
}

double squared_distance(
  const std::pair<double, double> & first_point,
  const std::pair<double, double> & second_point)
{
  // d^2 = (x1 - x2)^2 + (y1 - y2)^2
  // Using squared distance avoids sqrt in ranking comparisons.
  const double dx = first_point.first - second_point.first;
  const double dy = first_point.second - second_point.second;
  return (dx * dx) + (dy * dy);
}

std::optional<int> world_point_cost(
  const std::optional<OccupancyGrid2d> & costmap,
  const std::pair<double, double> & world_point)
{
  if (!costmap.has_value()) {
    return std::nullopt;
  }

  int costmap_x = 0;
  int costmap_y = 0;
  if (!costmap->worldToMapNoThrow(world_point.first, world_point.second, costmap_x, costmap_y)) {
    return std::nullopt;
  }
  return costmap->getCost(costmap_x, costmap_y);
}

bool is_world_point_blocked(
  const std::vector<std::optional<OccupancyGrid2d>> & costmaps,
  const std::pair<double, double> & world_point,
  int occ_threshold)
{
  // Multi-costmap policy: blocked if any map reports occupied over threshold.
  for (const auto & costmap : costmaps) {
    const auto cost = world_point_cost(costmap, world_point);
    if (cost.has_value() && *cost > occ_threshold) {
      return true;
    }
  }

  return false;
}

std::optional<std::pair<double, double>> choose_accessible_frontier_goal(
  const std::pair<double, double> & frontier_centroid,
  const std::vector<std::pair<int, int>> & accessible_cells,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<geometry_msgs::msg::Pose> & current_pose,
  double min_robot_distance)
{
  if (accessible_cells.empty()) {
    return std::nullopt;
  }

  // When a minimum robot distance is requested, prefer the nearest far-enough goal.
  const bool apply_min_distance = current_pose.has_value() && min_robot_distance > 0.0;
  const double min_robot_distance_sq = min_robot_distance * min_robot_distance;

  std::optional<std::pair<double, double>> best_any_point;
  // Distances are squared to avoid sqrt while preserving ordering.
  double best_any_distance = std::numeric_limits<double>::infinity();
  std::optional<std::pair<double, double>> best_far_point;
  double best_far_distance = std::numeric_limits<double>::infinity();

  for (const auto & [map_x, map_y] : accessible_cells) {
    const auto world = occupancy_map.mapToWorld(map_x, map_y);
    const double centroid_distance = squared_distance(world, frontier_centroid);
    if (centroid_distance < best_any_distance) {
      best_any_distance = centroid_distance;
      best_any_point = world;
    }

    if (apply_min_distance) {
      const double robot_dx = world.first - current_pose->position.x;
      const double robot_dy = world.second - current_pose->position.y;
      if ((robot_dx * robot_dx) + (robot_dy * robot_dy) >= min_robot_distance_sq &&
        centroid_distance < best_far_distance)
      {
        // Keep nearest point to centroid among points outside the robot-distance exclusion radius.
        best_far_distance = centroid_distance;
        best_far_point = world;
      }
    }
  }

  if (best_far_point.has_value()) {
    // Prefer far-enough candidate when min-distance gating is active.
    return best_far_point;
  }

  // Otherwise use best unconstrained candidate.
  return best_any_point;
}

std::optional<FrontierCandidate> build_frontier_candidate(
  const std::vector<FrontierPoint *> & new_frontier,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  const geometry_msgs::msg::Pose & current_pose,
  double min_goal_distance,
  FrontierSearchContext * search_context)
{
  if (new_frontier.size() <= static_cast<std::size_t>(MIN_FRONTIER_SIZE)) {
    // Reject tiny clusters; they are often noise at unknown/free boundaries.
    return std::nullopt;
  }

  FrontierSearchContext * context = search_context;
  std::optional<FrontierSearchContext> owned_context;
  if (context == nullptr) {
    if (!costmap.has_value()) {
      // Candidate construction depends on cost checks; no global costmap means no candidate.
      return std::nullopt;
    }
    owned_context.emplace(occupancy_map, *costmap, local_costmap);
    context = &(*owned_context);
  }

  // Compute centroid directly from frontier cells without allocating intermediate vectors.
  double centroid_sum_x = 0.0;
  double centroid_sum_y = 0.0;
  for (auto * frontier_point : new_frontier) {
    const auto world = context->world_point(frontier_point->mapX, frontier_point->mapY);
    centroid_sum_x += world.first;
    centroid_sum_y += world.second;
  }

  const std::pair<double, double> frontier_centroid{
    centroid_sum_x / static_cast<double>(new_frontier.size()),
    centroid_sum_y / static_cast<double>(new_frontier.size()),
  };

  context->begin_candidate_accessible_scan();

  const bool apply_min_goal_distance = min_goal_distance > 0.0;
  const double min_goal_distance_sq = min_goal_distance * min_goal_distance;
  std::optional<std::pair<double, double>> best_any_goal_point;
  std::optional<std::pair<double, double>> best_far_goal_point;
  double best_any_distance_sq = std::numeric_limits<double>::infinity();
  double best_far_distance_sq = std::numeric_limits<double>::infinity();

  for (auto * frontier_point : new_frontier) {
    // Candidate goal points are chosen from free, unblocked neighbors around frontier cells.
    for_each_neighbor(frontier_point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (!context->mark_accessible_cell_once(neighbor->mapX, neighbor->mapY)) {
        // Same accessible neighbor can be touched by multiple frontier cells.
        return;
      }

      if (
        occupancy_map.getCost(neighbor->mapX, neighbor->mapY) !=
        static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
      {
        // Only free neighbors are admissible navigation endpoints.
        return;
      }

      if (
        context->global_cost_blocked(neighbor->mapX, neighbor->mapY) ||
        context->local_cost_blocked(neighbor->mapX, neighbor->mapY))
      {
        // A blocked neighbor cannot be used as frontier goal candidate.
        return;
      }

      const auto world = context->world_point(neighbor->mapX, neighbor->mapY);
      const double centroid_distance_sq = squared_distance(world, frontier_centroid);
      if (centroid_distance_sq < best_any_distance_sq) {
        best_any_distance_sq = centroid_distance_sq;
        best_any_goal_point = world;
      }

      if (apply_min_goal_distance) {
        // Goal-distance gating uses squared radius:
        //   (x - xr)^2 + (y - yr)^2 >= min_goal_distance^2
        const double dx = world.first - current_pose.position.x;
        const double dy = world.second - current_pose.position.y;
        const double robot_distance_sq = (dx * dx) + (dy * dy);
        if (robot_distance_sq >= min_goal_distance_sq && centroid_distance_sq < best_far_distance_sq) {
          best_far_distance_sq = centroid_distance_sq;
          best_far_goal_point = world;
        }
      }
    });
  }

  const auto & goal_point = best_far_goal_point.has_value() ? best_far_goal_point : best_any_goal_point;
  if (!goal_point.has_value()) {
    // Cluster has no reachable free neighbor after cost filtering.
    return std::nullopt;
  }

  return FrontierCandidate{
    frontier_centroid,
    *goal_point,
    static_cast<int>(new_frontier.size()),
  };
}

std::pair<int, int> find_free(int mx, int my, const OccupancyGrid2d & occupancy_map)
{
  FrontierCache frontier_cache(occupancy_map.getSizeX(), occupancy_map.getSizeY());
  return find_free_with_cache(mx, my, occupancy_map, frontier_cache);
}

std::pair<int, int> find_free_with_cache(
  int mx,
  int my,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  // If robot starts in unknown/occupied cell, recover nearest reachable free seed for BFS.
  std::deque<FrontierPoint *> bfs;
  bfs.push_back(frontier_cache.getPoint(mx, my));

  while (!bfs.empty()) {
    FrontierPoint * location = bfs.front();
    bfs.pop_front();

    if (occupancy_map.getCost(location->mapX, location->mapY) == static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
      // First free hit in BFS radius is the nearest by grid distance.
      return {location->mapX, location->mapY};
    }

    for_each_neighbor(location, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (!has_classification(neighbor, PointClassification::MapClosed)) {
        // MapClosed acts as visited marker for this recovery BFS.
        set_classification(neighbor, PointClassification::MapClosed);
        bfs.push_back(neighbor);
      }
    });
  }

  return {mx, my};
}

FrontierSearchResult get_frontier(
  const geometry_msgs::msg::Pose & current_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double min_goal_distance,
  bool return_robot_cell)
{
  (void)return_robot_cell;

  // Two-level BFS:
  // 1) map queue expands navigable map area
  // 2) frontier queue grows each connected frontier cluster
  FrontierCache frontier_cache(occupancy_map.getSizeX(), occupancy_map.getSizeY());
  FrontierSearchContext search_context(occupancy_map, costmap, local_costmap);

  const auto [mx, my] = occupancy_map.worldToMap(current_pose.position.x, current_pose.position.y);
  // If robot projects into unknown space, find_free_with_cache nudges start to nearest free seed.
  const auto free_point = find_free_with_cache(mx, my, occupancy_map, frontier_cache);
  FrontierPoint * start = frontier_cache.getPoint(free_point.first, free_point.second);
  // Seed map BFS from a guaranteed free (or best effort) cell.
  start->classification = classification_flag(PointClassification::MapOpen);

  std::deque<FrontierPoint *> map_point_queue;
  map_point_queue.push_back(start);
  std::vector<FrontierCandidate> frontiers;

  while (!map_point_queue.empty()) {
    FrontierPoint * point = map_point_queue.front();
    map_point_queue.pop_front();

    if (has_classification(point, PointClassification::MapClosed)) {
      // Already fully processed from map queue.
      continue;
    }

    if (is_frontier_point(
        point,
        occupancy_map,
        costmap,
        local_costmap,
        frontier_cache,
        &search_context))
    {
      // Collect one connected frontier cluster.
      set_classification(point, PointClassification::FrontierOpen);
      std::deque<FrontierPoint *> frontier_queue;
      frontier_queue.push_back(point);
      std::vector<FrontierPoint *> new_frontier;

      while (!frontier_queue.empty()) {
        FrontierPoint * candidate = frontier_queue.front();
        frontier_queue.pop_front();

        if (
          has_classification(candidate, PointClassification::MapClosed) ||
          has_classification(candidate, PointClassification::FrontierClosed))
        {
          // Candidate already consumed by map or frontier phase.
          continue;
        }

        if (is_frontier_point(
            candidate,
            occupancy_map,
            costmap,
            local_costmap,
            frontier_cache,
            &search_context))
        {
          // Frontier BFS only expands cells that remain frontier-eligible under current maps.
          new_frontier.push_back(candidate);

          for_each_neighbor(candidate, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
            if (
              !has_classification(neighbor, PointClassification::FrontierOpen) &&
              !has_classification(neighbor, PointClassification::FrontierClosed) &&
              !has_classification(neighbor, PointClassification::MapClosed))
            {
              // FrontierOpen marks queued frontier candidates.
              set_classification(neighbor, PointClassification::FrontierOpen);
              frontier_queue.push_back(neighbor);
            }
          });
        }

        // FrontierClosed marks node as fully processed in this cluster-growth phase.
        set_classification(candidate, PointClassification::FrontierClosed);
      }

      for (auto * frontier_point : new_frontier) {
        // Prevent re-expansion from map queue once frontier cells are consumed.
        set_classification(frontier_point, PointClassification::MapClosed);
      }

      const auto frontier_candidate = build_frontier_candidate(
        new_frontier,
        occupancy_map,
        costmap,
        local_costmap,
        frontier_cache,
        current_pose,
        min_goal_distance,
        &search_context);

      if (frontier_candidate.has_value()) {
        // Preserve discovery order to keep output deterministic for downstream policy.
        frontiers.push_back(*frontier_candidate);
      }
    }

    // Continue map-space BFS through cells adjacent to free space.
    for_each_neighbor(point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (
        !has_classification(neighbor, PointClassification::MapOpen) &&
        !has_classification(neighbor, PointClassification::MapClosed))
      {
        bool has_free_neighbor = false;
        for_each_neighbor(neighbor, occupancy_map, frontier_cache, [&](FrontierPoint * candidate) {
          if (
            occupancy_map.getCost(candidate->mapX, candidate->mapY) ==
            static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
          {
            has_free_neighbor = true;
          }
        });

        if (has_free_neighbor) {
          // MapOpen marks node eligible for future map-queue expansion.
          set_classification(neighbor, PointClassification::MapOpen);
          map_point_queue.push_back(neighbor);
        }
      }
    });

    // MapClosed marks map-queue completion for this point.
    set_classification(point, PointClassification::MapClosed);
  }

  FrontierSearchResult result;
  result.frontiers = std::move(frontiers);
  result.robot_map_cell = {mx, my};
  return result;
}

std::vector<FrontierPoint *> get_neighbors(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  return iter_neighbors(point, occupancy_map, frontier_cache);
}

bool is_frontier_point(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  FrontierSearchContext * search_context)
{
  FrontierSearchContext * context = search_context;
  std::optional<FrontierSearchContext> owned_context;
  if (context == nullptr) {
    if (!costmap.has_value()) {
      return false;
    }
    owned_context.emplace(occupancy_map, *costmap, local_costmap);
    context = &(*owned_context);
  }

  // Frontier eligibility is cached per cell for the current search generation.
  const std::size_t cache_idx = context->cell_index(point->mapX, point->mapY);
  const auto cache_frontier_eligibility = [context, cache_idx](bool eligible) {
      context->frontier_eligibility_cache_stamp_[cache_idx] = context->frontier_eligibility_generation_;
      context->frontier_eligibility_cache_[cache_idx] = eligible ? 1 : 0;
    };

  if (context->frontier_eligibility_cache_stamp_[cache_idx] == context->frontier_eligibility_generation_) {
    // Fast path: frontier eligibility was already computed in this generation.
    return context->frontier_eligibility_cache_[cache_idx] != 0;
  }

  if (occupancy_map.getCost(point->mapX, point->mapY) != static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
    // Frontier candidates must be unknown cells bordering known free space.
    cache_frontier_eligibility(false);
    return false;
  }

  // A frontier point must touch free map space and must not be blocked by costmaps.
  bool blocked_neighbor = false;
  bool has_free_neighbor = false;
  for_each_neighbor(point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
    if (blocked_neighbor) {
      return;
    }

    const int map_cost = occupancy_map.getCost(neighbor->mapX, neighbor->mapY);

    if (context->global_cost_blocked(neighbor->mapX, neighbor->mapY)) {
      // Any blocked adjacent cell disqualifies this frontier candidate.
      blocked_neighbor = true;
      return;
    }

    if (context->local_cost_blocked(neighbor->mapX, neighbor->mapY)) {
      blocked_neighbor = true;
      return;
    }

    if (map_cost == static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
      has_free_neighbor = true;
    }
  });

  if (blocked_neighbor) {
    // Blocked adjacency takes precedence over free-neighbor condition.
    cache_frontier_eligibility(false);
    return false;
  }

  cache_frontier_eligibility(has_free_neighbor);
  return has_free_neighbor;
}

}  // namespace frontier_exploration_ros2
