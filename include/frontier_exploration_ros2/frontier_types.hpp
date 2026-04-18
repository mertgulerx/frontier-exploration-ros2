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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace frontier_exploration_ros2
{

// Occupancy threshold shared by frontier filtering and preemption checks.
constexpr int OCC_THRESHOLD = 50;
// Minimum raw cluster size required before frontier candidate materialization.
constexpr int MIN_FRONTIER_SIZE = 5;

// Lightweight convenience wrapper around nav_msgs/OccupancyGrid with map/world helpers.
class OccupancyGrid2d
{
public:
  enum class CostValues : int8_t
  {
    FreeSpace = 0,
    InscribedInflated = 100,
    LethalObstacle = 100,
    NoInformation = -1,
  };

  using GridMsg = nav_msgs::msg::OccupancyGrid;

  OccupancyGrid2d() = default;

  explicit OccupancyGrid2d(GridMsg::ConstSharedPtr grid)
  : map_(std::move(grid))
  {
  }

  explicit OccupancyGrid2d(GridMsg::SharedPtr grid)
  : map_(std::move(grid))
  {
  }

  explicit OccupancyGrid2d(const GridMsg & grid)
  : map_(std::make_shared<GridMsg>(grid))
  {
  }

  [[nodiscard]] const GridMsg & map() const
  {
    return checked_map();
  }

  [[nodiscard]] bool valid() const
  {
    return static_cast<bool>(map_);
  }

  [[nodiscard]] int getCost(int mx, int my) const
  {
    return static_cast<int>(checked_map().data[getIndex(mx, my)]);
  }

  [[nodiscard]] int getSizeX() const
  {
    return static_cast<int>(checked_map().info.width);
  }

  [[nodiscard]] int getSizeY() const
  {
    return static_cast<int>(checked_map().info.height);
  }

  [[nodiscard]] std::pair<double, double> mapToWorld(int mx, int my) const
  {
    const auto & info = checked_map().info;
    const double wx = info.origin.position.x + (static_cast<double>(mx) + 0.5) * info.resolution;
    const double wy = info.origin.position.y + (static_cast<double>(my) + 0.5) * info.resolution;
    return {wx, wy};
  }

  [[nodiscard]] std::pair<int, int> worldToMap(double wx, double wy) const
  {
    const auto & info = checked_map().info;
    if (wx < info.origin.position.x || wy < info.origin.position.y) {
      throw std::out_of_range("World coordinates out of bounds");
    }

    const int mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    const int my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    if (my < 0 || mx < 0 || my >= static_cast<int>(info.height) || mx >= static_cast<int>(info.width)) {
      throw std::out_of_range("World coordinates out of bounds");
    }

    return {mx, my};
  }

  [[nodiscard]] bool worldToMapNoThrow(double wx, double wy, int & mx, int & my) const noexcept
  {
    if (!map_) {
      return false;
    }

    const auto & info = map_->info;
    if (wx < info.origin.position.x || wy < info.origin.position.y) {
      return false;
    }

    const int maybe_mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    const int maybe_my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    if (
      maybe_my < 0 || maybe_mx < 0 ||
      maybe_my >= static_cast<int>(info.height) || maybe_mx >= static_cast<int>(info.width))
    {
      return false;
    }

    mx = maybe_mx;
    my = maybe_my;
    return true;
  }

  [[nodiscard]] bool isGeometryAlignedWith(const OccupancyGrid2d & other) const
  {
    constexpr double epsilon = 1e-9;
    const auto & this_info = checked_map().info;
    const auto & other_info = other.checked_map().info;
    return (
      std::abs(this_info.resolution - other_info.resolution) <= epsilon &&
      std::abs(this_info.origin.position.x - other_info.origin.position.x) <= epsilon &&
      std::abs(this_info.origin.position.y - other_info.origin.position.y) <= epsilon);
  }

private:
  [[nodiscard]] const GridMsg & checked_map() const
  {
    if (!map_) {
      throw std::logic_error("OccupancyGrid2d is not initialized");
    }
    return *map_;
  }

  [[nodiscard]] std::size_t getIndex(int mx, int my) const
  {
    return static_cast<std::size_t>(my) * checked_map().info.width + static_cast<std::size_t>(mx);
  }

  GridMsg::ConstSharedPtr map_;
};

class FrontierCache;

// Mutable BFS point state stored in FrontierCache.
struct FrontierPoint
{
  int classification{0};
  int mapX{0};
  int mapY{0};

  FrontierPoint() = default;
  FrontierPoint(int x, int y)
  : mapX(x), mapY(y)
  {
  }
};

// Cell-indexed cache with generation stamping for O(1) access without hash maps.
class FrontierCache
{
public:
  FrontierCache() = default;

  FrontierCache(int size_x, int size_y)
  {
    reset(size_x, size_y);
  }

  void reset(int size_x, int size_y)
  {
    if (size_x <= 0 || size_y <= 0) {
      size_x_ = 0;
      size_y_ = 0;
      points_.clear();
      point_generation_.clear();
      current_generation_ = 1;
      return;
    }

    const std::size_t cell_count =
      static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
    if (size_x_ != size_x || size_y_ != size_y || points_.size() != cell_count) {
      size_x_ = size_x;
      size_y_ = size_y;
      points_.assign(cell_count, FrontierPoint{});
      point_generation_.assign(cell_count, 0);
      current_generation_ = 1;
      return;
    }

    clear();
  }

  FrontierPoint * getPoint(int x, int y)
  {
    const std::size_t idx = indexOf(x, y);
    if (point_generation_[idx] != current_generation_) {
      point_generation_[idx] = current_generation_;
      FrontierPoint & point = points_[idx];
      point.classification = 0;
      point.mapX = x;
      point.mapY = y;
    }
    return &points_[idx];
  }

  void clear()
  {
    if (current_generation_ == std::numeric_limits<uint32_t>::max()) {
      std::fill(point_generation_.begin(), point_generation_.end(), 0);
      current_generation_ = 1;
      return;
    }
    ++current_generation_;
  }

private:
  [[nodiscard]] std::size_t indexOf(int x, int y) const
  {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(size_x_) +
           static_cast<std::size_t>(x);
  }

  int size_x_{0};
  int size_y_{0};
  std::vector<FrontierPoint> points_;
  std::vector<uint32_t> point_generation_;
  uint32_t current_generation_{1};
};

// Frontier output used by policy and goal dispatch.
struct FrontierCandidate
{
  FrontierCandidate() = default;

  // Compatibility constructor for existing nearest-style tests and simple callers.
  FrontierCandidate(
    std::pair<double, double> centroid_in,
    std::pair<double, double> goal_or_center_point_in,
    int size_in)
  : centroid(std::move(centroid_in)),
    center_point(goal_or_center_point_in),
    start_world_point(goal_or_center_point_in),
    goal_point(goal_or_center_point_in),
    size(size_in)
  {
  }

  FrontierCandidate(
    std::pair<double, double> centroid_in,
    std::pair<double, double> center_point_in,
    std::pair<int, int> center_cell_in,
    std::pair<int, int> start_cell_in,
    std::pair<double, double> start_world_point_in,
    std::optional<std::pair<double, double>> goal_point_in,
    int size_in)
  : centroid(std::move(centroid_in)),
    center_point(std::move(center_point_in)),
    center_cell(std::move(center_cell_in)),
    start_cell(std::move(start_cell_in)),
    start_world_point(std::move(start_world_point_in)),
    goal_point(std::move(goal_point_in)),
    size(size_in)
  {
  }

  // Mean world position of all cells in the frontier cluster.
  std::pair<double, double> centroid;
  // Center frontier cell projected to world coordinates; MRTSP dispatch baseline uses this point.
  std::pair<double, double> center_point;
  // Center frontier cell kept in map coordinates for geometry consistency and parity checks.
  std::pair<int, int> center_cell{0, 0};
  // First frontier cell observed while growing the cluster.
  std::pair<int, int> start_cell{0, 0};
  // start_cell converted to world coordinates for MRTSP path-cost calculations.
  std::pair<double, double> start_world_point;
  // Optional reachable goal point used by nearest-style navigation dispatch.
  std::optional<std::pair<double, double>> goal_point;
  // Number of cells in the underlying frontier cluster.
  int size{0};
};

enum class FrontierStrategy
{
  NEAREST,
  MRTSP,
};

// Primitive representation used by legacy/simple selection paths.
using PrimitiveFrontier = std::pair<double, double>;
// Unified frontier representation consumed by policy/core layers.
using FrontierLike = std::variant<PrimitiveFrontier, FrontierCandidate>;
using FrontierSequence = std::vector<FrontierLike>;
// Deterministic, quantized signature used for cache/publish equivalence checks.
using FrontierSignature = std::vector<std::array<int64_t, 4>>;

// Cached frontier snapshot keyed by map generations and robot cell.
struct FrontierSnapshot
{
  FrontierSequence frontiers;
  FrontierSignature signature;
  int map_generation{0};
  int decision_map_generation{0};
  int costmap_generation{0};
  int local_costmap_generation{0};
  std::pair<int, int> robot_map_cell{0, 0};
  double min_goal_distance{0.0};
  FrontierStrategy strategy{FrontierStrategy::NEAREST};
};

// Bit flags used while exploring map/frontier queues.
enum class PointClassification : int
{
  MapOpen = 1,
  MapClosed = 2,
  FrontierOpen = 4,
  FrontierClosed = 8,
};

// Logical action lifecycle tracked per active dispatch.
enum class GoalLifecycleState
{
  IDLE,
  SENDING,
  ACTIVE,
  SUPERSEDED,
  CANCELING,
};

// Selection output including reason label for logging/telemetry.
struct FrontierSelectionResult
{
  std::optional<FrontierLike> frontier;
  std::string mode;
};

}  // namespace frontier_exploration_ros2
