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

#include "frontier_exploration_ros2/monte_carlo_gain_estimator.hpp"

#include "frontier_exploration_ros2/frontier_search.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <utility>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

bool costmap_blocks(
  const std::optional<OccupancyGrid2d> & costmap,
  const std::pair<double, double> & point,
  int occ_threshold)
{
  const auto cost = world_point_cost(costmap, point);
  return cost.has_value() && *cost > occ_threshold;
}

bool ray_visible(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::pair<double, double> & target,
  int target_mx,
  int target_my,
  int occ_threshold,
  const MonteCarloGainConfig & config)
{
  const double dx = target.first - pose.pose.position.x;
  const double dy = target.second - pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1e-9) {
    return true;
  }

  const double step = std::max(decision_map.map().info.resolution * 0.5, 1e-6);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int step_index = 1; step_index <= steps; ++step_index) {
    const double t = std::min(1.0, static_cast<double>(step_index) / static_cast<double>(steps));
    const double sample_x = pose.pose.position.x + dx * t;
    const double sample_y = pose.pose.position.y + dy * t;

    int map_x = 0;
    int map_y = 0;
    if (!decision_map.worldToMapNoThrow(sample_x, sample_y, map_x, map_y)) {
      return false;
    }

    const bool endpoint = map_x == target_mx && map_y == target_my;
    if (!endpoint && config.occupied_blocks_visibility) {
      const int map_cost = decision_map.getCost(map_x, map_y);
      if (map_cost > occ_threshold) {
        return false;
      }
    }

    if (!endpoint && config.costmap_blocks_visibility) {
      const std::pair<double, double> sample_point{sample_x, sample_y};
      if (
        costmap_blocks(global_costmap, sample_point, occ_threshold) ||
        costmap_blocks(local_costmap, sample_point, occ_threshold))
      {
        return false;
      }
    }
  }

  return true;
}

double information_value(
  int map_cost,
  int occ_threshold,
  const MonteCarloGainConfig & config)
{
  if (map_cost == static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
    return config.unknown_value;
  }
  if (map_cost > occ_threshold) {
    return config.occupied_value;
  }
  return config.free_value;
}

}  // namespace

MonteCarloGainEstimator::MonteCarloGainEstimator(MonteCarloGainConfig config)
: config_(std::move(config)),
  rng_(static_cast<std::uint32_t>(std::max(0, config_.deterministic_seed)))
{
}

void MonteCarloGainEstimator::resetSession()
{
  rng_.seed(static_cast<std::uint32_t>(std::max(0, config_.deterministic_seed)));
}

std::string normalize_monte_carlo_gain_mode(std::string mode)
{
  std::transform(
    mode.begin(),
    mode.end(),
    mode.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (mode == "candidate_pose") {
    return mode;
  }
  return "viewpoint_pose";
}

MonteCarloGainResult MonteCarloGainEstimator::evaluatePose(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const geometry_msgs::msg::PoseStamped & pose,
  int occ_threshold)
{
  MonteCarloGainResult result;
  result.samples_requested = std::max(1, config_.sample_count);

  if (!config_.enabled) {
    return result;
  }

  int origin_x = 0;
  int origin_y = 0;
  if (!decision_map.worldToMapNoThrow(pose.pose.position.x, pose.pose.position.y, origin_x, origin_y)) {
    return result;
  }

  DeterministicRng local_rng(static_cast<std::uint32_t>(std::max(0, config_.deterministic_seed)));
  DeterministicRng & rng = config_.reset_seed_each_cycle ? local_rng : rng_;

  const double yaw = yaw_from_quaternion(pose.pose.orientation);
  const double fov_rad = std::clamp(config_.sensor_fov_deg, 1.0, 360.0) * (kPi / 180.0);
  const double yaw_offset_rad = config_.yaw_offset_deg * (kPi / 180.0);
  const double sensor_range = std::max(config_.sensor_range_m, config_.min_range_m + 1e-6);
  const double min_range = std::clamp(config_.min_range_m, 0.0, sensor_range);
  const double near_radius =
    std::clamp(config_.near_field_radius_fraction, 0.0, 1.0) * sensor_range;
  const int far_stride = std::max(1, config_.far_field_sample_stride);
  const auto started_at = std::chrono::steady_clock::now();
  const auto deadline =
    started_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(
      std::max(0.0, config_.runtime_budget_ms)));

  for (int sample_index = 0; sample_index < result.samples_requested; ++sample_index) {
    if (std::chrono::steady_clock::now() > deadline) {
      result.budget_exceeded = true;
      break;
    }

    const auto sample = sample_fov_area(
      rng,
      pose.pose.position.x,
      pose.pose.position.y,
      yaw,
      min_range,
      sensor_range,
      fov_rad,
      yaw_offset_rad);

    if (
      config_.use_mixed_resolution &&
      sample.radius > near_radius &&
      far_stride > 1 &&
      (sample_index % far_stride) != 0)
    {
      continue;
    }

    int sample_mx = 0;
    int sample_my = 0;
    if (!decision_map.worldToMapNoThrow(sample.x, sample.y, sample_mx, sample_my)) {
      result.out_of_map_samples += 1;
      continue;
    }

    result.samples_used += 1;
    const int map_cost = decision_map.getCost(sample_mx, sample_my);
    const double info_value = information_value(map_cost, occ_threshold, config_);
    if (info_value <= 0.0) {
      continue;
    }

    if (!ray_visible(
        decision_map,
        global_costmap,
        local_costmap,
        pose,
        {sample.x, sample.y},
        sample_mx,
        sample_my,
        occ_threshold,
        config_))
    {
      result.occluded_samples += 1;
      continue;
    }

    double resolution_decay = 1.0;
    if (
      config_.use_mixed_resolution &&
      config_.resolution_decay_lambda > 0.0 &&
      sample.radius > near_radius)
    {
      resolution_decay = std::exp(-config_.resolution_decay_lambda * (sample.radius - near_radius));
    }
    result.raw_gain += info_value * resolution_decay;
    if (map_cost == static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
      result.visible_unknown_samples += 1;
    }
  }

  if (config_.normalize_by_sample_count && result.samples_used > 0) {
    result.normalized_gain = result.raw_gain / static_cast<double>(result.samples_used);
  } else {
    result.normalized_gain = result.raw_gain;
  }

  result.valid = result.samples_used > 0 && !(result.budget_exceeded && config_.fail_open);
  return result;
}

}  // namespace frontier_exploration_ros2
