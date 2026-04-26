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

#include "frontier_exploration_ros2/rrt_viewpoint_sampler.hpp"

#include "frontier_exploration_ros2/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::cos(yaw * 0.5);
  orientation.z = std::sin(yaw * 0.5);
  return orientation;
}

std::pair<double, double> frontier_sampling_center(const FrontierCandidate & frontier)
{
  return frontier.goal_point.value_or(frontier.centroid);
}

std::pair<double, double> frontier_reference(const FrontierCandidate & frontier)
{
  return frontier.centroid;
}

double distance_between(
  const std::pair<double, double> & first,
  const std::pair<double, double> & second)
{
  return std::hypot(first.first - second.first, first.second - second.second);
}

bool occupancy_cell_free(
  const OccupancyGrid2d & decision_map,
  double x,
  double y,
  int occ_threshold)
{
  int map_x = 0;
  int map_y = 0;
  if (!decision_map.worldToMapNoThrow(x, y, map_x, map_y)) {
    return false;
  }
  const int cost = decision_map.getCost(map_x, map_y);
  return cost >= 0 && cost < occ_threshold;
}

bool hard_costmap_free(
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double x,
  double y,
  int occ_threshold)
{
  return !is_world_point_blocked(
    {global_costmap, local_costmap},
    {x, y},
    occ_threshold);
}

bool collision_free_point(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double x,
  double y,
  int occ_threshold,
  const RrtViewpointConfig & config)
{
  int map_x = 0;
  int map_y = 0;
  if (!decision_map.worldToMapNoThrow(x, y, map_x, map_y)) {
    return false;
  }
  if (config.use_occupancy_validation && !occupancy_cell_free(decision_map, x, y, occ_threshold)) {
    return false;
  }
  if (
    config.use_costmap_validation &&
    !hard_costmap_free(global_costmap, local_costmap, x, y, occ_threshold))
  {
    return false;
  }
  return true;
}

bool collision_free_segment(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const RrtNode2D & from,
  const RrtNode2D & to,
  int occ_threshold,
  const RrtViewpointConfig & config)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1e-9) {
    return false;
  }

  const double step = std::max(config.collision_check_step_m, 1e-4);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int step_index = 1; step_index <= steps; ++step_index) {
    const double t = static_cast<double>(step_index) / static_cast<double>(steps);
    const double x = from.x + dx * t;
    const double y = from.y + dy * t;
    if (!collision_free_point(
        decision_map,
        global_costmap,
        local_costmap,
        x,
        y,
        occ_threshold,
        config))
    {
      return false;
    }
  }
  return true;
}

bool line_of_sight_clear(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const std::pair<double, double> & viewpoint,
  const std::pair<double, double> & target,
  int occ_threshold,
  const RrtViewpointConfig & config)
{
  const double dx = target.first - viewpoint.first;
  const double dy = target.second - viewpoint.second;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1e-9) {
    return true;
  }
  const double step = std::max(decision_map.map().info.resolution * 0.5, 1e-6);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int step_index = 1; step_index <= steps; ++step_index) {
    const double t = std::min(1.0, static_cast<double>(step_index) / static_cast<double>(steps));
    const double x = viewpoint.first + dx * t;
    const double y = viewpoint.second + dy * t;
    int map_x = 0;
    int map_y = 0;
    if (!decision_map.worldToMapNoThrow(x, y, map_x, map_y)) {
      return false;
    }
    const bool endpoint = step_index == steps;
    if (!endpoint && decision_map.getCost(map_x, map_y) > occ_threshold) {
      return false;
    }
    if (
      !endpoint &&
      config.use_costmap_validation &&
      !hard_costmap_free(global_costmap, local_costmap, x, y, occ_threshold))
    {
      return false;
    }
  }
  return true;
}

std::optional<RrtNode2D> sample_tree_target(
  DeterministicRng & rng,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const FrontierCandidate & frontier,
  const RrtViewpointConfig & config)
{
  const auto center = frontier_sampling_center(frontier);
  const double goal_probability = std::clamp(config.goal_bias_probability, 0.0, 1.0);
  double frontier_probability = std::clamp(config.frontier_bias_probability, 0.0, 1.0);
  if (goal_probability + frontier_probability > 1.0) {
    frontier_probability = 1.0 - goal_probability;
  }

  const double selector = rng.uniform01();
  if (selector < goal_probability) {
    return RrtNode2D{center.first, center.second, -1, 0.0};
  }

  const bool sample_near_frontier = selector < goal_probability + frontier_probability;
  const double center_x = sample_near_frontier ? center.first : robot_pose.pose.position.x;
  const double center_y = sample_near_frontier ? center.second : robot_pose.pose.position.y;
  if (config.polar_sampling_enabled) {
    const auto sample = sample_polar_annulus(
      rng,
      center_x,
      center_y,
      config.min_sample_radius_m,
      config.sampling_radius_m,
      config.polar_area_uniform_radius,
      config.polar_angle_bins);
    return RrtNode2D{sample.x, sample.y, -1, 0.0};
  }

  const double radius = std::max(config.sampling_radius_m, config.min_sample_radius_m);
  return RrtNode2D{
    center_x + ((rng.uniform01() * 2.0) - 1.0) * radius,
    center_y + ((rng.uniform01() * 2.0) - 1.0) * radius,
    -1,
    0.0,
  };
}

int nearest_node_index(const std::vector<RrtNode2D> & nodes, const RrtNode2D & sample)
{
  int best_index = 0;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double dx = nodes[index].x - sample.x;
    const double dy = nodes[index].y - sample.y;
    const double distance_sq = (dx * dx) + (dy * dy);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_index = static_cast<int>(index);
    }
  }
  return best_index;
}

RrtNode2D extend_toward(
  const RrtNode2D & nearest,
  const RrtNode2D & sample,
  double step_size_m)
{
  const double dx = sample.x - nearest.x;
  const double dy = sample.y - nearest.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= std::max(step_size_m, 1e-6)) {
    return sample;
  }
  const double scale = std::max(step_size_m, 1e-6) / distance;
  return RrtNode2D{
    nearest.x + dx * scale,
    nearest.y + dy * scale,
    -1,
    0.0,
  };
}

}  // namespace

RrtViewpointSampler::RrtViewpointSampler(
  AdvancedViewpointSamplingConfig sampling_config,
  RrtViewpointConfig rrt_config)
: sampling_config_(std::move(sampling_config)),
  rrt_config_(std::move(rrt_config))
{
}

void RrtViewpointSampler::resetSession()
{
  generation_counter_ = 0U;
}

std::vector<ViewpointCandidate> RrtViewpointSampler::generateForFrontier(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const FrontierCandidate & frontier,
  std::size_t frontier_index,
  int occ_threshold,
  const std::string & frame_id,
  std::optional<std::chrono::steady_clock::time_point> deadline)
{
  if (!rrt_config_.enabled || rrt_config_.max_nodes <= 0 || rrt_config_.max_iterations <= 0) {
    return {};
  }

  const std::uint32_t seed =
    static_cast<std::uint32_t>(std::max(0, rrt_config_.deterministic_seed)) +
    static_cast<std::uint32_t>(frontier_index * 7919U) +
    (rrt_config_.reset_seed_each_cycle ? 0U : generation_counter_++);
  DeterministicRng rng(seed);

  std::vector<RrtNode2D> nodes;
  nodes.reserve(static_cast<std::size_t>(std::max(1, rrt_config_.max_nodes)));
  nodes.push_back(RrtNode2D{
    robot_pose.pose.position.x,
    robot_pose.pose.position.y,
    -1,
    0.0,
  });

  std::vector<ViewpointCandidate> viewpoints;
  viewpoints.reserve(static_cast<std::size_t>(std::max(1, sampling_config_.max_samples_per_frontier)));
  const auto reference = frontier_reference(frontier);
  const std::pair<double, double> robot_point{
    robot_pose.pose.position.x,
    robot_pose.pose.position.y,
  };

  for (int iteration = 0; iteration < rrt_config_.max_iterations; ++iteration) {
    if (deadline.has_value() && std::chrono::steady_clock::now() > *deadline) {
      break;
    }
    if (static_cast<int>(nodes.size()) >= rrt_config_.max_nodes) {
      break;
    }
    if (static_cast<int>(viewpoints.size()) >= sampling_config_.max_samples_per_frontier) {
      break;
    }

    const auto sample = sample_tree_target(rng, robot_pose, frontier, rrt_config_);
    if (!sample.has_value()) {
      continue;
    }

    const int nearest_index = nearest_node_index(nodes, *sample);
    RrtNode2D next = extend_toward(nodes[static_cast<std::size_t>(nearest_index)], *sample, rrt_config_.step_size_m);
    next.parent = nearest_index;
    const double segment_length = distance_between(
      {nodes[static_cast<std::size_t>(nearest_index)].x, nodes[static_cast<std::size_t>(nearest_index)].y},
      {next.x, next.y});
    next.accumulated_length_m =
      nodes[static_cast<std::size_t>(nearest_index)].accumulated_length_m + segment_length;

    const double robot_distance = distance_between(robot_point, {next.x, next.y});
    if (robot_distance < sampling_config_.min_goal_distance_m) {
      continue;
    }
    if (sampling_config_.max_goal_distance_m > 0.0 && robot_distance > sampling_config_.max_goal_distance_m) {
      continue;
    }

    if (!collision_free_point(
        decision_map,
        global_costmap,
        local_costmap,
        next.x,
        next.y,
        occ_threshold,
        rrt_config_))
    {
      continue;
    }
    if (!collision_free_segment(
        decision_map,
        global_costmap,
        local_costmap,
        nodes[static_cast<std::size_t>(nearest_index)],
        next,
        occ_threshold,
        rrt_config_))
    {
      continue;
    }
    if (
      sampling_config_.require_line_of_sight_to_frontier &&
      !line_of_sight_clear(
        decision_map,
        global_costmap,
        local_costmap,
        {next.x, next.y},
        reference,
        occ_threshold,
        rrt_config_))
    {
      continue;
    }

    nodes.push_back(next);
    const int node_index = static_cast<int>(nodes.size()) - 1;
    const double yaw = std::atan2(reference.second - next.y, reference.first - next.x);
    ViewpointCandidate viewpoint;
    viewpoint.pose.header.frame_id = frame_id;
    viewpoint.pose.pose.position.x = next.x;
    viewpoint.pose.pose.position.y = next.y;
    viewpoint.pose.pose.orientation = quaternion_from_yaw(yaw);
    viewpoint.frontier_index = frontier_index;
    viewpoint.parent_node_index = next.parent;
    viewpoint.valid = true;
    viewpoint.source = "rrt_viewpoint";
    viewpoint.robot_distance_m = robot_distance;
    viewpoint.frontier_distance_m = distance_between({next.x, next.y}, reference);
    (void)node_index;
    viewpoints.push_back(viewpoint);
  }

  return viewpoints;
}

}  // namespace frontier_exploration_ros2
