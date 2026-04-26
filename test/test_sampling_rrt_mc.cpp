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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <cstdint>

#include "frontier_exploration_ros2/monte_carlo_gain_estimator.hpp"
#include "frontier_exploration_ros2/rrt_viewpoint_sampler.hpp"
#include "frontier_exploration_ros2/sampling_utils.hpp"

namespace frontier_exploration_ros2
{
namespace
{

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<std::uint32_t>(width);
  msg.info.height = static_cast<std::uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

void set_cell(nav_msgs::msg::OccupancyGrid & msg, int x, int y, int value)
{
  msg.data[static_cast<std::size_t>(y * static_cast<int>(msg.info.width) + x)] =
    static_cast<int8_t>(value);
}

void set_rect(nav_msgs::msg::OccupancyGrid & msg, int x0, int y0, int x1, int y1, int value)
{
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      set_cell(msg, x, y, value);
    }
  }
}

geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  return pose;
}

FrontierCandidate make_frontier(double x, double y)
{
  return FrontierCandidate{{x, y}, {x, y}, 20};
}

}  // namespace

TEST(SamplingUtilsTests, PolarSamplesStayInsideAnnulus)
{
  DeterministicRng rng(7U);
  for (int i = 0; i < 200; ++i) {
    const auto sample = sample_polar_annulus(rng, 2.0, -1.0, 0.5, 2.0, true);
    const double radius = std::hypot(sample.x - 2.0, sample.y + 1.0);
    EXPECT_GE(radius, 0.5 - 1e-9);
    EXPECT_LE(radius, 2.0 + 1e-9);
  }
}

TEST(SamplingUtilsTests, AreaUniformRadiusDoesNotCollapseTowardCenter)
{
  DeterministicRng rng(11U);
  double radius_sum = 0.0;
  constexpr int samples = 1000;
  for (int i = 0; i < samples; ++i) {
    radius_sum += sample_polar_annulus(rng, 0.0, 0.0, 0.0, 1.0, true).radius;
  }
  // Uniform area sampling in a unit disk has expected radius 2/3. A radius = u
  // bug would sit near 0.5, so this wide smoke bound catches the obvious mistake.
  EXPECT_GT(radius_sum / static_cast<double>(samples), 0.60);
}

TEST(MonteCarloGainTests, FullyKnownFreeMapHasZeroGain)
{
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  MonteCarloGainConfig config;
  config.enabled = true;
  config.sample_count = 256;
  config.sensor_range_m = 8.0;
  config.deterministic_seed = 3;
  MonteCarloGainEstimator estimator(config);

  const auto result = estimator.evaluatePose(
    OccupancyGrid2d(map_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    50);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.visible_unknown_samples, 0);
  EXPECT_DOUBLE_EQ(result.normalized_gain, 0.0);
}

TEST(MonteCarloGainTests, VisibleUnknownProducesPositiveGain)
{
  auto map_msg = build_grid(20, 20, 0);
  set_rect(map_msg, 9, 0, 19, 19, -1);
  auto costmap_msg = build_grid(20, 20, 0);
  MonteCarloGainConfig config;
  config.enabled = true;
  config.sample_count = 512;
  config.sensor_range_m = 12.0;
  config.sensor_fov_deg = 360.0;
  config.deterministic_seed = 9;
  MonteCarloGainEstimator estimator(config);

  const auto result = estimator.evaluatePose(
    OccupancyGrid2d(map_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    50);

  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.visible_unknown_samples, 0);
  EXPECT_GT(result.normalized_gain, 0.0);
}

TEST(MonteCarloGainTests, OccupiedWallReducesVisibleUnknownGain)
{
  auto open_msg = build_grid(20, 20, 0);
  set_rect(open_msg, 9, 0, 19, 19, -1);
  auto walled_msg = open_msg;
  set_rect(walled_msg, 8, 0, 8, 19, 100);
  auto costmap_msg = build_grid(20, 20, 0);

  MonteCarloGainConfig config;
  config.enabled = true;
  config.sample_count = 1024;
  config.sensor_range_m = 12.0;
  config.deterministic_seed = 14;
  MonteCarloGainEstimator open_estimator(config);
  MonteCarloGainEstimator wall_estimator(config);

  const auto open_result = open_estimator.evaluatePose(
    OccupancyGrid2d(open_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    50);
  const auto wall_result = wall_estimator.evaluatePose(
    OccupancyGrid2d(walled_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    50);

  EXPECT_TRUE(open_result.valid);
  EXPECT_TRUE(wall_result.valid);
  EXPECT_GT(open_result.normalized_gain, wall_result.normalized_gain);
}

TEST(MonteCarloGainTests, SameSeedProducesSameGain)
{
  auto map_msg = build_grid(20, 20, 0);
  set_rect(map_msg, 9, 0, 19, 19, -1);
  auto costmap_msg = build_grid(20, 20, 0);
  MonteCarloGainConfig config;
  config.enabled = true;
  config.sample_count = 512;
  config.sensor_range_m = 12.0;
  config.deterministic_seed = 99;

  MonteCarloGainEstimator first(config);
  MonteCarloGainEstimator second(config);
  const auto first_result = first.evaluatePose(
    OccupancyGrid2d(map_msg), OccupancyGrid2d(costmap_msg), std::nullopt, make_pose(5.0, 10.0), 50);
  const auto second_result = second.evaluatePose(
    OccupancyGrid2d(map_msg), OccupancyGrid2d(costmap_msg), std::nullopt, make_pose(5.0, 10.0), 50);

  EXPECT_DOUBLE_EQ(first_result.normalized_gain, second_result.normalized_gain);
  EXPECT_EQ(first_result.visible_unknown_samples, second_result.visible_unknown_samples);
}

TEST(RrtViewpointSamplerTests, GeneratesDeterministicValidViewpoints)
{
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  AdvancedViewpointSamplingConfig sampling_config;
  sampling_config.enabled = true;
  sampling_config.max_samples_per_frontier = 8;
  sampling_config.max_goal_distance_m = 10.0;
  RrtViewpointConfig rrt_config;
  rrt_config.enabled = true;
  rrt_config.max_nodes = 32;
  rrt_config.max_iterations = 64;
  rrt_config.deterministic_seed = 4;
  rrt_config.reset_seed_each_cycle = true;

  RrtViewpointSampler first_sampler(sampling_config, rrt_config);
  RrtViewpointSampler second_sampler(sampling_config, rrt_config);
  const auto first = first_sampler.generateForFrontier(
    OccupancyGrid2d(map_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    make_frontier(12.0, 10.0),
    0,
    50,
    "map");
  const auto second = second_sampler.generateForFrontier(
    OccupancyGrid2d(map_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    make_frontier(12.0, 10.0),
    0,
    50,
    "map");

  ASSERT_FALSE(first.empty());
  ASSERT_EQ(first.size(), second.size());
  EXPECT_DOUBLE_EQ(first.front().pose.pose.position.x, second.front().pose.pose.position.x);
  EXPECT_DOUBLE_EQ(first.front().pose.pose.position.y, second.front().pose.pose.position.y);
  for (const auto & viewpoint : first) {
    EXPECT_TRUE(viewpoint.valid);
    EXPECT_EQ(viewpoint.pose.header.frame_id, "map");
  }
}

TEST(RrtViewpointSamplerTests, RejectsHardBlockedSamples)
{
  auto map_msg = build_grid(20, 20, 0);
  auto blocked_costmap_msg = build_grid(20, 20, 100);
  AdvancedViewpointSamplingConfig sampling_config;
  sampling_config.enabled = true;
  sampling_config.max_samples_per_frontier = 8;
  sampling_config.max_goal_distance_m = 10.0;
  RrtViewpointConfig rrt_config;
  rrt_config.enabled = true;
  rrt_config.max_nodes = 32;
  rrt_config.max_iterations = 64;
  rrt_config.deterministic_seed = 4;

  RrtViewpointSampler sampler(sampling_config, rrt_config);
  const auto viewpoints = sampler.generateForFrontier(
    OccupancyGrid2d(map_msg),
    OccupancyGrid2d(blocked_costmap_msg),
    std::nullopt,
    make_pose(5.0, 10.0),
    make_frontier(12.0, 10.0),
    0,
    50,
    "map");

  EXPECT_TRUE(viewpoints.empty());
}

}  // namespace frontier_exploration_ros2
