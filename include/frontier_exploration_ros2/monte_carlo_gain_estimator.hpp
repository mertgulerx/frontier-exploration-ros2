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

#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/sampling_utils.hpp"

namespace frontier_exploration_ros2
{

struct MonteCarloGainConfig
{
  bool enabled{false};
  std::string mode{"viewpoint_pose"};
  double weight{1.0};
  int sample_count{256};
  int max_candidates{10};
  double runtime_budget_ms{20.0};
  int deterministic_seed{0};
  bool reset_seed_each_cycle{false};
  double sensor_range_m{12.0};
  double sensor_fov_deg{360.0};
  double yaw_offset_deg{0.0};
  double min_range_m{0.05};
  bool occupied_blocks_visibility{true};
  bool costmap_blocks_visibility{false};
  double unknown_value{1.0};
  double free_value{0.0};
  double occupied_value{0.0};
  bool normalize_by_sample_count{true};
  bool use_mixed_resolution{true};
  double near_field_radius_fraction{0.60};
  int far_field_sample_stride{2};
  double resolution_decay_lambda{0.0};
  bool fail_open{true};
};

struct MonteCarloGainResult
{
  double normalized_gain{0.0};
  double raw_gain{0.0};
  int samples_requested{0};
  int samples_used{0};
  int visible_unknown_samples{0};
  int occluded_samples{0};
  int out_of_map_samples{0};
  bool budget_exceeded{false};
  bool valid{false};
};

// Optional Monte Carlo visible-information gain estimator.
//
// This is a 2D OccupancyGrid adaptation of the Monte Carlo gain idea from the
// 2026 multi-resolution field exploration paper. It estimates visible unknown
// area from a candidate viewpoint pose. It does not replace WFD frontier
// extraction and it does not require OctoMap. When the estimator fails, times
// out, or has no valid samples, the caller must fall back to the base frontier
// scoring behavior.
class MonteCarloGainEstimator
{
public:
  explicit MonteCarloGainEstimator(MonteCarloGainConfig config = {});

  MonteCarloGainResult evaluatePose(
    const OccupancyGrid2d & decision_map,
    const std::optional<OccupancyGrid2d> & global_costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    const geometry_msgs::msg::PoseStamped & pose,
    int occ_threshold);

  void resetSession();

private:
  MonteCarloGainConfig config_;
  DeterministicRng rng_;
};

std::string normalize_monte_carlo_gain_mode(std::string mode);

}  // namespace frontier_exploration_ros2
