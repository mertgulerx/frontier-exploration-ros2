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

#include "frontier_exploration_ros2/advanced_viewpoint_sampler.hpp"

#include "frontier_exploration_ros2/rrt_viewpoint_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <utility>

namespace frontier_exploration_ros2
{

AdvancedViewpointSampler::AdvancedViewpointSampler(
  AdvancedViewpointSamplingConfig sampling_config,
  RrtViewpointConfig rrt_config)
: sampling_config_(std::move(sampling_config)),
  rrt_config_(std::move(rrt_config))
{
}

void AdvancedViewpointSampler::resetSession()
{
}

std::string normalize_advanced_viewpoint_method(std::string method)
{
  std::transform(
    method.begin(),
    method.end(),
    method.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (method == "polar" || method == "hybrid") {
    return "frontier_local_rrt";
  }
  return "frontier_local_rrt";
}

std::vector<ViewpointCandidate> AdvancedViewpointSampler::generate(
  const OccupancyGrid2d & decision_map,
  const std::optional<OccupancyGrid2d> & global_costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<FrontierCandidate> & shortlisted_frontiers,
  int occ_threshold,
  const std::string & frame_id)
{
  if (!sampling_config_.enabled || !rrt_config_.enabled || shortlisted_frontiers.empty()) {
    return {};
  }

  const auto started_at = std::chrono::steady_clock::now();
  const auto deadline =
    started_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(
      std::max(0.0, sampling_config_.runtime_budget_ms)));
  const std::size_t max_frontiers = std::min(
    shortlisted_frontiers.size(),
    static_cast<std::size_t>(std::max(1, sampling_config_.max_frontiers)));
  const std::size_t max_total = static_cast<std::size_t>(std::max(1, sampling_config_.max_total_samples));

  std::vector<ViewpointCandidate> viewpoints;
  viewpoints.reserve(max_total);
  RrtViewpointSampler rrt_sampler(sampling_config_, rrt_config_);
  for (std::size_t frontier_index = 0; frontier_index < max_frontiers; ++frontier_index) {
    if (std::chrono::steady_clock::now() > deadline || viewpoints.size() >= max_total) {
      break;
    }
    auto frontier_viewpoints = rrt_sampler.generateForFrontier(
      decision_map,
      global_costmap,
      local_costmap,
      robot_pose,
      shortlisted_frontiers[frontier_index],
      frontier_index,
      occ_threshold,
      frame_id,
      deadline);
    for (auto & viewpoint : frontier_viewpoints) {
      if (viewpoints.size() >= max_total) {
        break;
      }
      viewpoints.push_back(std::move(viewpoint));
    }
  }

  return viewpoints;
}

}  // namespace frontier_exploration_ros2
