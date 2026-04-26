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

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/sampling_utils.hpp"
#include "frontier_exploration_ros2/viewpoint_candidate.hpp"

namespace frontier_exploration_ros2
{

// Optional RRT-based viewpoint sampling.
//
// This module does not replace Nav2 and does not command a path. It only
// proposes candidate sensing poses around already-detected frontier candidates.
// The selected pose is still dispatched through the normal NavigateToPose action
// path. Keep this module bounded by max nodes, max iterations, sample limits,
// and runtime budget so the explorer remains predictable on low-power robots.
class RrtViewpointSampler
{
public:
  RrtViewpointSampler(
    AdvancedViewpointSamplingConfig sampling_config,
    RrtViewpointConfig rrt_config);

  std::vector<ViewpointCandidate> generateForFrontier(
    const OccupancyGrid2d & decision_map,
    const std::optional<OccupancyGrid2d> & global_costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const FrontierCandidate & frontier,
    std::size_t frontier_index,
    int occ_threshold,
    const std::string & frame_id,
    std::optional<std::chrono::steady_clock::time_point> deadline = std::nullopt);

  void resetSession();

private:
  AdvancedViewpointSamplingConfig sampling_config_;
  RrtViewpointConfig rrt_config_;
  std::uint32_t generation_counter_{0};
};

}  // namespace frontier_exploration_ros2
