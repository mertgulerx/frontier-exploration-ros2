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
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/viewpoint_candidate.hpp"

namespace frontier_exploration_ros2
{

class AdvancedViewpointSampler
{
public:
  AdvancedViewpointSampler(
    AdvancedViewpointSamplingConfig sampling_config,
    RrtViewpointConfig rrt_config);

  std::vector<ViewpointCandidate> generate(
    const OccupancyGrid2d & decision_map,
    const std::optional<OccupancyGrid2d> & global_costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::vector<FrontierCandidate> & shortlisted_frontiers,
    int occ_threshold,
    const std::string & frame_id);

  void resetSession();

private:
  AdvancedViewpointSamplingConfig sampling_config_;
  RrtViewpointConfig rrt_config_;
};

std::string normalize_advanced_viewpoint_method(std::string method);

}  // namespace frontier_exploration_ros2
