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

#include <array>
#include <cstddef>
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

// Robot pose state required by the MRTSP start-node cost model.
struct RobotState
{
  std::array<double, 2> position{0.0, 0.0};
  double yaw{0.0};
};

// Relative weights for the distance-over-gain frontier scoring term.
struct CostWeights
{
  double distance_wd{1.0};
  double gain_ws{1.0};
};

// Dense directed cost matrix whose row/column zero represents the robot start node.
struct MrtspCostMatrix
{
  std::size_t dimension{0};
  std::vector<double> values;

  [[nodiscard]] double at(std::size_t row, std::size_t column) const
  {
    return values[row * dimension + column];
  }
};

// Uses frontier size as the exploration gain term in the MRTSP heuristic.
double frontier_information_gain(const FrontierCandidate & frontier);

// Estimates transition cost between two frontier nodes using center/start geometry from the paper.
double frontier_path_cost_with_start_world(
  const FrontierCandidate & source_frontier,
  const FrontierCandidate & target_frontier,
  const std::pair<double, double> & start_world_point,
  double sensor_effective_range_m);

// Estimates the initial robot-to-frontier transition cost for the synthetic start node.
double initial_frontier_path_cost(
  const std::array<double, 2> & robot_position,
  const FrontierCandidate & target_frontier,
  const std::pair<double, double> & start_world_point,
  double sensor_effective_range_m);

// Lower-bound motion time term used only on the start-node outgoing edges.
double lower_bound_time_cost(
  const RobotState & robot_state,
  const std::pair<double, double> & target_point,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax);

// Builds the directed MRTSP heuristic matrix consumed by the greedy ordering step.
MrtspCostMatrix build_cost_matrix(
  const std::vector<FrontierCandidate> & frontiers,
  const RobotState & robot_state,
  const CostWeights & weights,
  double sensor_effective_range_m,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax);

// Greedily traverses the MRTSP matrix and returns frontier indices in visit order.
std::vector<std::size_t> greedy_mrtsp_order(const MrtspCostMatrix & cost_matrix);

}  // namespace frontier_exploration_ros2
