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

#include "frontier_exploration_ros2/mrtsp_ordering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Shared distance helpers keep the heuristic formulas readable.
double euclidean(
  const std::pair<double, double> & lhs,
  const std::pair<double, double> & rhs)
{
  return std::hypot(lhs.first - rhs.first, lhs.second - rhs.second);
}

double euclidean(
  const std::array<double, 2> & lhs,
  const std::pair<double, double> & rhs)
{
  return std::hypot(lhs[0] - rhs.first, lhs[1] - rhs.second);
}

// Normalizes heading deltas into [-pi, pi] before angular-cost evaluation.
double angle_wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

double frontier_information_gain(const FrontierCandidate & frontier)
{
  // MRTSP currently uses cluster size as the gain proxy.
  return static_cast<double>(frontier.size);
}

double frontier_path_cost_with_start_world(
  const FrontierCandidate & source_frontier,
  const FrontierCandidate & target_frontier,
  const std::pair<double, double> & start_world_point,
  double sensor_effective_range_m)
{
  // Mirrors the paper's two-point frontier approximation and discounts sensor reach.
  const double dm = euclidean(source_frontier.center_point, target_frontier.center_point);
  const double dn = euclidean(source_frontier.center_point, target_frontier.centroid);
  const double du = euclidean(target_frontier.center_point, start_world_point);
  const double dv = euclidean(target_frontier.centroid, start_world_point);
  return std::max(dm + du, dn + dv) - sensor_effective_range_m;
}

double initial_frontier_path_cost(
  const std::array<double, 2> & robot_position,
  const FrontierCandidate & target_frontier,
  const std::pair<double, double> & start_world_point,
  double sensor_effective_range_m)
{
  // Start-node variant of the same frontier transition approximation.
  const double dm = euclidean(robot_position, target_frontier.center_point);
  const double dn = euclidean(robot_position, target_frontier.centroid);
  const double du = euclidean(target_frontier.center_point, start_world_point);
  const double dv = euclidean(target_frontier.centroid, start_world_point);
  return std::max(dm + du, dn + dv) - sensor_effective_range_m;
}

double lower_bound_time_cost(
  const RobotState & robot_state,
  const std::pair<double, double> & target_point,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax)
{
  // Uses the tighter of translation and heading-change bounds for the first dispatch cost.
  const double vmax = std::max(max_linear_speed_vmax, 1e-6);
  const double wmax = std::max(max_angular_speed_wmax, 1e-6);
  const double distance_term = euclidean(robot_state.position, target_point) / vmax;
  const double target_yaw = std::atan2(
    target_point.second - robot_state.position[1],
    target_point.first - robot_state.position[0]);
  const double yaw_delta = std::abs(angle_wrap(target_yaw - robot_state.yaw));
  const double heading_term = std::min(yaw_delta, (2.0 * kPi) - yaw_delta) / wmax;
  return std::min(distance_term, heading_term);
}

MrtspCostMatrix build_cost_matrix(
  const std::vector<FrontierCandidate> & frontiers,
  const RobotState & robot_state,
  const CostWeights & weights,
  double sensor_effective_range_m,
  double max_linear_speed_vmax,
  double max_angular_speed_wmax)
{
  const std::size_t frontier_count = frontiers.size();
  MrtspCostMatrix matrix;
  // Row/column zero is the synthetic robot start node.
  matrix.dimension = frontier_count + 1U;
  matrix.values.assign(matrix.dimension * matrix.dimension, std::numeric_limits<double>::infinity());

  // Returning to the start node is never chosen, so these edges stay zeroed for completeness.
  for (std::size_t row = 1; row < matrix.dimension; ++row) {
    matrix.values[row * matrix.dimension] = 0.0;
  }

  for (std::size_t row = 0; row < matrix.dimension; ++row) {
    for (std::size_t column = 1; column < matrix.dimension; ++column) {
      if (row == column) {
        continue;
      }

      const FrontierCandidate & target_frontier = frontiers[column - 1U];
      const double gain = weights.gain_ws * frontier_information_gain(target_frontier);
      if (gain <= 0.0) {
        continue;
      }

      if (row == 0U) {
        // The first step also pays a lower-bound motion-time term from the robot pose.
        const double path_cost = initial_frontier_path_cost(
          robot_state.position,
          target_frontier,
          target_frontier.start_world_point,
          sensor_effective_range_m);
        matrix.values[row * matrix.dimension + column] =
          ((weights.distance_wd * path_cost) / gain) +
          lower_bound_time_cost(
          robot_state,
          target_frontier.center_point,
          max_linear_speed_vmax,
          max_angular_speed_wmax);
      } else {
        // Frontier-to-frontier edges use the distance-over-gain term only.
        const double path_cost = frontier_path_cost_with_start_world(
          frontiers[row - 1U],
          target_frontier,
          target_frontier.start_world_point,
          sensor_effective_range_m);
        matrix.values[row * matrix.dimension + column] =
          (weights.distance_wd * path_cost) / gain;
      }
    }
  }

  return matrix;
}

std::vector<std::size_t> greedy_mrtsp_order(const MrtspCostMatrix & cost_matrix)
{
  if (cost_matrix.dimension <= 1U || cost_matrix.values.empty()) {
    return {};
  }

  // Greedy traversal starts from the synthetic robot node and visits each frontier once.
  std::vector<bool> visited(cost_matrix.dimension, false);
  visited[0] = true;
  std::vector<std::size_t> ordered_frontiers;
  ordered_frontiers.reserve(cost_matrix.dimension - 1U);
  std::size_t current_index = 0U;

  while (true) {
    std::size_t next_index = 0U;
    double best_cost = std::numeric_limits<double>::infinity();

    for (std::size_t candidate = 1U; candidate < cost_matrix.dimension; ++candidate) {
      if (visited[candidate]) {
        continue;
      }
      const double candidate_cost = cost_matrix.at(current_index, candidate);
      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        next_index = candidate;
      }
    }

    if (next_index == 0U || !std::isfinite(best_cost)) {
      break;
    }

    visited[next_index] = true;
    ordered_frontiers.push_back(next_index - 1U);
    current_index = next_index;
  }

  return ordered_frontiers;
}

}  // namespace frontier_exploration_ros2
