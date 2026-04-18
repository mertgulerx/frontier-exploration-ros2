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

#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

// Returns the world-space goal point used to dispatch navigation.
std::pair<double, double> frontier_position(const FrontierLike & frontier);

// Returns the frontier reference point used for comparison and distance heuristics.
std::pair<double, double> frontier_reference_point(const FrontierLike & frontier);

// Returns a comparable size metric for logging and policy checks.
int frontier_size(const FrontierLike & frontier);

// Builds a stable human-readable frontier description for logs.
std::string describe_frontier(const FrontierLike & frontier);

// Quantizes and sorts frontier coordinates to create a deterministic signature.
FrontierSignature frontier_signature(
  const FrontierSequence & frontiers,
  double frontier_visit_tolerance);

// Applies primitive selection policy (preferred, escape, close-range fallback).
FrontierSelectionResult select_primitive_frontier(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose,
  double frontier_selection_min_distance,
  double frontier_visit_tolerance,
  bool escape_active);

// Compares two frontiers with visit-tolerance semantics.
bool are_frontiers_equivalent(
  const std::optional<FrontierLike> & first_frontier,
  const std::optional<FrontierLike> & second_frontier,
  double frontier_visit_tolerance);

// Compares two frontier sequences element-by-element under tolerance.
bool are_frontier_sequences_equivalent(
  const FrontierSequence & first_frontier_sequence,
  const FrontierSequence & second_frontier_sequence,
  double frontier_visit_tolerance);

}  // namespace frontier_exploration_ros2
