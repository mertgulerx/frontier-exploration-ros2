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

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2::detail
{

// Shared angle constant for the visible-gain sensor model and degree/radian conversions.
constexpr double kPi = 3.14159265358979323846;

inline std::string normalize_suppressed_behavior(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (value == "return_to_start") {
    return value;
  }
  return "stay";
}

inline std::string strategy_to_string(FrontierStrategy strategy)
{
  return strategy == FrontierStrategy::MRTSP ? "mrtsp" : "nearest";
}

inline std::string status_to_string(int status)
{
  // Keep status logging allocation/localized in one helper.
  std::ostringstream oss;
  // Keep numeric status visible for unknown/new action status values.
  oss << status;
  return oss.str();
}

// Formats numeric thresholds and measured distances for preemption logs so runtime
// decisions can be read back against parameter values without manual unit decoding.
inline std::string format_meters(double value)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << value << " m";
  return oss.str();
}

// Extracts planar yaw from a full quaternion because the visible-gain helper only
// needs a 2D heading for the target-pose sensor model.
inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

// Rebuilds a quaternion from planar yaw so the target pose can be passed to the
// ray-cast helper using standard geometry message types.
inline geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::cos(yaw * 0.5);
  orientation.z = std::sin(yaw * 0.5);
  return orientation;
}

inline int quantize_bucket(double value, double quantum)
{
  const double safe_quantum = std::max(quantum, 1e-3);
  return static_cast<int>(std::llround(value / safe_quantum));
}

}  // namespace frontier_exploration_ros2::detail
