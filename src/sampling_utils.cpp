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

#include "frontier_exploration_ros2/sampling_utils.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

DeterministicRng::DeterministicRng(std::uint32_t seed)
: engine_(seed)
{
}

void DeterministicRng::seed(std::uint32_t seed_value)
{
  engine_.seed(seed_value);
}

double DeterministicRng::uniform01()
{
  return std::generate_canonical<double, 53>(engine_);
}

int DeterministicRng::uniformInt(int min_inclusive, int max_inclusive)
{
  if (max_inclusive < min_inclusive) {
    std::swap(min_inclusive, max_inclusive);
  }
  std::uniform_int_distribution<int> distribution(min_inclusive, max_inclusive);
  return distribution(engine_);
}

double normalize_angle_rad(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

PolarSample sample_polar_annulus(
  DeterministicRng & rng,
  double center_x,
  double center_y,
  double min_radius_m,
  double max_radius_m,
  bool area_uniform_radius,
  int angle_bins)
{
  const double safe_min_radius = std::max(0.0, std::min(min_radius_m, max_radius_m));
  const double safe_max_radius = std::max(safe_min_radius, max_radius_m);

  double angle_rad = 0.0;
  if (angle_bins > 0) {
    const int bin = rng.uniformInt(0, angle_bins - 1);
    angle_rad = (2.0 * kPi * static_cast<double>(bin)) / static_cast<double>(angle_bins);
  } else {
    angle_rad = 2.0 * kPi * rng.uniform01();
  }

  const double u = rng.uniform01();
  double radius = safe_min_radius;
  if (area_uniform_radius) {
    const double min_sq = safe_min_radius * safe_min_radius;
    const double max_sq = safe_max_radius * safe_max_radius;
    radius = std::sqrt(min_sq + u * (max_sq - min_sq));
  } else {
    radius = safe_min_radius + u * (safe_max_radius - safe_min_radius);
  }

  return PolarSample{
    center_x + std::cos(angle_rad) * radius,
    center_y + std::sin(angle_rad) * radius,
    radius,
    normalize_angle_rad(angle_rad),
  };
}

PolarSample sample_fov_area(
  DeterministicRng & rng,
  double origin_x,
  double origin_y,
  double yaw_rad,
  double min_range_m,
  double max_range_m,
  double fov_rad,
  double yaw_offset_rad)
{
  const double safe_min_range = std::max(0.0, std::min(min_range_m, max_range_m));
  const double safe_max_range = std::max(safe_min_range, max_range_m);
  const double safe_fov = std::clamp(fov_rad, 1e-6, 2.0 * kPi);
  const double radius = std::sqrt(
    (safe_min_range * safe_min_range) +
    rng.uniform01() * ((safe_max_range * safe_max_range) - (safe_min_range * safe_min_range)));
  const double local_angle = (rng.uniform01() * safe_fov) - (safe_fov * 0.5);
  const double angle = yaw_rad + yaw_offset_rad + local_angle;
  return PolarSample{
    origin_x + std::cos(angle) * radius,
    origin_y + std::sin(angle) * radius,
    radius,
    normalize_angle_rad(angle),
  };
}

}  // namespace frontier_exploration_ros2
