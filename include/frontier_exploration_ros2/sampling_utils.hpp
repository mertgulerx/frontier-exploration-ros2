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

#include <cstdint>
#include <random>

namespace frontier_exploration_ros2
{

class DeterministicRng
{
public:
  explicit DeterministicRng(std::uint32_t seed = 0U);

  void seed(std::uint32_t seed_value);
  double uniform01();
  int uniformInt(int min_inclusive, int max_inclusive);

private:
  std::mt19937 engine_;
};

struct PolarSample
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double angle_rad{0.0};
};

double normalize_angle_rad(double angle_rad);

PolarSample sample_polar_annulus(
  DeterministicRng & rng,
  double center_x,
  double center_y,
  double min_radius_m,
  double max_radius_m,
  bool area_uniform_radius,
  int angle_bins = 0);

PolarSample sample_fov_area(
  DeterministicRng & rng,
  double origin_x,
  double origin_y,
  double yaw_rad,
  double min_range_m,
  double max_range_m,
  double fov_rad,
  double yaw_offset_rad = 0.0);

}  // namespace frontier_exploration_ros2
