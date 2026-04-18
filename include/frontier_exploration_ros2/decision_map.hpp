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
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

// Paper-style grayscale encoding used by the decision-map filtering pipeline.
constexpr uint8_t PAPER_FREE = 255;
constexpr uint8_t PAPER_UNKNOWN = 205;
constexpr uint8_t PAPER_OCCUPIED = 0;

// Lightweight image wrapper for the paper-domain map representation.
struct PaperImage
{
  int width{0};
  int height{0};
  std::vector<uint8_t> data;

  [[nodiscard]] bool valid() const
  {
    return width > 0 && height > 0 && data.size() == static_cast<std::size_t>(width * height);
  }

  [[nodiscard]] uint8_t at(int x, int y) const
  {
    return data[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x)];
  }

  uint8_t & at(int x, int y)
  {
    return data[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x)];
  }
};

// Integer kernel offset reused by morphology-style operations.
struct KernelOffset
{
  int dx{0};
  int dy{0};
};

// Spatial kernel sample with its precomputed Gaussian weight.
struct WeightedKernelOffset
{
  int dx{0};
  int dy{0};
  float weight{0.0F};
};

// Runtime knobs for converting an occupancy map into a smoothed decision map.
struct DecisionMapConfig
{
  // When disabled, the output mirrors the raw occupancy classification without smoothing.
  bool optimization_enabled{true};
  // Threshold that separates free cells from occupied cells in the source occupancy grid.
  int occ_threshold{OCC_THRESHOLD};
  // Spatial sigma for bilateral filtering in paper-image cell units.
  double sigma_s{2.0};
  // Range sigma for bilateral filtering across occupied/unknown/free paper values.
  double sigma_r{30.0};
  // Radius of the circular dilation used to grow thresholded free-space support.
  int dilation_kernel_radius_cells{1};
};

// Materialized outputs of a full decision-map build.
struct DecisionMapResult
{
  PaperImage raw_image;
  std::vector<float> filtered_image;
  PaperImage threshold_image;
  PaperImage optimized_image;
  nav_msgs::msg::OccupancyGrid optimized_map_msg;
  OccupancyGrid2d decision_map;
};

// Incremental build outcome used by callers to reason about cache reuse.
struct DecisionMapBuildStatus
{
  bool reused_existing_output{false};
  bool output_changed{false};
  bool geometry_changed{false};
};

// Reusable buffers and caches for incremental decision-map updates.
struct DecisionMapWorkspace
{
  bool initialized{false};
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  bool last_optimization_enabled{true};
  int last_occ_threshold{OCC_THRESHOLD};
  double last_build_sigma_s{2.0};
  double last_build_sigma_r{30.0};
  int last_build_dilation_radius_cells{1};

  // Images for the current raw input, staging copy, and optimization outputs.
  PaperImage raw_image;
  PaperImage raw_scratch_image;
  std::vector<float> filtered_image;
  PaperImage threshold_image;
  PaperImage optimized_image;
  PaperImage dilation_scratch;

  // Cached occupancy-to-paper conversion for the active threshold.
  int cached_occ_threshold{OCC_THRESHOLD - 1};
  std::array<uint8_t, 256> occupancy_to_paper_lut{};

  // Cached bilateral-filter kernel samples and linear offsets for the current geometry.
  double cached_sigma_s{-1.0};
  int cached_filter_radius{-1};
  std::vector<WeightedKernelOffset> spatial_kernel_samples;
  int cached_spatial_linear_width{-1};
  std::vector<std::ptrdiff_t> spatial_linear_offsets;

  // Cached range weights across the three paper intensity classes.
  double cached_sigma_r{-1.0};
  std::array<float, 9> range_weight_lut{};

  // Cached circular dilation kernel and linear offsets for the current geometry.
  int cached_dilation_radius{-1};
  std::vector<KernelOffset> dilation_offsets;
  int cached_dilation_linear_width{-1};
  std::vector<std::ptrdiff_t> dilation_linear_offsets;

  // Dirty row spans drive partial recomputation of raw, filter, and dilation stages.
  std::vector<std::vector<std::pair<int, int>>> dirty_spans_by_row;
  std::vector<std::vector<std::pair<int, int>>> filter_spans_by_row;
  std::vector<std::vector<std::pair<int, int>>> dilation_spans_by_row;

  nav_msgs::msg::OccupancyGrid::SharedPtr optimized_map_msg{
    std::make_shared<nav_msgs::msg::OccupancyGrid>()};
};

// Converts occupancy costs into the paper-domain image used by the optimization pipeline.
PaperImage occupancy_grid_to_paper_image(
  const OccupancyGrid2d & occupancy_map,
  int occ_threshold);

// Converts a paper-domain image back into a standard occupancy grid message.
nav_msgs::msg::OccupancyGrid paper_image_to_occupancy_grid(
  const PaperImage & image,
  const nav_msgs::msg::OccupancyGrid & reference);

// Stateless bilateral filter helper used by tests and one-shot callers.
std::vector<float> bilateral_filter(
  const PaperImage & image,
  double sigma_s,
  double sigma_r);

// Incrementally rebuilds the decision map using reusable workspace caches.
DecisionMapBuildStatus build_decision_map(
  const OccupancyGrid2d & raw_map,
  const DecisionMapConfig & config,
  DecisionMapWorkspace & workspace);

// Convenience overload that builds the decision map without retaining caches.
DecisionMapResult build_decision_map(
  const OccupancyGrid2d & raw_map,
  const DecisionMapConfig & config);

}  // namespace frontier_exploration_ros2
