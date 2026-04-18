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

#include "frontier_exploration_ros2/decision_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kGeometryEpsilon = 1e-9;
// Threshold between UNKNOWN and FREE after filtering the paper-domain intensities.
constexpr float kFreeThreshold =
  static_cast<float>((static_cast<int>(PAPER_FREE) + static_cast<int>(PAPER_UNKNOWN)) / 2.0);

using RowSpan = std::pair<int, int>;
using RowSpanRows = std::vector<std::vector<RowSpan>>;

// Returns true when no row contains a dirty interval.
[[nodiscard]] bool row_spans_empty(const RowSpanRows & row_spans)
{
  for (const auto & row : row_spans) {
    if (!row.empty()) {
      return false;
    }
  }
  return true;
}

void clear_row_spans(RowSpanRows & row_spans, int height)
{
  row_spans.resize(static_cast<std::size_t>(std::max(0, height)));
  for (auto & row : row_spans) {
    row.clear();
  }
}

// Marks every row as dirty so downstream stages fully recompute.
void assign_full_row_spans(RowSpanRows & row_spans, int width, int height)
{
  clear_row_spans(row_spans, height);
  if (width <= 0) {
    return;
  }

  for (auto & row : row_spans) {
    row.emplace_back(0, width - 1);
  }
}

// Geometry changes invalidate the reusable buffers and linear-offset caches.
[[nodiscard]] bool same_geometry(
  const DecisionMapWorkspace & workspace,
  const nav_msgs::msg::OccupancyGrid & reference)
{
  return (
    workspace.width == static_cast<int>(reference.info.width) &&
    workspace.height == static_cast<int>(reference.info.height) &&
    std::abs(workspace.resolution - reference.info.resolution) <= kGeometryEpsilon &&
    std::abs(workspace.origin_x - reference.info.origin.position.x) <= kGeometryEpsilon &&
    std::abs(workspace.origin_y - reference.info.origin.position.y) <= kGeometryEpsilon);
}

// Resizes image storage without changing the existing semantic contents.
void assign_image_geometry(PaperImage & image, int width, int height)
{
  image.width = width;
  image.height = height;
  image.data.resize(static_cast<std::size_t>(width * height));
}

// Rebuilds workspace-owned buffers whenever map geometry changes.
void resize_workspace_for_geometry(
  DecisionMapWorkspace & workspace,
  const nav_msgs::msg::OccupancyGrid & reference)
{
  const int width = static_cast<int>(reference.info.width);
  const int height = static_cast<int>(reference.info.height);
  const std::size_t cell_count = static_cast<std::size_t>(width * height);

  workspace.width = width;
  workspace.height = height;
  workspace.resolution = reference.info.resolution;
  workspace.origin_x = reference.info.origin.position.x;
  workspace.origin_y = reference.info.origin.position.y;

  assign_image_geometry(workspace.raw_image, width, height);
  assign_image_geometry(workspace.raw_scratch_image, width, height);
  assign_image_geometry(workspace.threshold_image, width, height);
  assign_image_geometry(workspace.optimized_image, width, height);
  workspace.dilation_scratch.width = height;
  workspace.dilation_scratch.height = 1;
  workspace.dilation_scratch.data.resize(static_cast<std::size_t>(height), 0U);
  workspace.filtered_image.resize(cell_count, 0.0F);

  clear_row_spans(workspace.dirty_spans_by_row, height);
  clear_row_spans(workspace.filter_spans_by_row, height);
  clear_row_spans(workspace.dilation_spans_by_row, height);

  workspace.optimized_map_msg->header = reference.header;
  workspace.optimized_map_msg->info = reference.info;
  workspace.optimized_map_msg->data.resize(cell_count, -1);
}

// Maps occupancy costs into the three paper-domain intensity classes.
[[nodiscard]] uint8_t occupancy_cost_to_paper_value(int cost, int occ_threshold)
{
  if (cost >= occ_threshold) {
    return PAPER_OCCUPIED;
  }
  if (cost >= 0) {
    return PAPER_FREE;
  }
  return PAPER_UNKNOWN;
}

// Precomputes all possible int8 occupancy conversions for the current threshold.
void fill_occupancy_to_paper_lut(std::array<uint8_t, 256> & lut, int occ_threshold)
{
  for (int i = 0; i < 256; ++i) {
    lut[static_cast<std::size_t>(i)] = occupancy_cost_to_paper_value(
      static_cast<int>(static_cast<int8_t>(i)),
      occ_threshold);
  }
}

// The LUT changes only when the occupancy threshold changes.
void ensure_occupancy_to_paper_lut_cache(DecisionMapWorkspace & workspace, int occ_threshold)
{
  if (workspace.cached_occ_threshold == occ_threshold) {
    return;
  }

  fill_occupancy_to_paper_lut(workspace.occupancy_to_paper_lut, occ_threshold);
  workspace.cached_occ_threshold = occ_threshold;
}

// Compacts paper intensities into three indices for the range-weight lookup table.
[[nodiscard]] int paper_value_index(uint8_t value)
{
  if (value == PAPER_OCCUPIED) {
    return 0;
  }
  if (value == PAPER_UNKNOWN) {
    return 1;
  }
  return 2;
}

// Precomputes the spatial Gaussian kernel used by the bilateral filter.
void ensure_spatial_kernel_cache(DecisionMapWorkspace & workspace, double sigma_s)
{
  const double safe_sigma_s = std::max(sigma_s, 1e-6);
  const int radius = std::max(1, static_cast<int>(std::ceil(2.0 * safe_sigma_s)));
  if (
    workspace.cached_filter_radius == radius &&
    std::abs(workspace.cached_sigma_s - safe_sigma_s) <= kGeometryEpsilon)
  {
    return;
  }

  workspace.cached_sigma_s = safe_sigma_s;
  workspace.cached_filter_radius = radius;
  workspace.cached_spatial_linear_width = -1;
  workspace.spatial_kernel_samples.clear();
  workspace.spatial_kernel_samples.reserve(
    static_cast<std::size_t>((radius * 2 + 1) * (radius * 2 + 1)));

  const double denominator = 2.0 * safe_sigma_s * safe_sigma_s;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const double distance_sq = static_cast<double>((dx * dx) + (dy * dy));
      workspace.spatial_kernel_samples.push_back(
        WeightedKernelOffset{
          dx,
          dy,
          static_cast<float>(std::exp(-(distance_sq / denominator))),
        });
    }
  }
}

// Converts 2-D filter offsets into linear offsets for the current map width.
void ensure_spatial_linear_offset_cache(DecisionMapWorkspace & workspace)
{
  if (workspace.cached_spatial_linear_width == workspace.width) {
    return;
  }

  workspace.cached_spatial_linear_width = workspace.width;
  workspace.spatial_linear_offsets.clear();
  workspace.spatial_linear_offsets.reserve(workspace.spatial_kernel_samples.size());
  for (const auto & sample : workspace.spatial_kernel_samples) {
    workspace.spatial_linear_offsets.push_back(
      static_cast<std::ptrdiff_t>(sample.dy) * static_cast<std::ptrdiff_t>(workspace.width) +
      static_cast<std::ptrdiff_t>(sample.dx));
  }
}

// Precomputes bilateral range weights for occupied/unknown/free paper values.
void ensure_range_lut_cache(DecisionMapWorkspace & workspace, double sigma_r)
{
  const double safe_sigma_r = std::max(sigma_r, 1e-6);
  if (std::abs(workspace.cached_sigma_r - safe_sigma_r) <= kGeometryEpsilon) {
    return;
  }

  workspace.cached_sigma_r = safe_sigma_r;
  constexpr std::array<uint8_t, 3> paper_values{PAPER_OCCUPIED, PAPER_UNKNOWN, PAPER_FREE};
  const double denominator = 2.0 * safe_sigma_r * safe_sigma_r;
  for (std::size_t center_index = 0; center_index < paper_values.size(); ++center_index) {
    for (std::size_t shifted_index = 0; shifted_index < paper_values.size(); ++shifted_index) {
      const double diff =
        static_cast<double>(paper_values[center_index]) -
        static_cast<double>(paper_values[shifted_index]);
      workspace.range_weight_lut[center_index * paper_values.size() + shifted_index] =
        static_cast<float>(std::exp(-((diff * diff) / denominator)));
    }
  }
}

// Builds the circular dilation stencil used after thresholding.
void ensure_dilation_kernel_cache(DecisionMapWorkspace & workspace, int radius)
{
  const int safe_radius = std::max(0, radius);
  if (workspace.cached_dilation_radius == safe_radius) {
    return;
  }

  workspace.cached_dilation_radius = safe_radius;
  workspace.cached_dilation_linear_width = -1;
  workspace.dilation_offsets.clear();
  workspace.dilation_offsets.reserve(
    static_cast<std::size_t>((safe_radius * 2 + 1) * (safe_radius * 2 + 1)));
  for (int dy = -safe_radius; dy <= safe_radius; ++dy) {
    for (int dx = -safe_radius; dx <= safe_radius; ++dx) {
      if ((dx * dx) + (dy * dy) <= (safe_radius * safe_radius)) {
        workspace.dilation_offsets.push_back(KernelOffset{dx, dy});
      }
    }
  }
}

// Converts dilation offsets into linear form for fast interior-cell access.
void ensure_dilation_linear_offset_cache(DecisionMapWorkspace & workspace)
{
  if (workspace.cached_dilation_linear_width == workspace.width) {
    return;
  }

  workspace.cached_dilation_linear_width = workspace.width;
  workspace.dilation_linear_offsets.clear();
  workspace.dilation_linear_offsets.reserve(workspace.dilation_offsets.size());
  for (const auto & offset : workspace.dilation_offsets) {
    workspace.dilation_linear_offsets.push_back(
      static_cast<std::ptrdiff_t>(offset.dy) * static_cast<std::ptrdiff_t>(workspace.width) +
      static_cast<std::ptrdiff_t>(offset.dx));
  }
}

// Coalesces overlapping or adjacent dirty intervals within one row.
void normalize_row_spans(std::vector<RowSpan> & row_spans)
{
  if (row_spans.empty()) {
    return;
  }

  std::sort(row_spans.begin(), row_spans.end());

  std::size_t write_index = 0;
  for (std::size_t read_index = 1; read_index < row_spans.size(); ++read_index) {
    if (row_spans[read_index].first <= row_spans[write_index].second + 1) {
      row_spans[write_index].second =
        std::max(row_spans[write_index].second, row_spans[read_index].second);
      continue;
    }

    ++write_index;
    row_spans[write_index] = row_spans[read_index];
  }

  row_spans.resize(write_index + 1);
}

// Expands dirty spans by each stage's support radius so only affected cells are recomputed.
void build_expanded_row_spans(
  const RowSpanRows & source_spans_by_row,
  int horizontal_radius,
  int vertical_radius,
  int width,
  int height,
  DecisionMapWorkspace & workspace,
  RowSpanRows & target_spans_by_row)
{
  clear_row_spans(target_spans_by_row, height);
  if (width <= 0 || height <= 0) {
    return;
  }

  auto & touched_rows = workspace.dilation_scratch.data;
  if (touched_rows.size() < static_cast<std::size_t>(height)) {
    touched_rows.resize(static_cast<std::size_t>(height), 0U);
  }
  std::fill_n(touched_rows.begin(), height, 0U);

  const int safe_horizontal_radius = std::max(0, horizontal_radius);
  const int safe_vertical_radius = std::max(0, vertical_radius);

  for (int y = 0; y < height; ++y) {
    for (const auto & span : source_spans_by_row[static_cast<std::size_t>(y)]) {
      const int expanded_start = std::max(0, span.first - safe_horizontal_radius);
      const int expanded_end = std::min(width - 1, span.second + safe_horizontal_radius);
      const int row_min = std::max(0, y - safe_vertical_radius);
      const int row_max = std::min(height - 1, y + safe_vertical_radius);
      for (int target_y = row_min; target_y <= row_max; ++target_y) {
        target_spans_by_row[static_cast<std::size_t>(target_y)].emplace_back(
          expanded_start,
          expanded_end);
        touched_rows[static_cast<std::size_t>(target_y)] = 1U;
      }
    }
  }

  for (int y = 0; y < height; ++y) {
    if (touched_rows[static_cast<std::size_t>(y)] != 0U) {
      normalize_row_spans(target_spans_by_row[static_cast<std::size_t>(y)]);
    }
  }
}

// Re-materializes the raw paper image and records which cells actually changed.
void populate_raw_scratch_and_detect_dirty(
  const OccupancyGrid2d & occupancy_map,
  DecisionMapWorkspace & workspace,
  bool geometry_changed)
{
  clear_row_spans(workspace.dirty_spans_by_row, workspace.height);

  const auto & map_data = occupancy_map.map().data;
  for (int y = 0; y < workspace.height; ++y) {
    const std::size_t row_offset =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(workspace.width);
    int span_start = -1;

    for (int x = 0; x < workspace.width; ++x) {
      const std::size_t idx = row_offset + static_cast<std::size_t>(x);
      const uint8_t paper_value =
        workspace.occupancy_to_paper_lut[static_cast<uint8_t>(map_data[idx])];
      workspace.raw_scratch_image.data[idx] = paper_value;

      const bool changed =
        geometry_changed ||
        !workspace.initialized ||
        workspace.raw_image.data[idx] != paper_value;

      if (changed) {
        if (span_start < 0) {
          span_start = x;
        }
      } else if (span_start >= 0) {
        workspace.dirty_spans_by_row[static_cast<std::size_t>(y)].emplace_back(span_start, x - 1);
        span_start = -1;
      }
    }

    if (span_start >= 0) {
      workspace.dirty_spans_by_row[static_cast<std::size_t>(y)].emplace_back(
        span_start,
        workspace.width - 1);
    }
  }
}

// Recomputes only the requested row spans for bilateral filtering and thresholding.
void recompute_filtered_threshold_row_spans(
  const PaperImage & raw_image,
  std::vector<float> & filtered_image,
  PaperImage * threshold_image,
  const RowSpanRows & row_spans_by_row,
  const DecisionMapWorkspace & workspace)
{
  const auto * raw_data = raw_image.data.data();
  auto * filtered_data = filtered_image.data();
  auto * threshold_data = threshold_image ? threshold_image->data.data() : nullptr;
  const auto * range_lut = workspace.range_weight_lut.data();
  const int radius = workspace.cached_filter_radius;
  const int width = raw_image.width;
  const int height = raw_image.height;

  // Border cells clamp neighborhood access back into image bounds.
  auto recompute_border_cell = [&](int x, int y, std::size_t idx) {
      const uint8_t center_value = raw_data[idx];
      const int center_index = paper_value_index(center_value);
      float weighted_sum = 0.0F;
      float normalization = 0.0F;

      for (const auto & sample : workspace.spatial_kernel_samples) {
        const int sample_x = std::clamp(x + sample.dx, 0, width - 1);
        const int sample_y = std::clamp(y + sample.dy, 0, height - 1);
        const uint8_t shifted_value =
          raw_data[static_cast<std::size_t>(sample_y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(sample_x)];
        const int shifted_index = paper_value_index(shifted_value);
        const float weight =
          sample.weight *
          range_lut[static_cast<std::size_t>(center_index * 3 + shifted_index)];
        weighted_sum += weight * static_cast<float>(shifted_value);
        normalization += weight;
      }

      filtered_data[idx] =
        normalization > std::numeric_limits<float>::epsilon() ?
        (weighted_sum / normalization) :
        static_cast<float>(center_value);

      if (threshold_data != nullptr) {
        threshold_data[idx] =
          center_value == PAPER_OCCUPIED ?
          PAPER_OCCUPIED :
          (filtered_data[idx] >= kFreeThreshold ? PAPER_FREE : PAPER_UNKNOWN);
      }
    };

  // Interior cells can use precomputed linear offsets with no per-sample bounds checks.
  auto recompute_interior_cell = [&](std::size_t idx) {
      const uint8_t center_value = raw_data[idx];
      const int center_index = paper_value_index(center_value);
      float weighted_sum = 0.0F;
      float normalization = 0.0F;
      const auto base_index = static_cast<std::ptrdiff_t>(idx);

      for (std::size_t sample_index = 0; sample_index < workspace.spatial_kernel_samples.size();
        ++sample_index)
      {
        const uint8_t shifted_value =
          raw_data[static_cast<std::size_t>(base_index + workspace.spatial_linear_offsets[sample_index])];
        const int shifted_index = paper_value_index(shifted_value);
        const float weight =
          workspace.spatial_kernel_samples[sample_index].weight *
          range_lut[static_cast<std::size_t>(center_index * 3 + shifted_index)];
        weighted_sum += weight * static_cast<float>(shifted_value);
        normalization += weight;
      }

      filtered_data[idx] =
        normalization > std::numeric_limits<float>::epsilon() ?
        (weighted_sum / normalization) :
        static_cast<float>(center_value);

      if (threshold_data != nullptr) {
        threshold_data[idx] =
          center_value == PAPER_OCCUPIED ?
          PAPER_OCCUPIED :
          (filtered_data[idx] >= kFreeThreshold ? PAPER_FREE : PAPER_UNKNOWN);
      }
    };

  for (int y = 0; y < height; ++y) {
    const auto & row_spans = row_spans_by_row[static_cast<std::size_t>(y)];
    if (row_spans.empty()) {
      continue;
    }

    const std::size_t row_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    const bool interior_y = y >= radius && y < (height - radius);

    for (const auto & span : row_spans) {
      const int border_left_end = interior_y ? std::min(span.second, radius - 1) : span.second;
      for (int x = span.first; x <= border_left_end; ++x) {
        recompute_border_cell(x, y, row_offset + static_cast<std::size_t>(x));
      }

      if (interior_y) {
        const int interior_start = std::max(span.first, radius);
        const int interior_end = std::min(span.second, width - radius - 1);
        for (int x = interior_start; x <= interior_end; ++x) {
          recompute_interior_cell(row_offset + static_cast<std::size_t>(x));
        }

        const int border_right_start = std::max(interior_end + 1, span.first);
        for (int x = border_right_start; x <= span.second; ++x) {
          if (x < radius || x >= width - radius) {
            recompute_border_cell(x, y, row_offset + static_cast<std::size_t>(x));
          }
        }
      }
    }
  }
}

[[nodiscard]] int8_t paper_value_to_occupancy_cost(uint8_t value)
{
  if (value == PAPER_FREE) {
    return 0;
  }
  if (value == PAPER_OCCUPIED) {
    return 100;
  }
  return -1;
}

// Fast path used when optimization is disabled and raw classifications should be forwarded.
[[nodiscard]] bool copy_raw_to_output_row_spans(
  const PaperImage & raw_image,
  std::vector<float> & filtered_image,
  PaperImage & threshold_image,
  PaperImage & optimized_image,
  nav_msgs::msg::OccupancyGrid & optimized_map_msg,
  const RowSpanRows & row_spans_by_row,
  bool force_write_output)
{
  bool output_changed = false;
  for (int y = 0; y < raw_image.height; ++y) {
    const auto & row_spans = row_spans_by_row[static_cast<std::size_t>(y)];
    if (row_spans.empty()) {
      continue;
    }

    const std::size_t row_offset =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(raw_image.width);
    for (const auto & span : row_spans) {
      for (int x = span.first; x <= span.second; ++x) {
        const std::size_t idx = row_offset + static_cast<std::size_t>(x);
        const uint8_t raw_value = raw_image.data[idx];
        filtered_image[idx] = static_cast<float>(raw_value);
        threshold_image.data[idx] = raw_value;
        if (optimized_image.data[idx] != raw_value) {
          optimized_image.data[idx] = raw_value;
          output_changed = true;
        } else if (force_write_output) {
          optimized_image.data[idx] = raw_value;
        }

        if (force_write_output || optimized_map_msg.data[idx] != paper_value_to_occupancy_cost(raw_value)) {
          optimized_map_msg.data[idx] = paper_value_to_occupancy_cost(raw_value);
        }
      }
    }
  }

  return output_changed;
}

// Applies circular dilation over thresholded free-space support on the requested spans.
[[nodiscard]] bool recompute_dilation_row_spans(
  const PaperImage & raw_image,
  const PaperImage & threshold_image,
  PaperImage & optimized_image,
  nav_msgs::msg::OccupancyGrid & optimized_map_msg,
  const RowSpanRows & row_spans_by_row,
  const DecisionMapWorkspace & workspace,
  bool force_write_output)
{
  const auto * raw_data = raw_image.data.data();
  const auto * threshold_data = threshold_image.data.data();
  auto * optimized_data = optimized_image.data.data();
  auto * optimized_msg_data = optimized_map_msg.data.data();
  const int radius = workspace.cached_dilation_radius;
  const int width = raw_image.width;
  const int height = raw_image.height;
  bool output_changed = false;

  // Border cells keep explicit bounds checks because the stencil may cross image edges.
  auto recompute_border_cell = [&](int x, int y, std::size_t idx) {
      uint8_t optimized_value = PAPER_UNKNOWN;
      if (raw_data[idx] == PAPER_OCCUPIED) {
        optimized_value = PAPER_OCCUPIED;
      } else {
        for (const auto & offset : workspace.dilation_offsets) {
          const int sample_x = x + offset.dx;
          const int sample_y = y + offset.dy;
          if (
            sample_x < 0 || sample_y < 0 ||
            sample_x >= width || sample_y >= height)
          {
            continue;
          }

          if (threshold_data[
              static_cast<std::size_t>(sample_y) * static_cast<std::size_t>(width) +
              static_cast<std::size_t>(sample_x)] == PAPER_FREE)
          {
            optimized_value = PAPER_FREE;
            break;
          }
        }
      }

      if (optimized_data[idx] != optimized_value) {
        optimized_data[idx] = optimized_value;
        output_changed = true;
      } else if (force_write_output) {
        optimized_data[idx] = optimized_value;
      }

      const int8_t occupancy_value = paper_value_to_occupancy_cost(optimized_value);
      if (force_write_output || optimized_msg_data[idx] != occupancy_value) {
        optimized_msg_data[idx] = occupancy_value;
      }
    };

  // Interior cells reuse linear offsets and skip per-neighbor coordinate math.
  auto recompute_interior_cell = [&](std::size_t idx) {
      uint8_t optimized_value = PAPER_UNKNOWN;
      if (raw_data[idx] == PAPER_OCCUPIED) {
        optimized_value = PAPER_OCCUPIED;
      } else {
        const auto base_index = static_cast<std::ptrdiff_t>(idx);
        for (const auto linear_offset : workspace.dilation_linear_offsets) {
          if (threshold_data[static_cast<std::size_t>(base_index + linear_offset)] == PAPER_FREE) {
            optimized_value = PAPER_FREE;
            break;
          }
        }
      }

      if (optimized_data[idx] != optimized_value) {
        optimized_data[idx] = optimized_value;
        output_changed = true;
      } else if (force_write_output) {
        optimized_data[idx] = optimized_value;
      }

      const int8_t occupancy_value = paper_value_to_occupancy_cost(optimized_value);
      if (force_write_output || optimized_msg_data[idx] != occupancy_value) {
        optimized_msg_data[idx] = occupancy_value;
      }
    };

  for (int y = 0; y < height; ++y) {
    const auto & row_spans = row_spans_by_row[static_cast<std::size_t>(y)];
    if (row_spans.empty()) {
      continue;
    }

    const std::size_t row_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    const bool interior_y = y >= radius && y < (height - radius);

    for (const auto & span : row_spans) {
      const int border_left_end = interior_y ? std::min(span.second, radius - 1) : span.second;
      for (int x = span.first; x <= border_left_end; ++x) {
        recompute_border_cell(x, y, row_offset + static_cast<std::size_t>(x));
      }

      if (interior_y) {
        const int interior_start = std::max(span.first, radius);
        const int interior_end = std::min(span.second, width - radius - 1);
        for (int x = interior_start; x <= interior_end; ++x) {
          recompute_interior_cell(row_offset + static_cast<std::size_t>(x));
        }

        const int border_right_start = std::max(interior_end + 1, span.first);
        for (int x = border_right_start; x <= span.second; ++x) {
          if (x < radius || x >= width - radius) {
            recompute_border_cell(x, y, row_offset + static_cast<std::size_t>(x));
          }
        }
      }
    }
  }

  return output_changed;
}

}  // namespace

PaperImage occupancy_grid_to_paper_image(
  const OccupancyGrid2d & occupancy_map,
  int occ_threshold)
{
  PaperImage image;
  const auto & grid = occupancy_map.map();
  image.width = static_cast<int>(grid.info.width);
  image.height = static_cast<int>(grid.info.height);
  image.data.resize(static_cast<std::size_t>(image.width * image.height), PAPER_UNKNOWN);

  std::array<uint8_t, 256> occupancy_to_paper_lut{};
  fill_occupancy_to_paper_lut(occupancy_to_paper_lut, occ_threshold);
  for (std::size_t i = 0; i < image.data.size(); ++i) {
    image.data[i] = occupancy_to_paper_lut[static_cast<uint8_t>(grid.data[i])];
  }

  return image;
}

nav_msgs::msg::OccupancyGrid paper_image_to_occupancy_grid(
  const PaperImage & image,
  const nav_msgs::msg::OccupancyGrid & reference)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = reference.header;
  msg.info = reference.info;
  msg.data.resize(image.data.size(), -1);

  for (std::size_t i = 0; i < image.data.size(); ++i) {
    msg.data[i] = paper_value_to_occupancy_cost(image.data[i]);
  }

  return msg;
}

std::vector<float> bilateral_filter(
  const PaperImage & image,
  double sigma_s,
  double sigma_r)
{
  // One-shot helper: build a minimal workspace and treat the whole image as dirty.
  DecisionMapWorkspace workspace;
  workspace.width = image.width;
  workspace.height = image.height;
  workspace.raw_image = image;
  workspace.filtered_image.assign(image.data.size(), 0.0F);
  clear_row_spans(workspace.filter_spans_by_row, image.height);
  assign_full_row_spans(workspace.filter_spans_by_row, image.width, image.height);

  ensure_spatial_kernel_cache(workspace, sigma_s);
  ensure_spatial_linear_offset_cache(workspace);
  ensure_range_lut_cache(workspace, sigma_r);
  recompute_filtered_threshold_row_spans(
    image,
    workspace.filtered_image,
    nullptr,
    workspace.filter_spans_by_row,
    workspace);
  return workspace.filtered_image;
}

DecisionMapBuildStatus build_decision_map(
  const OccupancyGrid2d & raw_map,
  const DecisionMapConfig & config,
  DecisionMapWorkspace & workspace)
{
  DecisionMapBuildStatus status;
  const auto & reference = raw_map.map();
  // Geometry changes require workspace reallocation; config changes require stage-wide recompute.
  const bool geometry_changed = !workspace.initialized || !same_geometry(workspace, reference);
  const bool config_changed =
    !workspace.initialized ||
    workspace.last_optimization_enabled != config.optimization_enabled ||
    workspace.last_occ_threshold != config.occ_threshold ||
    std::abs(workspace.last_build_sigma_s - config.sigma_s) > kGeometryEpsilon ||
    std::abs(workspace.last_build_sigma_r - config.sigma_r) > kGeometryEpsilon ||
    workspace.last_build_dilation_radius_cells != config.dilation_kernel_radius_cells;

  if (geometry_changed) {
    resize_workspace_for_geometry(workspace, reference);
    status.geometry_changed = true;
  } else {
    workspace.optimized_map_msg->header = reference.header;
    workspace.optimized_map_msg->info = reference.info;
  }

  ensure_occupancy_to_paper_lut_cache(workspace, config.occ_threshold);
  populate_raw_scratch_and_detect_dirty(raw_map, workspace, geometry_changed);

  // Any configuration change invalidates stage-local reuse, so mark the full image dirty.
  if (config_changed) {
    assign_full_row_spans(workspace.dirty_spans_by_row, workspace.width, workspace.height);
  }
  if (row_spans_empty(workspace.dirty_spans_by_row)) {
    status.reused_existing_output = true;
    status.output_changed = false;
    return status;
  }

  std::swap(workspace.raw_image.data, workspace.raw_scratch_image.data);
  workspace.raw_image.width = workspace.width;
  workspace.raw_image.height = workspace.height;

  const bool force_full_output_write = status.geometry_changed || !workspace.initialized || config_changed;

  // Disabled optimization means the paper-domain raw classification becomes the final output.
  if (!config.optimization_enabled) {
    status.output_changed = status.geometry_changed;
    if (copy_raw_to_output_row_spans(
        workspace.raw_image,
        workspace.filtered_image,
        workspace.threshold_image,
        workspace.optimized_image,
        *workspace.optimized_map_msg,
        workspace.dirty_spans_by_row,
        force_full_output_write))
    {
      status.output_changed = true;
    }

    workspace.initialized = true;
    workspace.last_optimization_enabled = config.optimization_enabled;
    workspace.last_occ_threshold = config.occ_threshold;
    workspace.last_build_sigma_s = config.sigma_s;
    workspace.last_build_sigma_r = config.sigma_r;
    workspace.last_build_dilation_radius_cells = config.dilation_kernel_radius_cells;
    status.reused_existing_output = !status.output_changed;
    return status;
  }

  ensure_spatial_kernel_cache(workspace, config.sigma_s);
  ensure_spatial_linear_offset_cache(workspace);
  ensure_range_lut_cache(workspace, config.sigma_r);
  ensure_dilation_kernel_cache(workspace, config.dilation_kernel_radius_cells);
  ensure_dilation_linear_offset_cache(workspace);

  // Dirty spans expand by each stage's support so partial recomputation remains exact.
  if (config_changed) {
    assign_full_row_spans(workspace.filter_spans_by_row, workspace.width, workspace.height);
    assign_full_row_spans(workspace.dilation_spans_by_row, workspace.width, workspace.height);
  } else {
    build_expanded_row_spans(
      workspace.dirty_spans_by_row,
      workspace.cached_filter_radius,
      workspace.cached_filter_radius,
      workspace.width,
      workspace.height,
      workspace,
      workspace.filter_spans_by_row);
    build_expanded_row_spans(
      workspace.dirty_spans_by_row,
      workspace.cached_filter_radius + std::max(0, config.dilation_kernel_radius_cells),
      workspace.cached_filter_radius + std::max(0, config.dilation_kernel_radius_cells),
      workspace.width,
      workspace.height,
      workspace,
      workspace.dilation_spans_by_row);
  }

  recompute_filtered_threshold_row_spans(
    workspace.raw_image,
    workspace.filtered_image,
    &workspace.threshold_image,
    workspace.filter_spans_by_row,
    workspace);

  status.output_changed = status.geometry_changed;
  if (recompute_dilation_row_spans(
      workspace.raw_image,
      workspace.threshold_image,
      workspace.optimized_image,
      *workspace.optimized_map_msg,
      workspace.dilation_spans_by_row,
      workspace,
      force_full_output_write))
  {
    status.output_changed = true;
  }

  workspace.initialized = true;
  workspace.last_optimization_enabled = config.optimization_enabled;
  workspace.last_occ_threshold = config.occ_threshold;
  workspace.last_build_sigma_s = config.sigma_s;
  workspace.last_build_sigma_r = config.sigma_r;
  workspace.last_build_dilation_radius_cells = config.dilation_kernel_radius_cells;
  status.reused_existing_output = !status.output_changed;
  return status;
}

DecisionMapResult build_decision_map(
  const OccupancyGrid2d & raw_map,
  const DecisionMapConfig & config)
{
  // Convenience wrapper for callers that do not need incremental workspace reuse.
  DecisionMapWorkspace workspace;
  (void)build_decision_map(raw_map, config, workspace);

  DecisionMapResult result;
  result.raw_image = workspace.raw_image;
  result.filtered_image = workspace.filtered_image;
  result.threshold_image = workspace.threshold_image;
  result.optimized_image = workspace.optimized_image;
  result.optimized_map_msg = *workspace.optimized_map_msg;
  result.decision_map = OccupancyGrid2d(result.optimized_map_msg);
  return result;
}

}  // namespace frontier_exploration_ros2
