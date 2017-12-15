/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SAMPLE_CARTO_CORE_MAP_PROBABILITY_GRID_H_
#define SAMPLE_CARTO_CORE_MAP_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "src/common/math.h"
#include "src/common/port.h"
#include "src/core/map/probability_values.h"
#include "src/core/map/map_limits.h"
#include "src/core/map/xy_index.h"

namespace sample_carto
{
namespace core
{
namespace map
{

// Represents a 2D grid of probabilities.
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               kUnknownProbabilityValue) {}


  // Returns the limits of this ProbabilityGrid.
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      assert(cells_[update_indices_.back()] >= kUpdateMarker);
      cells_[update_indices_.back()] -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability) {
    uint16& cell = cells_[ToFlatIndex(cell_index)];
    CHECK_EQ(cell, kUnknownProbabilityValue);
    cell = ProbabilityToValue(probability);
    known_cells_box_.extend(cell_index.matrix());
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table) {
    assert(table.size() >= kUpdateMarker);
    const int flat_index = ToFlatIndex(cell_index);
    uint16& cell = cells_[flat_index];
    if (cell >= kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(flat_index);
    cell = table[cell];
    assert(cell >= kUpdateMarker);
    known_cells_box_.extend(cell_index.matrix());
    return true;
  }

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const {
    if (limits_.Contains(cell_index)) {
      return ValueToProbability(cells_[ToFlatIndex(cell_index)]);
    }
    return kMinProbability;
  }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           cells_[ToFlatIndex(cell_index)] != kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    if (known_cells_box_.isEmpty()) {
      *offset = Eigen::Array2i::Zero();
      *limits = CellLimits(1, 1);
    } else {
      *offset = known_cells_box_.min().array();
      *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                           known_cells_box_.sizes().y() + 1);
    }
  }

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  void GrowLimits(const Eigen::Vector2f &point)
  {
    assert(update_indices_.empty());
    while (!limits_.Contains(limits_.GetCellIndex(point)))
    {
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));
      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      std::vector<uint16> new_cells(new_size,
                                    kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i)
      {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j)
        {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      if (!known_cells_box_.isEmpty())
      {
        known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
      }
    }
  }


 private:
  // Converts a 'cell_index' into an index into 'cells_'.
  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    assert(limits_.Contains(cell_index));
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker.
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
};

}
}
}

#endif  // SAMPLE_CARTO_CORE_MAP_PROBABILITY_GRID_H_
