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

#ifndef SAMPLE_CARTO_CORE_MAP_RANGE_DATA_INSERTER_H_
#define SAMPLE_CARTO_CORE_MAP_RANGE_DATA_INSERTER_H_

#include <utility>
#include <vector>

#include "src/common/lua_parameter_dictionary.h"
#include "src/common/port.h"
#include "src/core/map/probability_grid.h"
#include "src/core/map/xy_index.h"
#include "src/sensor/point_cloud.h"
#include "src/sensor/range_data.h"

namespace sample_carto {
namespace core {
namespace map {

class RangeDataInserterOptions
{
  public:
    void Create(common::LuaParameterDictionary *parameter_dictionary);
    double hit_probability_;
    double miss_probability_;
    bool insert_free_space_;
};

class RangeDataInserter
{
  public:
    RangeDataInserter(const RangeDataInserterOptions& options);
    RangeDataInserter(const RangeDataInserter &) = delete;
    RangeDataInserter &operator=(const RangeDataInserter &) = delete;

    // Inserts 'range_data' into 'probability_grid'.
    void Insert(const sensor::RangeData &range_data,
                ProbabilityGrid *probability_grid) const;

  private:
    const RangeDataInserterOptions options_;
    const std::vector<uint16> hit_table_;
    const std::vector<uint16> miss_table_;
};
}
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_MAP_RANGE_DATA_INSERTER_H_
