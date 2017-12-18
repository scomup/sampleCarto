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

#ifndef SAMPLE_CARTO_CORE_MAP_RAY_CASTING_H_
#define SAMPLE_CARTO_CORE_MAP_RAY_CASTING_H_

#include <vector>

#include "src/common/port.h"
#include "src/core/map/probability_grid.h"
#include "src/sensor/point_cloud.h"
#include "src/sensor/range_data.h"
#include "src/transform/transform.h"

namespace sample_carto {
namespace core {
namespace map {
// For each ray in 'range_data', inserts hits and misses into
// 'probability_grid'. Hits are handled before misses.
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table, bool insert_free_space,
              ProbabilityGrid* probability_grid);
}
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_MAP_RAY_CASTING_H_
