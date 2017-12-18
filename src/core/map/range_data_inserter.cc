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

#include "range_data_inserter.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "src/core/map/ray_casting.h"
#include "src/core/map/xy_index.h"
#include "glog/logging.h"

namespace sample_carto
{
namespace core
{
namespace map
{

RangeDataInserter::RangeDataInserter(const RangeDataInserterOptions& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyOdds(
          Odds(options.hit_probability_))),
      miss_table_(ComputeLookupTableToApplyOdds(
          Odds(options.miss_probability_))) {}

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               ProbabilityGrid* const probability_grid) const {
    // By not finishing the update after hits are inserted, we give hits priority
    // (i.e. no hits will be ignored because of a miss in the same cell).
    CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space_,
             CHECK_NOTNULL(probability_grid));
    probability_grid->FinishUpdate();
}

void RangeDataInserterOptions::Create(common::LuaParameterDictionary *const parameter_dictionary)
{
    hit_probability_ = parameter_dictionary->GetDouble("hit_probability");
    miss_probability_ = parameter_dictionary->GetDouble("miss_probability");
    insert_free_space_ = parameter_dictionary->GetBool("insert_free_space");
    CHECK_GT(hit_probability_, 0.5);
    CHECK_LT(miss_probability_, 0.5);
}
}
} // namespace core
} // namespace sample_carto
