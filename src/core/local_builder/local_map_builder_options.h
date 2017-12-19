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

#ifndef SAMPLE_CARTOCORE_2D_LOCAL_MAP_BUILDER_OPTIONS_H_
#define SAMPLE_CARTOCORE_2D_LOCAL_MAP_BUILDER_OPTIONS_H_

#include "src/common/lua_parameter_dictionary.h"
#include "src/core/local_builder/motion_filter.h"
#include "src/core/map/submaps.h"

namespace sample_carto {
namespace core {

class LocalMapBuilderOptions
{
  public:
    void Create(common::LuaParameterDictionary* const parameter_dictionary);
    double min_range_;
    double max_range_;
    double min_z_;
    double max_z_;
    double missing_data_ray_length_;
    int scans_per_accumulation_;
    bool use_online_correlative_scan_matching_;
    MotionFilterOptions motion_filter_options_; 
    map::SubmapsOptions submaps_options_; 
};

}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTOCORE_2D_LOCAL_MAP_BUILDER_OPTIONS_H_
