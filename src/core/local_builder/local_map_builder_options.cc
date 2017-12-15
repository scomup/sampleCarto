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

#include "src/core/local_builder/local_map_builder_options.h"
#include "src/core/local_builder/motion_filter.h"

namespace sample_carto
{
namespace core
{

LocalMapBuilderOptions::LocalMapBuilderOptions(common::LuaParameterDictionary *const parameter_dictionary)
{

    min_range_ = parameter_dictionary->GetDouble("min_range");
    max_range_ = parameter_dictionary->GetDouble("max_range");
    min_z_ = parameter_dictionary->GetDouble("min_z");
    max_z_ = parameter_dictionary->GetDouble("max_z");
    missing_data_ray_length_ = parameter_dictionary->GetDouble("missing_data_ray_length");
    scans_per_accumulation_ = parameter_dictionary->GetInt("scans_per_accumulation");
    use_online_correlative_scan_matching_ = parameter_dictionary->GetBool("use_online_correlative_scan_matching");
    motion_filter_options_.Create(parameter_dictionary->GetDictionary("motion_filter").get());
}

} // namespace core
} // namespace sample_carto
