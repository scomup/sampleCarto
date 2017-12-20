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

#include "src/core/sparse_pose_graph/constraint_builder_options.h"
#include "src/core/scan_matching/ceres_scan_matcher.h"
#include "src/core/scan_matching/fast_correlative_scan_matcher.h"

namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

void ConstraintBuilderOptions::Create(common::LuaParameterDictionary *const parameter_dictionary)
{
    sampling_ratio_ = parameter_dictionary->GetDouble("sampling_ratio");
    max_constraint_distance_ = parameter_dictionary->GetDouble("max_constraint_distance");
    min_score_ = parameter_dictionary->GetDouble("min_score");
    global_localization_min_score_ = parameter_dictionary->GetDouble("global_localization_min_score");
    loop_closure_translation_weight_ = parameter_dictionary->GetDouble("loop_closure_translation_weight");
    loop_closure_rotation_weight_ = parameter_dictionary->GetDouble("loop_closure_rotation_weight");
    log_matches_ = parameter_dictionary->GetBool("log_matches");
    fast_correlative_scan_matcher_options_.Create(parameter_dictionary->GetDictionary("fast_correlative_scan_matcher").get());
    ceres_scan_matcher_options_.Create(parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
};

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto
