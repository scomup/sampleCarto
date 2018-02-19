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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_OPTIONS_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_OPTIONS_H_

#include "src/common/lua_parameter_dictionary.h"
#include "src/core/scan_matching/ceres_scan_matcher.h"
#include "src/core/scan_matching/fast_correlative_scan_matcher.h"
#include "src/core/scan_matching/real_time_correlative_scan_matcher.h"

namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

class ConstraintBuilderOptions
{
public:
  void Create(common::LuaParameterDictionary *const parameter_dictionary);
  double sampling_ratio_;
  double max_constraint_distance_;
  double min_score_;
  double global_localization_min_score_;
  double loop_closure_translation_weight_;
  double loop_closure_rotation_weight_;
  bool log_matches_;

  scan_matching::FastCorrelativeScanMatcherOptions fast_correlative_scan_matcher_options_;
  scan_matching::CeresScanMatcherOptions ceres_scan_matcher_options_;
};


}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_OPTIONS_H_
