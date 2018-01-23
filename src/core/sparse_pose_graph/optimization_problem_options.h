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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIONS_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIONS_H_

#include "src/common/lua_parameter_dictionary.h"
#include "src/core/scan_matching/ceres_scan_matcher.h"

namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

class OptimizationProblemOptions
{
public:
  void Create(common::LuaParameterDictionary *const parameter_dictionary);
      double huber_scale_;
      double acceleration_weight_;
      double rotation_weight_;
      double consecutive_scan_translation_penalty_factor_;
      double consecutive_scan_rotation_penalty_factor_;
      double fixed_frame_pose_translation_weight_;
      double fixed_frame_pose_rotation_weight_;
      bool log_solver_summary_;
      scan_matching::CeresSolverOptions ceres_solver_options_;
};

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIONS_H_
