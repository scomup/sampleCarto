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

#include "src/core/sparse_pose_graph/optimization_problem_options.h"

namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

void OptimizationProblemOptions::Create(common::LuaParameterDictionary* const parameter_dictionary) {
      acceleration_weight_ = parameter_dictionary->GetDouble("acceleration_weight");
      rotation_weight_ = parameter_dictionary->GetDouble("rotation_weight");
      consecutive_scan_translation_penalty_factor_ = parameter_dictionary->GetDouble("consecutive_scan_translation_penalty_factor");
      consecutive_scan_rotation_penalty_factor_ = parameter_dictionary->GetDouble("consecutive_scan_rotation_penalty_factor");
      fixed_frame_pose_translation_weight_ = parameter_dictionary->GetDouble("fixed_frame_pose_translation_weight");
      fixed_frame_pose_rotation_weight_ = parameter_dictionary->GetDouble("fixed_frame_pose_rotation_weight");
      log_solver_summary_ = parameter_dictionary->GetBool("log_solver_summary");
      ceres_solver_options_.Create(parameter_dictionary->GetDictionary("ceres_solver_options").get());
}

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto
