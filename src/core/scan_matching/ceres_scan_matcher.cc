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

#include "ceres_scan_matcher.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "src/common/lua_parameter_dictionary.h"
#include "src/core/map/probability_grid.h"
#include "src/core/scan_matching/occupied_space_cost_functor.h"
#include "src/core/scan_matching/rotation_delta_cost_functor.h"
#include "src/core/scan_matching/translation_delta_cost_functor.h"
#include "src/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace sample_carto {
namespace core {
namespace scan_matching {


void CeresSolverOptions::Create(common::LuaParameterDictionary* const parameter_dictionary){
    use_nonmonotonic_steps_ = parameter_dictionary->GetBool("use_nonmonotonic_steps");
    max_num_iterations_ = parameter_dictionary->GetNonNegativeInt("max_num_iterations");
    num_threads_ = parameter_dictionary->GetNonNegativeInt("num_threads");
    CHECK_GT(max_num_iterations_, 0);
    CHECK_GT(num_threads_, 0);
}

void CeresScanMatcherOptions::Create(common::LuaParameterDictionary* const parameter_dictionary){
    occupied_space_weight_ = parameter_dictionary->GetDouble("occupied_space_weight");
    translation_weight_ = parameter_dictionary->GetDouble("translation_weight");
    rotation_weight_ = parameter_dictionary->GetDouble("rotation_weight");
    ceres_solver_options_.Create(parameter_dictionary->GetDictionary("ceres_solver_options").get());
}

CeresScanMatcher::CeresScanMatcher(
    const CeresScanMatcherOptions& options)
    :options_(options){
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
  ceres_solver_options_.use_nonmonotonic_steps = options.ceres_solver_options_.use_nonmonotonic_steps_;
  ceres_solver_options_.max_num_iterations = options.ceres_solver_options_.max_num_iterations_;
  ceres_solver_options_.num_threads = options.ceres_solver_options_.num_threads_;
}

CeresScanMatcher::~CeresScanMatcher() {}

void CeresScanMatcher::Match(const transform::Rigid2d& previous_pose,
                             const transform::Rigid2d& initial_pose_estimate,
                             const sensor::PointCloud& point_cloud,
                             const map::ProbabilityGrid& probability_grid,
                             transform::Rigid2d* const pose_estimate,
                             ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight_, 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor, ceres::DYNAMIC,
                                      3>(
          new OccupiedSpaceCostFunctor(
              options_.occupied_space_weight_ /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, probability_grid),
          point_cloud.size()),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.translation_weight_, 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 2, 3>(
          new TranslationDeltaCostFunctor(options_.translation_weight_,
                                          previous_pose)),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight_, 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 1, 3>(
          new RotationDeltaCostFunctor(options_.rotation_weight_,
                                       ceres_pose_estimate[2])),
      nullptr, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace core
}  // namespace sample_carto
