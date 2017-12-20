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

#ifndef SAMPLE_CARTO_CORE_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define SAMPLE_CARTO_CORE_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "src/common/lua_parameter_dictionary.h"
#include "src/core/map/probability_grid.h"
#include "src/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace sample_carto {
namespace core {
namespace scan_matching {


class CeresSolverOptions
{
  public:
    void Create(common::LuaParameterDictionary* const parameter_dictionary);
    double use_nonmonotonic_steps_;
    uint max_num_iterations_;
    uint num_threads_;
};

class CeresScanMatcherOptions
{
  public:
    void Create(common::LuaParameterDictionary* const parameter_dictionary);
    double occupied_space_weight_;
    double translation_weight_;
    double rotation_weight_;
    CeresSolverOptions ceres_solver_options_; 
};

// Align scans with an existing map using Ceres.
class CeresScanMatcher {
 public:
  explicit CeresScanMatcher(const CeresScanMatcherOptions& options);
  virtual ~CeresScanMatcher();

  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const transform::Rigid2d& previous_pose,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud,
             const map::ProbabilityGrid& probability_grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const CeresScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
