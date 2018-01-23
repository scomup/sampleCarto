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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_


#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "src/common/port.h"
//#include "src/core/sparse_pose_graph/sparse_pose_graph.h"
#include "src/sensor/odometry_data.h"
#include "src/transform/transform_interpolation_buffer.h"
#include "src/core/sparse_pose_graph/optimization_problem_options.h"
#include "src/core/sparse_pose_graph/optimization_problem.h"
#include "src/core/sparse_pose_graph/constraint.h"

namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

struct NodeData {
  double time;
  transform::Rigid2d initial_pose;
  transform::Rigid2d pose;
};

struct SubmapPoseData {
  transform::Rigid2d pose;
};

// Implements the SPA loop closure method.
class OptimizationProblem {
 public:
 //using Constraint = SparsePoseGraph::Constraint;

  explicit OptimizationProblem(const OptimizationProblemOptions& options);
  ~OptimizationProblem();

  OptimizationProblem(const OptimizationProblem&) = delete;
  OptimizationProblem& operator=(const OptimizationProblem&) = delete;

  void AddOdometerData(const sensor::OdometryData& odometry_data);
  void AddNode(double time,
               const transform::Rigid2d &initial_pose,
               const transform::Rigid2d &pose);
  void TrimNode(const int node_id);
  void AddSubmap(const transform::Rigid2d& submap_pose);
  void TrimSubmap(const int submap_id);

  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses.
  //void Solve(const std::set<int>& frozen_trajectories);
  // Optimizes the global poses.
  void Solve(const std::vector<Constraint>& constraints, const std::set<int>& frozen_trajectories);

  const std::map<int, NodeData>& node_data() const;
  const std::map<int, transform::Rigid2d>& submap_data();
 private:
  struct TrajectoryData {
    // TODO(hrapp): Remove, once we can relabel constraints.
    int next_submap_index = 0;
    int next_node_index = 0;
  };
  OptimizationProblemOptions options_;
  std::map<int, NodeData> node_data_;
  transform::TransformInterpolationBuffer odometry_data_;
  std::map<int, transform::Rigid2d> submap_pose_data_;
  TrajectoryData trajectory_data_;
};

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
