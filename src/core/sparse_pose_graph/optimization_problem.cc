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
#include "src/core/sparse_pose_graph/optimization_problem.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "src/common/math.h"
#include "src/core/sparse_pose_graph/spa_cost_function.h"
#include "src/sensor/odometry_data.h"
#include "src/transform/transform.h"
#include "src/transform/transform_interpolation_buffer.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace sample_carto
{
namespace core
{
namespace sparse_pose_graph
{

namespace
{

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d &pose)
{
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3> &values)
{
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

} // namespace

OptimizationProblem::OptimizationProblem(
    const OptimizationProblemOptions &options)
    : options_(options) {}

OptimizationProblem::~OptimizationProblem() {}

void OptimizationProblem::AddOdometerData(const sensor::OdometryData &odometry_data)
{
  odometry_data_.Push(odometry_data.time, odometry_data.pose);
}

void OptimizationProblem::AddNode(
    const double time,
    const transform::Rigid2d &initial_pose, const transform::Rigid2d &pose)
{
  node_data_.emplace(trajectory_data_.next_node_index, NodeData{time, initial_pose, pose});
  ++trajectory_data_.next_node_index;
}

void OptimizationProblem::TrimNode(const int node_id)
{
  //auto& node_data = node_data_.at(node_id.trajectory_id);
  //CHECK(node_data.erase(node_id.node_index));
  //if (!node_data.empty() ) {
  //  auto node_it = node_data.begin();
  //}
}

void OptimizationProblem::AddSubmap(const transform::Rigid2d &submap_pose)
{

  submap_pose_data_.emplace(trajectory_data_.next_submap_index,
                       SubmapPoseData{submap_pose});
  ++trajectory_data_.next_submap_index;
}

void OptimizationProblem::TrimSubmap(const int submap_id)
{
  CHECK(submap_pose_data_.erase(submap_id));
}

void OptimizationProblem::SetMaxNumIterations(const int32 max_num_iterations)
{
  options_.ceres_solver_options_.max_num_iterations_ = max_num_iterations;
}

void OptimizationProblem::Solve(const std::vector<Constraint> &constraints)
{
  if (node_data_.empty())
  {
    // Nothing to optimize.
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapPoseData.
  std::map<int, std::array<double, 3>> C_submaps;
  std::map<int, std::array<double, 3>> C_nodes;
  bool first_submap = true;

  for (const auto &index_submap_data : submap_pose_data_)
  {
    const int submap_index = index_submap_data.first;
    const SubmapPoseData &submap_data = index_submap_data.second;

    C_submaps.emplace(submap_index,
                      FromPose(submap_data.pose));
    problem.AddParameterBlock(
        C_submaps.at(submap_index).data(), 3);
    if (first_submap)
    {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_index).data());
    }
  }

  for (const auto &index_node_data : node_data_)
  {
    const int node_index = index_node_data.first;
    const NodeData &node_data = index_node_data.second;
    C_nodes.emplace(node_index, FromPose(node_data.pose));
    problem.AddParameterBlock(C_nodes.at(node_index).data(),3);
  }

  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint &constraint : constraints)
  {
    //transform::Rigid2d s_p = submap_pose_data_[0][constraint.submap_id.submap_index].pose;
    //transform::Rigid2d c_p = node_data_[0][constraint.node_id.node_index].pose;
    //transform::Rigid2d diff = s_p.inverse() * c_p;
    //transform::Rigid2d zbar_ij = transform::Project2D(constraint.pose.zbar_ij);
    //transform::Rigid2d error = zbar_ij.inverse() * diff;
    //std::cout<<error<<std::endl;

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
            new SpaCostFunction(constraint.pose)),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id.submap_index).data(),
        C_nodes.at(constraint.node_id.node_index).data());
  }

  // Add penalties for violating odometry or changes between consecutive scans
  // if odometry is not available.
  
    if (node_data_.empty())
    {
      continue;
    }

    for (auto node_it = node_data_.begin();;)
    {
      const int node_index = node_it->first;
      const NodeData &node_data = node_it->second;
      ++node_it;
      if (node_it == node_data_.end())
      {
        break;
      }

      const int next_node_index = node_it->first;
      const NodeData &next_node_data = node_it->second;

      if (next_node_index != node_index + 1)
      {
        continue;
      }

      const bool odometry_available =
          trajectory_id < odometry_data_.size() &&
          odometry_data_.Has(
              node_data_[next_node_index].time) &&
          odometry_data_.Has(
              node_data_[node_index].time);
      const transform::Rigid3d relative_pose =
          odometry_available
              ? odometry_data_.Lookup(node_data.time).inverse() *
                    odometry_data_.Lookup(next_node_data.time)
              : transform::Embed3D(node_data.initial_pose.inverse() *
                                   next_node_data.initial_pose);
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
              new SpaCostFunction(Constraint::Pose{
                  relative_pose,
                  options_.consecutive_scan_translation_penalty_factor(),
                  options_.consecutive_scan_rotation_penalty_factor()})),
          nullptr /* loss function */,
          C_nodes[node_index].data(),
          C_nodes[next_node_index].data());
    }
  

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary())
  {
    LOG(INFO) << summary.FullReport();
  }

  for (auto &index_submap_data : submap_pose_data_)
  {
    index_submap_data.second.pose = ToPose(C_submaps.at(index_submap_data.first));
  }

  for (auto &index_node_data : node_data_)
  {
    index_node_data.second.pose = ToPose(C_nodes.at(index_node_data.first));
  }
}

const std::map<int, NodeData> &OptimizationProblem::node_data()const
{
  return node_data_;
}
/*
const std::map<int, transform::Rigid2d> &OptimizationProblem::submap_data()const
{
  return submap_pose_data_;
}*/

} // namespace sparse_pose_graph
} // namespace core
} // namespace sample_carto
