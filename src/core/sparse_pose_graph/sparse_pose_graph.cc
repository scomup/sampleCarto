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

#include "sparse_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "src/common/make_unique.h"
#include "src/common/math.h"
#include "glog/logging.h"

namespace sample_carto {
namespace core {

void SparsePoseGraphOptions::Create(common::LuaParameterDictionary *const parameter_dictionary)
{
  optimize_every_n_scans_ = parameter_dictionary->GetInt("optimize_every_n_scans");
  matcher_translation_weight_ = parameter_dictionary->GetDouble("matcher_translation_weight");
  matcher_rotation_weight_ = parameter_dictionary->GetDouble("matcher_rotation_weight");
  max_num_final_iterations_ = parameter_dictionary->GetNonNegativeInt("max_num_final_iterations");
  global_sampling_ratio_ = parameter_dictionary->GetDouble("global_sampling_ratio");
  log_residual_histograms_ = parameter_dictionary->GetBool("log_residual_histograms");
  optimization_problem_options_.Create(parameter_dictionary->GetDictionary("optimization_problem").get());
  constraint_builder_options_.Create(parameter_dictionary->GetDictionary("constraint_builder").get());

}

SparsePoseGraph::SparsePoseGraph(
    const SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options_),
      constraint_builder_(options_.constraint_builder_options_, thread_pool),
      localization_samplers_(common::make_unique<FixedRatioSampler>(options_.global_sampling_ratio_))
 {}

SparsePoseGraph::~SparsePoseGraph() {
  //WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  //CHECK(work_queue_ == nullptr);
}

std::vector<int> SparsePoseGraph::GrowSubmapTransformsAsNeeded(const std::vector<std::shared_ptr<const map::Submap>> &insertion_submaps)
{
  CHECK(!insertion_submaps.empty());
  const auto &submap_data = optimization_problem_.submap_data();
  if (insertion_submaps.size() == 1)
  {
    // If we don't already have an entry for the first submap, add one.
    if (submap_data.empty())
    {
      optimization_problem_.AddSubmap(sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0]));
    }
    CHECK_EQ(submap_data.size(), 1);
    const int submap_id = 0;
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const int last_submap_id = submap_data.rbegin()->first;
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id);
    optimization_problem_.AddSubmap(
        first_submap_pose *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0])
                .inverse() *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
    return {last_submap_id, last_submap_id + 1};
  }
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const int front_submap_id = last_submap_id - 1;
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}


void SparsePoseGraph::AddScan(
    std::shared_ptr<const Node::Data> constant_data,
    const transform::Rigid3d& pose,
    const std::vector<std::shared_ptr<const map::Submap>>& insertion_submaps) {
      
      
  const transform::Rigid3d optimized_pose(pose);

  common::MutexLocker locker(&mutex_);
  nodes_[++num_nodes_] = Node{constant_data, optimized_pose};

  //trajectory_nodes_.Append(
  //    trajectory_id, mapping::TrajectoryNode{constant_data, optimized_pose});
  //++num_trajectory_nodes_;
  //connected_components_.Add(trajectory_id);

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (submap_data_.size() == 0 || submap_data_.back().submap != insertion_submaps.back())
  {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    submap_data_.push_back(SubmapData());
    submap_data_.back().submap = insertion_submaps.back();
  }
  
 // Make sure we have a sampler for this trajectory.
  if (localization_samplers_ == nullptr) {
    localization_samplers_ = common::make_unique<FixedRatioSampler>(options_.global_sampling_ratio_);
  }

  /*// We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(
        trajectory_id, insertion_submaps, newly_finished_submap,
        transform::Project2D(pose *
                             transform::Rigid3d::Rotation(
                                 constant_data->gravity_alignment.inverse())));
  });
  */
  
}

SparsePoseGraph::SubmapDataWithPose SparsePoseGraph::GetSubmapDataUnderLock(const int submap_id)
{
  if (submap_data_[submap_id].state == SubmapState::kTrimmed)
  {
    return {};
  }
  auto submap = submap_data_[submap_id].submap;
  //todo(liu)
  return {submap, submap->local_pose()};
}


SparsePoseGraph::SubmapDataWithPose SparsePoseGraph::GetSubmapData(const int submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

std::vector<SparsePoseGraph::SubmapDataWithPose> SparsePoseGraph::GetAllSubmapData()
{
  common::MutexLocker locker(&mutex_);
  std::vector<SparsePoseGraph::SubmapDataWithPose> all_submap_data;
  for (size_t submap_index = 0; submap_index < submap_data_.size(); ++submap_index)
  {
    all_submap_data.emplace_back(GetSubmapDataUnderLock(submap_index));
  }

  return all_submap_data;
}


void SparsePoseGraph::AddWorkItem(const std::function<void()>& work_item) {
  if (work_queue_ == nullptr) {
    work_item();
  } else {
    work_queue_->push_back(work_item);
  }
}


void SparsePoseGraph::AddOdometerData(const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=](){
    optimization_problem_.AddOdometerData(odometry_data);
  });
}

void SparsePoseGraph::ComputeConstraintsForScan(std::vector<std::shared_ptr<const map::Submap>> insertion_submaps,const bool newly_finished_submap, const transform::Rigid2d &pose)
{
  
  const std::vector<int> submap_ids = GrowSubmapTransformsAsNeeded(insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const int matching_id = submap_ids.front();
  const transform::Rigid2d optimized_pose =
      optimization_problem_.submap_data().at(matching_id) *
      sparse_pose_graph::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
      pose;
  const int node_id = !optimization_problem_.node_data().empty() ? optimization_problem_.node_data().rbegin()->first + 1 : 0;

  const auto &scan_data = nodes_.at(node_id).constant_data;
  optimization_problem_.AddNode(scan_data->time, pose, optimized_pose);
  for (size_t i = 0; i < insertion_submaps.size(); ++i)
  {
    const int submap_id = submap_ids[i];
    // Even if this was the last scan added to 'submap_id', the submap will only
    // be marked as finished in 'submap_data_' further below.
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid2d constraint_transform =
        sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
        pose;
    constraints_.push_back(sparse_pose_graph::Constraint{submap_id,
                                                         node_id,
                                                         {transform::Embed3D(constraint_transform),
                                                          options_.matcher_translation_weight_,
                                                          options_.matcher_rotation_weight_},
                                                         sparse_pose_graph::Constraint::INTRA_SUBMAP});
  }

  for (int submap_id = 0; submap_id < (int)submap_data_.size(); ++submap_id)
  {
    if (submap_data_.at(submap_id).state == SubmapState::kFinished)
    {
      CHECK_EQ(submap_data_.at(submap_id).node_ids.count(node_id), 0);
      ComputeConstraint(node_id, submap_id);
    }
  }

  if (newly_finished_submap)
  {
    const int finished_submap_id = submap_ids.front();
    
    SubmapData &finished_submap_data = submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap_id);
    
  }
  constraint_builder_.NotifyEndOfScan();
  ++num_scans_since_last_loop_closure_;
  
  if (options_.optimize_every_n_scans_ > 0 &&
      num_scans_since_last_loop_closure_ > options_.optimize_every_n_scans_)
  {
     CHECK(!run_loop_closure_);
    run_loop_closure_ = true;
    // If there is a 'work_queue_' already, some other thread will take care.
    if (work_queue_ == nullptr)
    {
      work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
      HandleWorkQueue();
    }
  }
}

void SparsePoseGraph::ComputeConstraint(const int node_id, const int submap_id)
{

  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Only globally match against submaps not in this trajectory.
  if (localization_samplers_->Pulse())
  {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        nodes_.at(node_id).constant_data.get());
  }
  else
  {
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_.submap_data().at(submap_id) *
        optimization_problem_.node_data().at(node_id).pose;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        nodes_.at(node_id).constant_data.get(),
        initial_relative_pose);
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(const int submap_id)
{
  const auto &submap_data = submap_data_.at(submap_id);
  const auto &node_data = optimization_problem_.node_data();

  for (const auto &index_node_data : node_data)
  {
    const int node_id = index_node_data.first;
    CHECK(!nodes_.at(node_id).trimmed());
    if (submap_data.node_ids.count(node_id) == 0)
    {
      ComputeConstraint(node_id, submap_id);
    }
  }
}

void SparsePoseGraph::HandleWorkQueue()
{

  constraint_builder_.WhenDone(
      [this](const sparse_pose_graph::ConstraintBuilder::Result &result) {
        {
          common::MutexLocker locker(&mutex_);
          constraints_.insert(constraints_.end(), result.begin(), result.end());
        }
        //RunOptimization();

        // Update the trajectory connectivity structure with the new
        // constraints.
        for (const sparse_pose_graph::Constraint &constraint : result)
        {
          CHECK_EQ(constraint.tag,
                   sparse_pose_graph::Constraint::INTER_SUBMAP);
        }

        common::MutexLocker locker(&mutex_);
        num_scans_since_last_loop_closure_ = 0;
        run_loop_closure_ = false;
        while (!run_loop_closure_)
        {
          if (work_queue_->empty())
          {
            LOG(INFO) << "We caught up. Hooray!";
            work_queue_.reset();
            return;
          }
          work_queue_->front()();
          work_queue_->pop_front();
        }
        // We have to optimize again.
        HandleWorkQueue();
      });
}
/* 
void SparsePoseGraph::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_scans_at_start =
      constraint_builder_.GetNumFinishedScans();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return constraint_builder_.GetNumFinishedScans() ==
               num_trajectory_nodes_;
      },
      1.)) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. * (constraint_builder_.GetNumFinishedScans() -
                             num_finished_scans_at_start) /
                         (num_trajectory_nodes_ - num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone([this, &notification](
      const sparse_pose_graph::ConstraintBuilder::Result& result) {
    common::MutexLocker locker(&mutex_);
    constraints_.insert(constraints_.end(), result.begin(), result.end());
    notification = true;
  });
  locker.Await([&notification]() { return notification; });
}

void SparsePoseGraph::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

void SparsePoseGraph::RunFinalOptimization() {
  WaitForAllComputations();
  optimization_problem_.SetMaxNumIterations(
      options_.max_num_final_iterations());
  RunOptimization();
  optimization_problem_.SetMaxNumIterations(
      options_.optimization_problem_options()
          .ceres_solver_options()
          .max_num_iterations());
}

void SparsePoseGraph::RunOptimization() {
  
  if (optimization_problem_.submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_ and
  // frozen_trajectories_ when executing the Solve. Solve is time consuming, so
  // not taking the mutex before Solve to avoid blocking foreground processing.
  optimization_problem_.Solve(constraints_, frozen_trajectories_);
  common::MutexLocker locker(&mutex_);

  const auto& submap_data = optimization_problem_.submap_data();
  const auto& node_data = optimization_problem_.node_data();
  for (int trajectory_id = 0;
       trajectory_id != static_cast<int>(node_data.size()); ++trajectory_id) {
    const int num_nodes = trajectory_nodes_.num_indices(trajectory_id);
    for (const auto& node_data_index : node_data.at(trajectory_id)) {
      const mapping::NodeId node_id{trajectory_id, node_data_index.first};
      auto& node = trajectory_nodes_.at(node_id);
      node.pose =
          transform::Embed3D(node_data_index.second.pose) *
          transform::Rigid3d::Rotation(node.constant_data->gravity_alignment);
    }
    // Extrapolate all point cloud poses that were added later.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        optimized_submap_transforms_, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();
    int last_optimized_node_index =
        node_data.at(trajectory_id).empty()
            ? 0
            : node_data.at(trajectory_id).rbegin()->first;
    for (int node_index = last_optimized_node_index + 1; node_index < num_nodes;
         ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      auto& node_pose = trajectory_nodes_.at(node_id).pose;
      node_pose = old_global_to_new_global * node_pose;
    }
  }
  optimized_submap_transforms_ = submap_data;

  //TrimmingHandle trimming_handle(this);
  //for (auto& trimmer : trimmers_) {
  //  trimmer->Trim(&trimming_handle);
  //}
  
}

std::vector<std::vector<mapping::TrajectoryNode>>
SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_.data();
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  std::vector<Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                       trajectory_id);
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {

  return connected_components_.Components();
}

int SparsePoseGraph::num_submaps(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  if (trajectory_id >= submap_data_.num_trajectories()) {
    return 0;
  }
  return submap_data_.num_indices(trajectory_id);
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
SparsePoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
      all_submap_data(submap_data_.num_trajectories());
  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    all_submap_data[trajectory_id].reserve(
        submap_data_.num_indices(trajectory_id));
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      all_submap_data[trajectory_id].emplace_back(GetSubmapDataUnderLock(
          mapping::SubmapId{trajectory_id, submap_index}));
    }
  }
  return all_submap_data;
}

transform::Rigid3d SparsePoseGraph::ComputeLocalToGlobalTransform(
    const std::vector<std::map<int, sparse_pose_graph::SubmapPoseData>>&
        submap_transforms,
    const int trajectory_id) const {
  if (trajectory_id >= static_cast<int>(submap_transforms.size()) ||
      submap_transforms.at(trajectory_id).empty()) {
    return transform::Rigid3d::Identity();
  }

  const int submap_index = submap_transforms.at(trajectory_id).rbegin()->first;
  const mapping::SubmapId last_optimized_submap_id{trajectory_id, submap_index};
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
             submap_transforms.at(trajectory_id).at(submap_index).pose) *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) {
  if (submap_data_.at(submap_id).state == SubmapState::kTrimmed) {
    return {};
  }
  auto submap = submap_data_.at(submap_id).submap;
  if (submap_id.trajectory_id <
      static_cast<int>(optimized_submap_transforms_.size())) {
    const size_t submap_data_index = submap_id.submap_index;
    if (submap_data_index <
        optimized_submap_transforms_.at(submap_id.trajectory_id).size()) {
      // We already have an optimized pose.
      return {submap, transform::Embed3D(optimized_submap_transforms_
                                             .at(submap_id.trajectory_id)
                                             .at(submap_data_index)
                                             .pose)};
    }
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}*/

}  // namespace core
}  // namespace sample_carto
