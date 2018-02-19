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

#include "src/core/sparse_pose_graph/constraint_builder.h"

#include <cmath>
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
#include "src/common/thread_pool.h"
#include "glog/logging.h"



namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

transform::Rigid2d ComputeSubmapPose(const map::Submap& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder::ConstraintBuilder(
    const ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options)
      ,thread_pool_(thread_pool)
      ,sampler_(options.sampling_ratio_)
      ,ceres_scan_matcher_(options.ceres_scan_matcher_options_)
      {}

ConstraintBuilder::~ConstraintBuilder() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(pending_computations_.size(), 0);
  CHECK_EQ(submap_queued_work_items_.size(), 0);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder::MaybeAddConstraint(
    const int submap_id, const map::Submap* const submap,
    const int node_id,
    const Node::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {

  //(liu) do not use this rule
  //if (initial_relative_pose.translation().norm() >
  //    options_.max_constraint_distance_) {
  //  return;
  //}
  if (sampler_.Pulse()) {
    common::MutexLocker locker(&mutex_);
    constraints_.emplace_back();
    auto* const constraint = &constraints_.back();
    ++pending_computations_[current_computation_];
    const int current_computation = current_computation_;
    ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
        submap_id, &submap->probability_grid(), [=]()  {
          ComputeConstraint(submap_id, submap, node_id,
                            false,   /* match_full_submap */
                            constant_data, initial_relative_pose, constraint);
          FinishComputation(current_computation);
        });
  }
}

void ConstraintBuilder::MaybeAddGlobalConstraint(
    const int submap_id, const map::Submap* const submap,
    const int node_id,
    const Node::Data* const constant_data) {
  common::MutexLocker locker(&mutex_);
  constraints_.emplace_back();
  auto* const constraint = &constraints_.back();
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      submap_id, &submap->probability_grid(), [=]() {
        ComputeConstraint(submap_id, submap, node_id,
                          true, /* match_full_submap */
                          constant_data,
                          transform::Rigid2d::Identity(), constraint);
        FinishComputation(current_computation);
      });
}

void ConstraintBuilder::NotifyEndOfScan() {
  common::MutexLocker locker(&mutex_);
  ++current_computation_;
}

void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  thread_pool_->Schedule(
      [this, current_computation] { FinishComputation(current_computation); });
}

void ConstraintBuilder::ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
    const int submap_id, const map::ProbabilityGrid* const submap,
    const std::function<void()>& work_item) {
  if (submap_scan_matchers_[submap_id].fast_correlative_scan_matcher !=
      nullptr) {
    thread_pool_->Schedule(work_item);
  } else {
    submap_queued_work_items_[submap_id].push_back(work_item);
    if (submap_queued_work_items_[submap_id].size() == 1) {
      thread_pool_->Schedule(
          [=]() { ConstructSubmapScanMatcher(submap_id, submap); });
    }
  }
}

void ConstraintBuilder::ConstructSubmapScanMatcher(
    const int submap_id, const map::ProbabilityGrid* const submap) {
  auto submap_scan_matcher =
      common::make_unique<scan_matching::FastCorrelativeScanMatcher>(
          *submap, options_.fast_correlative_scan_matcher_options_);
  common::MutexLocker locker(&mutex_);
  submap_scan_matchers_[submap_id] = {submap, std::move(submap_scan_matcher)};
  for (const std::function<void()>& work_item :
       submap_queued_work_items_[submap_id]) {
    thread_pool_->Schedule(work_item);
  }
  submap_queued_work_items_.erase(submap_id);
}

const ConstraintBuilder::SubmapScanMatcher*
ConstraintBuilder::GetSubmapScanMatcher(const int submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}
/*
template <typename T>
std::vector<T> sort_indexes(const std::vector<T> &v)
{

  // initialize original index locations
  std::vector<T> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
            [&v](T i1, T i2) { return v[i1] < v[i2]; });

  return idx;
}
bool hough(sensor::PointCloud point_cloud)
{
  int step_theta = 1;
  std::vector<double> ctheta(180/step_theta);
  std::vector<double> stheta(180/step_theta);
  for (int i = 0; i < 180; i+=step_theta)
  {
    double theta = -M_PI / 2 * M_PI / 2 * i;
    ctheta[i/step_theta] = cos(theta);
    stheta[i/step_theta] = sin(theta);
  }
  int max_distance = 200;
  int offset = max_distance / 2;
  std::vector<std::vector<uint>> accum;
  accum.resize(180/step_theta, std::vector<uint>(max_distance));
  std::vector<uint> hough_res(180/step_theta);


  for (auto &p : point_cloud)
  {
    for (int i = 0; i < 180/step_theta; i+=step_theta)
    {
      int accum_idx = std::round((ctheta[i/step_theta] * p.x() + stheta[i/step_theta] * p.y()) * 10) + offset;
      assert(accum_idx < max_distance - 1);
      ++accum[i/step_theta][accum_idx];
    }
  }


  for (int i = 0; i < 180; i+=step_theta)
  {
    hough_res[i/step_theta] = *std::max_element(accum[i/step_theta].begin(), accum[i/step_theta].end());
  }

  std::vector<uint> order = sort_indexes<uint>(hough_res);
  for (int i = 0; i < 180; i+=step_theta)
  {
    std::cout << hough_res[i/step_theta] << ","<< "\n";
    std::cout << order[i/step_theta] << ","<< "\n";
  }
  std::cout << "\n";

  return 1;
}
*/

void ConstraintBuilder::ComputeConstraint(
    const int submap_id, const map::Submap *const submap,
    const int node_id, bool match_full_submap,
    const Node::Data *const constant_data,
    const transform::Rigid2d &initial_relative_pose,
    std::unique_ptr<Constraint> *constraint)
{
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;
  const SubmapScanMatcher *const submap_scan_matcher =
      GetSubmapScanMatcher(submap_id);

  // The 'constraint_transform' (submap i <- scan j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in scan j,
  // - the initial guess 'initial_pose' for (map <- scan j),
  // - the result 'pose_estimate' of Match() (map <- scan j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  //if(!hough(constant_data->filtered_gravity_aligned_point_cloud))//(liu)
  //  return;

  if (match_full_submap)
  {
    if (submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score_, &score, &pose_estimate))
    {
      CHECK_GT(score, options_.global_localization_min_score_);
    }
    else
    {
      return;
    }
  } else {
    //std::cout <<"Try to add submap_"<< submap_id << "->node_"<<node_id<<"\n";
    if (submap_scan_matcher->fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score_, &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score_);
    } else {
      return;
    }
  }
  
  {
    common::MutexLocker locker(&mutex_);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate, pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher->probability_grid,
                            &pose_estimate, &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight_,
                                    options_.loop_closure_rotation_weight_},
                                   Constraint::INTER_SUBMAP});
  //std::cout <<"Add submap_"<< submap_id << "->node_"<<node_id<<" successful!!!\n";
  if (options_.log_matches_) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder::FinishComputation(const int computation_index) {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    if (--pending_computations_[computation_index] == 0) {
      pending_computations_.erase(computation_index);
    }
    if (pending_computations_.empty()) {
      CHECK_EQ(submap_queued_work_items_.size(), 0);
      if (when_done_ != nullptr) {
        for (const std::unique_ptr<Constraint>& constraint : constraints_) {
          if (constraint != nullptr) {
            result.push_back(*constraint);
          }
        }
        if (options_.log_matches_) {
          LOG(INFO) << constraints_.size() << " computations resulted in "
                    << result.size() << " additional constraints.";
        }
        constraints_.clear();
        callback = std::move(when_done_);
        when_done_.reset();
      }
    }
  }
  if (callback != nullptr) {
    (*callback)(result);
  }
}

int ConstraintBuilder::GetNumFinishedScans() {
  common::MutexLocker locker(&mutex_);
  if (pending_computations_.empty()) {
    return current_computation_;
  }
  return pending_computations_.begin()->first;
}

void ConstraintBuilder::DeleteScanMatcher(const int submap_id) {
  common::MutexLocker locker(&mutex_);
  CHECK(pending_computations_.empty());
  submap_scan_matchers_.erase(submap_id);
}

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto
