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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "src/core/sparse_pose_graph/fixed_ratio_sampler.h"
#include "src/common/math.h"
#include "src/common/mutex.h"
#include "src/transform/transform.h"
#include "src/common/thread_pool.h"
#include "src/core/scan_matching/ceres_scan_matcher.h"
#include "src/core/scan_matching/fast_correlative_scan_matcher.h"
#include "src/core/map/submaps.h"
#include "src/sensor/point_cloud.h"
//#include "src/core/sparse_pose_graph/sparse_pose_graph.h"
#include "src/core/sparse_pose_graph/constraint_builder_options.h"
#include "src/core/sparse_pose_graph/constraint.h"
#include "src/core/sparse_pose_graph/node.h"
#include "src/core/scan_matching/real_time_correlative_scan_matcher.h"



namespace sample_carto {
namespace core {
namespace sparse_pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const map::Submap& submap);

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder {
 public:
  //using Constraint = SparsePoseGraph::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder(
      const ::sample_carto::core::sparse_pose_graph::ConstraintBuilderOptions &options,
      common::ThreadPool *thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(
      const int submap_id, const map::Submap* submap,
      const int node_id,
      const Node::Data* const constant_data,
      const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const int submap_id, const map::Submap* submap,
      const int node_id,
      const Node::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfScan();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished scans.
  int GetNumFinishedScans();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const int submap_id);

 private:
  struct SubmapScanMatcher {
    const map::ProbabilityGrid* probability_grid;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
        fast_correlative_scan_matcher;
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      const int submap_id, const map::ProbabilityGrid* submap,
      const std::function<void()>& work_item) ;

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  void ConstructSubmapScanMatcher(const int submap_id,
                                  const map::ProbabilityGrid* submap);

  // Returns the scan matcher for a submap, which has to exist.
  const SubmapScanMatcher* GetSubmapScanMatcher(
      const int submap_id) ;

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(
      const int submap_id, const map::Submap* submap,
      const int node_id, bool match_full_submap,
      const Node::Data* const constant_data,
      const transform::Rigid2d& initial_relative_pose,
      std::unique_ptr<Constraint>* constraint) ;

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  void FinishComputation(int computation_index) ;

  const ConstraintBuilderOptions options_;
  common::ThreadPool* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_;

  // Index of the scan in reaction to which computations are currently
  // added. This is always the highest scan index seen so far, even when older
  // scans are matched against a new submap.
  int current_computation_  = 0;

  // For each added scan, maps to the number of pending computations that were
  // added for it.
  std::map<int, int> pending_computations_ ;

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  std::deque<std::unique_ptr<Constraint>> constraints_ ;

  // Map of already constructed scan matchers by 'submap_id'.
  std::map<int, SubmapScanMatcher> submap_scan_matchers_;

  // Map by 'submap_id' of scan matchers under construction, and the work
  // to do once construction is done.
  std::map<int, std::vector<std::function<void()>>>
      submap_queued_work_items_ ;

  FixedRatioSampler sampler_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;
  scan_matching::RealTimeCorrelativeScanMatcher real_time_correlative_scan_matcher_;

};

}  // namespace sparse_pose_graph
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
