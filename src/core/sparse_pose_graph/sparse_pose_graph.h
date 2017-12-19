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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_BASE_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_BASE_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "src/common/lua_parameter_dictionary.h"
#include "src/common/mutex.h"
#include "src/core/map/submaps.h"
#include "src/core/sparse_pose_graph/node.h"
#include "src/core/sparse_pose_graph/fixed_ratio_sampler.h"
#include "src/transform/rigid_transform.h"
#include "src/sensor/odometry_data.h"
#include "src/sensor/range_data.h"

namespace sample_carto
{
namespace core
{

class SparsePoseGraphOptions
{
public:
  void Create(common::LuaParameterDictionary *const parameter_dictionary);
  int optimize_every_n_scans_;
  //*options.mutable_constraint_builder_options() =
  //    sparse_pose_graph::CreateConstraintBuilderOptions(
  //        parameter_dictionary->GetDictionary("constraint_builder").get());
  double matcher_translation_weight_;
  double matcher_rotation_weight_;
  //*options.mutable_optimization_problem_options() =
  //    sparse_pose_graph::CreateOptimizationProblemOptions(
  //        parameter_dictionary->GetDictionary("optimization_problem").get());
  uint max_num_final_iterations_;
  double global_sampling_ratio_;
  bool log_residual_histograms_;
};

class SparsePoseGraph
{
public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint
  {
    struct Pose
    {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };

    int submap_id; // 'i' in the paper.
    int node_id;   // 'j' in the paper.

    // Pose of the scan 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where scan 'j' was inserted into
    // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
    // into submap 'i').
    enum Tag
    {
      INTRA_SUBMAP,
      INTER_SUBMAP
    } tag;
  };

  struct SubmapDataWithPose
  {
    std::shared_ptr<const map::Submap> submap;
    transform::Rigid3d pose;
  };
  enum class SubmapState { kActive, kFinished, kTrimmed };
  struct SubmapData {
    std::shared_ptr<const map::Submap> submap;

    // IDs of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<int> node_ids;
    SubmapState state = SubmapState::kActive;
  };


  ~SparsePoseGraph();
  SparsePoseGraph(const SparsePoseGraph &) = delete;
  SparsePoseGraph &operator=(const SparsePoseGraph &) = delete;
  SparsePoseGraph(const SparsePoseGraphOptions &options);

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  //virtual void FreezeTrajectory(int trajectory_id) = 0;

  // Adds a 'submap' from a proto with the given 'initial_pose' to the frozen
  // trajectory with 'trajectory_id'.
  //virtual void AddSubmapFromProto(int trajectory_id,
  //                                const transform::Rigid3d& initial_pose,
  //                                const proto::Submap& submap) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  //virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Computes optimized poses.
  //void RunFinalOptimization() = 0;

  // Gets the current trajectory clusters.
  // virtual std::vector<std::vector<int>> GetConnectedTrajectories();

  // Return the number of submaps for the given 'trajectory_id'.
  //virtual int num_submaps(int trajectory_id) = 0;

  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'.
  //virtual SubmapData GetSubmapData(const SubmapId& submap_id) = 0;

  // Returns data for all Submaps by trajectory.
  //virtual std::vector<std::vector<SubmapData>> GetAllSubmapData() = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  //virtual transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) = 0;

  // Returns the current optimized trajectories.
  //virtual std::vector<std::vector<TrajectoryNode>> GetTrajectoryNodes() = 0;

  // Returns the collection of constraints.
  //virtual std::vector<Constraint> constraints() = 0;
  void AddScan(
      std::shared_ptr<const Node::Data> constant_data,
      const transform::Rigid3d &pose,
      const std::vector<std::shared_ptr<const map::Submap>> &insertion_submaps);
  //void AddImuData(int trajectory_id, const sensor::ImuData& imu_data);
  void AddOdometerData(int trajectory_id,
                       const sensor::OdometryData &odometry_data);


SubmapDataWithPose GetSubmapData(const int submap_id);

std::vector<SubmapDataWithPose> GetAllSubmapData();


private:
  SubmapDataWithPose GetSubmapDataUnderLock(const int submap_id);

  const SparsePoseGraphOptions options_;
  common::Mutex mutex_;
  //std::vector<std::map<int, transform::Rigid2d>> submap_data_;
  std::vector<SubmapData> submap_data_;
  std::unique_ptr<FixedRatioSampler> localization_samplers_;
  std::map<int, transform::Rigid2d> optimized_submap_transforms_;

};

} // namespace core
} // namespace sample_carto

#endif // SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_H_
