/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef SAMPLE_CARTO_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
#define SAMPLE_CARTO_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_

#include "src/sensor/point_cloud.h"
#include "src/sensor/odometry_data.h"
#include "src/core/local_builder/local_map_builder.h"
//#include "src/mapping/sparse_pose_graph.h"
#include "src/common/make_unique.h"


namespace sample_carto {
namespace core {

class GlobalMapManager{
 public:
  GlobalMapManager(const LocalMapBuilderOptions options):
    //thread_pool_(4),
   local_map_builder_(options)
   //,sparse_pose_graph_2d_(std::move(common::make_unique<SparsePoseGraph2D>(sparse_pose_graphe_options ,&thread_pool_))) 
   {};

  ~GlobalMapManager(){};

  GlobalMapManager(const GlobalMapManager&) = delete;
  GlobalMapManager& operator=(const GlobalMapManager&) = delete;

  //const PoseEstimate& pose_estimate() const {
  //  return local_map_builder_.pose_estimate();
  //}

  void AddRangefinderData(const double time, const Eigen::Vector3f &origin, const sensor::PointCloud &ranges)
  {
    auto insertion_result = local_map_builder_.AddRangeData( time, sensor::RangeData{origin, ranges, {}});
    //std::cout<<"All pending works:" << sparse_pose_graph_2d_->constraint_builder_.GetAllPendingWork() <<std::endl;
    if (insertion_result != nullptr)//temp_liu
      submap_ = insertion_result->insertion_submaps.back();//temp_liu
    /*
    if (insertion_result == nullptr)
    {
      return;
    }
    sparse_pose_graph_2d_->AddScan(
        insertion_result->constant_data, insertion_result->pose_observation,
        0, insertion_result->insertion_submaps);
    */
  }

  void AddSensorData(const sensor::OdometryData& odometry_data){
    local_map_builder_.AddOdometerData(odometry_data);
    //sparse_pose_graph_2d_->AddOdometerData(0, odometry_data);
  }


  //std::unique_ptr<SparsePoseGraph2D>& sparse_pose_graph() { return sparse_pose_graph_2d_; }
  
  std::shared_ptr<const map::Submap>& submap() { return submap_; } //temp_liu

private:
  //common::ThreadPool thread_pool_;
  LocalMapBuilder local_map_builder_;
  std::shared_ptr<const map::Submap> submap_;//temp_liu
  //std::unique_ptr<SparsePoseGraph2D> sparse_pose_graph_2d_;
};

}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
