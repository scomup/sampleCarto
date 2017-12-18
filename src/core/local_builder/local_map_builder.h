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

#ifndef SAMPLE_CARTO_CORE_2D_LOCAL_MAP_BUILDER_H_
#define SAMPLE_CARTO_CORE_2D_LOCAL_MAP_BUILDER_H_

#include <memory>

#include "local_map_builder_options.h"

//#include "src/mapping/scan_matching/real_time_correlative_scan_matcher.h"
//#include "src/mapping/scan_matching/ceres_scan_matcher.h"
#include "src/core/map/submaps.h"
#include "src/core/local_builder/motion_filter.h"
#include "src/sensor/odometry_data.h"
#include "src/sensor/range_data.h"
#include "src/transform/rigid_transform.h"
#include "src/core/local_builder/pose_extrapolator.h"
#include "src/core/local_builder/motion_filter.h"




#include "nav_msgs/GetMap.h"

namespace sample_carto
{
namespace core
{

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
class LocalMapBuilder
{
  public:

    struct InsertionResult
    {
        //std::shared_ptr<const TrajectoryNode::Data> constant_data;
        transform::Rigid3d pose_observation;
        std::vector<std::shared_ptr<const map::Submap>> insertion_submaps;
    };

    explicit LocalMapBuilder(const LocalMapBuilderOptions &options);
    ~LocalMapBuilder();

    LocalMapBuilder(const LocalMapBuilder&) = delete;
    LocalMapBuilder& operator=(const LocalMapBuilder&) = delete;

    //const PoseEstimate &pose_estimate() const;
    

    // Range data must be approximately horizontal for 2D SLAM.
    std::unique_ptr<InsertionResult> AddRangeData(double, const sensor::RangeData &range_data);
    void AddOdometerData(const sensor::OdometryData &odometry_data);

  private:

    
    std::unique_ptr<InsertionResult> AddAccumulatedRangeData(double time, const sensor::RangeData &range_data);
    
    sensor::RangeData TransformAndFilterRangeData(
        const transform::Rigid3f &gravity_alignment,
        const sensor::RangeData &range_data) const;

    // Scan matches 'gravity_aligned_range_data' and fill in the
    // 'pose_observation' with the result.
    void ScanMatch(const double time, const transform::Rigid2d &pose_prediction,
                   const sensor::RangeData &gravity_aligned_range_data,
                   transform::Rigid2d *pose_observation);
                   

    // Lazily constructs a PoseExtrapolator.
    void InitializeExtrapolator(double time);

    
    const LocalMapBuilderOptions options_;
    MotionFilter motion_filter_;   
    map::ActiveSubmaps active_submaps_; 
    //scan_matching::RealTimeCorrelativeScanMatcher real_time_correlative_scan_matcher_;
    //scan_matching::CeresScanMatcher ceres_scan_matcher_;
    std::unique_ptr<core::PoseExtrapolator> extrapolator_;
    int num_accumulated_ = 0;
    transform::Rigid3f first_pose_estimate_ = transform::Rigid3f::Identity();
    sensor::RangeData accumulated_range_data_;


    nav_msgs::GetMap::Response map_;
    

};

}
}

#endif // SAMPLE_CARTO_CORE_2D_LOCAL_MAP_BUILDER_H_
