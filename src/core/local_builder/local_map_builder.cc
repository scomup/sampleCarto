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

#include "local_map_builder.h"

#include <limits>
#include <memory>

#include "src/common/make_unique.h"
#include "src/sensor/range_data.h"
#include "src/transform/transform.h"

namespace sample_carto
{
namespace core
{

LocalMapBuilder::LocalMapBuilder(const LocalMapBuilderOptions &options)
    :options_(options)
    ,motion_filter_(options.motion_filter_options_)
    ,active_submaps_(options.submaps_options_)
    ,real_time_correlative_scan_matcher_(options_.real_time_correlative_scan_matcher_options_)
    ,ceres_scan_matcher_(options_.ceres_scan_matcher_options_)
{
}

LocalMapBuilder::~LocalMapBuilder() {}


//transform::Rigid2<double> Project2D(const transform::Rigid3<double>& transform) {
//    transform::Rigid2<double> a;
//  return a;
//}


std::unique_ptr<LocalMapBuilder::InsertionResult> LocalMapBuilder::AddAccumulatedRangeData(double time, const sensor::RangeData &range_data)
{
    
    // Transforms 'range_data' into a frame where gravity direction is
    // approximately +z.

    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(Eigen::Quaterniond(1, 0, 0, 0));
    const sensor::RangeData gravity_aligned_range_data = TransformAndFilterRangeData(gravity_alignment.cast<float>(), range_data);
    if (gravity_aligned_range_data.returns.empty())
    {
        LOG(WARNING) << "Dropped empty horizontal range data.";
        return nullptr;
    }
    // Computes a gravity aligned pose prediction.
    const transform::Rigid3d non_gravity_aligned_pose_prediction = extrapolator_->ExtrapolatePose(time);
    const transform::Rigid2d pose_prediction = transform::Project2D(non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

    transform::Rigid2d pose_estimate_2d = pose_prediction;//add liu
    ScanMatch(time, pose_prediction, gravity_aligned_range_data, &pose_estimate_2d);

    const transform::Rigid3d pose_estimate =
        transform::Embed3D(pose_estimate_2d) * gravity_alignment;
    extrapolator_->AddPose(time, pose_estimate);

    
    if (motion_filter_.IsSimilar(time, pose_estimate))
    {
        return nullptr;
    }
    
    // Querying the active submaps must be done here before calling
    // InsertRangeData() since the queried values are valid for next insertion.
    std::vector<std::shared_ptr<const map::Submap>> insertion_submaps;
    for (const std::shared_ptr<map::Submap> &submap : active_submaps_.submaps())
    {
        insertion_submaps.push_back(submap);
    }
    
    active_submaps_.InsertRangeData(
        TransformRangeData(gravity_aligned_range_data,
                           transform::Embed3D(pose_estimate_2d.cast<float>())));

    return common::make_unique<InsertionResult>(InsertionResult{
        std::make_shared<const Node::Data>(
            Node::Data{
                time,
                gravity_aligned_range_data.returns}),
        pose_estimate, std::move(insertion_submaps)});
}

sensor::RangeData LocalMapBuilder::TransformAndFilterRangeData(
    const transform::Rigid3f &gravity_alignment,
    const sensor::RangeData &range_data) const
{
    const sensor::RangeData cropped = sensor::CropRangeData( sensor::TransformRangeData(range_data, gravity_alignment), options_.min_z_, options_.max_z_);
    return sensor::RangeData{cropped.origin, cropped.returns, cropped.misses};
}

std::unique_ptr<LocalMapBuilder::InsertionResult>
LocalMapBuilder::AddRangeData(const double time, const sensor::RangeData &range_data)
{
    
    // Initialize extrapolator now if we do not ever use an IMU.
    InitializeExtrapolator(time);

    if (extrapolator_ == nullptr)
    {
        // Until we've initialized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return nullptr;
    }
    if (num_accumulated_ == 0)
    {
        first_pose_estimate_ = extrapolator_->ExtrapolatePose(time).cast<float>();
        accumulated_range_data_ =
            sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}};
    }
    

    const transform::Rigid3f tracking_delta = first_pose_estimate_.inverse() * extrapolator_->ExtrapolatePose(time).cast<float>();
    const sensor::RangeData range_data_in_first_tracking = sensor::TransformRangeData(range_data, tracking_delta);
 
    // Drop any returns below the minimum range and convert returns beyond the
    // maximum range into misses.
    for (const Eigen::Vector3f &hit : range_data_in_first_tracking.returns)
    {
        const Eigen::Vector3f delta = hit - range_data_in_first_tracking.origin;
        const float range = delta.norm();
        if (range >= options_.min_range_)
        {
            if (range <= options_.max_range_)
            {
                accumulated_range_data_.returns.push_back(hit);
            }
            else
            {
                accumulated_range_data_.misses.push_back(
                    range_data_in_first_tracking.origin +
                    options_.missing_data_ray_length_ / range * delta);
            }
        }
    }
    ++num_accumulated_;

     if (num_accumulated_ >= options_.scans_per_accumulation_)
    {
        num_accumulated_ = 0;
        return AddAccumulatedRangeData(time, sensor::TransformRangeData(accumulated_range_data_, tracking_delta.inverse()));
    }
    
    return nullptr;
}

void LocalMapBuilder::InitializeExtrapolator(const double time)
{
    
    if (extrapolator_ != nullptr)
    {
        return;
    }
    // We derive velocities from poses which are at least 1 ms apart for numerical
    // stability. Usually poses known to the extrapolator will be further apart
    // in time and thus the last two are used.
    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    extrapolator_ = common::make_unique<core::PoseExtrapolator>(kExtrapolationEstimationTimeSec);
    extrapolator_->AddPose(time, transform::Rigid3d::Identity());
    
}

void LocalMapBuilder::ScanMatch(
    const double time, const transform::Rigid2d &pose_prediction,
    const sensor::RangeData &gravity_aligned_range_data,
    transform::Rigid2d *const pose_observation)
{

    std::shared_ptr<const map::Submap> matching_submap = active_submaps_.submaps().front();
    transform::Rigid2d initial_ceres_pose = pose_prediction;

    if (options_.use_online_correlative_scan_matching_)
    {
        real_time_correlative_scan_matcher_.Match(
            pose_prediction, gravity_aligned_range_data.returns,
            matching_submap->probability_grid(), &initial_ceres_pose);
    }

    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(
        pose_prediction, initial_ceres_pose, gravity_aligned_range_data.returns,
        matching_submap->probability_grid(), pose_observation, &summary);
        
}


void LocalMapBuilder::AddOdometerData(
    const sensor::OdometryData &odometry_data)
{
    
    if (extrapolator_ == nullptr)
    {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
    }
    extrapolator_->AddOdometryData(odometry_data);
    
}



} // namespace core
} // namespace sample_carto
