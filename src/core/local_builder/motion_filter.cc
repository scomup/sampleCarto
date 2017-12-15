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

#include "src/core/local_builder//motion_filter.h"

#include "src/transform/transform.h"
#include "glog/logging.h"

namespace sample_carto {
namespace core {

void MotionFilterOptions::Create(common::LuaParameterDictionary *const parameter_dictionary)
{
  max_time_seconds_ = parameter_dictionary->GetDouble("max_time_seconds");
  max_distance_meters_ = parameter_dictionary->GetDouble("max_distance_meters");
  max_angle_radians_ = parameter_dictionary->GetDouble("max_angle_radians");
}

MotionFilter::MotionFilter(const MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const double time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of scans to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 &&
      time - last_time_ <= options_.max_time_seconds_ &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters_ &&
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians_) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping_3d
}  // namespace cartographer
