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

#ifndef SAMPLE_CARTO_CORE_3D_MOTION_FILTER_H_
#define SAMPLE_CARTO_CORE_3D_MOTION_FILTER_H_

#include <limits>

#include "src/common/lua_parameter_dictionary.h"
#include "src/transform/rigid_transform.h"

namespace sample_carto {
namespace core {

class MotionFilterOptions
{
  public:
    void Create(common::LuaParameterDictionary *const parameter_dictionary);
    double max_time_seconds_;
    double max_distance_meters_;
    double max_angle_radians_;
};

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(double time, const transform::Rigid3d& pose);

 private:
  const MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  double last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // SAMPLE_CARTO_CORE_3D_MOTION_FILTER_H_
