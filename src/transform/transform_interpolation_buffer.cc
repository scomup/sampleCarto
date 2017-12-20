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

#include "src/transform/transform_interpolation_buffer.h"

#include <algorithm>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "src/transform/transform.h"
#include "glog/logging.h"

namespace sample_carto {
namespace transform {


void TransformInterpolationBuffer::Push(const double time,
                                        const transform::Rigid3d& transform) {
  if (!timestamped_transforms_.empty()) {
    CHECK_GE(time, latest_time()) << "New transform is older than latest.";
  }
  timestamped_transforms_.push_back(TimestampedTransform{time, transform});
}

bool TransformInterpolationBuffer::Has(const double time) const {
  if (timestamped_transforms_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

transform::Rigid3d TransformInterpolationBuffer::Lookup(
    const double time) const {
  CHECK(Has(time)) << "Missing transform for: " << time;
  auto start = std::lower_bound(
      timestamped_transforms_.begin(), timestamped_transforms_.end(), time,
      [](const TimestampedTransform& timestamped_transform,
         const double time) {
        return timestamped_transform.time < time;
      });
  const auto end = start;
  if (end->time == time) {
    return end->transform;
  }
  --start;
  if (start->time == time) {
    return start->transform;
  }

  const double duration = end->time - start->time;
  const double factor = (time - start->time) / duration;
  const Eigen::Vector3d origin =
      start->transform.translation() +
      (end->transform.translation() - start->transform.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start->transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end->transform.rotation()));
  return transform::Rigid3d(origin, rotation);
}

double TransformInterpolationBuffer::earliest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.front().time;
}

double TransformInterpolationBuffer::latest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.back().time;
}

bool TransformInterpolationBuffer::empty() const {
  return timestamped_transforms_.empty();
}

}  // namespace transform
}  // namespace sample_carto
