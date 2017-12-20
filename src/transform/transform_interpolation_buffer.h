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

#ifndef SAMPLE_CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define SAMPLE_CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <vector>

#include "src/transform/rigid_transform.h"

namespace sample_carto {
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(double time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  bool Has(double time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(double time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  double earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  double latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  struct TimestampedTransform {
    double time;
    transform::Rigid3d transform;
  };

  std::vector<TimestampedTransform> timestamped_transforms_;
};

}  // namespace transform
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
