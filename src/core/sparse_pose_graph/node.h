/*
 * Copyright 2016 The sample_carto Authors
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

#ifndef SAMPLE_CARTO_CORE_TRAJECTORY_NODE_H_
#define SAMPLE_CARTO_CORE_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "src/sensor/range_data.h"
#include "src/transform/rigid_transform.h"

namespace sample_carto {
namespace core {

struct Node {
  struct Data {
    double time;
    sensor::PointCloud filtered_gravity_aligned_point_cloud;
  };

  double time() const { return constant_data->time; }
  bool trimmed() const { return constant_data == nullptr; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data;

  transform::Rigid3d pose;
};

}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_TRAJECTORY_NODE_H_
