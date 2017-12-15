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

#ifndef SAMPLE_CARTO_CORE_POSE_EXTRAPOLATOR_H_
#define SAMPLE_CARTO_CORE_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "src/sensor/odometry_data.h"
#include "src/transform/rigid_transform.h"

namespace sample_carto {
namespace core {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(double pose_queue_duration);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;


  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  double GetLastPoseTime() const;

  void AddPose(double time, const transform::Rigid3d& pose);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(double time);

 private:
  void UpdateVelocitiesFromPoses();
  void TrimOdometryData();
  Eigen::Quaterniond ExtrapolateRotation(const double time);
  Eigen::Vector3d ExtrapolateTranslation(const double time);

  const double pose_queue_duration_;
  struct TimedPose {
    double time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_POSE_EXTRAPOLATOR_H_
