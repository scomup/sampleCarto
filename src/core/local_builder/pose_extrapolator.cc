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

#include "pose_extrapolator.h"

#include <algorithm>

#include "src/common/make_unique.h"
#include "src/transform/transform.h"
//#include "glog/logging.h"

namespace sample_carto {
namespace core {

PoseExtrapolator::PoseExtrapolator(double pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration){}


double PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return 0;
  }
  return timed_pose_queue_.back().time;
}
void PoseExtrapolator::AddPose(const double time,
                               const transform::Rigid3d& pose) {

  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  TrimOdometryData();
}


void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest =
      odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest =
      odometry_data_.back();
  const double odometry_time_delta = odometry_data_oldest.time - odometry_data_newest.time;
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time);
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;

  //auto pose_now = ExtrapolatePose(odometry_data.time);// add by liu
  //AddPose(odometry_data.time, pose_now);


}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const double time) {
  // TODO(whess): Keep the last extrapolated pose.
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  return transform::Rigid3d::Translation(ExtrapolateTranslation(time)) *
         newest_timed_pose.pose *
         transform::Rigid3d::Rotation(ExtrapolateRotation(time));
}


void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = newest_time - oldest_time;
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " ms";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(const double time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =time - newest_timed_pose.time;
    if (odometry_data_.size() < 2) {
      return transform::AngleAxisVectorToRotationQuaternion(
              Eigen::Vector3d(angular_velocity_from_poses_ * extrapolation_delta));

    }
    return transform::AngleAxisVectorToRotationQuaternion(
      Eigen::Vector3d(angular_velocity_from_odometry_ * extrapolation_delta));
}
  

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(const double time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =time - newest_timed_pose.time;
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace core
}  // namespace sample_carto
