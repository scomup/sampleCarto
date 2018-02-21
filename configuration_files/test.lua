-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 10,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

--- Slower but more precise scan matching
TRAJECTORY_BUILDER_2D.use_online_correlavtive_scan_matching = false
--- merging multiple scan data together
TRAJECTORY_BUILDER_2D.scans_per_accumulation = 3
--- n node per submap
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 40
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
--- baselink to laser
TRAJECTORY_BUILDER_2D.baselink_to_laser_x = 0
TRAJECTORY_BUILDER_2D.baselink_to_laser_y = 0
TRAJECTORY_BUILDER_2D.baselink_to_laser_theta = 0
--- the search window size for inter constraint
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45)
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
--- the min score for inter constraint
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.90
--- sampling ratio for finding constraint
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 1.
--- log info
MAP_BUILDER.sparse_pose_graph.constraint_builder.log_matches = false
MAP_BUILDER.sparse_pose_graph.log_residual_histograms = false

return options
