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

  use_odometry = false,

  --- baselink to laser
  baselink_to_laser_x = 0,
  baselink_to_laser_y = 0,
  baselink_to_laser_theta = 0,
}
--- Slower but more precise scan matching
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
--- merging multiple scan data together
TRAJECTORY_BUILDER_2D.scans_per_accumulation = 1
--- n node per submap
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 60
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
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
