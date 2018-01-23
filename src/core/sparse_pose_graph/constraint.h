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

#ifndef SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_H_
#define SAMPLE_CARTO_CORE_SPARSE_POSE_GRAPH_CONSTRAINT_H_

#include "src/transform/rigid_transform.h"



namespace sample_carto
{
namespace core
{
namespace sparse_pose_graph
{
// A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
// pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
// 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
struct Constraint
{
    struct Pose
    {
        transform::Rigid3d zbar_ij;
        double translation_weight;
        double rotation_weight;
    };

    int submap_id; // 'i' in the paper.
    int node_id;   // 'j' in the paper.

    // Pose of the scan 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where scan 'j' was inserted into
    // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
    // into submap 'i').
    enum Tag
    {
        INTRA_SUBMAP,
        INTER_SUBMAP
    } tag;
};

}
}
}

#endif