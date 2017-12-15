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

#ifndef SAMPLE_CARTOmapping_SUBMAPS_H_
#define SAMPLE_CARTOmapping_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "src/common/lua_parameter_dictionary.h"
#include "src/mapping/submaps.h"
#include "src/mapping/map_limits.h"
#include "src/mapping/probability_grid.h"
#include "src/mapping/proto/submaps_options.pb.h"
#include "src/mapping/range_data_inserter.h"
#include "src/sensor/range_data.h"
#include "src/transform/rigid_transform.h"

namespace sample_carto
{
namespace mapping
{

// Converts the given probability to log odds.
inline float Logit(float probability)
{
    return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability)
{
    const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                         254.f / (kMaxLogOdds - kMinLogOdds)) +
                      1;
    CHECK_LE(1, value);
    CHECK_GE(255, value);
    return value;
}

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid &probability_grid);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary *parameter_dictionary);

class Submap
{
  public:
    Submap(const MapLimits &limits, const Eigen::Vector2f &origin);
    //explicit Submap(const mapping::proto::Submap2D &proto);

    //void ToProto(mapping::proto::Submap *proto) const;

    const ProbabilityGrid &probability_grid() const { return probability_grid_; }
    bool finished() const { return finished_; }

    //void ToResponseProto(
    //    const transform::Rigid3d &global_submap_pose,
    //    mapping::proto::SubmapQuery::Response *response) const;

    // Insert 'range_data' into this submap using 'range_data_inserter'. The
    // submap must not be finished yet.
    void InsertRangeData(const sensor::RangeData &range_data,
                         const RangeDataInserter &range_data_inserter);
    void Finish();

    // Local SLAM pose of this submap.
    transform::Rigid3d local_pose() const { return local_pose_; }

    // Number of RangeData inserted.
    int num_range_data() const { return num_range_data_; }

  protected:
    void SetNumRangeData(const int num_range_data)
    {
        num_range_data_ = num_range_data;
    }

  private:
    const transform::Rigid3d local_pose_;
    ProbabilityGrid probability_grid_;
    bool finished_ = false;
    int num_range_data_ = 0;
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps
{
  public:
    explicit ActiveSubmaps(const proto::SubmapsOptions &options);

    ActiveSubmaps(const ActiveSubmaps &) = delete;
    ActiveSubmaps &operator=(const ActiveSubmaps &) = delete;

    // Returns the index of the newest initialized Submap which can be
    // used for scan-to-map matching.
    int matching_index() const;

    // Inserts 'range_data' into the Submap collection.
    void InsertRangeData(const sensor::RangeData &range_data);

    std::vector<std::shared_ptr<Submap>> submaps() const;

  private:
    void FinishSubmap();
    void AddSubmap(const Eigen::Vector2f &origin);

    const proto::SubmapsOptions options_;
    int matching_submap_index_ = 0;
    std::vector<std::shared_ptr<Submap>> submaps_;
    RangeDataInserter range_data_inserter_;
};

} // namespace mapping
} // namespace sample_carto

#endif // SAMPLE_CARTOmapping_SUBMAPS_H_
