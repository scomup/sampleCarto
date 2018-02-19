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

#include "fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "src/common/math.h"
#include "src/core/map/probability_grid.h"
#include "src/sensor/point_cloud.h"
#include "src/transform/transform.h"
#include "glog/logging.h"

namespace sample_carto
{
namespace core
{
namespace scan_matching
{
void FastCorrelativeScanMatcherOptions::Create(common::LuaParameterDictionary *const parameter_dictionary)
{
  linear_search_window_ = parameter_dictionary->GetDouble("linear_search_window");
  angular_search_window_ = parameter_dictionary->GetDouble("angular_search_window");
  branch_and_bound_depth_ = parameter_dictionary->GetInt("branch_and_bound_depth");
}
namespace
{

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
class SlidingWindowMaximum
{
public:
  void AddValue(const float value)
  {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back())
    {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value)
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front())
    {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurence, and so on.
  std::deque<float> non_ascending_maxima_;
};

} // namespace

PrecomputationGrid::PrecomputationGrid(
    const map::ProbabilityGrid &probability_grid, const map::CellLimits &limits,
    const int width, std::vector<float> *reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells)
{
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float> &intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y)
  {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        probability_grid.GetProbability(Eigen::Array2i(0, y)));
    for (int x = -width + 1; x != 0; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells)
      {
        current_values.AddValue(
            probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));
      current_values.AddValue(
          probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x)
  {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells)
      {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid::ComputeCellValue(const float probability) const
{
  const int cell_value = common::RoundToInt(
      (probability - map::kMinProbability) *
      (255.f / (map::kMaxProbability - map::kMinProbability)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

class PrecomputationGridStack
{
public:
  PrecomputationGridStack(
      const map::ProbabilityGrid &probability_grid,
      const FastCorrelativeScanMatcherOptions &options)
  {
    CHECK_GE(options.branch_and_bound_depth_, 1);
    const int max_width = 1 << (options.branch_and_bound_depth_ - 1);
    precomputation_grids_.reserve(options.branch_and_bound_depth_);
    std::vector<float> reusable_intermediate_grid;
    const map::CellLimits limits = probability_grid.limits().cell_limits();
    reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                       limits.num_y_cells);
    for (int i = 0; i != options.branch_and_bound_depth_; ++i)
    {
      const int width = 1 << i;
      precomputation_grids_.emplace_back(probability_grid, limits, width,
                                         &reusable_intermediate_grid);
    }
  }

  const PrecomputationGrid &Get(int index)
  {
    return precomputation_grids_[index];
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

private:
  std::vector<PrecomputationGrid> precomputation_grids_;
};

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const map::ProbabilityGrid &probability_grid,
    const FastCorrelativeScanMatcherOptions &options)
    : options_(options),
      limits_(probability_grid.limits()),
      precomputation_grid_stack_(
          new PrecomputationGridStack(probability_grid, options)) {}

FastCorrelativeScanMatcher::~FastCorrelativeScanMatcher() {}

bool FastCorrelativeScanMatcher::Match(
    const transform::Rigid2d &initial_pose_estimate,
    const sensor::PointCloud &point_cloud, const float min_score, float *score,
    transform::Rigid2d *pose_estimate) const
{
  const SearchParameters search_parameters(options_.linear_search_window_,
                                           options_.angular_search_window_,
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher::MatchFullSubmap(
    const sensor::PointCloud &point_cloud, float min_score, float *score,
    transform::Rigid2d *pose_estimate) const
{
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(), // Linear search window, 1e6 cells/direction.
      M_PI,                       // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d &initial_pose_estimate,
    const sensor::PointCloud &point_cloud, float min_score, float *score,
    transform::Rigid2d *pose_estimate) const
{
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  const std::vector<Candidate> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  const Candidate best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  if (best_candidate.score > min_score)
  {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

std::vector<Candidate>
FastCorrelativeScanMatcher::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan> &discrete_scans,
    const SearchParameters &search_parameters) const
{
  std::vector<Candidate> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

std::vector<Candidate>
FastCorrelativeScanMatcher::GenerateLowestResolutionCandidates(
    const SearchParameters &search_parameters) const
{
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size)
    {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size)
      {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher::ScoreCandidates(
    const PrecomputationGrid &precomputation_grid,
    const std::vector<DiscreteScan> &discrete_scans,
    const SearchParameters &search_parameters,
    std::vector<Candidate> *const candidates) const
{
  for (Candidate &candidate : *candidates)
  {
    int count = 0;
    int sum = 0;
    for (const Eigen::Array2i &xy_index :
         discrete_scans[candidate.scan_index])
    {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      //sum += precomputation_grid.GetValue(proposed_xy_index);
      //(liu) the new score algorithm give higher score to a scandidate
      // whose points laid on the obstacle area.

      double p = PrecomputationGrid::ToProbability(precomputation_grid.GetValue(proposed_xy_index));
      if (p < 0.49 || p > 0.51)
      {
        count++;
      }
      if (p > 0.51)
      {
        sum++;
      }
    }
    //candidate.score = PrecomputationGrid::ToProbability(sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
    //count = ((double)count > (double)discrete_scans[candidate.scan_index].size()*0.7) ? count : discrete_scans[candidate.scan_index].size();
    candidate.score = sum / static_cast<float>(count);
    //std::cout<<"candidate.score:"<<candidate.score<<"\n";

  }
  std::sort(candidates->begin(), candidates->end(), std::greater<Candidate>());
}

Candidate FastCorrelativeScanMatcher::BranchAndBound(
    const std::vector<DiscreteScan> &discrete_scans,
    const SearchParameters &search_parameters,
    const std::vector<Candidate> &candidates, const int candidate_depth,
    float min_score) const
{


  std::vector<Candidate> this_level_candidates(candidates);
  std::vector<Candidate> next_level_candidates;

  ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth),
                  discrete_scans, search_parameters,
                  &this_level_candidates);
  Candidate best_candidate(this_level_candidates[0]);
  for (int i = 0; i < static_cast<int>(this_level_candidates.size()); i++)
  {
    if (this_level_candidates[i].score < min_score)
    {
      this_level_candidates.erase(this_level_candidates.begin() + i, this_level_candidates.end());
    }
  }
  if (this_level_candidates.size() == 0)
  {
    //return a bad result.
    best_candidate.score = 0.0;
    return best_candidate;
  }

  if (candidate_depth == 0)
  {
    //(liu)return a bad result, if there is a candidate far away from the best one.
    for (const Candidate &candidate : this_level_candidates)
    {
      double diff = std::hypot((candidate.x - best_candidate.x), (candidate.y - best_candidate.y));
      if (diff >= 0.2 /*&& candidate.score > best_candidate.score *0.95*/)
      {
        //return a bad result.
        best_candidate.score = 0.0;
        return best_candidate;
      }
    }
    return best_candidate;
  }
  for (const Candidate &candidate : this_level_candidates)
  {
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width})
    {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x)
      {
        break;
      }
      for (int y_offset : {0, half_width})
      {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y)
        {
          break;
        }
        next_level_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
  }

  Candidate next_level_best = BranchAndBound(discrete_scans, search_parameters,
                                             next_level_candidates, candidate_depth - 1,
                                             min_score); //(liu)

  return next_level_best;
}


} // namespace scan_matching
} // namespace core
} // namespace sample_carto
