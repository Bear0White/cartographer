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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
/*
 * 概率栅格，从Grid2D派生出来的
 */
class ProbabilityGrid : public Grid2D {
 public:
  // 从MapLimits中构造
  explicit ProbabilityGrid(const MapLimits& limits,
                           ValueConversionTables* conversion_tables);

  // 从proto中构造
  explicit ProbabilityGrid(const proto::Grid2D& proto,
                           ValueConversionTables* conversion_tables);

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  // 给某个指定的栅格设置概率值
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  // 这个东西是干嘛的？“应用查找表”？实际上是用来更新网格的占率值用的
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  // 返回栅格类型，是一个枚举量
  GridType GetGridType() const override;

  // Returns the probability of the cell with 'cell_index'.
  // 返回某个栅格的概率值
  float GetProbability(const Eigen::Array2i& cell_index) const;

  // 转换到proto
  proto::Grid2D ToProto() const override;

  // 生成一个包含所有有效数据的网格
  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  
  // 应该是用来回应某个service，此处略过
  bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const override;

 private:
  ValueConversionTables* conversion_tables_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
