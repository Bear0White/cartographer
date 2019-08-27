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
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// 构造函数，主要调用了基类Grid2D的构造函数
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

// 构造函数：从proto构造
ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(proto, conversion_tables), conversion_tables_(conversion_tables) {
  CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
// 设置某个网格的概率值(浮点)，只有在这个网格之前是未知的才可以。
// 设置的值是浮点，但是内部存储的是整形的空率
// 注意设置之后会更新mutable_known_cells_box（） 里面实时维护了一个有效区域
// （怎么还复杂化了，定义一个更新函数，不管之前是不是未知，都可以更新不行么）
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// 注意设置之后会更新mutable_known_cells_box（） 里面实时维护了一个有效区域
/*
 * 这个函数用来更新栅格中的概率值，用查表法来更新，用到的参数table是预先产生的
 * 在哪里被调用？二维情况下，在probability_grid_range_data_inserter_2d模块中被调用，它是一个插入器，里面唯一的方法就是Insert，
 * 把一帧极光数据插入到一个二维概率栅格中,本质就是对栅格的数据更新操作。在更新时，就会调用下面的方法，来更新某个栅格数据。这个插入器
 * 内部有相关的查找表，是预先计算出来的，用于更新。查找表的本体是ProbabilityGridRangeDataInserter2D中的hit_table_和miss_table_
 * 生成查找表用的是probability_values.cc中
 * ComputeLookupTableToApplyOdds和ComputeLookupTableToApplyCorrespondenceCostOdds这两个函数
 * 该部分正好对应于论文中更新栅格数据的部分
 */ 
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  // 非常重要：如果这个点拥有了更新标志位，那么就不再更新，立即退出
  // mutable_corrxxx_cost_cells()返回的是correspondence_cost_cells_成员，即栅格数据本体
  if (*cell >= kUpdateMarker) {
    return false;
  }
  // 下面这个方法来自基类grid_2d，返回的是内部记录已经更新过的网格
  mutable_update_indices()->push_back(flat_index);
  // 查表法得到更新后的数据，而且查到的数据是带有更新标志位的
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  // 以下的方法来自基类grid_2d，返回的是内部维护的一个描述“有效数据区域”的对象，某个网格更新数据后，它就成了有效的
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

// 返回栅格类型：概率栅格
GridType ProbabilityGrid::GetGridType() const {
  return GridType::PROBABILITY_GRID;
}

// Returns the probability of the cell with 'cell_index'.
// 获取某个栅格的概率值(浮点)，若无，则kMinProbability。最小占率和最大空率都表示未知值
// 返回的值是浮点，但是网格内部存储的是整形的空率
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

// 转换proto
proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

// 重新生成一个网格对象，仅仅包含有效的数据区域
std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  // 下面是基类的方法，计算出有效区域的偏移量和范围
  ComputeCroppedLimits(&offset, &cell_limits);
  
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

// 应该是用来回应service的，暂且不论
bool ProbabilityGrid::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  common::FastGzipString(cells, texture->mutable_cells());
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

}  // namespace mapping
}  // namespace cartographer
