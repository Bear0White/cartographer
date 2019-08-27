/*
 * Copyright 2018 The Cartographer Authors
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
#include "cartographer/mapping/2d/grid_2d.h"

namespace cartographer {
namespace mapping {
namespace {

// 从proto获取数据
float MinCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::Grid2D: min_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMinCorrespondenceCost;
  } else {
    return proto.min_correspondence_cost();
  }
}

// 从proto获取数据
float MaxCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::Grid2D: max_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMaxCorrespondenceCost;
  } else {
    return proto.max_correspondence_cost();
  }
}
}  // namespace

// 配置参数格式转换：从lua到proto
proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::GridOptions2D options;
  const std::string grid_type_string =
      parameter_dictionary->GetString("grid_type");
  proto::GridOptions2D_GridType grid_type;
  CHECK(proto::GridOptions2D_GridType_Parse(grid_type_string, &grid_type))
      << "Unknown GridOptions2D_GridType kind: " << grid_type_string;
  options.set_grid_type(grid_type);
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  return options;
}

// 构造函数：仅仅是初始化相关成员而已
// 比较有价值的就是从查找表参数中获得一张从[1, 32767]到[min占率，max占率]的查找表，
//   浮点和整形的未知值是kUnknownCorrespondenceValue(零)和max占率
Grid2D::Grid2D(const MapLimits& limits, float min_correspondence_cost,
               float max_correspondence_cost,
               ValueConversionTables* conversion_tables)
    : limits_(limits),
      correspondence_cost_cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownCorrespondenceValue),
      min_correspondence_cost_(min_correspondence_cost),
      max_correspondence_cost_(max_correspondence_cost),
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
          max_correspondence_cost, min_correspondence_cost,
          max_correspondence_cost)) {
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

// 从proto中构造
Grid2D::Grid2D(const proto::Grid2D& proto,
               ValueConversionTables* conversion_tables)
    : limits_(proto.limits()),
      correspondence_cost_cells_(),
      min_correspondence_cost_(MinCorrespondenceCostFromProto(proto)),
      max_correspondence_cost_(MaxCorrespondenceCostFromProto(proto)),
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
          max_correspondence_cost_, min_correspondence_cost_,
          max_correspondence_cost_)) {
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
  if (proto.has_known_cells_box()) {
    const auto& box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  correspondence_cost_cells_.reserve(proto.cells_size());
  for (const auto& cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());
    correspondence_cost_cells_.push_back(cell);
  }
}

// Finishes the update sequence.
/* 
 * 初次看到这里简直一头雾水：这是什么东西
 * 首先，网格中的存储的数据是uint16的整数，表示一个整形的占率。
 * 然而，整数的最高位专门用来记录这个整数有没有被更新过，其他15位才记录占率的值
 * 下面的kUpdateMarker在probability_values.h定义，值为1u << 15，就是最高位是1的其他都是零的uint16整数的值
 * 所以猜测过程是这样的：
 * - 初始状态，所有数据没有更新，都没有标志位；
 * - 迭代时，更新过的数据，都设置标志位，然后压入update_indices_中
 * - 结束迭代，调用FinishUpdate方法，把所有压入update_indices_的东西全部弹出并取消标志位
 * 这样的意义何在？猜测多个激光点可能落在一个网格内，而这个网格的数据更新一次就够了（详见carto论文),不应该重复更新
 * 那么更新的过程在哪里？
 * update_indices_这个成员，通过mutable_update_incices()方法被索引，被派生类probabilty_grid调用
 * 调用者是ProbabilityGrid::ApplyLookupTable，它的作用是更新某个栅格的数值。它会检查这个数值有没有更新标志位，如果有代表本轮它被更新过，则不处理。
 * 否则就通过查表进行更新（这个过程会设置更新标志位），然后添加到update_indices_中
 * 【话说回来你搞个集合不就行了，这么麻烦！！】
 */
void Grid2D::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
// 计算有效数据区域的偏移量和区域范围，主要调用了known_cells_box_，后者实时记录了有效数据范围
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
// 必要是时增大地图以容纳point，必须在FinishUpdate调用后立即调用，在ApplyLookupTable前调用
// 这个东西会改动栅格大小和栅格内的数据，会让某些记录失效
void Grid2D::GrowLimits(const Eigen::Vector2f& point) {
  GrowLimits(point, {mutable_correspondence_cost_cells()},
             {kUnknownCorrespondenceValue});
}

// 增大网格，实际上是创建了新的网格对象，包括新的尺寸和数据，感觉是类中最有技术含量的函数了
void Grid2D::GrowLimits(const Eigen::Vector2f& point,
                        const std::vector<std::vector<uint16>*>& grids,
                        const std::vector<uint16>& grids_unknown_cell_values) {
  // 检查一下，所有数据都完成了一轮操作，调用完FinishUpdate方法
  CHECK(update_indices_.empty());
  // 进行以下操作，直至足以包含目标点为止
  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    // 首先，尝试把整体尺寸扩大一倍，更新MapLimits
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    // 新的MapLimits在原来基础上各个方向都扩大了一倍，但是中心点是不变的，具体画图
    
    // 以下的部分是更新网格数据，把原来网格的数据塞入新网格的位置中去
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;

    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
      std::vector<uint16> new_cells(new_size,
                                    grids_unknown_cell_values[grid_index]);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      *grids[grid_index] = new_cells;
    }
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

// 转换成proto
proto::Grid2D Grid2D::ToProto() const {
  proto::Grid2D result;
  *result.mutable_limits() = mapping::ToProto(limits_);
  *result.mutable_cells() = {correspondence_cost_cells_.begin(),
                             correspondence_cost_cells_.end()};
  CHECK(update_indices().empty()) << "Serializing a grid during an update is "
                                     "not supported. Finish the update first.";
  if (!known_cells_box().isEmpty()) {
    auto* const box = result.mutable_known_cells_box();
    box->set_max_x(known_cells_box().max().x());
    box->set_max_y(known_cells_box().max().y());
    box->set_min_x(known_cells_box().min().x());
    box->set_min_y(known_cells_box().min().y());
  }
  result.set_min_correspondence_cost(min_correspondence_cost_);
  result.set_max_correspondence_cost(max_correspondence_cost_);
  return result;
}

}  // namespace mapping
}  // namespace cartographer
