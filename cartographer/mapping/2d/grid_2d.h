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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/2d/grid_2d.pb.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {

// 配置参数格式转换
proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

// 枚举：栅格类型
enum class GridType { PROBABILITY_GRID, TSDF };

/*
 * 2D栅格：里面保存了数据实体和一个MapLimits对象，等等。前者记录了数据，后者对栅格的原点和尺寸等信息进行了描述
 * 确切来说MapLimits描述了分辨率，栅格尺寸，和一个max点，表示最大坐标处对应的物理位置点
 */
class Grid2D : public GridInterface {
 public:
  Grid2D(const MapLimits& limits, float min_correspondence_cost,
         float max_correspondence_cost,
         ValueConversionTables* conversion_tables);
  explicit Grid2D(const proto::Grid2D& proto,
                  ValueConversionTables* conversion_tables);

  // Returns the limits of this Grid2D.
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate();

  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
    if (!limits().Contains(cell_index)) return max_correspondence_cost_;
    return (*value_to_correspondence_cost_table_)
        [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
  }

  virtual GridType GetGridType() const = 0;

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
               kUnknownCorrespondenceValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f& point);

  // 裁剪栅格，只需要囊括有效数据点即可
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

  // 转换到Proto
  virtual proto::Grid2D ToProto() const;

  // 从参数类型上来看，应该是用来回应相关服务的，暂不研究
  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 protected:
  // 自动增长栅格大小
  void GrowLimits(const Eigen::Vector2f& point,
                  const std::vector<std::vector<uint16>*>& grids,
                  const std::vector<uint16>& grids_unknown_cell_values);

  // 返回栅格数据本体
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  
  // 返回已经更新过的栅格索引
  const std::vector<int>& update_indices() const { return update_indices_; }
  
  // 返回囊括所有有效数据点的最小栅格区域
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  // 返回栅格数据本体(的指针)
  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }
  
  // 返回已经更新过的栅格索引(的指针), 这个东西在派生类probability_grid中使用
  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  // 返回囊括所有有效数据点的最小栅格区域（的指针）
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  // 把一个二维的索引转换为一维的索引，用来访问栅格数据
  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

 private:
  // 描述栅格的分辨率，栅格尺寸，最大坐标的物理位置
  MapLimits limits_;
  // 栅格数据本体
  std::vector<uint16> correspondence_cost_cells_;
  // 最小最大的浮点占率值
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  // 用于记录每次更新时，已经更新的网格点，indices是index的复数
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  // 用于裁剪栅格，它实时记录了囊括所有有效数据点的最小栅格区域
  Eigen::AlignedBox2i known_cells_box_;
  // 一个指向了浮点数组的指针
  const std::vector<float>* value_to_correspondence_cost_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
