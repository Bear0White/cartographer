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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
/*
 * 描述一个栅格地图的边界。
 * 构造函数：
 *  - 以(resolution, max, CellLimits)构造：初始化相关成员即可
 *  - 从proto流中构造
 * 方法：
 *  - resolution 返回分辨率
 *  - max 返回x和y的最大边界，单位米
 *  - cell_limits 返回cell_limits_成员
 *  - GetCellIndex 
 *  - Contains 是否包含了某个离散坐标的栅格
 * 注意：
 *  整个栅格的基准点实际上是max点！它没有原点！得到的离散坐标值也是相对于max点来说的！
 */ 
class MapLimits {
 public:
  /* 
   * 构造函数：分辨率，xy方向的最大值(单位米)，离散化的边界限制CellLimits
   * 内部仅仅实现了成员初始化而已
   */
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  // 从proto中构造
  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  // 返回分辨率(单位米)
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  // xy方向上的最大值，单位米
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  // 离散化的网格限制, 内部只有两个成员：x和y方向上的网格数量，仅此而已
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  /*
   * 看起来最别扭的一个函数：虽然看起来是给定一个物理坐标，返回它在网格的离散坐标
   * 但是根本不是这样的！返回的是距离max坐标的位置差，即距离max点相距几个网格，网格数目是向下取整得到的
   * 这么回头看，整个类，在定位时的基准点是max点，它是没有原点的！
   * 所以说，整个类里面的信息可以这么组织：
   * 以max点作为基准点，内部的CellLimits描述了栅格的数量，以分辨率作为媒介映射到物理空间
   * 后续的所有GetCellIndex以及后续的GetCellCenter都要注意了！
   * 【得到的离散坐标值是相对于max点来说的！！！！】
   */
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
        // 注意：RoundToInt是四舍五入取整，对(x-0.5)四舍五入就是直接向下取整，例如1.8->1 0.9->0 1.2->1
  }

  // Returns the center of the cell at 'cell_index'.
  // 给定某个栅格坐标，计算该栅格中心点位置的物理坐标（单位米）
  // 注意这个栅格坐标是相对于max点来说的，详见GetCellIndex函数注释
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const {
    return {max_.x() - resolution() * (cell_index[1] + 0.5),
            max_.y() - resolution() * (cell_index[0] + 0.5)};
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  // 给出一个二维坐标，判断是否包含在目前的栅格中，判断依据是CellLimits
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  // 分辨率，单位米
  double resolution_;
  // x和y方向上的最大值，单位米
  Eigen::Vector2d max_;
  // 内部只有两个成员：x和y方向上的网格数量，仅此而已
  CellLimits cell_limits_;
};

// 把MapLimits转换到proto格式
inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
