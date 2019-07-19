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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/probability_values.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
// 代价函数类，类的名字可以任意。类中有一个模板函数，重载了()运算符，这个函数就是代价函数本体；类的构造函数通常用来传递常量
// 这里注意一下概念：待优化变量，常量，残差。
class OccupiedSpaceCostFunction2D {
 public:
  // 构造函数，通常用来传入常量
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}
  // 代价函数本体，本质是重载()运算符，第一个参数是待优化变量，最后一个参数是残差，注意参数的类型和格式
  // 注意，参数中可以有多个待优化变量，最后一个参数为残差，也是允许的
  // 待优化变量：从扫描帧到栅格地图的位姿转换关系
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 根据pose产生3*3的2D变换矩阵transform。
    // 再次声明，此处的pose是三维向量，但确切来说，
    // 它并不代表位姿，而是从扫描帧到栅格地图的位姿变换
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
    
    // 以下的类在下文中定义，作用是对一个栅格四周做扩展。
    // 为什么做扩展？后续在求残差的时候，要用到栅格在某个坐标处的值，这个值由ceres的一个差值类BiCubicInterpolator计算产生。
    // 这个差值类对离散的栅格数据做差值，使其更平滑。
    // 但是差值类在构造的时候，以栅格做参数，要求栅格是无穷的；
    // 所以需要对原来有限的栅格四周做一个很大的扩展，扩展后的栅格就是adapter，把它交给ceres差值类
    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();
    
    // 对扫描中的每个点都做一次残差计算，可见最后的残差是多维的，每个维度对应一个点的代价值。
    // 此处还没有理解为什么残差可以是多维的？最后求max的时候难道把残差再做一个二范数么
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      // 把单个点的坐标扩展为齐次坐标
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      // 求出这个点经过变换后，对应的栅格地图上的位置点
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      // 评估栅格地图在这个位置点上的概率值
      // 此处参考ceres官网，差值对象的Evaluate方法，可以求出给定二维点处的值以及偏导。
      // 此处求出的是给定点的值，存入了残差项内。
      // 关于坐标转换：此处是比较难理解的，似乎栅格坐标系和实体坐标系转换挺难的，到底是怎么定义的
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      // 最后乘以一个系数作为最终结果
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    // 数据的维度，例如RGB图像的数据维度就是3，这个枚举类型在ceres的插值类中规定必须具备
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}
    
    // 获得某个2D坐标处的值，这个方法在ceres的插值类中规定必须具备
    // 不难理解，这个函数实现了这样的功能：对原来的栅格做扩展，给出新栅格的坐标，返回新栅格在坐标处的值
    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const Grid2D& grid_;
  };

  /*    栅格扩展示意图
   *    000000000000000000
   *    000000000000000000
   *    000011111111110000
   *    000011111111110000
   *    000011111111110000
   *    000000000000000000
   *    000000000000000000
   */

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const Grid2D& grid_;
};

}  // namespace

// 创造代价函数，这个是代价函数类的高层次调用，在这个函数中会构造代价函数类，
// 并把相应的常量传入，并且会规定变量维度，求解方式等细节
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size());
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
