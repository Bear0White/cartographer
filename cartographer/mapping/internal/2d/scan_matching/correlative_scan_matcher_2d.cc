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

#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 构造函数：
// linear_search_window 单边平移搜索范围
// angular_search_window 单边旋转搜索范围
// point_cloud 点云，即扫描帧
// resolution 栅格分辨率
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 以下的代码是根据论文中公式，计算旋转搜索步长值angular_perturbation_step_size
  float max_scan_range = 3.f * resolution;
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));
  // 计算单边旋转搜索窗口数量
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  // 计算所有旋转搜索窗口数量，注意这里的名字起得很草率
  num_scans = 2 * num_angular_perturbations + 1;

  // 计算单边平移搜索窗口数量
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);
  // 填充linear_bounds数组，数组元素类型是LinearBounds，里面记录了x和y的搜索边界，单位是像素
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
  /*
   * 函数总结：
   * 构造函数的作用主要为：
   * 计算旋转搜索步长，这个由论文公式给出，与最大激光扫描距离有关；而平移搜索步长仅仅是栅格分辨率的大小。
   * 计算旋转搜索窗口的个数num_scans，对应于linear_bounds线性搜索边界数组的长度。
   * 这里强调一下线性搜索边界数组，里面记录了每个旋转搜索窗口下的平移搜索窗口大小。
   * 所以搜索窗口的产生是有顺序的：先按旋转值展开，再按照平移值展开
   */ 
  
}

// 仅供测试使用，略去
SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

/*
 * 调整类中的linear_bounds成员，里面记录了每个旋转搜索窗口下的平移搜索范围，对它进行调整，尽量缩小平移搜索范围
 * 【这里有个很不解的地方,貌似还涉及到了坐标变换？？】
 * 至少从MapLimits和CellLimits来看，里面只有max而没有min，所以猜测坐标应该和图像坐标一致
 * 这么来说，linear_bounds中的min_x是x方向偏移量，这个数一定是负数，因为搜索应该是双边的；绝对值就是把扫描帧向左平移的量。
 * 而在下面中linear_bounds中min_x是所有扫描点中最小的那个-xy_index，？？？
 */
void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                                   const CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i]) {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

/*
 * 把不同旋转搜索窗口应用到扫描帧上，产生一个数组；数组中的元素，就是应用了不同旋转变换后的点云数据
 */
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);
  
  // delta_theta 是不同旋转搜索窗口对应的旋转角度值
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  // 老实说这段代码也太别扭了吧，直接用delta_theta迭代不就行了
  return rotated_scans;
}

/*
 * 把所有的扫描帧都应用平移变换，然后进行离散化。
 * 注意这里有个平移变换,函数名起得太差劲了
 */
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());
  for (const sensor::PointCloud& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const sensor::RangefinderPoint& point : scan) {
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.position.head<2>();
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
