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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

/*
 * 关联扫描匹配器，这是作为“实时扫描匹配器”和“快速关联扫描匹配器”的基类出现的，里面定义了一些基本操作和数据类型
 */

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 定义数据类型：离散化扫描帧，本体是一个"二维整数向量"的数组
typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  // 其实这个东西应该叫做“平移边界”，表示移动窗口在x和y方向的平移量的边界，是一个闭区间。
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };
  // 构造函数，非常重要。
  // 参数：linear_search_window 单边平移搜索范围，单位米
  // angular_search_window 单边旋转搜索范围，单位弧度
  // 实际搜索范围是两倍的单边范围，例如，单边旋转搜索范围为pi/3，则整个搜索的范围是-pi/3 ~ pi/3。
  // point_cloud 点云数据
  // resolution 栅格地图分辨率，单位米
  // 搜索窗口的使用过程：首先给定一个扫描帧，一个栅格地图，一个初始位姿估计（如果没有则默认零）
  // 首先把扫描帧应用初始位姿估计，直观来说，就是把扫描帧“对准”在栅格地图上
  // 产生旋转窗口，根据论文公式可以计算出旋转步长，然后根据配置参数“单边旋转量”，产生一个个独立的窗口
  // 对于每个旋转窗口，再去应用平移窗口，并计算分值
  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);

  // For testing.
  // 此处应该仅仅用于测试，略去。
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  // 尽量地缩紧搜索窗口
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);

  // 旋转搜索窗格的个数
  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  // 
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
/* 一个候选人，记录了一个具体的旋转和平移量。注意，它内部没有具体的点云数据。
 * 但是它们的表示方法是不同的：旋转窗口由上文的函数GenerateRotatedScans来生成
 * 本质是一个向量，里面记录了应用了各个不同旋转窗口的点云数据，而候选人类中的scan_index成员记录了旋转窗口的index；
 * 平移窗口由成员x_index_offset和y_index_offset来记录
 * 此处再次强调一下：旋转窗口预先都应用到了扫描帧上，然后存入了数组，随用随取，避免重复计算；而平移窗口，随用随算，毕竟平移变换的计算量要小得多。
 */
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  // 此处重点说明一下：GenerateRotatedScans函数产生了一系列的旋转窗口下的点云数据，这里表示一个旋转量对应于哪个旋转窗口
  int scan_index = 0;

  // Linear offset from the initial pose.
  // 平移偏移量
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  // 下面的x和y和orientation其实都是由上面的scan_index和xy便宜量，结合搜索步长来计算得到的
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
