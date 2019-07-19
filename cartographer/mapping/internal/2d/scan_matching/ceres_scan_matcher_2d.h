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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

// Ceres扫描匹配器，利用ceres完成精准扫描匹配

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 配置参数从lua到proto的转换
proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
// 类：Ceres扫描匹配器的定义
// 构造函数：由参数来构造
// 方法：Match，把某个点云匹配到栅格地图上
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  // 最核心的方法，把某个点云匹配到某个栅格，使用ceres进行非线性优化，待优化量是点云到栅格的相对位姿
  // 参数：target_translation, 这个是期望的位姿平移向量，这个东西是直接参与到translation_delta代价函数当中的。
  // 简单来说，函数在匹配的时候，会向期望的位姿平移向量靠拢
  // 参数：initial_pose_estimate，初始位姿估计
  // Carto的文档中提到，ceres非线性优化是快速而准确的，但是受初始值影响巨大，所以一定要确保初始值是可靠的才行
  // 参数：point_cloud，点云数据
  // 参数：grid，二维的栅格地图
  // 参数：pose_estimate，位姿估计，用来传出最后的匹配结果
  // 参数：summary，结果概要，用来传出优化的结果
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 // 私有成员：仅仅有匹配器的参数和Ceres的参数而已，其中后者是前者内容的一部分
 private:
  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
