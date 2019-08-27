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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "absl/container/flat_hash_set.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {

// Voxel filter for point clouds. For each voxel, the assembled(聚合，组装) point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.

/*
 * 体素滤波器，完成对激光点的滤波操作
 * 构造函数：以浮点类型的分辨率进行构造，分辨率用以对激光点数据做离散化
 * 方法：Filter，分别定义了对三种数据的滤波操作：
 * - PointCloud
 * - TimedPointCloud
 * - vector<TimedPointCloudOriginData::RangeMeasurement>
 * 关于滤波操作：
 * 可以概括为：“把点云数据以指定分辨率做离散化，过滤空间中过于靠近的点”
 * 在滤波过程中，可以动态调节分辨率，使得尽量满足参数min_num_points指定的最小数量，又充分滤掉过近的点
 */
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  // 构造函数：需要分辨率参数，仅仅用于初始化resolution_成员
  explicit VoxelFilter(float size) : resolution_(size) {}

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Returns a voxel filtered copy of 'point_cloud'.
  // 对PointCloud类型的数据进行滤波
  PointCloud Filter(const PointCloud& point_cloud);

  // Same for TimedPointCloud.
  // 对TimedPointCloud类型的数据进行滤波
  TimedPointCloud Filter(const TimedPointCloud& timed_point_cloud);

  // Same for RangeMeasurement.
  // 对vector<TimedPointCloudOriginData::RangeMeasurement>类型的数据进行滤波
  std::vector<TimedPointCloudOriginData::RangeMeasurement> Filter(
      const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
          range_measurements);

 private:
  // 键的类型，是一个32*3的bitset，这个键是用于voxel_set_中的，后者是一个哈希容器
  using KeyType = std::bitset<3 * 32>;
  // 把一个三维整形向量转换成KeyType格式
  static KeyType IndexToKey(const Eigen::Array3i& index);
  // 把一个浮点三维坐标值离散化
  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const;

  // 分辨率，用于GetCellIndex，实现对浮点坐标的离散化
  float resolution_;
  // 体素集合，用来直接参与滤波操作
  absl::flat_hash_set<KeyType> voxel_set_;
};

/*
 * 自适应体素滤波器的配置参数格式转换：从lua到proto
 */
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

/*
 * 自适应体素滤波器
 * 构造函数：从配置参数中构造
 * 方法：Filter，对点云进行滤波处理,核心还是调用了VoxelFilter类中的Filter方法
 */
class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
