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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

namespace {

/*
 * 根据最大距离值去对点云(一帧扫描数据)进行滤波，直接把点云中超出最大距离的激光点去掉
 * 重申，点云PointCloud是激光点RangefinderPoint的数组，后者仅有一个position成员，是一个三维向量
 */
PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  PointCloud result;
  for (const RangefinderPoint& point : point_cloud) {
    if (point.position.norm() <= max_range) {
      result.push_back(point);
    }
  }
  return result;
}

PointCloud AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloud& point_cloud) {
  // 如果点云数据已经很少，少于指定的"最少点数目",那么没必要滤波，直接返回
  // [猜测数量点太少，应该是没法用的]
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  // 以options.max_length为参数构建体素滤波器，对点云数据进行滤波
  // 显然，这里的max_length是分辨率，滤波过程是“把点云数据以指定分辨率做离散化，过滤空间中过于靠近的点”
  PointCloud result = VoxelFilter(options.max_length()).Filter(point_cloud);
  // 如果发现过滤后有足够的点，足够指定的"最少点数目",则可以直接返回
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  
  /*
   * 如果滤波后没有足够的点，则不断降低分辨率，直至为原分辨率的百分之一，以期可以获得更多的点
   * 具体的过程也很有趣，用了二分搜索(比较这里是浮点范围的搜索，没法直接用STL),来获得最佳分辨率
   * -------
   * for high = ori_high; high > ori_high / 100; high /= 2.0:
   *    low = high / 2;
   *    if 以low滤波后数量足够:
   *      在范围[low, high]中从右向左搜索, 直到找到分辨率可以满足条件则更新结果，或者搜索到1.1*low处
   *    返回结果
   * 返回结果
   * -------
   * 整个搜索过程值得学习，毕竟不是从数组里面找到某个元素，离散的有穷的搜索空间可以用STL，这里是浮点，空间是连续无穷的
   * 
   * 不过话说回来，为什么不直接在[ori_high/100, ori_high]区间进行二分搜索？
   */

  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(low_length).Filter(point_cloud);
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate =
            VoxelFilter(mid_length).Filter(point_cloud);
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

}  // namespace

/*
 * Filter 方法，参数是PointCloud类型
 * 过程很简单，对参数中所有的激光点做离散化，然后滤除过于靠近的点。
 * 每个离散的位置处有一个点就够了，后续落在同一个离散位置的点统统过滤
 */
PointCloud VoxelFilter::Filter(const PointCloud& point_cloud) {
  PointCloud results;
  for (const RangefinderPoint& point : point_cloud) {
    auto it_inserted =
        voxel_set_.insert(IndexToKey(GetCellIndex(point.position)));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

/*
 * Filter 方法，参数是TimedPointCloud类型
 * 虽然参数类型不同，但是过程基本一致
 */
TimedPointCloud VoxelFilter::Filter(const TimedPointCloud& timed_point_cloud) {
  TimedPointCloud results;
  for (const TimedRangefinderPoint& point : timed_point_cloud) {
    auto it_inserted =
        voxel_set_.insert(IndexToKey(GetCellIndex(point.position)));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

/*
 * Filter 方法，参数是vector<TimedPointCloudOriginData::RangeMeasurement>类型
 * 虽然参数类型不同，但是过程基本一致
 */
std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter::Filter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements) {
  std::vector<TimedPointCloudOriginData::RangeMeasurement> results;
  for (const auto& range_measurement : range_measurements) {
    auto it_inserted = voxel_set_.insert(
        IndexToKey(GetCellIndex(range_measurement.point_time.position)));
    if (it_inserted.second) {
      results.push_back(range_measurement);
    }
  }
  return results;
}

/*
 * KeyType是一个32*3的bitset，把一个三维整形向量依次填充到其中去
 * 第一个元素的二进制形式填充到bitset的前32位，第二个元素到中间32位，... 以此类推
 */
VoxelFilter::KeyType VoxelFilter::IndexToKey(const Eigen::Array3i& index) {
  KeyType k_0(static_cast<uint32>(index[0]));
  KeyType k_1(static_cast<uint32>(index[1]));
  KeyType k_2(static_cast<uint32>(index[2]));
  return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
}

/*
 * 把一个三维点的坐标离散化
 * 相当于根据resolution_建立立体网格，求出三维点在网格中的位置
 * 具体计算过程也仅仅是坐标除以分辨率然后做四舍五入
 * [总觉得对于立体网格来说，向下取整而不是四舍五入更合理些吧，如果是四舍五入，
 * 就相当于把三维点对准在了网格“刻度”处而不是网格“内部”,不过这对于滤波整体结果影响不大]
 */
Eigen::Array3i VoxelFilter::GetCellIndex(const Eigen::Vector3f& point) const {
  Eigen::Array3f index = point.array() / resolution_;
  return Eigen::Array3i(common::RoundToInt(index.x()),
                        common::RoundToInt(index.y()),
                        common::RoundToInt(index.z()));
}

/*
 * 自适应体素滤波器的配置参数格式转换：从lua到proto
 */
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AdaptiveVoxelFilterOptions options;
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}

/*
 * 自适应体素滤波器的构造函数，仅仅初始化options_成员
 */
AdaptiveVoxelFilter::AdaptiveVoxelFilter(
    const proto::AdaptiveVoxelFilterOptions& options)
    : options_(options) {}

/*
 * 自适应体素滤波器中唯一的方法：Filter
 * 参数：PointCloud，点云，数据格式是激光点的数组
 * 内容：仅仅是调用AdaptivelyVoxelFiltered和FilterByMaxRange函数
 * 后者会过滤超出范围的激光点，前者过滤空间中过于靠近的激光点，同时动态调节分辨率以满足最少点数目min_num_points的要求
 */
PointCloud AdaptiveVoxelFilter::Filter(const PointCloud& point_cloud) const {
  return AdaptivelyVoxelFiltered(
      options_, FilterByMaxRange(point_cloud, options_.max_range()));
}

}  // namespace sensor
}  // namespace cartographer
