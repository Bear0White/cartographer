/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

/*
 * 带时间的点云Data， 注意里面有时间，有原点，还有“带时间的点云”
 * “带时间的点云”是一个数组，元素类型是“带时间的扫描点”，后者里面包括了一个扫描点的三维坐标，和扫描点捕获到的相对时间
 * 注意这个时间是相对于最后一个扫描点来说的，所以猜测最后一个扫描点的时间就是整个“带时间的点云Data”中的time
 * 这个应该是描述旋转式雷达的，每个扫描点的时间都是不一样的。
 * 注意里面是有原点坐标的，所以它可以表示经过变换到全局坐标下的扫描数据
 */
struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;
  TimedPointCloud ranges;
};

/*
 * 带时间的点云和原点Data，这名字起得云里雾里的
 * 本体包含了一个时间点数据，和一个原点数组，以及扫描值数组。扫描值中包括了一个带时间的扫描点和一个原点index
 * 猜测这个是描述旋转式雷达用的，而且比“带时间的点云Data”类型更精准，它考虑到了扫描过程中雷达本体的移动。
 * 也就是说，一组扫描数据有N个，那么就有N个激光点数据，N个扫描时间点数据，N个雷达本体的位姿数据
 * 所以可以解释为：time是整个结构体的基准时间，猜测是最后一个扫描点的捕获时间；origins是N个扫描点被捕获时的雷达本体位姿；
 * ranges是每次扫描的数据，包括扫描到的激光点数据（包括捕获到的时间）以及此时的雷达位置index
 */
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

// 把“带时间的点云Data”转换到proto
// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// 从proto中读取"带时间的点云Data"
// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
