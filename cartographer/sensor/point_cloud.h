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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

/*
 * 本文中定义了点云数据，分为带时间和不带时间的版本。本体是一个激光扫描点的数组，而扫描点有带时间和不带时间的版本。
 * 带时间的版本每个激光点都带有一个时间，所以猜测是针对于旋转式的雷达。这个时间点是相对于最后一个扫描点被捕获的时间来说的，是一个相对时间
 * 此外还定义了带强度值的点云数据，内部是带时间的点云数据和强度值数组
 * 另外定义了(带时间和不带时间的)点云的位姿变换，以给定z值做截断等操作
 */

// Stores 3D positions of points.
// For 2D points, the third entry is 0.f.
// “点云”即一个扫描帧，本质是一个“扫描点”的数组
// 注意：扫描点的数据结构中，仅仅有一个名为position的成员，是一个三维坐标
using PointCloud = std::vector<RangefinderPoint>;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
// 注意：带时间的点云，就是一组“带时间的扫描点”数据。
// “带时间的扫描点”指的是每个扫描点都带有一个时间,其结构体中带有一个名为postion的三维坐标，和一个名为time的时间数据
// 猜测针对的是旋转式的雷达，每个扫描点的捕获时间是不同的。结构体中的time是一个相对时间，相对于最后一个扫描点的捕获时间来说的
using TimedPointCloud = std::vector<TimedRangefinderPoint>;

// 带强度的点云数据，内部是“带时间的点云数据”和一个存储强度值的浮点数组
struct PointCloudWithIntensities {
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
// 对点云数据应用三维位姿变换
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
// 对带时间的点云数据应用三维位姿变换
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
// 把一个点云数据以某个z区间做截断
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
// 把一个带时间的点云数据以某个z区间做截断
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
