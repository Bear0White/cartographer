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

#ifndef CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
#define CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

/*
 * 本模块定义了激光扫描点数据，分为带时间和不带时间的版本，但是数据本体都是一个三维空间坐标
 * 从应用场景来看，带时间的版本是用来描述旋转式雷达，它的各个扫描点的捕获时刻是不同的
 * 此外还定义了与三维刚体变换的相乘操作，与proto的相互转换，以及带时间和不带时间版本的相互转换等
 */


// Stores 3D position of a point observed by a rangefinder sensor.
// 一个扫描点数据，即一个三维空间坐标
struct RangefinderPoint {
  Eigen::Vector3f position;
};

// Stores 3D position of a point with its relative measurement time.
// See point_cloud.h for more details.
// 一个带时间的扫描点数据，根据注释，这个时间是相对时间。
// 所以这个应该是描述那种旋转式扫描器，每个扫描点的捕获时间是不一样的。
// 给定某个时间点作为基准（例如最后一个扫描点获取的时间),那么所有的扫描点都记录一个相对于基准点的相对时间即可
struct TimedRangefinderPoint {
  Eigen::Vector3f position;
  float time;
};

// 对于一个扫描点数据应用三维刚体变换，重载了*运算符
template <class T>
inline RangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                  const RangefinderPoint& rhs) {
  RangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

// 对于一个带时间的扫描点数据应用三维刚体变换，重载了*运算符
template <class T>
inline TimedRangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                       const TimedRangefinderPoint& rhs) {
  TimedRangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

// 扫描点数据的等号定义
inline bool operator==(const RangefinderPoint& lhs,
                       const RangefinderPoint& rhs) {
  return lhs.position == rhs.position;
}

// 带时间的扫描点数据的等号定义
inline bool operator==(const TimedRangefinderPoint& lhs,
                       const TimedRangefinderPoint& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

// 从proto获取扫描点数据
inline RangefinderPoint FromProto(
    const proto::RangefinderPoint& rangefinder_point_proto) {
  return {transform::ToEigen(rangefinder_point_proto.position())};
}

// 把扫描点数据转换为proto格式
inline proto::RangefinderPoint ToProto(
    const RangefinderPoint& rangefinder_point) {
  proto::RangefinderPoint proto;
  *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
  return proto;
}

// 从proto中构造带时间的扫描点数据
inline TimedRangefinderPoint FromProto(
    const proto::TimedRangefinderPoint& timed_rangefinder_point_proto) {
  return {transform::ToEigen(timed_rangefinder_point_proto.position()),
          timed_rangefinder_point_proto.time()};
}

// 把带时间的扫描点数据转换为proto
inline proto::TimedRangefinderPoint ToProto(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  proto::TimedRangefinderPoint proto;
  *proto.mutable_position() =
      transform::ToProto(timed_rangefinder_point.position);
  proto.set_time(timed_rangefinder_point.time);
  return proto;
}

// 把 带时间的扫描点数据 转换为 (不带时间的)扫描点数据
inline RangefinderPoint ToRangefinderPoint(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  return {timed_rangefinder_point.position};
}

// 把 (不带时间的)扫描点数据 转换为 带时间的扫描点数据
inline TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& rangefinder_point, const float time) {
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
