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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
 * 关于TimedPointCloudData的定义：
 * struct TimedPointCloudData { common::Time time; Eigen::Vector3f origin; TimedPointCloud ranges; };
 * using TimedPointCloud = std::vector<TimedRangefinderPoint>;
 * struct TimedRangefinderPoint { Eigen::Vector3f position; float time; };
 */

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {

  /*
   * 这个函数做了什么：它调整了current_start_和end_两个量，然后去调用CropAndMerge来获取结果并返回
   */

  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  // id_to_pending_data_ 类型是 map<string, sensor::TimedPointCloudData>，相当于传感器数据记录表
  // 注意，这里是一个map，意味着一个传感器只能有一组数据。这里的组指的是旋转式雷达的一个扫描周期
  // 如果记录表中已经有这个传感器的数据了
  if (id_to_pending_data_.count(sensor_id) != 0) {
    // 更新区间起始点
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    // 终止点就是该记录表中该传感器的时间点
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    // 更新记录表
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    return result;
  }
  // 如果记录表中没有这个传感器的数据，那就添加进去
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  // 如果期望的传感器列表中，有的传感器没有数据被记录，则返回空数据
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  // 如果所有的期望传感器数据都有是数据：调整区间起始点，是当前的end
  current_start_ = current_end_;
  // 以下是调整区间终止点，是所有记录表中的最旧的时刻
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

/*
 *
 * 看到最后忍不住就要骂人了，数据结构这么混乱复杂
 */
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    // TimedPointCloudData 里面包含了common::Time time; Eigen::Vector3f origin; TimedPointCloud ranges;
    sensor::TimedPointCloudData& data = it->second;
    // 带时间的点云本质是一个数组，元素包含了扫描点坐标和相对时间
    sensor::TimedPointCloud& ranges = it->second.ranges;

    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                                                                  origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }

  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
