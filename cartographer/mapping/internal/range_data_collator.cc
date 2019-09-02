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
 * 这个类中维护了一个id_to_pending_data_成员,是一个从sensorID到TimedPointCloudData的哈希，它的本意是记录多个激光雷达的数据，
 * 这里的雷达偏向于指代“旋转式雷达”（当然也适用于瞬时式),每次添加数据，就会记录下新的数据并更新这个成员，同时把一定时间区间内的所有激光雷达数据，
 * 按时序封装到TimedPointCloudOriginData类型中返回。
 *
 * 关于TimedPointCloudData的定义：
 * struct TimedPointCloudData { common::Time time; Eigen::Vector3f origin; TimedPointCloud ranges; };
 * using TimedPointCloud = std::vector<TimedRangefinderPoint>;
 * struct TimedRangefinderPoint { Eigen::Vector3f position; float time; };
 * 简而言之，TimedPointCloudData里面有时刻，原点，"带时间戳的扫描点"数组，可以认为是旋转式雷达的一个扫描帧
 * 如果是瞬时式的话，那么各个激光点的时间戳都是一样的
 */
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {

  /*
   * 这个函数做了什么：添加传感器数据，然后返回某个时间区间内所有传感器数据
   * 具体来说，通过id_to_pending_data_.emplace添加数据，通过调整current_start_和current_end_两个量，并调用CropAndMerge来返回结果
   * 如果id_to_pending_data_（简称记录表)已经有这个传感器的数据了，那么添加数据会抹掉原有数据，必须以原有数据的尾部时间作为区间终点，返回所有传感器数据
   * 如果记录表没有这个传感器数据，那么就以所有传感器数据的最旧数据作为区间终点，返回数据
   * [可以看出，整个测量是想到保守的，尽量返回较旧的数据。为什么这样呢？]
   * 
   *    |  |  |  |
   *             |
   * --------------------------- sensor1
   *         |  |  |  |
   *                  |
   * --------------------------- sensor2
   * 
   * 一帧数据包含多个激光点，每个激光点都有时间戳。一帧数据整体有一个时间戳，是最晚的激光点时间戳。如上图所示，一帧有四个激光点，一帧的时间戳是最后的时间戳。
   * 假设此时刚刚加入了sensor2的数据，那么是否该以sensor2的时间戳，作为区间终点，返回数据？显然不可以，毕竟下一帧可能是sensor1的数据，里面包含了senosr2的时间戳之前的激光点
   * 所以，尽量以较早的时间戳作为区间终点，返回数据，可以保证“这个区间内的数据都接收完了，不会有新的数据没接收"的情况
   */

  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  // id_to_pending_data_ 类型是 map<string, sensor::TimedPointCloudData>，相当于传感器数据记录表
  // 注意，这里是一个map，意味着一个传感器只能有一帧数据。这里的帧指的是旋转式雷达的一个扫描周期
  
  // 如果记录表中已经有这个传感器的数据了
  if (id_to_pending_data_.count(sensor_id) != 0) {
    // 交替更新区间起始点
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
  // 如果所有的期望传感器数据都有是数据：交替更新区间起始点
  current_start_ = current_end_;
  
  // 以下是调整时间区间终止点，是所有记录表中的最旧的时刻
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;

  return CropAndMerge();
}

/*
 * 看到最后忍不住就要骂人了，数据结构这么混乱复杂
 *  struct TimedPointCloudOriginData {
      struct RangeMeasurement {
        TimedRangefinderPoint point_time;
        size_t origin_index;
      };
      common::Time time;
      std::vector<Eigen::Vector3f> origins;
      std::vector<RangeMeasurement> ranges;
    };
    从下文看，这个结构体是存储多个传感器的激光点数据的。
 */

/*
 * 以[current_start_, current_end_]作为时间区间，
 * 把区间内id_to_pending_data_中所有传感器的数据都拷贝进TimedPointCloudOriginData对象进行返回，
 * 对象里的数据都是经过时间戳排序的，对象的time元素就是current_end_,
 * 同时去除id_to_pending_data_已拷贝过的，无用的数据。
 * 也就是说，返回的是多个传感器在特定时间区间内的所有激光点数据。
 * [我总感觉有点问题，为了避免重复，应该是取半开半闭区间内的数据，更加合理]
 */
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  //用it去遍历某个传感器的某个帧
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    // TimedPointCloudData 里面包含了common::Time time; Eigen::Vector3f origin; TimedPointCloud ranges;
    sensor::TimedPointCloudData& data = it->second;
    // 带时间的点云本质是一个数组，元素包含了扫描点坐标和相对时间
    // 这里的ranges就认为是一个帧，里面包括了一组激光点，各个激光点都拥有独立的时间戳，这个时间戳是相对于整个帧的相对时间
    sensor::TimedPointCloud& ranges = it->second.ranges;

    // overlap_begin: 从ranges.begin到ranges.end,找到第一个时间点>=current_start_的条目
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }
    // overlap_end: 从overlap_begin到ranges.end,找到第一个大于currnet_end_的条目
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
    // 区间[overlap_begin,overlap_end)的扫描点，全部拷贝进入result中。
    // 注意overlap_begin是第一个时间点>=current_start_的条目，overlap_end是第一个大于currnet_end_的条目，那么[overlap_begin,overlap_end)
    // 就是[current_start_, current_end_]区间内所有条目
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
    // 清理无用的数据点，用到了overlap_end，注意这里的end依然是“尾后”的意思，上文只是处理了[overlap_begin,overlap_end)内的数据
    // 如果overlap_end已经是目前帧的尾后了，说明前面的数据全都拷贝过了，这帧数据就可以丢弃了。删除该帧，然后迭代it
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    // 如果overlap_end等于当前帧的begin，那么[overlap_begin,overlap_end)就是空，这意味着ranges的所有数据都不在指定区间内，此次无需做处理，直接迭代下一个传感器的数据即可
    } else if (overlap_end == ranges.begin()) {
      ++it;
    // 下面这种情况，就是当前帧的数据处理了一部分，那么就把已经处理完的数据去除，把剩下的数据保存作为data，再迭代it即可
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }
  
  // 对result中的元素按照时间戳进行排序
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
