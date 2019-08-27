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

#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
// 计算占空比的对数,log(odds), 占空比=占率/空率
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

// 最大占率对应的占空比对数，与最小占率对应的占空比对数
const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
// 求一个占率的占空比对数，然后把这个值转换到整数空间
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets
// 'insertion_finished' when the map no longer changes and is ready for loop
// closing.
/*
 * 一个子地图，会跟踪有多少帧数据插入其中，如果超过预设值，则设置insertion_finished，此后子地图不再改变，可以参与后端优化
 * 它有一个local_pose，应该是整个submap相对于整个地图的位姿吧，但是为什么注释说实在local map 坐标系下？
 * 这个东西是一个接口类，内容非常单薄，会向下派生出2D和3D的子地图类。
 * 构造：以一个local_submap_pose为参数[还不清楚这个指的是子地图在全局的位姿？还是什么]
 * 方法：
 *    local_pose 获取local_pose_
 *    num_range_data和set_num_range_data 返回和设置插入的激光帧的数目
 *    insertion_finished和set_insertion_finished 返回和设置该子地图是否完结
 */
class Submap {
 public:
  // 构造函数，仅仅有一个local_pose做参数
  Submap(const transform::Rigid3d& local_submap_pose)
      : local_pose_(local_submap_pose) {}
  virtual ~Submap() {}

  // proto转换相关
  virtual proto::Submap ToProto(bool include_grid_data) const = 0;
  virtual void UpdateFromProto(const proto::Submap& proto) = 0;

  // Fills data into the 'response'.
  // 用以应答对子地图的请求
  virtual void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      proto::SubmapQuery::Response* response) const = 0;

  // Pose of this submap in the local map frame.
  // 返回local_pose_
  transform::Rigid3d local_pose() const { return local_pose_; }

  // Number of RangeData inserted.
  // 返回和设置多少帧数据被插入
  int num_range_data() const { return num_range_data_; }
  void set_num_range_data(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

  // 返回和设置该子地图是否完结
  bool insertion_finished() const { return insertion_finished_; }
  void set_insertion_finished(bool insertion_finished) {
    insertion_finished_ = insertion_finished;
  }

 private:
  // local_pose_ [到底是子地图在全局的位姿，还是在local map的位姿？]
  const transform::Rigid3d local_pose_;
  // 有多少帧数据被插入
  int num_range_data_ = 0;
  // 是否完结
  bool insertion_finished_ = false;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
