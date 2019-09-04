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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

/*
 * Collator类相当于是对OrderedMultiQueue类的高级封装，核心操作都是调用了后者的方法
 * 此类直接在MapBuilder调用，后者是整个carto的最高调用接口，所以此类是极高层次的接口
 * 数据主体是OrderedMultiQueue类型的queue_成员，所有操作直接与它相关
 */
class Collator : public CollatorInterface {
 public:
  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  // 添加轨迹，根据轨迹号和传感器ID集合，生成所有的QueueKey，一个一个添加给queue_
  void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) override;
  
  // 给queue_所有队列都标记为Finished
  void FinishTrajectory(int trajectory_id) override;

  // 给某个轨迹号下的某个传感器队列添加数据，注意添加后会自动执行queue_的派发操作
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

  // 直接调用queue_的Flush:把所有未完成的Queue使用标记成完成并执行派发操作，此举会清理掉所有数据
  void Flush() override;
  
  // 返回queue_的阻塞派发的轨迹号
  absl::optional<int> GetBlockingTrajectoryId() const override;

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  // 数据主体：一个OrderedMultiQueue，本体是多个传感器数据队列，里面是按时序的传感器数据
  // 用一个QueueKey索引一个队列，QueueKey包括了一个轨迹号和传感器ID
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  // 哈希，从轨迹号到对应的所有QueueKey数组。其中QueueKey包括了一个轨迹号和传感器ID
  absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
