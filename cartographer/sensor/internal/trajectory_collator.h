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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_

#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

// Waits to see at least one data item for all sensor ids and dispatches data
// in merge-sorted order. Contrary to 'Collator', it does not wait for other
// trajectories.
// Also contrary to 'Collator', whose output is deterministic, the sequence in
// which data is dispatched is not sorted, so non-deterministic input sequences
// will result in non-deterministic output.
/*
 * 建议和Collator类放在一起看，两者都是从接口类继承下来的，核心都是OrderedMultiQueue
 *      OrderedMultiQueue是多个传感器数据队列，由一个QueueKey索引，里面包括轨迹号和传感器ID
 *      方法包括添加队列，向队列添加数据，冲水等操作
 * 此类和Collator不同的是，后者用了OrderedMultiQueue类型的成员queue_,但是这里是哈希<int, OrderedMultiQueue> trajectory_to_queue_
 * 尽管在我看来，必要性并不大，因为QueueKey里面就已经有了轨迹号，相当于是两层哈希，仅仅提高了根据轨迹号索引所有队列的效率，仅此而已
 * 功能上，和Collator并没有什么区别
 */
class TrajectoryCollator : public CollatorInterface {
 public:
  TrajectoryCollator() {}

  TrajectoryCollator(const TrajectoryCollator&) = delete;
  TrajectoryCollator& operator=(const TrajectoryCollator&) = delete;
   
  // 根据轨迹号和传感器ID集合，添加所有的传感器数据队列
  void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) override;

  // 给指定ID号下的所有传感器队列都标记为Finished
  void FinishTrajectory(int trajectory_id) override;

  // 给指定队列和传感器ID(从data中得到)下的队列添加数据，注意此后会触发派发操作
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;
  
  // 对所有的Queue组都执行Flush，此举会标记所有队列为Finish，并派发所有数据
  void Flush() override;

  // 半成品，返回空对象
  absl::optional<int> GetBlockingTrajectoryId() const override;

  // 评估用
  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  metrics::Counter* GetOrCreateSensorMetric(const std::string& sensor_id,
                                            int trajectory_id);

  static cartographer::metrics::Family<metrics::Counter>*
      collator_metrics_family_;

  // Holds individual counters for each trajectory/sensor pair.
  // 用来评估
  absl::flat_hash_map<std::string, metrics::Counter*> metrics_map_;

  // 整个数据主体
  // 一个轨迹号对应一个OrderedMultiQueue，但感觉不必要，因为后者中的索引就包含了轨迹号
  absl::flat_hash_map<int, OrderedMultiQueue> trajectory_to_queue_;

  // Map of trajectory ID to all associated QueueKeys.
  // 记录轨迹号与QueueKey队列的Map
  absl::flat_hash_map<int, std::vector<QueueKey>> trajectory_to_queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_
