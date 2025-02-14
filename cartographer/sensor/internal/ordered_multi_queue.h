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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace sensor {

/*
 * 显然这个东西是用来排序用的，里面两个元素，一个是轨迹号，一个是传感器名称
 */
struct QueueKey {
  int trajectory_id;
  std::string sensor_id;

  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

/*
 * 这个东西叫做有序多个队列，显然应该是多个queue，每个queue都由一个QueueKey来索引。即，一个轨迹号和一个传感器名称可以唯一指定一个queue
 * 整个数据本体是一个哈希，从 QueueKey-> queue
 * 从后续的函数可以看出，每个queue中保存了一组传感器数据，传感器数据也是有序的，排序的方式是时间
 * 它的意义在于整合多个传感器数据队列
 * 构造：默认构造函数
 * 方法：
 *  AddQueue 添加一个queue
 *  MarkQueuAsFinished 把一个queue标记为完成，然后执行派发操作
 *  Add 对某个queue添加数据，然后执行派发操作
 *  Flush 把所有Queue标记为完成，然后执行派发操作
 *  GetBlocker 返回当前阻塞派发的队列
 * 关于派发操作：
 *   派发操作就是从所有的数据队列中，一个一个取出最旧的数据，弹出并用回调函数做处理，直到某个数据队列为空，或其他阻塞情况为止。
 *   派发操作时，如果某队列标记为Finish，则意味着不再有新数据添加到队列中，可以放心大胆地弹出数据做处理，甚至直接删除整个队列。
 *   派发操作还会关注某个轨迹号下所有队列的公共起始时间，这个时间指的是所有队列首元素最大时间戳。如果有数据远远旧于这个时间，则直接丢弃。
 *   具体还有很多细节。
 */

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue();
  OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  // 新增一个Queue
  void AddQueue(const QueueKey& queue_key, Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  // 把一个Queue标记为完成，然后执行Dispatch
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  // 对某个指定的Queue添加数据，再执行Dispatch
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  // 把所有的Queue都标记为完成，再执行Dispatch
  void Flush();

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

 private:
  // 显然，OrderedMultiQueue维护了多个队列，每个队列都由以下来定义：内含一个队列本体，一个回调函数，一个是否Finish的标记
  struct Queue {
    // BlockingQueue是一个线程安全的队列
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_ = common::Time::min();

  //仅仅用在GetCommonStartTime中，用于记录查找结果，避免重复查找
  std::map<int, common::Time> common_start_time_per_trajectory_;

  // 整个数据的本体
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
