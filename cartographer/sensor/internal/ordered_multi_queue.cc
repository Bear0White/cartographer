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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

// 添加一个queue，仅仅是在queues_里面添加了空元素并初始化了它的callback而已
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

// 给某个指定的Queue标记为完成，然后调用Dispatch
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);
  queue.finished = true;
  Dispatch();
}

// 给某个指定的Queue添加数据
// 如果这个Queue不存在，则return；否则给Queue添加数据，然后Dispatch
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  it->second.queue.Push(std::move(data));
  Dispatch();
}

// 遍历所有的Queue，把所有未完成的Queue使用MarkQueueAsFinished标记成完成并调用Dispatch，此举会清理掉所有数据
void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

// 得到blocker_
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/*
 * 最难搞的一个函数了
 * 最外层是一个while，不断循环：每次循环都找到所有队列中，首元素时间戳最小的那个元素，尝试弹出并用回调函数处理；
 * 直到遇见某个队列为空，或其他阻塞情况为止。
 * 其中会调用GetCommonStartTime，使用的场景看，应该是某个轨迹下所有队列都有数据后，所有首元素的最大时间戳。只会计算一次，随后用查表来得到。
 * 阻塞情况请查看具体实现。
 */
void OrderedMultiQueue::Dispatch() {
  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    // 遍历所有的Queue，如果遇见未Finish的空队列，就会终止函数。同时会查找首元素时间戳最小者，记作next_data
    for (auto it = queues_.begin(); it != queues_.end();) {
      // 尝试取出Queue的头部元素
      const auto* data = it->second.queue.Peek<Data>();
      // 如果Queue没有头部元素，即Queue已经空了
      if (data == nullptr) {
        // 如果Queue已经被标记为结束，则直接把这个Queue从队列中删掉，迭代下一个Queue
        if (it->second.finished) {
          queues_.erase(it++); //这样做不太好，似乎迭代器会失效？
          continue;
        }
        // 如果Queue没有标记为结束，则把这个Queue的键存入blocker_中并从整个函数中返回，视作阻塞情况
        // 所以，如果发现有空的Queue却没有标记结束，则保存在blocker_并结束整个函数，注意是整个函数
        CannotMakeProgress(it->first);
        return;
      }
      // 用“擂台法”去搜索所有Queue里面，头部元素的时间最小的那个
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      ++it;
    }
    // 如果next_data为空，显然此时queues_为空,直接返回
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // GetCom...返回的是这个轨迹号下所有队列首元素最大时间戳。根据上文流程，如果有队列为空则函数会终止，所以走到这一步，一定是所有队列都有数据
    // 注意：这个数值仅在初始化阶段才计算一次，以后就直接查表。这意味着整个过程中值是唯一的
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);
    
    // 下面是对next_data进行处理，它是所有队列的时间戳最小者，即最旧的数据。绝大部分都会进入第一个条件分支，弹出元素并用回调函数处理。
    // 其他两个分支，即数据旧于公共时间点，要么阻塞，要么直接丢弃，仅在某些情况下才用回调函数处理。
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      last_dispatched_time_ = next_data->GetTime();
      // 弹出元素，并用回调函数做处理
      next_queue->callback(next_queue->queue.Pop());
    } else if (next_queue->queue.Size() < 2) {
      // 如果元素不足两个，又没有标记结束，则不能判断是否该丢弃，大概是因为整个算法在计算时，有的数据至少要保留两条，如从里程计数据估计速度的情况
      // 在此处阻塞，终止函数，并保存阻塞项
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      }
      // 如果标记结束了，则可以高高兴兴地弹出元素并用回调函数做处理
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
      // 目标数据在指定时间点之前，而且目标后面的数据依然在指定时间点之前，那么就弹出目标数据，不做任何处理；
      //                         如果目标后面的数据在指定时间点之后，那么就弹出目标数据并用回调函数处理
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

// 名字很奇怪，无法执行程序。实际内部的有效操作也仅仅把queue_key保存到blocker_中
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    if (entry.second.queue.Size() > kMaxQueueSize) {
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

/*
 * 查找某个轨迹号的公用起始时间，毕竟一个轨迹号可以有很多传感器的队列
 * 核心用到了common_start_time_per_trajectory_成员，这个东西就是用来记录查找结果的，避免重复查找
 * 首先看看有没有记录，有的话直接返回；如果没有，则插入一条记录，并且寻找该轨迹号的所有Queue，把头部元素时间戳最大者作为结果返回，并更新这个记录
 * 简单来说，返回所有该轨迹号下的所有Queue中，队首元素最大的时间戳
 * [如果某个记录一旦插入，则对该轨迹号的任何查找都会直接返回该条记录，如果队列更新了怎么办？也就是说这个函数只在最初时刻才运作，之后就是查表而已]
 * [这个函数只在Dispatch调用，而流程中，只有所有queue都有数据时，才会调用。
 * 所以函数的本意是：在初始阶段，某个轨迹号下所有传感器队列都有数据后，查找所有首元素时间戳最大者。
 * 物理意义就是，这个时刻之后，所有传感器队列都有数据了（当然啦，carto认为传感器数据都是周期性连贯的）。所以叫做公共起始时间
 * ]
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  // 成员common_start_time_per_trajectory_仅仅在这个函数中有用到，是用来记录查找结果的，避免重复查找
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  // emplace函数原型是这样的：pair<iterator,bool> emplace (Args&&... args);
  // 返回的是迭代器和bool的一个pair，如果插入成功，则返回新元素的迭代器和true；否则返回已存在的元素的迭代器和false
  common::Time& common_start_time = emplace_result.first->second;
  // 如果插入成功了，则原先记录表里面是没有这项的：找到该轨迹号下所有Queues中队首元素时间最大者，作为common_start_time返回
  if (emplace_result.second) {
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        // 注意这里的common_start_time是引用，修改其值会影响原数据
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer
