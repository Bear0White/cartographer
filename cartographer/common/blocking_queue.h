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

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
/*
 * 一个线程安全的阻塞式队列，用于生产消费者模式
 * 整体用了absl中的互斥锁，虽然整体能读懂，但不了解执行过程
 * [例如，如果队列为空，那pop时会等待，但是已经占用了互斥锁，其他线程还怎么push？莫非await自动包含了释放的过程？]
 */
template <typename T>
class BlockingQueue {
 public:
  // 此处的零表示无限长度，用来初始化队列
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  // 空参数的构造函数：调用另一个版本的构造函数，去构造一个表示无限长度的队列
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.
  // 构造函数：构造某个指定长度的队列
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  // 把一个东西压入队列，如果队列满，则阻塞。压入时调用了std::move
  void Push(T t) {
    // 此处predicate是一个lamda函数，EXCLUSIVE_LOCKS_REQUIRED暂且不明（大概是absl库里面的），猜测是这个函数执行时会需要互斥锁
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      //函数中直接调用了QueueNotFullCondition方法，返回是否“非满”
      return QueueNotFullCondition();
    };
    // 请求锁
    absl::MutexLock lock(&mutex_);
    // 等待predicate函数返回真
    mutex_.Await(absl::Condition(&predicate));
    // push操作，调用了std::move
    deque_.push_back(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  // 带超时的压入操作，和上文的压入操作仅仅差别是，使用了带超时的Await函数
  bool PushWithTimeout(T t, const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueNotFullCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  // 弹出元素，如果队列是空的，则阻塞到非空
  T Pop() {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  // 带超时的Pop版本
  T PopWithTimeout(const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Peek, but can timeout. Returns nullptr in this case.
  // 返回队首元素，如果对列为空，则等待至超时
  template <typename R>
  R* PeekWithTimeout(const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  // 返回队首元素，如果队列为空，则直接返回空指针
  template <typename R>
  const R* Peek() {
    absl::MutexLock lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  // 返回队列长度
  size_t Size() {
    absl::MutexLock lock(&mutex_);
    return deque_.size();
  }

  // Blocks until the queue is empty.
  // 等待队列为空
  void WaitUntilEmpty() {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));
  }

 private:
  // Returns true iff the queue is empty.
  // 如果队列为空则返回true
  bool QueueEmptyCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return deque_.empty();
  }

  // Returns true iff the queue is not full.
  // 如果队列非满，则返回true
  bool QueueNotFullCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    // 如何定义非满？这里的queue_size_是预设的最大容量，如果预设的容量是无限，或者当前size小于最大容量，则判定非满
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  // absl的互斥锁
  absl::Mutex mutex_;
  // 被锁保护的变量
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
