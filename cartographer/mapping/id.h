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

#ifndef CARTOGRAPHER_MAPPING_ID_H_
#define CARTOGRAPHER_MAPPING_ID_H_

#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <tuple>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace internal {

/*
 * 以下三个函数分别返回某个对象t的time()函数结果，和time成员
 * 但是搜遍整个项目，都没找到它们在哪里被调用
 */
template <class T>
auto GetTimeImpl(const T& t, int) -> decltype(t.time()) {
  return t.time();
}
template <class T>
auto GetTimeImpl(const T& t, unsigned) -> decltype(t.time) {
  return t.time;
}
template <class T>
common::Time GetTime(const T& t) {
  return GetTimeImpl(t, 0);
}

}  // namespace internal

// Uniquely identifies a trajectory node using a combination of a unique
// trajectory ID and a zero-based index of the node inside that trajectory.
/*
 * 表征一条轨迹上的某个节点，用一个轨迹号和一个节点号来表示
 * 并定义了相等，不等，小于符号，猜测用于排序等用途。
 */
struct NodeId {
  NodeId(int trajectory_id, int node_index)
      : trajectory_id(trajectory_id), node_index(node_index) {}

  int trajectory_id;
  int node_index;

  bool operator==(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) ==
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }

  bool operator!=(const NodeId& other) const { return !operator==(other); }

  bool operator<(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) <
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }

  void ToProto(proto::NodeId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_node_index(node_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const NodeId& v) {
  return os << "(" << v.trajectory_id << ", " << v.node_index << ")";
}

// Uniquely identifies a submap using a combination of a unique trajectory ID
// and a zero-based index of the submap inside that trajectory.
/*
 *  表征一个Submap，用一个轨迹号和submap编号来表示
 * 定义了等于，不等，小于符号，用于排序等场合
 */
struct SubmapId {
  SubmapId(int trajectory_id, int submap_index)
      : trajectory_id(trajectory_id), submap_index(submap_index) {}

  int trajectory_id;
  int submap_index;

  bool operator==(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) ==
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  bool operator!=(const SubmapId& other) const { return !operator==(other); }

  bool operator<(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) <
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  void ToProto(proto::SubmapId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_submap_index(submap_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const SubmapId& v) {
  return os << "(" << v.trajectory_id << ", " << v.submap_index << ")";
}

/*
 * 用两个首尾迭代器定义了一个Range类型，可以提供begin()和end()方法
 */
template <typename IteratorType>
class Range {
 public:
  Range(const IteratorType& begin, const IteratorType& end)
      : begin_(begin), end_(end) {}

  IteratorType begin() const { return begin_; }
  IteratorType end() const { return end_; }

 private:
  IteratorType begin_;
  IteratorType end_;
};

// Reminiscent of std::map, but indexed by 'IdType' which can be 'NodeId' or
// 'SubmapId'.
// Note: This container will only ever contain non-empty trajectories. Trimming
// the last remaining node of a trajectory drops the trajectory.
/*
 * 类似于std::map，但是可以用NodeID或SubmapID来查找数据
 * [整个项目的印象就是封装封装再封装，调用起来一层一层一层的，总觉得吧，为了弄个高端的类出来简直不择手段]
 */
template <typename IdType, typename DataType>
class MapById {
 private:
 // 这个结构体定义在最下部的private标签下，这里只是做个声明，莫非是怕编译器找不到？
  struct MapByIndex;
  // struct MapByIndex{ bool can_append_ = true; std::map<int, DataType> data_; };

 public:
  // 下面这个是作为查询结果来用的
  struct IdDataReference {
    IdType id;
    const DataType& data;
  };
  
  /* 类：常量迭代器，是一个很好的迭代器类范本
   * 作用：整个索引结构是分层的，上层是轨迹号，下层是ID号（可以是节点ID或submapID),一个轨迹号加上一个ID号唯一索引一条数据。
   * 迭代器直接索引了数据，通过迭代器可以遍历整个数据，以轨迹号为主次序，ID号为副次序来实现遍历
   */

    /*
     *   整体结构是这样的：整体是一个哈希，一个轨迹号对应一个元素，元素内部也是一个哈希，一个ID对应一个数据 
     *   {
     *      0   :  {0:数据   1:数据    2:数据}
     *      1   :  {0:数据   1:数据    2:数据}         
     *      2   :  {0:数据   1:数据    2:数据}
     *      3   :  {0:数据   1:数据    2:数据}
     *    }
     */
  class ConstIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = IdDataReference;
    using difference_type = int64;
    using pointer = std::unique_ptr<const IdDataReference>;
    using reference = const IdDataReference&;

    /*
     * 迭代器的构造函数，需要一个MapById对象，和轨迹号。注意前者是整个外层的类，迭代器在它内部定义的
     */
    explicit ConstIterator(const MapById& map_by_id, const int trajectory_id)
        //trajectories_ 是 map<int, MapByIndex>类型的， 而MapByIndex是{ bool can_append_ = true; std::map<int, DataType> data_; }
        // 里面包括了can_append_成员，和一个data_，后者是一个[id-数据]集
        // map::lower_bound(key):返回map中第一个大于或等于key的迭代器指针[map中的key不是唯一的么]
        : current_trajectory_(
              map_by_id.trajectories_.lower_bound(trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ = current_trajectory_->second.data_.begin();
        // current_xxx 指向了一个{轨迹号:[id-数据]集结构体}，它的second就是[id-数据]集结构体，后者的data_就是[id-数据]集，它的begin就是首元素迭代器
        AdvanceToValidDataIterator();
      }
      /*
       * 首先搞清楚里面两个至关重要的成员：current_trajecotry_和end_trajectory_, 别看叫这个名字，实际上却是迭代器
       * 整个ConstIterator类都是定义在MapById类中的，而后者存放数据的本体就是trajectories_, 是一个从int到MapByIndex的哈希，后者定义是
       * { bool can_append_ = true; std::map<int, DataType> data_; }
       * 看到这里，有必要对实际意义做一个解读：trajectorries_里面存放了从轨迹号到MapByIndex的映射，就是说一个轨迹号，可以查到一个MapByIndex；
       * 后者存放了某个编号到某个数据的映射，以下称为[id-数据]集，编号可以是节点ID，也可以是Submap的ID
       * 如果完全认作map里面的key是唯一的，那么current_trajectory_就指向某个轨迹号对应的元素，end_trajectory_指向了整个map的尾后
       * 然后：如果current_xxx不等于尾后，意味着map存在这个轨迹号对应的元素，那么current_data就指向了这个轨迹号下的[id-数据]集中的第一个元素
       * 可以简单地认为：current_trajecotry_指向了某个轨迹号，current_data_指向了这个轨迹号下的某个[id-数据]元素
       */
    }

    // 给定了具体的IdType，里面包含了轨迹号和ID号，用来初始化一个迭代器
    explicit ConstIterator(const MapById& map_by_id, const IdType& id)
        : current_trajectory_(map_by_id.trajectories_.find(id.trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ =
            current_trajectory_->second.data_.find(MapById::GetIndex(id));
        if (current_data_ == current_trajectory_->second.data_.end()) {
          current_trajectory_ = end_trajectory_;
        }
      }
    }

    /*
     * 解引用，返回当前元素。
     * 返回的是IdDataReference，包含了IdType{轨迹号，ID号}和数据内容
     * 可见，当前的元素由两个迭代器指定：
     * current_trajectory_指向了轨迹号，
     * current_data_指向了ID号
     */
    IdDataReference operator*() const {
      CHECK(current_trajectory_ != end_trajectory_);
      return IdDataReference{
          IdType{current_trajectory_->first, current_data_->first},
          current_data_->second};
    }

    /*
     * 重载->符号，返回的是解引用结果的智能指针
     */
    std::unique_ptr<const IdDataReference> operator->() const {
      return absl::make_unique<const IdDataReference>(this->operator*());
    }
    
    /*
     * current_data_指向了某个轨迹号下的某个[ID-数据]元素，
     * 把current_data_移向下一个元素，如果该轨迹号下后续没有元素，则移向下一个轨迹号的首元素，同时更新current_trajectory_
     */
    ConstIterator& operator++() {
      CHECK(current_trajectory_ != end_trajectory_);
      ++current_data_;
      AdvanceToValidDataIterator();
      return *this;
    }
    /*
     * 整个过程就是++操作的逆过程：尝试把current_data_移向上一个元素，如果没有上一个元素，则移动到上一个轨迹号的数据尾
     */
    ConstIterator& operator--() {
      while (current_trajectory_ == end_trajectory_ ||
             current_data_ == current_trajectory_->second.data_.begin()) {
        --current_trajectory_;
        current_data_ = current_trajectory_->second.data_.end();
      }
      --current_data_;
      return *this;
    }
    // 比较连个迭代器是否相等：
    // 当两个迭代器其中之一的轨迹号是无效的时候，看看是否都无效；否则比较轨迹号和ID号
    bool operator==(const ConstIterator& it) const {
      if (current_trajectory_ == end_trajectory_ ||
          it.current_trajectory_ == it.end_trajectory_) {
        return current_trajectory_ == it.current_trajectory_;
      }
      return current_trajectory_ == it.current_trajectory_ &&
             current_data_ == it.current_data_;
    }

    bool operator!=(const ConstIterator& it) const { return !operator==(it); }

   private:
   /*
    * 更新至可用的数据迭代器
    * current_data_应该指向了某个轨迹号下的，某个[id-数据]元素，如果它指向了尾后元素，代表这个轨迹号下的[id-数据]集内容是空的，
    * 此时把current_trajectory_移向下一个轨迹号，尝试把current_data_指向这个轨迹号下的[id-数据]集的首元素
    * 从整个过程来看，轨迹号是最高层，id是底层，整个数据的遍历过程，是按这两个层次进行的有序遍历
    */
    void AdvanceToValidDataIterator() {
      CHECK(current_trajectory_ != end_trajectory_);
      while (current_data_ == current_trajectory_->second.data_.end()) {
        ++current_trajectory_;
        if (current_trajectory_ == end_trajectory_) {
          return;
        }
        current_data_ = current_trajectory_->second.data_.begin();
      }
    }
    
    // 指向了当前轨迹号对应的{轨迹号-[id-数据]}元素，简单认为，它记录了轨迹号
    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
    // 指向整个map的尾后迭代器，赋值后是不做更改的
    typename std::map<int, MapByIndex>::const_iterator end_trajectory_;
    // 指向了某个轨迹号对应的[id-数据]元素，简单认为，它记录了ID号
    typename std::map<int, DataType>::const_iterator current_data_;
  };
  
  /*
   * 下面这个东西，仅仅迭代轨迹号
   */
  class ConstTrajectoryIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = int;
    using difference_type = int64;
    using pointer = const int*;
    using reference = const int&;
    
    /*
     * 根据一个指向{int-MapByIndex}的迭代器来构造，直接初始化成员current_trajectory_即可
     * 实际意义上，简单理解为指向某个轨迹号
     */
    explicit ConstTrajectoryIterator(
        typename std::map<int, MapByIndex>::const_iterator current_trajectory)
        : current_trajectory_(current_trajectory) {}

    // 返回当前轨迹号
    int operator*() const { return current_trajectory_->first; }
    
    // 指向下一个轨迹号
    ConstTrajectoryIterator& operator++() {
      ++current_trajectory_;
      return *this;
    }

    // 指向下一个轨迹号
    ConstTrajectoryIterator& operator--() {
      --current_trajectory_;
      return *this;
    }

    // 判断等于
    bool operator==(const ConstTrajectoryIterator& it) const {
      return current_trajectory_ == it.current_trajectory_;
    }
    //判断不等
    bool operator!=(const ConstTrajectoryIterator& it) const {
      return !operator==(it);
    }

   private:
    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
  };

  // Appends data to a 'trajectory_id', creating trajectories as needed.
  // 给一个轨迹号的[id-数据]集追加元素
  IdType Append(const int trajectory_id, const DataType& data) {
    CHECK_GE(trajectory_id, 0);
    // 根据轨迹号，找到对应的[id-数据集]结构体
    auto& trajectory = trajectories_[trajectory_id];
    // 检测是否允许追加
    CHECK(trajectory.can_append_);
    // 找到下一个可用的ID号，然后追加
    const int index =
        trajectory.data_.empty() ? 0 : trajectory.data_.rbegin()->first + 1;
    trajectory.data_.emplace(index, data);
    return IdType{trajectory_id, index};
  }

  // Returns an iterator to the element at 'id' or the end iterator if it does
  // not exist.
  // 找到某个IdType（包含了轨迹号和ID号)对应的迭代器
  ConstIterator find(const IdType& id) const {
    return ConstIterator(*this, id);
  }

  // Inserts data (which must not exist already) into a trajectory.
  // 整体和Append很像，但是会设置"不可追加"
  void Insert(const IdType& id, const DataType& data) {
    CHECK_GE(id.trajectory_id, 0);
    CHECK_GE(GetIndex(id), 0);
    // trajectories_是一个{轨迹号，[id-数据]集}的map, map可以用[]索引么
    auto& trajectory = trajectories_[id.trajectory_id];
    // 插入数据后就把一个[id-数据]集设置为不可追加
    trajectory.can_append_ = false;
    // 对这个[id-数据]集追加元素
    CHECK(trajectory.data_.emplace(GetIndex(id), data).second);
  }

  // Removes the data for 'id' which must exist.
  void Trim(const IdType& id) {
    // 找到轨迹号对应的[id-数据]集结构体
    auto& trajectory = trajectories_.at(id.trajectory_id);
    // 找到ID号对应的迭代器
    const auto it = trajectory.data_.find(GetIndex(id));
    CHECK(it != trajectory.data_.end()) << id;
    // 如果它是队列中的最后一个元素，则设置为不可追加，因为追加后就无法保证序号连续
    if (std::next(it) == trajectory.data_.end()) {
      // We are removing the data with the highest index from this trajectory.
      // We assume that we will never append to it anymore. If we did, we would
      // have to make sure that gaps in indices are properly chosen to maintain
      // correct connectivity.
      trajectory.can_append_ = false;
    }
    // 把[id-数据]元素删除
    trajectory.data_.erase(it);
    // 如果该轨迹号下面为空，则删除它
    if (trajectory.data_.empty()) {
      trajectories_.erase(id.trajectory_id);
    }
  }
  
  // 可以看出，Insert和Trim都会设置为不可追加，不可追加在Append方法中被检测

  // 是否包含某个元素
  bool Contains(const IdType& id) const {
    return trajectories_.count(id.trajectory_id) != 0 &&
           trajectories_.at(id.trajectory_id).data_.count(GetIndex(id)) != 0;
  }
  
  // const索引某个位置的元素
  const DataType& at(const IdType& id) const {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }
  
  // 索引某个位置的元素
  DataType& at(const IdType& id) {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }

  // Support querying by trajectory.
  // 索引某个轨迹中最开头的元素
  ConstIterator BeginOfTrajectory(const int trajectory_id) const {
    return ConstIterator(*this, trajectory_id);
  }
  // 其实是索引下一个轨迹号中开头的元素，对于ConstIterator来说，尾部元素的下一个元素，就是下一个轨迹号起始元素
  ConstIterator EndOfTrajectory(const int trajectory_id) const {
    return BeginOfTrajectory(trajectory_id + 1);
  }

  // Returns 0 if 'trajectory_id' does not exist.
  // 某个轨迹号下所有元素的数目
  size_t SizeOfTrajectoryOrZero(const int trajectory_id) const {
    return trajectories_.count(trajectory_id)
               ? trajectories_.at(trajectory_id).data_.size()
               : 0;
  }

  // Returns count of all elements.
  // 所有元素的数目
  size_t size() const {
    size_t size = 0;
    for (const auto& item : trajectories_) {
      size += item.second.data_.size();
    }
    return size;
  }

  // Returns Range object for range-based loops over the nodes of a trajectory.
  // 某个轨迹号的Range
  Range<ConstIterator> trajectory(const int trajectory_id) const {
    return Range<ConstIterator>(BeginOfTrajectory(trajectory_id),
                                EndOfTrajectory(trajectory_id));
  }

  // Returns Range object for range-based loops over the trajectory IDs.
  // 所有轨迹号（构成的map)的Range
  Range<ConstTrajectoryIterator> trajectory_ids() const {
    return Range<ConstTrajectoryIterator>(
        ConstTrajectoryIterator(trajectories_.begin()),
        ConstTrajectoryIterator(trajectories_.end()));
  }
  // 所有数据的begin和end
  ConstIterator begin() const { return BeginOfTrajectory(0); }
  ConstIterator end() const {
    return BeginOfTrajectory(std::numeric_limits<int>::max());
  }
  
  // 是否为空
  bool empty() const { return begin() == end(); }

  // Returns an iterator to the first element in the container belonging to
  // trajectory 'trajectory_id' whose time is not considered to go before
  // 'time', or EndOfTrajectory(trajectory_id) if all keys are considered to go
  // before 'time'.
  ConstIterator lower_bound(const int trajectory_id,
                            const common::Time time) const {
    if (SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      return EndOfTrajectory(trajectory_id);
    }

    const std::map<int, DataType>& trajectory =
        trajectories_.at(trajectory_id).data_;
    if (internal::GetTime(std::prev(trajectory.end())->second) < time) {
      return EndOfTrajectory(trajectory_id);
    }
    auto left = trajectory.begin();
    auto right = std::prev(trajectory.end());
    // 二分法去查找
    while (left != right) {
      const int middle = left->first + (right->first - left->first) / 2;
      const auto lower_bound_middle = trajectory.lower_bound(middle);
      if (internal::GetTime(lower_bound_middle->second) < time) {
        left = std::next(lower_bound_middle);
      } else {
        right = lower_bound_middle;
      }
    }

    return ConstIterator(*this, IdType{trajectory_id, left->first});
  }

 private:
  struct MapByIndex {
    bool can_append_ = true;
    std::map<int, DataType> data_;
  };

  // GetIndex: 从NodeId或SubmapId中剥离出ID号
  static int GetIndex(const NodeId& id) { return id.node_index; }
  static int GetIndex(const SubmapId& id) { return id.submap_index; }

  std::map<int, MapByIndex> trajectories_;

  /*
   * 思考：如果单纯地迭代数据，直接用一个{trajectory ID, other ID}结构体作为键值，数据为内容，放入容器中不就可以了么
   * 只要定义好<操作，容器里面的条目都是按轨迹号为主次序，ID为副次序拍好的，不就可以依次迭代了么
   * 用两层map的唯一好处就是方便分层次访问，应该仅此而已了
   */
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ID_H_
