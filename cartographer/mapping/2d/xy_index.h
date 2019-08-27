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

#ifndef CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_
#define CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/2d/cell_limits.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
 * 网格边界，通常在MapLimits中使用，描述一个离散网格在x和y方向上的数量
 * 内部也仅仅有两个成员：num_x_cells，num_y_cells而已
 */
struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  explicit CellLimits(const proto::CellLimits& cell_limits)
      : num_x_cells(cell_limits.num_x_cells()),
        num_y_cells(cell_limits.num_y_cells()) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

// 转换到Proto
inline proto::CellLimits ToProto(const CellLimits& cell_limits) {
  proto::CellLimits result;
  result.set_num_x_cells(cell_limits.num_x_cells);
  result.set_num_y_cells(cell_limits.num_y_cells);
  return result;
}

/*
 * 一个对二维矩阵进行遍历的迭代器，遍历时以行为主次序，列为副次序
 * 构造函数：
 *  - 以Eigen::Array2i类型的最小边界和最大边界为参数构造
 *  - 以CellLimits为参数构造
 * 方法：
 *  - 重载++: 迭代下一个位置
 *  - 重载*: 返回当前位置坐标，是Eigen::Array2i格式
 *  - ==和!= :返回是否等于和不等于另一个迭代器
 *  - begin(): 返回开始位置，就是构造函数中min参数描述的位置
 *  - end(): 返回尾后位置，就是构造函数中max参数描述的位置进行++后到达的下一个位置
 */
// Iterates in row-major order through a range of xy-indices.
class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
 public:
  // Constructs a new iterator for the specified range.
  XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                       const Eigen::Array2i& max_xy_index)
      : min_xy_index_(min_xy_index),
        max_xy_index_(max_xy_index),
        xy_index_(min_xy_index) {}

  // Constructs a new iterator for everything contained in 'cell_limits'.
  // 注意以CellLimits构造时，num_x_cells是网格数目，对应到迭代器范围就是[0, num-1]
  explicit XYIndexRangeIterator(const CellLimits& cell_limits)
      : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                             Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator& operator++() {
    // This is a necessary evil. Bounds checking is very expensive and needs to
    // be avoided in production. We have unit tests that exercise this check
    // in debug mode.
    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i& operator*() { return xy_index_; }

  bool operator==(const XYIndexRangeIterator& other) const {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator& other) const {
    return !operator==(other);
  }

  XYIndexRangeIterator begin() {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }
  
  // 这里留意一下：end就是最后一个位置进行++操作后到达的位置，因为end()本来就是配合++来使用的
  XYIndexRangeIterator end() {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

 private:
  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_
