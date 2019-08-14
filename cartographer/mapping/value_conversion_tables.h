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

#ifndef CARTOGRAPHER_MAPPING_VALUE_CONVERSION_TABLES_H_
#define CARTOGRAPHER_MAPPING_VALUE_CONVERSION_TABLES_H_

#include <map>
#include <vector>

#include "cartographer/common/port.h"
#include "glog/logging.h"

/* 
 * 数值转换表，为了实现从uint16整数空间转换到浮点空间而做的查找表。
 * 查找表本质就是一个浮点数组，元素下标即整数值，元素内容即对应的浮点值，以此完成快速查找。
 * 每次创建新的查找表，都会在类内部进行存储，避免重复创建。
 */

namespace cartographer {
namespace mapping {

// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.
class ValueConversionTables {
 public:
  // 唯一的方法：获取一个查找表
  // 映射的方式是从16位整形到浮点，但是16位整形的最高位是作为标志位来使用的，所以映射范围是：
  // 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound]
  const std::vector<float>* GetConversionTable(float unknown_result,
                                               float lower_bound,
                                               float upper_bound);

 private:
  // 我还不知道浮点也可以作为map的键值
  std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                            float /* upper_bound */>,
           std::unique_ptr<const std::vector<float>>>
      bounds_to_lookup_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_VALUE_CONVERSION_TABLES_H_
