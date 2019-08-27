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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

/*
 * 此处先声明几个自定义的名词：
 * 占率：即Probability，表示栅格被占据的概率
 * 空率：即CorrespondenceCost，表示栅格没有被占据的概率
 * 占率+空率=1
 * 占空比：即Odd，占率与空率的比值
 * 函数名中的Value通常指的是整形值
 * 此部分内容严格对应论文中栅格概率更新的公式
 */

namespace cartographer {
namespace mapping {

namespace {

// 用数值计算的方法，把一个浮点数float_value，从[lower_bound, upper_bound]映射到[1, 32767]整数空间
inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound) {
 // common::Clamp(v,min,max): 把v截断在[min,max]中
 // common::RoundToInt: 四舍五入取整
  const int value =
      common::RoundToInt(
          (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

}  // namespace

// 根据占率求占空比
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

// 根据占空比求占率
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

// 根据空率求占率
inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}

// 根据占率求空率
inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

// 以下几个常量，用来指定占率和空率的上下界
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
// 把占率截断在指定上下界之内
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}
// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 把空率截断在指定上下界之内
inline float ClampCorrespondenceCost(const float correspondence_cost) {
  return common::Clamp(correspondence_cost, kMinCorrespondenceCost,
                       kMaxCorrespondenceCost);
}

// 以下常量，用来指定占率和空率的“未知值”
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
// 以下这个量，是更新标志位（对应的数值）。栅格地图中直接存储的是u16位整形空率，最高位就是更新标志位，
// 表示这个值是否（在一轮迭代中）得到更新，避免重复更新。其他的15位才真正记录数值
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
// 用数值计算的方式，求某个空率对应的整数值
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
  return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}

// Converts a probability to a uint16 in the [1, 32767] range.
// 用数值计算的方式，求某个占率对应的整数值
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

// 以下变量定义在同名的cc文件中，用来返回占率和空率从整数到浮点的查找表
extern const std::vector<float>* const kValueToProbability;
extern const std::vector<float>* const kValueToCorrespondenceCost;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
// 用查找表的方式，求某个整形占率对应的浮点值
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 用查找表的方式，求某个整形空率对应的浮点值
inline float ValueToCorrespondenceCost(const uint16 value) {
  return (*kValueToCorrespondenceCost)[value];
}

// 计算某个整形占率对应的整形空率
inline uint16 ProbabilityValueToCorrespondenceCostValue(
    uint16 probability_value) {
  // 如果是未知值，直接返回对应的未知值
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  // 是否带了更新标志位？如果带了，则把标志位取下，进行转换后，最终结果再加上
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = CorrespondenceCostToValue(
      ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

// 计算某个整形空率对应的整形占率
inline uint16 CorrespondenceCostValueToProbabilityValue(
    uint16 correspondence_cost_value) {
  // 如果是未知值，直接返回对应的未知值
  if (correspondence_cost_value == kUnknownCorrespondenceValue)
    return kUnknownProbabilityValue;
  // 是否带了更新标志位？如果带了，则把标志位取下，进行转换后，最终结果再加上
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = ProbabilityToValue(CorrespondenceCostToProbability(
      ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

// 下面这两个函数分别实现对hits点和miss点的数据更新
std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
