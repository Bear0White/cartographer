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

#include "cartographer/mapping/probability_values.h"

#include "absl/memory/memory.h"

// 声明：本文件中的probability称作占率，CorrespondenceCost称为空率，Odd称为占空比或赔率

namespace cartographer {
namespace mapping {

namespace {

/* 首先，栅格地图存储的是整形的空率值，用一个16位int来表示。
 * 然而最高位用来表示是否(在一轮迭代中)被更新过，剩余15位用来存储数据
 * 所以存储范围是0-32767
 * 下面这个kValueCount就表示15位可以存储的整数个数，即32768
 */
constexpr int kValueCount = 32768;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
/*
 * 用数值计算的方式，求某个整数value，从[1, 32767]映射到某个[lower_bound, upper_bound]的浮点空间的值。
 * 其中unknown_value会映射到unknow_result
 * 从具体场合来说，unknown_value的值往往是零，这就实现了[0,32767]到浮点空间的映射
 */
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  return value * kScale + (lower_bound - kScale);
}

/*
 * 生成一个查找表(的指针),用来实现从整形空间[1, 32767]到浮点空间[lower_bound, upper_bound]的快速查找
 * 其中unknown_value会映射到unknow_result
 * 从应用场合来说，就是实现整形占率(或空率)转换到浮点占率（或空率)的查找表。
 * 栅格内部存储的数据本体是整形空率，我们就以空率来说明
 * 注意里面有个kRepetitionCount，重复次数，值是2.
 * 结果就是查找表的长度是65536，其中[0,32767]就是最高位是零的16位整数所表示的范围，
 * 即更新标志位是零的整形空率数值范围；而[32768,65535]就是更新标志位是一的整形空率数值范围。
 * 这就实现了所有的整形空率（不论有没有更新标志位）都可以实现查找，其中有没有标志位，转换的结果都是相同的
 */
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

/*
 * 这是对PrecomputeValueToBoundedFloat函数的包装，赋予它指定的参数。
 * 最终生成一个查找表, 实现占率从整形到浮点的映射
 * 具体是:[1, 32767]到[kMinProbability,kMaxProbability]的映射，
 * 并且kUnknownProbabilityValue作为未知值映射到kMinProbability
 */
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

/*
 * 这是对PrecomputeValueToBoundedFloat函数的包装，赋予它指定的参数。
 * 最终生成一个查找表, 实现空率从整形到浮点的映射
 * 具体是：从[1, 32767]到[kMinCorrespondenceCost, kMaxCorrespondenceCost]的映射，
 * 并且kUnknownCorrespondenceValue作为未知值映射到kMaxCorrespondenceCost
 */
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

/*
 * 返回PrecomputeValueToProbability方法得到的指针，即，返回占率从整形到浮点的映射的查找表
 * [1, 32767]到[kMinProbability,kMaxProbability],kUnknownProbabilityValue到kMinProbability
 */
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

/*
 * 返回PrecomputeValueToCorrespondenceCost方法得到的指针，即，返回空率从整形到浮点的映射的查找表
 * [1, 32767]到[kMinCorrespondenceCost, kMaxCorrespondenceCost],kUnknownCorrespondenceValue到kMaxCorrespondenceCost
 */
const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

/*
 * 生成一个查找表，用来实现栅格地图中的更新
 * 论文中，如果某个点是hits状态，则更新：
 * M_{new}(x) = clamp(odds^{-1}(odds(M_{old}(x)) \dot odds(p_{hit})))
 * 其中p_{hit}是一个常量。
 * 对比一下就可以看出，
 * - 函数参数odds对应公式中的odds(p_{hit})，
 * - 函数ProbabilityFromOdds对应公式odds^{-1},
 * - 变量(*kValueToProbability)[cell]))对应M_{old}(x)
 * 可见，函数实现的功能是：指定odds(p_{hit})生成查找表,查找时，给出一个整形占率，返回它更新后的结果
 * 实际上，在RangeDataInserter2D中的构造函数，就初始化hit_table_成员为：
 * ComputeLookupTableToApplyOdds(Odds(options_.hit_probability())))
 * 这个hit_table_直接用于更新过程
 * 最后强调一点：最后的数值都是带更新标志位的，就是在这一步骤加上的，怎么看怎么别扭
 */
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  // 下面是对result[0]的处理，它是对输入值是零的查找结果，而零对应kUnknownProbabilityValue（定义在头文件中），
  // 即未知的占率，就是说第一次观测到这个网格，那么根据公式应该直接给它p_{hits}
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

/*
 * 生成一个查找表，用来实现栅格地图的更新
 * 同上文的ComputeLookupTableToApplyOdds函数类似，都是对论文中更新公式的实现
 * 不同的是，这个是对miss点的处理过程，不做仔细研究
 */
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
