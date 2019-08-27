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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
// 超像素常数
constexpr int kSubpixelScale = 1000;

// 提取range_data里面的原点和所有hits和miss的激光点，根据这些点的坐标对栅格进行扩充
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  // 下面这个是Eigen的box，应该是用来记录一个矩形区域的上下界的
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  // Padding(填充) around bounding box to avoid numerical issues at cell boundaries.
  // 下面这个是一个很小的填充值，防止在边界发生一些问题
  constexpr float kPadding = 1e-6f;
  // 扩展bounding_box，使得可以囊括所有hits和miss的激光点位置
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  // 根据bounding_box对栅格进行扩充，GrowLimits方法会扩充栅格直至包含指定坐标
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

/*
 * 以下方法是Insert操作的主体，实现了把一帧激光数据插入到网格中，来完成网格数据更新
 * 参数：
 * - range_data：激光帧，包括了原点坐标，hits点坐标数组(或者称returns)，miss点坐标数组
 * - hit_table：用于对hits点进行数据更新的查找表
 * - miss_table：用于对miss点进行数据更新的查找表
 * - insert_free_space：是否对miss点进行处理
 * - probability_grid: 概率栅格对象
 */
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  
  // 扩展网格，使得它可以包含激光帧里面的所有激光点坐标
  GrowAsNeeded(range_data, probability_grid);

  // 以下对原本的栅格进行细化，得到细化的栅格框架。猜测最主要的目的是计算miss栅格
  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  
  // begin点就是激光原点在细化栅格下的坐标
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  // ends数组存储了所有激光(击中障碍物的)点，也是细化栅格下的坐标
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    // 装填ends数组
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    // 以下这个函数直接实现了对hits点的更新操作：给定一个坐标值，按照查找表hit_table实现数据更新
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }
  
  // 如果insert_free_space为False，则不再插入free space，即不再处理miss点
  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  // 对miss点进行处理，miss点是原点在激光末点之间的点
  for (const Eigen::Array2i& end : ends) {
    // 下面的函数就是指定起始点和终止点，计算在这两点之间的线段所经过的栅格坐标（不含起始点和终止点的栅格坐标）。
    // 参数kSubpixelScale是细化栅格尺度，begin和end是物理坐标值在某栅格下经过细化得到的离散坐标值
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      // 直接实现了对miss点的更新操作：给定一个坐标值，按照查找表miss_table实现数据更新
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  // 下面这个乍一看不太懂，怎么还有个miss？现在想想，论文中的miss点是通过原点到hits点的连线计算出来的，
  // 而激光帧中原本就带有的misses应该是传感器返回的数据，所以推测情况是：激光打出去，一路飞出没有碰到障碍物，没有得到测距数据。
  // 猜测range_data里面的misses应该填充了一个较大的坐标值，所以从原点到这个坐标一路都是空的
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    // 更新过程同上文
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

// 配置参数：从lua到proto
proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

/* 根据配置参数去初始化
 * 重要操作：初始化hit_table_和miss_table。此处的操作要结合下面两个函数来看：
 * ComputeLookupTableToApplyCorrespondenceCostOdds 对应论文中hits点的更新过程，给定odds(p_{hit}),实现更新的查找表
 * ComputeLookupTableToApplyCorrespondenceCostOdds 对应论文中miss点的更新过程，给定odds(p_{miss}), 实现更新的查找表
 * 有了查找表之后，输入旧的网格数据，就可以查找得到新的网格数据。注意得到的数据中是包含更新标志位的
 * 这里也可以看出，配置参数中hit_probability和miss_probability对应论文中的p_hit和p_miss
 */
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

/*
 * 最核心也是唯一的方法：插入扫描帧到网格（以实现网格的更新）
 * 主体操作在CastRays函数，然后调用了概率栅格的FinishUpdate方法，后者会把本次迭代中所有已经更新过的栅格的更新标志位全都去掉
 * 具体是这样的：所有网格都有一个更新标志位，初始为零。当捕捉到一帧激光数据，则调用下面的方法进行Insert操作，以实现对网格数据的更新。
 * 被更新后的每个栅格，都会置位更新标志位；如果一个栅格已经有标志位，则不再更新它。
 * (更新栅格发生在CastRays方法中，使用查找表来更新，而查找表hit_table_和miss_table_存储的数据都是带标志位的)
 * 所以一个栅格只会被更新一次，避免多个激光点落在一个栅格，遍历激光点时使得栅格重复更新.[你搞个集合不就完了嘛！这么麻烦！]
 * 当完成所有的更新后，再调用FinishUpdate方法去掉所有的更新标志位，本次更新结束。
 * 所以说，之所以这么折腾就是避免在一次Insert操作时对同一栅格重复更新的！
 * 参数：
 * - range_data：包含了原点坐标，hits点(或者称returns)坐标数组和miss点坐标数组
 * - grid：栅格地图
 */
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
