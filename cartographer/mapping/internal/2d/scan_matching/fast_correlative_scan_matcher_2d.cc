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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
/*
 * 滑动窗口类，这个类仅仅用于产生预计算栅格而已，本体上维护一个队列，可以返回队列最大值
 * 本体上维护一个(非严格)单调递减队列，主要操作有压入，弹出，求最大值。
 */
class SlidingWindowMaximum {
 public:
  // 向队列尾部添加元素，如果尾部有更小的元素，则把这些元素全部弹出。
  // 从而保证了添加元素之后队列依然是单调递减的
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }
  // 给定一个值，如果队列头部元素等于这个值，则弹出队列头部元素
  // 之所以这么设计，是在产生预计算栅格时用的，非常巧妙
  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }
  // 返回队列的最大值，即头部元素
  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }
  // 检查队列是否为空？
  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  // 队列本体
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

// 配置参数的格式转换：从Lua到proto
proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

/*
 * 预计算栅格的构造函数，函数中去产生预计算栅格
 * 注意最后这个参数，显然是用来避免重复分配释放资源而设立的中间量
 * 提前预警：这个函数写得非常艰深难懂，具体参考md文件，里面有具体说明
 */
PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),
      max_score_(1.f - grid.GetMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  /*
   * 此处注意一些参数的初始化：cells_是预计算栅格(以下简称P)的本体，尺寸是wide_limits_，等于原有栅格M尺寸加上width-1
   * 偏移量offset_是常量，在GetValue方法中使用
   */
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  
  /*
   * intermediate是中间量，注意尺寸，在x方向上是wide_limits_，在y方向上是limits_
   * 以下的代码就是计算intermediate(以下简称I)，I(x,y)是M中以(x,y)作为结尾的1*width大小的滑动窗口的最大值
   * 这是通过遍历y来实现的，注意代码中关于滑动窗口压入和弹出相关的操作，非常巧妙
   */
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    
    current_values.CheckIsEmpty();
  }                                         
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  /*
   * 下面是计算最终的P，P的本体是cells_(以下简称C)
   * 如果把x和y做一个替换，你会发现下面的算法与上面的算法如出一辙。
   * 所以以下的作用：遍历x，用滑动窗口计算得到C，其中C(x,y)表示I中以(x,y)作为结尾的width*1尺寸的滑动窗口的最大值
   * 又因为I记录了每个横向滑动窗口的最大值，所以C(x,y)表示M中以(x,y)作为结尾的width*width尺寸的滑动窗口的最大值
   */
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

// 把浮点数从[min_score_, max_score]投射到[0,255]上
uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

// 本体是一个容器，用来盛放不同深度的预计算栅格。在构造函数中，就计算好了不同深度的预计算栅格，并存入容器。
PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  // 最大宽度，等于2^(配置参数中深度值)
  // 例如深度3，则最大宽度是4；深度4，最大宽度是8.
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  
  // 开始计算所有的预计算栅格，宽度的范围是1到2^(depth-1)
  // 注意了，最底层的预计算栅格宽度是1，等同原始栅格。这个类的max_depth方法，返回了数组长度减一，即配置参数深度减一。
  // 所以最高层的预计算栅格宽度是2^(配置参数深度-1)，也就是2^max_depth
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

// 构造函数，主要是参数初始化而已
FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

// 匹配操作，最顶层的接口之一。里面的操作就是参数初始化，构造SearchParameters，以及MatchWithSearchParameters
bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

// 全地图匹配，最顶层的接口之一。和Match方法唯一的区别在于它没有初始位姿参数，而且搜索范围是整个地图
// 所以制定SearchParameters的时候，单边平移搜索范围是一百万像素，单边旋转搜索范围是pi。而且初始位姿是整个地图的中心点。
// 以此实现在所有地图上进行搜索
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

/*
 * 最核心的方法，整个匹配过程的直接执行者
 */
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);
  
  // 以下方法和real_time_correlative_scan_matcher中如出一辙
  // 把点云数据应用上初始位姿估计中的旋转变换
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 产生不同旋转搜索窗口下的点云数据
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  // 对这些点云数据应用初始位姿估计的平移变换，再离散化
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 调整搜索窗口大小
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());
  
  /* 产生最低分辨率的候选人们，并计算他们的分值。
   * 一个候选人本体描述了一个搜索窗口。这里强调一下，分辨率是相对于预计算栅格来说的，分辨率最低就是宽度最大的预计算栅格，
   * 这个值是2^max_depth，其中max_depth是预计算栅格类的方法，返回的是与计算栅格数组长度减一，这个值也是配置参数中深度值减一，相关参考预计算栅格构造函数
   * 分辨率会影响到候选人（即搜索窗口）的平移搜索步长，但对旋转步长没有影响。平移搜索步长就是预计算栅格的宽度
   * 综上所述，这个过程就是：取出最顶层预计算栅格，然后设定平移步长是它的宽度，去产生搜索窗口；
   * 对原始点云数据应用搜索窗口，然后对齐在最顶层预计算栅格上，得到匹配分值，并把候选者们排序
   * 具体参考函数的实现过程
   */

  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);

  // 用分值定界去求最优方案，最最核心的运算过程
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  
  // 如果最优方案高于预先设定的得分，则方案被采纳，整理结果并传出score和pose_estimate，整个函数返回true；否则整个函数返回false
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

// 计算最低分辨率候选人的分数：先产生最低分辨率后续人(们)，然后计算它的分数，注意ScoreCandidates会排序
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

/*
 * 产生最低分辨率的候选人们
 * 注意：candidate描述了一个旋转和平移窗口，这个窗口是施加在扫描帧上。一个候选人是一个具体的搜索窗口，而不是点云数据
 * 从下文的产生候选人的过程来看，平移搜索步长变成了2^max_depth, 但是旋转搜索步长没有变化；
 * 旋转搜索窗口的数目在search_parameters里面，这个变量用配置参数初始化，以后都没有更改过。旋转搜索窗口一个不少地被遍历了
 */
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  // 平移搜索步长变成了2^max_depth.参考PrecomputationGridStack2D构造函数，这个数值刚好是最高层预计算栅格的宽度
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  
  // 以下是根据平移搜索步长，来计算总共的搜索窗口数量
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  // 用搜索窗口数量去预分配数组，提高性能
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  // 产生所有的搜索窗口
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/*
 * 计算候选人们的分值。
 * 参数：precomputaion_grid，预计算栅格，里面是以某个宽度，对原栅格进行区域最大值求解后的栅格，本质是经过加工后的栅格地图而已
 * 参数：discrete_scans，离散扫描帧的数组，从使用场合来看，里面是不同旋转窗口下的扫描帧数据
 * 参数：search_parameters，函数根本就没用到
 * 参数：candidates，后续人，里面记录了旋转窗口的ID，x和y的偏移量。
 * 注意：candidates描述了一个旋转和平移窗口，这个窗口是施加在扫描帧上，然后把扫描帧与栅格进行对齐，来计算得分
 * 输出：最后会对candidates数组按照得分进行排序。得分的计算也很简单，把激光点投射在栅格地图的位置上的占率做累加，再归一化即可
 */
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

/* 
 * 整个分支定界函数是一个递归函数
 * 首先，明确概念。分值定界，分的是什么？定的是什么？操作对象是什么
 * 分值定界的操作对象是候选人的集合，每个候选人表示了一个搜索窗口。
 * 参数: discrete_scans，不同旋转搜索窗口下的点云数据的离散化，用于计算搜索窗口的得分
 * 参数：search_parameters，搜索参数，记录了搜索边界等信息，用处较多
 * 参数：candidates，待处理的后续人（搜索窗口）的集合，分支定界操作的主体对象
 * 参数：candidate_depth，后续人深度，这个值是对应于预计算栅格来说的，后者的宽度就是2^depth
 * 参数：min_score，人为指定的得分最小值，一旦某个分值小于这个值，直接剪去
 */ 

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  // 递归的终点。注意ScoreCandidates函数会对候选人队列进行排序，又一个名不副实的函数啊
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }
  
  // 构造函数，旋转窗口ID，x偏移量，y偏移量，搜索参数
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;

  // 开始进入最重要的流程：分支
  for (const Candidate2D& candidate : candidates) {
    // 分数很小，直接剪枝【等等，这里难道不该是continue吗】
    // 【因为candidates里面是排好序的，如果当前的分数小，后面的会更小】
    if (candidate.score <= min_score) {
      break;
    }

    // 以下部分是构造higher_resolution_candidates，更高分辨率的候选人集合
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    // 下面的代码实现了分支：把原有的候选人切分成四个。
    // 新候选人相对于原候选人来说，旋转量不变，平移量变成了四种(x,y各两种，分别是原来的偏移量，和原来偏移量加一半宽度)
    // 注意以下的范围for循环，花括号里面是离散值，例如：for(int i : {0, 2}): cout<<i; 输出的是02
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    // 对分裂产生的新后续人进行评分并排序，排序结果用于递归求解下一个分支定界
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

/*
 * 整个函数做个总结：
 * 首先整个分支定界，它的操作对象是搜索窗口的集合。
 * 代码中，操作对象以参数传入，是一个已经按照分值排好序的候选人数组，每个候选人都带有旋转量，x偏移量，y偏移量，分值等成员
 * 分值定界的参数：离散化点云、搜索参数、最小分值这些都是不变的，迭代的量只有候选人集合和搜索深度
 * 分值定界函数的目标：算出给定候选人数组中最佳者
 * 首先，看看分值定界的最初次调用：最初的搜索深度(depth)是用户配置参数深度-1，2^depth就是预计算栅格(P)的宽度，宽度值也是平移搜索步长
 * 以2^depth作为平移搜索步长，(旋转搜索步长不变)，产生各个搜索窗口；与宽度为2^depth的预计算栅格进行匹配，计算出候选人的所有得分，然后对候选人数组排序
 * 以上步骤就是对最低分辨率候选人的处理过程.注意预计算栅格是在计算候选人分值的时候用到，预计算栅格的宽度与候选人的平移步长是对应的
 * 然后迭代，深度优先搜索：（注意参数candidates以及排好序并算好了分数）
 * F(candidates, depth):
 *    if depth == 0: 
 *        return candidates[0]
 *    best = min()
 *    for someone in candidates:
 *        if someone.score is too low: 
 *            break the for-loop;
 *        children = empty()
 *        branch someone to children
 *        compute score and sort children
 *        best = max(best, F(children, depth-1))
 *    return best
 */

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
