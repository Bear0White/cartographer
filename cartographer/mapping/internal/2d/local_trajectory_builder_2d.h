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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/2d/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

/*
 * 几乎可以认为是前端的最核心模块
 * 对调用的mapping下的模块做一个简要整理：
 * 2d/submap_2d
 * internal/2d/scan_matching/ceres_scan_matcher_2d
 * internal/2d/scan_matching/real_time_correlative_scan_matcher_2d
 * internal/motion_filter
 * internal/range_data_collator
 * pose_extrapolator
 */

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.

/*
 * 局部轨迹构建者，是整个前端的最高层调用者
 */
class LocalTrajectoryBuilder2D {
 public:
  /*
   * 插入结果，就是把一帧扫描数据插入到子地图当中。
   * 与TrajectoryBuilderInterface中定义的InsertionResult相比，这里缺了一个NodeId.
   */
  struct InsertionResult {
    //Data, 节点上记录的数据，包括时间，重力方向角，经过滤波和重力校正的点云数据，在局部SLAM中的位姿【这个局部SLAM指的是什么？】
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // 一个子地图(指针)数组，记录了一帧数据插入到了哪些子地图中
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  };
  
  /*
   * 匹配结果，包括了时间点，本地pose[?], 扫描数据，一个插入结果的指针
   */
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter. 如果被motion filter过滤掉了则返回空
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  /*
   * 构造函数，从proto流里面获取配置参数，并需要一个传感器名称数组，表示期望接受的传感器数据
   * 强调一下，sensor_id这里的ID不是index，而是传感器的名字
   * 传感器名称数组仅仅用来初始化range_data_collator_成员
   */
  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  /*
   * 添加一帧扫描数据，需要参数：
   * 传感器名称，和TimedPointCloudData类型的扫描数据
   * 根据注释所说，这里的RangeData应该值的是旋转扫描的传感器。
   * TimedPointCloudData::time是最后一个激光点被捕获的时间，TimedPointCloudData::ranges对象包含了相对于这个时刻的相对时间
   * 预计内部会完成扫描匹配和插入操作，最后返回一个MatchingResult类型的结果
   */
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  
  //添加一帧IMU数据
  void AddImuData(const sensor::ImuData& imu_data);
  
  // 添加一帧里程计数据
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  
  // 下面这个东西应该是作为评价指标来用的，暂且不论
  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,
      const absl::optional<common::Duration>& sensor_duration);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  // 配置参数
  const proto::LocalTrajectoryBuilderOptions2D options_;
  // 已激活的子地图，内部包含两个子地图，可以完成插入扫描帧的操作
  ActiveSubmaps2D active_submaps_;

  // 运动过滤器，在较短时间内发生的位姿变化较小的数据会被过滤掉
  MotionFilter motion_filter_;
  
  // 实时对应扫描匹配器：用滑动窗口实现粗匹配
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  // Ceres扫描匹配器：用Ceres实现精准匹配
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  //位姿推算器
  std::unique_ptr<PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  // 该轨迹上的累积数据[?]
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  RangeDataCollator range_data_collator_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
