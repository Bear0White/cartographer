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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

// 下面这些东西估计都是做评价用的，暂且不论
static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// 局部轨迹建立者的构造函数，仅仅是成员的初始化而已
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/*
 * 把扫描数据转换到重力矫正坐标系下并滤波：
 * 参数：重力矫正坐标系的位姿变换，点云数据
 * 操作：直接把点云应用上重力校正坐标系的位姿变换，截取指定z值范围内的数据，然后用sensor::VoxelFilter进行滤波
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.returns),
      sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.misses)};
}

/*
 * 扫描匹配，最核心的方法之一
 * 具体来说，是给active_submaps_中的前图(旧图)做的匹配
 * 但是没关系，最后计算得到的位姿是相对于整个全局物理坐标系的，而不仅仅是相对某个子地图的
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  // 如果没有子地图可用
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 待匹配的图是旧图(那新图怎么办？不用担心，因为匹配的结果是全局的，而不仅仅是相对于旧图的)
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  
  // 以下就是做匹配的全过程，先用滑动窗口做匹配(如果配置了相关参数)，再用Ceres做匹配
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  
  // 如果配置参数中有相关配置，则用实时相关匹配器做匹配，作为ceres匹配器的初始值
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  // 用ceres匹配器做精准匹配
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  
  // 以下猜测是做评估
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  return pose_observation;
}

/*
 * 添加激光数据，最长最难看的一个函数
 * 主要用于多个激光雷达的情况，而且雷达是旋转式雷达，各个激光点都有独立的时间戳。（如果是瞬时雷达，则所有激光点时间戳都是一致的）
 * 多个雷达就会出现A雷达和B雷达的某些激光点在时间上是重叠的情况。这个函数就是处理时间重叠，把某个时间段内所有雷达的激光点进行统一规整，然后
 * 调用AddAccumulatedRangeData添加到子地图中进行扫描匹配和插入操作。
 * 在本类中，这是唯一调用AddAccumu...方法的公开入口。
 * 具体来说，调用range_data_collator_.AddRangeData就可以返回一定时段内所有雷达规整后的数据了(简称规整数据)，
 * 但是这个时间段不能由外部指定，于是在成员accumulated_range_data_上压入规整数据，当数据到了一定数量后，才添加到子地图中进行扫描匹配和插入操作
 * 
 * 所以这个函数的作用可以概括为：对于多雷达的情况，规整一段时间内的雷达数据至一定数量，然后交给AddAccum...函数进行扫描匹配和插入操作
 * 内部也顺便进行了位姿计算，基于范围的激光数据滤波等操作
 * 
 * 首先强调一下参数，带时间的点云Data，实际是一个激光扫描帧，内部包含了一个基准时间，一个原点，各个扫描点的空间位置和相对时间(相对于基准时间)
 * 通常扫描帧的基准时间，是整个帧内部所有激光时间的最晚时间。每个激光点时间戳，由基准时间+相对时间得到
 * 这种数据类型猜测用来描述旋转式雷达用的
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // unsynchronized就是非同步的意思，而collator是整理器的意思，就是把原来非同步的数据进行整理
  // 这个整理器可以记录多个激光的数据，每次添加数据后，都可以把一定区段内所有传感器的激光点数据进行封装并返回
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }
  
  // 下面的这个时间点，是所有synchronized_data(简称规整数据)里面数据时间戳的最晚时间
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    // 初始化推算器：创建extrapolator_对象(如果没有)，并添加一个初始时刻的初始位姿
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  // 下面是计算规整数据里面最早的激光点时间戳
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  
  // 没看懂为什么推算器的位姿队列末尾时间戳，大于规整数据首元素时间戳，就终止函数呢
  // 至少可以得到一个态度：那就是位姿推算器的基础上，去推算激光帧的位姿，这是有时序的
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  // range_data_poses用来存放，在每个激光点时间戳的时刻下，用位姿推算器推算的车体的位姿
  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  for (const auto& range : synchronized_data.ranges) {
    // 每个激光点的实际时间
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    // 如果激光点时间晚于推算器的“最后推算点”时间（这个值等同位姿队列最末时间),那么就不能用推算器来推算这个激光点时刻下的位姿
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    // 推算激光点时刻下的车体位姿，存入range_data_poses
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  //在必要时初始化accumulated_range_data_
  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 下面是对accumulated_range_data_添加激光点数据，处理过程中会对超出配置参数max_range外的激光点视作miss点(即无穷远点，而非击空点)
  // 并增加计数num_accumulated_
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // hit: 遍历的某个具体的激光点
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    // orgin_in_local: 全局坐标系下激光点中心的坐标(这么说origin是相对于车体来说的？)
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    // hit_in_local: 全局坐标系下hit点的坐标
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    
    // 以范围进行滤波，并对accumulated_range_data_执行压入操作
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  // 更新累加计数
  ++num_accumulated_;

  // 当累加计数足够大时，可以把数据推送给AddAccu...函数进行子地图扫描匹配和插入操作了，不做细节研究
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;
    num_accumulated_ = 0;
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation();
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }
  return nullptr;
}

// 调用扫描匹配和插入操作的高层方法
// 唯一调用InsertIntoSubmap方法的地方：插入一帧累积数据
// 里面从位姿推算器去获得位姿，然后用扫描匹配去精准计算位姿，然后插入激光帧到子地图(们)中去
// 上句的“位姿”是相对于全局的位姿
// 里面重力矫正什么的，好混乱的
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  // 如果激光数据本体没有有效数据
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 推算一下当前车体位姿
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 应该是结合重力方向，把位姿投射到2D平面去
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());
  // 用自适应滤波器对激光数据进行滤波
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(options_.adaptive_voxel_filter_options())
          .Filter(gravity_aligned_range_data.returns);
  // 滤波给滤没了：退出
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 扫描匹配，得到的结果是激光帧相对于active_submaps_旧图的位姿
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // 把2D位姿重新转换回来么
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 给推算器，可见推算器里面的位姿队列存储的最终位姿都是用IMU或里程计再用扫描匹配才得到的
  extrapolator_->AddPose(time, pose_estimate);

  // 把激光帧也重新转回3D空间
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // 插入激光帧
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());
  
  // 下面应该都是做评估用的
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回的最终结果
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// 实现插入扫描帧到子地图的顶层函数
// 插入之前已经用扫描匹配计算出位姿了，所以内部的核心操作也仅仅是active_submaps_.InsertRangeData
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 上面这些参数，唯一用于插入子地图的就是range_data_in_local，其他的全都原封不动返回了，真不知道谷歌卖什么药

  // 运动过滤器唯一用到的地方：如果在较短时间内发生了较小的运动，则过滤掉
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 插入一帧激光数据，引发active_submpas_更新它内部的所有子地图，可能会引发新建子地图的操作
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  // 插入后，把结果返回
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 添加IMU数据，直接调用extrapolator_的添加IMU数据方法
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  // 下面的函数，只会在没有推算器对象时才会初始化
  InitializeExtrapolator(imu_data.time);
  // 给推算器添加IMU数据
  extrapolator_->AddImuData(imu_data);
}

// 添加里程计数据：直接调用extrapolator_的添加里程计数据方法即可
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 初始化推算器：创建extrapolator_对象(如果没有)，并添加一个初始时刻的初始位姿
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant());
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

// 应该是评估用，暂且不论
void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
