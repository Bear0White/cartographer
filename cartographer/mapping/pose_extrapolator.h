/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
/*
 * 估计一段时间内的线速度和角速度，以追踪位姿。会使用IMU和里程计数据（如果可用）
 * 也就是说，是IMU和里程计的一个积分器而已，对各个时刻的数据进行积分，评估当前的位姿
 */
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;

  void AddPose(common::Time time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  // 从poses里面去估计速度
  void UpdateVelocitiesFromPoses();
  // 过滤过期的Imu数据
  void TrimImuData();
  // 过滤过期的里程计数据
  void TrimOdometryData();
  // 更新IMU追踪者
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;

  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  // 有效时限：用于位姿队列，每次向位姿队列中添加数据时，都会把超出有效时限的过期数据仅保留一个，其他去掉。
  // 而在清理过期的IMU和里程计数据时，也是以位姿队列的起始元素时间戳为依据的
  const common::Duration pose_queue_duration_;

  // 带时间戳的位姿
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  // 存储已经推算好的位姿结果, 以下简称位姿队列。
  // 非常重要，当前时刻的位姿，一定是从以往的位姿基础上，加上从传感器得到的位姿增量，去计算得到的
  std::deque<TimedPose> timed_pose_queue_;
  // 从队列中推算出的当前的线速度和角速度
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  // 应该是重力常数吧
  const double gravity_time_constant_;
  // IMU数据的队列
  std::deque<sensor::ImuData> imu_data_;

  // 以下分别是IMU追踪者，里程计追踪者，推算者。前者才是整个操作的本体，后两者只是前者的一个临时记录器而已
  // IMU追踪者，实现对IMU状态的追踪，本质是一个积分器
  std::unique_ptr<ImuTracker> imu_tracker_;
  // 这个东西只有两个地方有用
  // AddPose()中用imu_tracker_去给它赋值
  // AddOdometryData() 用它的旋转量去计算旋转量增量
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  // 这个东西仅仅在三个地方有用：
  // GetLastExtrapolatedTime() 返回它的时间戳
  // AddPose()中用imu_tracker_去给它赋值
  // ExtrapolatePose() 用它的旋转量去计算旋转量增量
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  
  // 缓存最近一次推算的位姿结果，便于多次查询
  TimedPose cached_extrapolated_pose_;

  // 用来存储里程计数据的队列
  std::deque<sensor::OdometryData> odometry_data_;
  // 从里程计队列中推算出的线速度和角速度
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
