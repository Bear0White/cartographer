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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

/*
 * IMU追踪器，说明白点，就是实现了IMU数据的积分操作，从而跟踪IMU的位姿
 * IMU原始数据输入包括了线性加速度和角速度
 * [下面注释没有看懂，roll/pitch没有漂移，yaw有漂移？莫非指的是机器人在平面运动的情况]
 * 实际上维护了两个数据：重力方向向量，和空间角
 * 构造函数：根据重力常数和起始时间点去构造
 * 方法：
 * - Advance：根据内部记录的角速度值去更新重力方向向量和空间角
 * - AddImuLinearAccelerationObservation： 根据线加速度观测值去更新重力方向向量和空间角
 * - AddImuAngularVelocityObservation：根据角速度观测值去更新内部记录的角速度值
 * 思考：看完之后很困惑，难道Advance不该和第三个方法放一起吗？实际上在PoseExtrapolator类中调用它们的时候，通常都按照
 *   一定的顺序执行
 * 注意：本模块直接对IMU实现了积分，其中包括了很多物理公式的实现，由于时间原因不做研究
 */

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  // 把状态更新到指定时刻
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  // 添加线性加速度观测值
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  // 添加角速度观测值
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  // 返回当前时间
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  // 返回当前空间角
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  common::Time time_;
  common::Time last_linear_acceleration_time_;
  // 记录当前的空间角
  Eigen::Quaterniond orientation_;
  // 记录当前的重力方向向量
  Eigen::Vector3d gravity_vector_;
  // 当前的角速度
  Eigen::Vector3d imu_angular_velocity_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
