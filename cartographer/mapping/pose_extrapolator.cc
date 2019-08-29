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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
 * 关于IMU追踪者：
 * 里面用到了IMU追踪者，其中只有三个方法：
 * - AddImuLinearAccelerationObservation 添加某个线性加速度观测值，更新内部数据
 * - AddImuAngularVelocityObservation 添加某个角速度观测值，更新内部数据
 * - Advance 利用内部存储的数据，推算状态到某个时间点
 * 详细内容请查看imu_tracker模块
 * 这里大致说一下：两个添加观测值的方法，通常是连在一起用的，目的是更新此刻的内部状态，
 *   而Advance是推算状态到某个时间点
 * 这里再说一下imu追踪器三个方法的使用顺序：
 * - 如果单纯地向推算IMU在某个时刻的状态，则用Advance；
 * - 如果想向它添加观测数据，则尽量推算它的状态到观测点时刻，再添加观测数据，来完成内部状态的更新
 * 如果还不能理解的话，可以认为观测值是控制量的反映，在控制量的作用下状态发生突变；否则状态会按之前的趋势推进下去
 */

/*
 * 位姿队列是最核心的数据体，大部分操作都是从位姿队列展开。位姿队列存储了每次迭代后计算出的位姿结果，同时会清理其中的过期数据。
 * 新的位姿是在上次迭代产生的位姿(即位姿队列尾元素)基础上，结合从那时到此时的"IMU或里程计算出的位姿差量"，来计算新的位姿
 * IMU追踪器里面维护了IMU的状态，核心状态就是姿态角。
 * 但姿态角并不直接用来反映车体姿态，而是用两个时刻的姿态角的差值去反映车体在两时刻间的位姿差量
 * 里程计没有追踪器去维护它的位姿状态，仅仅是从里程计数据中直接推算角速度和线速度而已，以此推算某不同时刻间的车体位姿差量
 * 回过头来，看carto官方文档，它说imu坐标系和“tracking_frame"应该出在同一个位置，尽管可以有旋转变换。因为就是用imu的姿态来表征后者的姿态（变化量）的
 */

// 构造函数：仅仅完成成员的初始化而已
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

/*
 * 隐式构造函数：比构造函数多了一个ImuData，这个是作为一个最初的传感器输入数值来用的
 * 内部会调用构造函数，然后：
 * 1. 添加一帧IMU数据
 * 2. 构造它的imu_tracker_成员，并向它添加“IMU线性加速度观测值”和“IMU角速度观测值”，并更新它
 * [注意：在ImuTracker类中笔者做了说明，它的状态更新过程通常都按序调用这三种方法]
 * 3. 添加Pose
 */
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  // 创建一个推算器
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  // 添加imu数据，仅仅是给imu_data_成员追加数据然后清理过期数据而已
  extrapolator->AddImuData(imu_data);
  // 在imu_data.time这个时间点上创建imu追踪者对象，然后添加这个时间点上的观测数据，去更新它内部状态
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  //[imu_tracker_在构造时，时间点就已经是imu_data.time了，那么Advance不会有实质动作]
  extrapolator->imu_tracker_->Advance(imu_data.time);
  // 用此刻IMU追踪者推算出的方向角作为pose，去添加到位姿队列
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 返回位姿队列的最新时间戳，若无则返回极小值
common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

// 返回推算跟踪器的时间戳，若无则返回极小值
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

/* 
 * 唯一对位姿队列添加数据的操作
 * 添加某个时刻的pose到位姿队列，并从位姿队列更新速度值
 * 内部会：
 * - 构造IMU追踪者(如果之前没有)，
 * - 用imu_data_更新imu追踪器，
 * - 清理过期的IMU和里程计数据
 * - 更新里程计追踪者和推进器追踪者，这两个东西是用来保存IMU的临时状态的
 */
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  // 如果imu追踪者是空的，那么以参数time与imu_data_中最小的时间，去构造imu追踪者
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  // 把pose添加到位姿队列
  timed_pose_queue_.push_back(TimedPose{time, pose});
  // 清理位姿队列过期数据：最后一条时间是time，只保留一条与time差距在pose_queue_duration_以内的数据,更早的数据都清理掉
  // 至少保留两条数据（计算速度至少需要两条)
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  // 从位姿队列中计算线速度和角速度，更新成员
  UpdateVelocitiesFromPoses();
  // 用imu_data_去更新imu追踪器的状态
  AdvanceImuTracker(time, imu_tracker_.get());
  // 清理过期的IMU和里程计数据
  TrimImuData();
  TrimOdometryData();
  // 构建里程计追踪者和推进器追踪者，可见它们只是在AddPose时对imu_tracker_的一个临时拷贝而已，其他函数没有对它们的写操作
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

// 在imu_data_队列中添加数据，然后删除(相对于位姿队列)过期的数据,仅此而已
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

// 添加里程计数据
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  // 添加里程计数据到相关队列，然后清理(相对于位姿队列)过期的数据
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  // 如果数据队列中不足两个，则立即返回
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // 以下从里程计数据队列中计算角速度和线速度，更新成员angular_velocity_from_odometry_和linear_velocity_from_odometry_
  
  // 计算角速度：取最新与最旧的里程计数据做差，求旋转量差值，除以时间差
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;

  // 计算线速度
  if (timed_pose_queue_.empty()) {
    return;
  }
  // 坦白来说，下面这个东西没看太懂。
  // 第一个量是在最新里程计数据的时刻点，里程计所在坐标系的线速度
  // 第二个量是在最新里程计数据的时刻点，里程计所在坐标系的方向角
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
      // odometry_imu_tracker_是在添加位姿队列的函数中更新的，它的时刻和位姿队列最后一个元素的时间是一样的，
      // 所以Extra...函数返回的是位姿队列末尾时刻到odometry_data_newest.time时刻，IMU的姿态变换差量，由于IMU固定在车体上，所以也是车体的姿态差量
      // 位姿队列末尾元素乘以这个差量，就是odometry_data_newest.time时刻车体的姿态.
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

// 推算某个时刻的位姿：如果有缓存，直接从缓存中取；否则，取出位姿队列中最后一条数据，在它的基础上
// 用ExtrapolateTranslation ExtrapolateRotation 计算出平移和旋转增量叠加得到
// 这是ExtrapolateTrans...函数唯一调用的地方
// 注意用到了extrapolation_imu_tracker_，用以计算旋转增量
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    //下面的Extra...函数，计算从位姿队列最后一个时间戳到指定时刻，按系统线速度应该运动的平移量
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    // 下面的Extra...函数，计算IMU推算到指定时刻的位姿，与extr...的位姿的旋转差值。
    // 注意每次往位姿队列添加元素时都会更新extra..,所以里面保存的状态，恰好对应位姿队列中最后一个元素添加时，IMU的状态
    // 所以可以解释为：Extra...函数返回的是两个IMU状态的旋转量差值，从extra...的状态，与time时刻的IMU状态的差值。
    // 而前者存储了位姿队列末尾元素的时刻点的IMU状态，就说说Extra函数返回的是IMU从位姿队列末尾时刻到time时刻的旋转变换
    // 假设从位姿队列末尾时刻到time时刻，IMU旋转了某个角度；那么time时刻的位姿，就是那个时刻的位姿应用这个角度变换即可
    // [ 为什么这个相乘的顺序那么别扭呢？不应该是后面乘前面吗？]
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

// 推算重力方向：把imu追踪者更新到指定时间点，然后取出它内部的方向角[而不是重力向量?]
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

/*
 * 从位姿队列中更新线速度值linear_velocity_from_poses_和角速度值angular_velocity_from_poses_
 * 步骤：从位姿队列中取出最新和最旧的位姿数据，通过位姿差值和时间差值做除法来计算线速度和角速度
 * [AddPose方法会清理太旧的数据，所以求解的值可以代表当前一小段时间内的线速度和角速度]
 */
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  // 位姿队列数据不足，无法计算，则直接退出
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  // 最新的位姿和时间戳
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  // 最旧的位姿和时间戳
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  
  // 以下用线性的方法计算线速度和角速度，用以更新相关的成员
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  // 如果时间差过小，则估计不准，此时警告并退出函数。此处的pose_queue_duration_就是这个时间差值
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  // 平移差值，就是平移向量的差值
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
  // 旋转差值，计算稍微复杂些：_G^A R = G->A, _G^B R = G->B, _A^B R = _A^G R * _G^B R 
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

// 去除已经过期的IMU数据，过期是相对于位姿队列来说的，后者存储了已经推算好的位姿结果，
// 如果某个时刻的位姿已经推算好了，那么这个时刻以前的数据仅仅保留一条就够用了
// 至少会保留一条IMU数据
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    // 注意：当第二个IMU数据小于等于位姿队列最后的时间戳时，会pop，
    // 那么最终状态就是第一个数据在时间戳之前，第二个数据在时间戳之后.
    // 甚至所有数据都在时间戳之后，但猜测这很难发生，毕竟位姿队列是根据IMU等数据推算出来的
    imu_data_.pop_front();
  }
}

// 去除已经过期的里程计数据，过期是相对于位姿队列来说的，后者存储了已经推算好的位姿结果，
// 如果某个时刻的位姿已经推算好了，那么这个时刻以前的数据仅仅保留一条就够用了
// 至少会保留2条里程计数据，因为计算速度至少需要2条数据
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    // 和TrimImuData方法类似，最终状态就是第一个数据在时间戳之前，第二个数据在时间戳之后
    odometry_data_.pop_front();
  }
}

/*
 * 更新IMU追踪者：利用所有的imu数据(即imu_data_)把某个IMU追踪者更新到给定时间点
 * 最核心的方法之一
 * 具体步骤为：从IMU追踪者的状态时间戳开始，直到指定的时间点之间的所有IMU数据依次输入到IMU追踪者内完成更新
 */
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  // 如果发现没有可用的IMU数据：用默认的方法去更新IMU追踪者
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    // 我们希望把状态更新到time，但是发现没有imu数据可用，或者imu数据全都在time之后，那么就无法用imu数据去计算time的状态
    imu_tracker->Advance(time);
    // 用一个fake的重力方向更新线加速度。这个fake的重力方向就是[0,0,1]。理想情况。
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    // 注意，如果有里程计数据可用，则用里程计计算得到的角速度，否则才用根据位姿队列计算得到的角速度
    // 总结：如果没有IMU数据可用，则伪造线加速度和角速度，作为本时刻的IMU观测值，输入到IMU追踪者中进行更新
    // 因为时间点一致，所以最后没必要调用Advance
    return;
  }
  /*
   * 以下是一个简单的示例：
   * time line ----------------------------------------------------------------
   *             ^imu_tracker   ^imu_data1  ^imu_data2  ^imu_data3   ^time
   */
  // 对于所有数据都高于IMU追踪者时刻的情况：
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  // 老实说，我有些怀疑以上的必要性，因为即使没有上述操作，下面的操作照样也会实现用第一条IMU数据进行Advance操作

  // 找到第一个大于等于IMU追踪者时刻的IMU数据
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  // 从大于等于IMU追踪者时刻的IMU数据开始，到最后一条小于目标时刻的IMU数据，依次输入到IMU追踪者中实现它的更新
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  // 这里的Advance更新状态到指定时间点，完成整个操作   
  imu_tracker->Advance(time);
  /*
   * 顺便补充以下lower_bound的最常见的用法：需要algorithm头文件，可以有四个参数：迭代器起始位置，终止位置，某个参数值X，一个比较函数。
   * 函数的参数通常有两个：迭代器源类型的const引用，和参数值X的const引用类型。这两个形参分别由迭代器和参数值X传入。
   * 返回使得整个函数返回false的第一个迭代器位置。[注意是函数不成立的第一个位置]
   * Advance可以认为是根据IMU追踪者之前记录的角速度去更新整个状态
   */
}

// ---- 获得系统与期望时刻的旋转与平移差值。从代码可见，求旋转时用到了IMU数据，求平移时优先用到了里程计数据  ---//
//-----------------------------------------------------------------------------------------------------//

// 返回成员IMU更新到指定时间点时的位姿，与指定的参数IMU追踪者的位姿，两者的旋转差值。使用到了成员IMU
// 一个imu_tracker里面维护了一个IMU的状态量，所以最终返回的是两个IMU状态的差值
// 而在调用的时候，参数imu_tracker_ 要么是extrapolation_imu_tracker_，要么是odometry_tracker_, 这两个东西其实是记录了本体IMU在某个时刻的状态
// 所以求解的其实是本体IMU在不同时刻的位姿差值而已，因为IMU是固定在车体上的，所以也反映了车体在不同时刻的位姿差值，这最好在具体场合中去理解
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  // 利用IMU数据队列，更新参数中的IMU追踪器，到指定的时刻点
  AdvanceImuTracker(time, imu_tracker);
  // 返回成员IMU追踪器的旋转量的逆，乘以参数IMU追踪器的旋转量
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

// 以位姿队列中最新的数据时间戳为基础，以当前的线速度，从这个时间戳到指定的时刻，平移的量
// 计算过程就是时间差乘以线速度，优先使用里程计得到的线速度
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
