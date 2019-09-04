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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

// Collates sensor data using a sensor::CollatorInterface, then passes it on to
// a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.
// 用sensor::CollatorInterface去整理数据，然后传递给轨迹建立者接口
/*
 * 规整轨迹构建者：继承于轨迹构建者接口
 * 从代码搜索的结果来看，这个类仅仅在MapBuilder中被调用，调用时内部会产生GlobalTrajectoryBuilder对象
 */
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
 public:
  using SensorId = TrajectoryBuilderInterface::SensorId;

  // 构造函数：
  // 参数：配置参数，轨迹号，期望传感器ID集合，一个“轨迹构建者接口”指针
  // 在MapBuilder中，用一个GlobalTrajectoryBuilder指针用作了最后一个参数
  CollatedTrajectoryBuilder(
      const proto::TrajectoryBuilderOptions& trajectory_options,
      sensor::CollatorInterface* sensor_collator, int trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
  ~CollatedTrajectoryBuilder() override {}

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
      delete;

  // 以下分别实现了添加各种传感器数据的功能，分别对AddSensorData实现了重载
  // 几乎所有的方法都调用了私有方法AddData，并把数据封装成Dispatchable对象的智能指针
  // 猜测这个对象就是用来记录一帧传感器数据用的，可以提供时间戳，数据内容，和加入到轨迹构建者中的函数

  // 添加激光点云：直接调用私有方法AddData
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }

  // 添加IMU：直接调用AddData
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, imu_data));
  }

  // 添加里程计：直接调用AddData
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
  }

  // 如果“规整固定坐标”标志位被设置，则直接调用AddData；否则调用wrapped_trajectory_builder_的AddSensorData方法
  // 注意：实际在MapBuilder中调用时，wrapped...是一个GlobalTrajectoryBuilder对象
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    if (collate_fixed_frame_) {
      AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id,
                                               fixed_frame_pose_data);
  }

  // 如果“规整地标”标志位被设置，则直接调用AddData；否则调用wrapped_trajectory_builder_的AddSensorData方法
  // 注意：实际在MapBuilder中调用时，wrapped...是一个GlobalTrajectoryBuilder对象
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    if (collate_landmarks_) {
      AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id, landmark_data);
  }

  // 添加本地SLAM结果数据：直接调用AddData
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    AddData(std::move(local_slam_result_data));
  }

 private:
  // 最核心的私有成员：AddData
  void AddData(std::unique_ptr<sensor::Data> data);
  // 处理规整数据
  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  // 数据规整器
  sensor::CollatorInterface* const sensor_collator_;
  // 标志位：是否规整地标，从配置参数读取
  const bool collate_landmarks_;
  // 标志位：是否规整固定坐标，从配置参数读取
  const bool collate_fixed_frame_;
  // 轨迹号
  const int trajectory_id_;
  // 包裹轨迹构建者，在MapBuilder中调用时，实际上是一个GlobalTrajectoryBuilder对象
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
