/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// 以下猜测是评估用
static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

/*
 * 全局轨迹构建者：继承于轨迹构建者接口
 * 内部成员含有本地轨迹构建者和位姿图，可见里面融合了前端和后端的功能
 * 推测：前端的最高接口是轨迹构建者，后端的最高接口是位姿图，而这个类直接调用了两者，是核心的任务协调者
 * 构造：
 *    参数：本地轨迹建立者，轨迹号，位姿图，本地SLAM结果回调函数
 *    内部仅仅实现成员初始化而已
 * 方法：
 *    AddSensorData 对多种传感器数据类型实现重载，实现添加数据的操作
 *      重载的数据类型有：
 *      激光点云，IMU，里程计，固定坐标值，路标
 *    AddLocalSlamResultData 对其参数local_slam_result_data调用了它的AddToPoseGraph方法
 */
template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.
  // 构造函数：参数：本地轨迹建立者，轨迹号，位姿图，本地SLAM结果回调函数
  // 内部仅仅实现成员初始化而已
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback) {}
  ~GlobalTrajectoryBuilder() override {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;
  
  // 添加激光帧数据，里面进行了本地轨迹建立者的AddRangeData函数，和位姿图的AddNode函数，并执行了本地SLAM结果回调函数，等等。
  // 回调函数仅仅在这里执行
  void AddSensorData(
      const std::string& sensor_id,
      //点云数据类型：里面有时间，有原点，还有“带时间戳的激光点”数组
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    
    // 首先，调用本地轨迹建立者local_trajectory_builder_的AddRangeData函数，返回匹配结果matching_result
    // 注意后者会完成激光数据扫描匹配和插入操作
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);
    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    // 接下来会调用位姿图pose_graph_的AddNode方法，同时会把匹配结果matching_result的数据整理成insertion_result
    kLocalSlamMatchingResults->Increment();
    std::unique_ptr<InsertionResult> insertion_result;
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
          node_id, matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
    // 此处会调用“本地slam结果回调函数”进行处理
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local),
          std::move(insertion_result));
    }
  }
  
  // 添加IMU数据：调用了本地轨迹构建者的AddImuData和位姿图的AddImuData方法，仅此而已
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddImuData(imu_data);
    }
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }
  
  // 添加里程计数据：调用了本地轨迹构建者的AddOdometryData和位姿图的AddOdometryData方法，仅此而已
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }

  // 添加固定坐标位姿数据：调用了位姿图的AddFixedFramePoseData方法
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

  // 添加路标数据：调用了位姿图的AddLandmarkData方法
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
  }

  // 添加本地SLAM结果数据：参数local_slam_result_data，调用了它的AddToPoseGraph方法
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }

 private:
  //轨迹号
  const int trajectory_id_;
  // 核心成员：位姿图
  PoseGraph* const pose_graph_;
  // 核心成员：本地轨迹建立者：SLAM前端最高接口
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
  // 本地SLAM结果回调函数，仅在AddSensorData方法中调用
  LocalSlamResultCallback local_slam_result_callback_;
};

}  // namespace

// 构造一个全局轨迹构建者对象，返回一个智能指针。参数直接用于构造对象
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback);
}

// 3D的情况，不做研究
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback);
}

// 猜测是评估使用，不做研究
void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
  auto* results = factory->NewCounterFamily(
      "mapping_global_trajectory_builder_local_slam_results",
      "Local SLAM results");
  kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
  kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer
