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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

// 配置参数格式转换
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// 这个东西估计只是个简单的声明吧
class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
/*
 * 从注释上看，这东西是整个全局SLAM的管理者，包括局部slam用于位姿估计，扫描匹配去检测回环，和一个稀疏的位姿图参与优化
 */
class TrajectoryBuilderInterface {
 public:
  /*
   * 结构体：插入结果
   * 包括：
   * NodeId：由轨迹号和节点号构成
   * Data(指针)：节点上记录的数据，包括时间，重力方向角，经过滤波和重力校正的点云数据，在局部SLAM中的位姿
   * Submap(指针)数组：插入到的submap们
   */
  struct InsertionResult {
    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  // 回调函数：这个回调函数是当一个RangeData数据进来时被调用
  // 主要有5个参数：trajectory_id，该帧数据的插入时刻Time, 插入到Local Slam中的pose,RangeData数据本身和插入的结果InsertionResult。
  // 如果该帧数据成功插入了submap中则返回节点id:NodeId，否则返回空指针
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;
  
  // 结构体：SensorId。内部包含了一个枚举的传感器类型，一个string的传感器名称（id），还定义了相等和小于符号
  struct SensorId {
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;
    std::string id;

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  // 构造函数，目前是空的
  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;
  
  // 以下定义了同名的AddSensorData，分别添加不同类型的传感器数据
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::LandmarkData& landmark_data) = 0;

  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  // 根据注释，这个可以直接添加局部SLAM的结果到位姿图中，注意不适于带LocalTrajectoryBuilder的轨迹
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

// 与proto的格式转换
proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
