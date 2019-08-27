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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.

/*
 * MapBuilder接口类
 */
class MapBuilderInterface {
 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;
  
  /*  定义如下
   *  using LocalSlamResultCallback =
   *  std::function<void(int // trajectory ID,
   *                     common::Time,
   *                     transform::Rigid3d // local pose estimate,
   *                     sensor::RangeData // in local frame,
   *                     std::unique_ptr<const InsertionResult>)>;
   */ 

  using SensorId = TrajectoryBuilderInterface::SensorId;

  /* 定义如下
   *   struct SensorId {
   *        enum class SensorType { RANGE, IMU, ODOMETRY, FIXED_FRAME_POSE, LANDMARK, LOCAL_SLAM_RESULT };
   *        SensorType type;
   *        std::string id;
   *    };
   */

  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index.
  // 创建一个新轨迹建立者，返回它的ID
  virtual int AddTrajectoryBuilder(
      // 期望的传感器名称
      const std::set<SensorId>& expected_sensor_ids,
      // 配置参数
      const proto::TrajectoryBuilderOptions& trajectory_options,
      // 局部slam的回调函数
      LocalSlamResultCallback local_slam_result_callback) = 0;

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  // 从序列中恢复一个轨迹的数据并返回它的index，如果请求它的轨迹建立者会返回空指针
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0;

  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  // 给定一个轨迹号，求它的轨迹建立者。如果没有，则返回空指针
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  // 标记某个轨迹是“结束”状态，即不再向它添加传感器数据
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  // 把submap数据转换为SubmapQuery::Response格式，用来回应对Submap的请求。如果出错，则返回错误字符串，否则返回空字符串。
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // 序列化整个SLAM建图状态数据（而不仅仅是地图数据）到proto流中。如果include_unfinished_submaps选项被设置，则未完成的submap也会包含进去
  virtual void SerializeState(bool include_unfinished_submaps,
                              io::ProtoStreamWriterInterface* writer) = 0;

  // Serializes the current state to a proto stream file on the host system. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // Returns true if the file was successfully written.
  // 序列化整个SLAM建图状态数据（而不仅仅是地图数据）到proto流中，从后续具体的实现来看，主要调用了SerializeState方法
  virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  // Loads the SLAM state from a proto stream. Returns the remapping of new
  // trajectory_ids.
  // 从proto流中加载SLAM建图状态数据，返回proto流中的轨迹号 -> 最终恢复出来的轨迹号的对应关系
  // 返回的map中，前者就是proto中的轨迹号，后者是最终的轨迹号
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadState(io::ProtoStreamReaderInterface* reader, bool load_frozen_state) = 0;

  // Loads the SLAM state from a pbstream file. Returns the remapping of new
  // trajectory_ids.
  // 从文件中加载SLAM建图状态数据，返回文件中轨迹号 -> 恢复的轨迹号的对应关系
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadStateFromFile(const std::string& filename, bool load_frozen_state) = 0;

  // 返回系统中已有的轨迹建立者的数目
  virtual int num_trajectory_builders() const = 0;

  // 返回一个poseGraph接口指针
  virtual mapping::PoseGraphInterface* pose_graph() = 0;

  // 返回所有轨迹建立者的配置参数。整体是一个数组，表示每个轨迹的轨迹建立者配置参数
  virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
