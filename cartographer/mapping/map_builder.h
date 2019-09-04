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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// 配置参数格式转换：从lua到proto
proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// 把整个SLAM栈组织在一起，包括了轨迹建立者（用于局部SLAM）和位姿图（用于回环优化）
/*
 * 整个carto的最顶层调用接口
 * 相关的服务，如创建轨迹，结束轨迹，请求子地图等等，都可以在本类中找到对应的处理函数
 * 构造函数：从参数中构造
 * 方法：
 * AddTrajectoryBilder：新建一个轨迹建立者
 * FinishTrajectory：标记某个轨迹是终止状态
 * 此外，还有序列化建图状态，从文件加载建图状态，等等。核心方法只有以上两个
 * 
 */
class MapBuilder : public MapBuilderInterface {
 public:
  // 唯一合法的构造函数：从配置参数中构造
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;
  
  // 创建一个新轨迹建立者，返回它的ID
  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  // 从序列中恢复一个轨迹的数据并返回它的index，如果请求它的轨迹建立者会返回空指针
  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  // 标记某个轨迹是“结束”状态，即不再向它添加传感器数据
  void FinishTrajectory(int trajectory_id) override;

  // 把submap数据转换为SubmapQuery::Response格式，用来回应对Submap的请求。如果出错，则返回错误字符串，否则返回空字符串。
  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  // 序列化整个SLAM建图状态数据（而不仅仅是地图数据）到proto流中。如果include_unfinished_submaps选项被设置，则未完成的submap也会包含进去
  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  // 序列化整个SLAM建图状态数据（而不仅仅是地图数据）到proto流中，从后续具体的实现来看，主要调用了SerializeState方法
  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  // 从proto流中加载SLAM建图状态数据，返回proto流中的轨迹号 -> 最终恢复出来的轨迹号的对应关系
  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  // 从文件中加载SLAM建图状态数据，返回文件中轨迹号 -> 恢复的轨迹号的对应关系
  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  //返回一个poseGraph接口指针
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  // 返回系统中已有的轨迹建立者的数目
  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  // 给定一个轨迹号，求它的轨迹建立者。如果没有，则返回空指针
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  // 返回所有轨迹建立者的配置参数。整体是一个数组，表示每个轨迹的轨迹建立者配置参数
  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  // 配置参数
  const proto::MapBuilderOptions options_;
  // 线程池
  common::ThreadPool thread_pool_;

  // 位姿图（指针)
  std::unique_ptr<PoseGraph> pose_graph_;

  // 传感器数据整理者（指针）
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  // 轨迹建立者（指针)数组
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  // 轨迹建立者配置参数数组
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

/*
 * 这里强调一下“轨迹”的概念
 * 这里的轨迹是非常高层次的概念，在我看来，不应该叫轨迹，应该叫“任务”
 * 轨迹是调用“服务”来实现新建和结束的，不是程序自发结束的。每个轨迹都可以配置不同的参数。
 * 例如：第一个轨迹是建图，第二个轨迹是用另一套参数建图，第三个轨迹是定位
 * 但注意了，轨迹建立者有很多，但是pose_graph却只有一个，用来对所有的子地图进行优化
 * [那么在后端的优化时，所有的不同轨迹建立者创建的子图是一视同仁吗？
 *  猜测：既然轨迹建立者和位姿图是分开的两部分，所以作用是分开的才对。轨迹规划者负责产生子地图，以及描述它们之间的关系
 *  具体在位姿图的内容中解读]
 */