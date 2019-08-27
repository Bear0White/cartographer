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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  absl::optional<ConstantPoseData> constant_pose_data;
};

/*
 * 轨迹节点，每个节点记录了某个时刻的传感器信息，一个轨迹由若干节点构成。
 * 整个结构体包括了Data的定义。结构体内部包括了一个Data的智能指针，一个本地位姿数据
 */ 

struct TrajectoryNode {
  
  /*
   * Data, 节点上记录的数据，包括时间，重力方向角，经过滤波和重力校正的点云数据，在局部SLAM中的位姿
   * 最后的局部slam的位姿猜测是相对于子地图的位姿吧？但为什么没有子地图相关的数据？
   */
  struct Data {
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaterniond gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;
    sensor::PointCloud low_resolution_point_cloud;
    Eigen::VectorXf rotational_scan_matcher_histogram;

    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;
  };

  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data;

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;
};

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data);
TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
