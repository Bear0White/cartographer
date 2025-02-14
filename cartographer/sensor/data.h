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

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {

// 这个应该是声明吧？如果没有包含它的h文件，下文中调用时找不到定义
namespace mapping {
class TrajectoryBuilderInterface;
}

namespace sensor {

/*
 * Data类定义了一个基础的传感器数据结构，是一个虚类
 * 构造函数：以一个传感器名称构造（注意此处的ID不是index)
 * 方法：
 *  GetTime返回时间戳, 
 *  GetSensorId返回传感器ID, 
 *  AddToTrajectoryBuilder添加到轨迹构建者
 */
class Data {
 public:
  explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
  virtual ~Data() {}

  virtual common::Time GetTime() const = 0;
  const std::string &GetSensorId() const { return sensor_id_; }
  virtual void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *trajectory_builder) = 0;

 protected:
  const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
