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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

/*
 * Dispatchable类，从Data类中继承
 * Data类定义了一个基础的传感器数据结构
 *    构造函数：以一个传感器名称构造（注意此处的ID不是index)
 *    方法：GetTime, GetSensorId, AddToTrajectoryBuilder
 * Dispatchable类中也仅仅增加了一个模板类型DataType而已，这是最主要的区别
 * 构造函数：string类型的sensor_id，一个DataType类型的数据对象
 * 方法：GetTime：返回数据对象的时间戳
 *      AddToTrajectoryBuilder：调用参数的AddSensorData方法而已
 *      data：返回数据对象本体
 * 概括：猜测这个东西就是用来记录一帧传感器数据用的，可以提供时间戳，数据内容，和加入到轨迹构建者中的途径
 */
template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const override { return data_.time; }
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  const DataType &data() const { return data_; }

 private:
  const DataType data_;
};

/*
 * 创建一个Dispatchable对象的智能unique指针
 */
template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
