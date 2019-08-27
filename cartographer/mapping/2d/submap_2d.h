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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// 配置参数格式转换：从lua到proto
proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

/*
 * 子地图
 * 构造：一个二维坐标，一个Grid2D指针，一个查找表指针
 * 方法：
 *    grid：返回内部的Grid2D成员
 *    InsertRangeData：使用某个插入器，把某个扫描帧插入子地图
 *    Finish：标记某个子地图为结束，不再插入数据
 */
class Submap2D : public Submap {
 public:
  // 构造函数：需要参数
  Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
           ValueConversionTables* conversion_tables);
  
  // 构造函数：从流中构造
  explicit Submap2D(const proto::Submap2D& proto,
                    ValueConversionTables* conversion_tables);

  // 转换到proto
  proto::Submap ToProto(bool include_grid_data) const override;

  // 从一个proto流中更新地图
  void UpdateFromProto(const proto::Submap& proto) override;
  // 如果得到一个对子地图数据的请求，尝试把地图数据转换为proto::SubmapQuery::Response格式
  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;
  
  // 返回地图的栅格数据
  const Grid2D* grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  // 把某个扫描帧插入到子地图中去，使用一个插入器。注意这里是不含扫描匹配的
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserterInterface* range_data_inserter);
  
  // 结束一个子地图，不再插入扫描数据
  void Finish();

 private:
  // 地图数据的本体
  std::unique_ptr<Grid2D> grid_;
  
  // 查找表，用于在更新地图时进行快速地在整形和浮点空间进行转换，而且可以记录所有生成的查找表，避免重复生成
  // 构造函数中有查找表参数，这样构造新Submap时可以把之前用到的查找表共享出去，提高效率
  ValueConversionTables* conversion_tables_;
};

// ------------------  以下是 ActiveSubmaps2D 的内容---------------------------

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.

/*
 * 第一个激活的子地图会被创建，当处理第一个扫描帧的时候。除了这个初始化过程外，正常过程中维护两个子地图：旧的和新的。
 * [没看懂注释里面的]
 * 一旦一定数量的扫描帧被插入，子地图就认为是“初始化完毕”。旧的那个不在改变，新的那个就变成了旧的，用来做扫描匹配。同时一个新的地图被创建。
 * 旧的和新的，正常情况两个都没满。旧的里面更多些。帧插入两者中，等旧的满了，就交给后端，这里就直接抛弃了。新的就成了旧的，同时创建新的。
 * 最初的情况是什么呢？？
 */
class ActiveSubmaps2D {
 public:
  // 构造函数：从参数中构造
  explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Inserts 'range_data' into the Submap collection.
  // 插入一个扫描帧到某些子地图中（应该是两个或一个），返回一个指针数组，表示插入到的子地图们
  std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
      const sensor::RangeData& range_data);
  
  // 返回内部包含的子地图(指针)数组
  std::vector<std::shared_ptr<const Submap2D>> submaps() const;

 private:
  // 创建一个激光帧插入器
  std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  // 创建一个网格
  std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
  // 完结一个submap
  void FinishSubmap();
  // 添加一个submap
  void AddSubmap(const Eigen::Vector2f& origin);

  // 配置参数
  const proto::SubmapsOptions2D options_;
  // submap数组
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  // 激光帧插入器
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
  
  // 查找表
  ValueConversionTables conversion_tables_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

/*
 *
 * 层次关系分析：
 * ActiveSubmaps2D内部有Submaps数组
 * Submap里面有Grid2D
 * Grid2D里面有MapLimits和栅格实体数据
 * 关于插入操作：ActiveSubmaps2D里面有InsertRangeData方法，让内部的Submap执行InsertRangeData方法；后者调用内部的插入器执行Insert方法。
 * 插入器的Insert方法需要参数：激光帧和栅格地图，会遍历激光点，然后调用栅格的相关更新函数完成更新
 */