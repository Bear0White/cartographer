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

#ifndef CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
#define CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/proto/ceres_solver_options.pb.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace common {

// 从Lua字典里面提取相关参数，转换为proto格式
proto::CeresSolverOptions CreateCeresSolverOptionsProto(
    common::LuaParameterDictionary* parameter_dictionary);

// 把proto格式的配置参数转换为Ceres可接受的结构体格式
ceres::Solver::Options CreateCeresSolverOptions(
    const proto::CeresSolverOptions& proto);

/*
 * Summary
 * 这个文件的主要作用就是对ceres相关的配置参数做格式转换。
 * 从配置文件中读取的数据最初是Lua字典格式的，但是最终交付给ceres使用，一定是ceres自己要求的格式。
 * 而信息在传递的过程中，会用到proto格式，proto成了信息传递的中间格式。
 */


}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
