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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>

namespace cartographer {

/*
 * 本文件中的主要内容：
 * 定义了int8/16/32/64与uint8/16/32/64数据类型
 * 实现浮点数的四舍五入取整，通过调用std::lround函数实现四舍五入
 * 实现字符串的压缩与解压操作，原理不做深究
 * 虽然内容很简单，但对剥离核心逻辑与底层平台有重要作用
 */

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {

// 对float进行四舍五入取整得到int
inline int RoundToInt(const float x) { return std::lround(x); }

// 对double进行四舍五入取整得到int
inline int RoundToInt(const double x) { return std::lround(x); }

// 对float进行四舍五入取整得到int64
inline int64 RoundToInt64(const float x) { return std::lround(x); }

// 对double进行四舍五入取整得到int64
inline int64 RoundToInt64(const double x) { return std::lround(x); }

// 压缩字符串，原理不做深究
inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  boost::iostreams::filtering_ostream out;
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}

// 解压字符串，原理不做深究
inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
