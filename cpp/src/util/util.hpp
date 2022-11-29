/*
* SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#pragma once

#include "global/global.hpp"
#include <type_traits>

PROJECT_NAMESPACE_START

namespace util {

namespace fs {

extern bool   existFile(const String& fileName);
extern String getDirName(const String& fileName);
extern String getFileName(const String& fileName);
} // namespace fs

namespace str {
extern void split(const String& s, const String& delims, Vector<String>& tokens);
}

namespace enumUtil {

template <typename E> constexpr auto getVal(E e) {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

template <typename T> struct StrValPair {
  static_assert(std::is_enum<T>::value);
  using value_type = T;
  const T           value;
  const char* const str;
};

// Templated helper functions.
// Mapping is some type of standard container that supports find_if()
// V is the type of the enum whose value we wish to look up
template <typename Mapping, typename V> String val2Str(Mapping a, V value) {
  auto pos = std::find_if(
      std::begin(a), std::end(a),
      [&value](const typename Mapping::value_type& t) { return (t.value == value); });
  if (pos != std::end(a)) {
    return pos->str;
  }
  return "";
}

template <typename Mapping>
typename Mapping::value_type::value_type str2Val(Mapping a, const String& str) {
  static_assert(std::is_enum<typename Mapping::value_type::value_type>::value);
  auto pos = std::find_if(
      std::begin(a), std::end(a),
      [&str](const typename Mapping::value_type& t) { return (t.str == str); });
  if (pos != std::end(a)) {
    return pos->value;
  }
  typename Mapping::value_type::value_type val{};
  return val;
}

} // namespace enumUtil

} // namespace util

PROJECT_NAMESPACE_END
