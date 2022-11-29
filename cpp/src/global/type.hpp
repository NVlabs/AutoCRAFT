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

#include "namespace.hpp"
#include <cstdint>
#include <sstream>
#include <string>

#include <vector>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <stack>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <tuple>
#include <variant>

PROJECT_NAMESPACE_START

using Int  = std::int32_t;
using Uint = std::uint32_t;
using Real = double;
using Byte = char;

constexpr Int  MAX_INT    = std::numeric_limits<Int>::max() / 3;
constexpr Int  MIN_INT    = std::numeric_limits<Int>::lowest() / 3;
constexpr Uint MAX_UINT   = std::numeric_limits<Uint>::max() / 3;
constexpr Uint MIN_UINT   = std::numeric_limits<Uint>::lowest() / 3;
constexpr Real MAX_REAL   = std::numeric_limits<Real>::max() / 3;
constexpr Real MIN_REAL   = std::numeric_limits<Real>::lowest() / 3;
constexpr Real EPSILON    = 1e-8;
constexpr Real TIME_SCALE = 1000000.00;
constexpr Real MEM_SCALE  = 1024.0;

// Type aliases
                                                          using String    = std::string;
template <typename T, typename U>                         using Pair      = std::pair<T, U>;
template <typename... Args>                               using Tuple     = std::tuple<Args...>;
template <typename T>                                     using Queue     = std::queue<T>;
template <typename T>                                     using Stack     = std::stack<T>;
template <typename T, size_t N>                           using Array     = std::array<T, N>;
template <typename T, class Alloc = std::allocator<T>>    using Vector    = std::vector<T, Alloc>;
template <typename T>                                     using List      = std::list<T>;
template <typename T>                                     using Set       = std::set<T>;
template <typename T, typename U>                         using Map       = std::map<T, U>;
template <typename T>                                     using Uset      = std::unordered_set<T>;
template <typename T, typename U>                         using Umap      = std::unordered_map<T, U>;
template <typename T>                                     using UniquePtr = std::unique_ptr<T>;
template <typename T>                                     using SharedPtr = std::shared_ptr<T>;
template <typename T>                                     using WeakPtr   = std::weak_ptr<T>;
template <typename... Args>                               using Variant   = std::variant<Args...>;
PROJECT_NAMESPACE_END
