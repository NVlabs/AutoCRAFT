/*
* SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <parallel_hashmap/phmap.h>

#include "global/namespace.hpp"

PROJECT_NAMESPACE_START

template<typename K>
using FlatHashSet = phmap::flat_hash_set<K>;

template<typename K, typename V>
using FlatHashMap = phmap::flat_hash_map<K, V>;

template<typename K>
using ParallelFlatHashSet = phmap::parallel_flat_hash_set<K>;

template<typename K, typename V>
using ParallelFlatHashMap = phmap::parallel_flat_hash_map<K, V>;

template<typename K>
using NodeHashSet = phmap::node_hash_set<K>;

template<typename K, typename V>
using NodeHashMap = phmap::node_hash_map<K, V>;

template<typename K>
using ParallelNodeHashSet = phmap::parallel_node_hash_set<K>;

template<typename K, typename V>
using ParallelNodeHashMap = phmap::parallel_node_hash_map<K, V>;

PROJECT_NAMESPACE_END

