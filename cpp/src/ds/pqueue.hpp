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

#include <ext/pb_ds/priority_queue.hpp>

#include "global/namespace.hpp"

PROJECT_NAMESPACE_START


template<typename T, typename Cmp = std::less<T> >
using Pqueue = __gnu_pbds::priority_queue<T, Cmp>;

template<typename T, typename Cmp = std::less<T> >
using BinaryHeap = __gnu_pbds::priority_queue<T, Cmp, __gnu_pbds::binary_heap_tag>;

template<typename T, typename Cmp = std::less<T> >
using BinomialHeap = __gnu_pbds::priority_queue<T, Cmp, __gnu_pbds::binomial_heap_tag>;

template<typename T, typename Cmp = std::less<T> >
using RCBinomialHeap = __gnu_pbds::priority_queue<T, Cmp, __gnu_pbds::rc_binomial_heap_tag>;

template<typename T, typename Cmp = std::less<T> >
using PairingHeap = __gnu_pbds::priority_queue<T, Cmp, __gnu_pbds::pairing_heap_tag>;

template<typename T, typename Cmp = std::less<T> >
using ThinHeap = __gnu_pbds::priority_queue<T, Cmp, __gnu_pbds::thin_heap_tag>;

PROJECT_NAMESPACE_END

