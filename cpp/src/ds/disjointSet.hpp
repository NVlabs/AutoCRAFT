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

#include "global/namespace.hpp"

PROJECT_NAMESPACE_START

class DisjointSet {
 public:
  DisjointSet()
    : parent(0), rnk(0), n(0) {}
  DisjointSet(Int n) { init(n); }
  ~DisjointSet() {
    delete [] parent;
    delete [] rnk;
  }

  void init(Int n) {
    this->n = n;
    parent = new Int[n];
    rnk = new Int[n];
    for (Int i = 0; i < n; ++i) {
      rnk[i] = 0;
      parent[i] = i;
    }
  }
  // Find set
  Int find(Int u) {
    return (u == parent[u] ? u : parent[u] = find(parent[u]));
  }

  // Union by rank
  void merge(Int x, Int y) {
    x = find(x), y = find(y);
    if (x == y)
      return;
    if (rnk[x] > rnk[y])
      parent[y] = x;
    else // If rnk[x] <= rnk[y]
      parent[x] = y;
    if (rnk[x] == rnk[y])
      rnk[y]++;
  }

  // Number of disjoint sets
  Int nSets() {
    Int nSets = 0;
    for (Int i = 0; i < n; ++i)
      if (parent[i] == i) ++nSets;
    return nSets;
  }

 private:
  Int *parent, *rnk;
  Int n;
};

PROJECT_NAMESPACE_END

