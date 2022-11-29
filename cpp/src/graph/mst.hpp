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

#include <lemon/kruskal.h>
#include <lemon/smart_graph.h>

#include "global/global.hpp"

PROJECT_NAMESPACE_START

template <typename CostType, typename GraphType = lemon::SmartGraph>
class MinimumSpanningTree {

  using Graph   = GraphType;
  using Node    = typename GraphType::Node;
  using Edge    = typename GraphType::Edge;
  using EdgeMap = typename GraphType::template EdgeMap<CostType>;

public:
  explicit MinimumSpanningTree(const Int numNodes) : _edgeMap(_graph) {
    _vNodes.reserve(numNodes);
    for (Int i = 0; i < numNodes; ++i) {
      _vNodes.emplace_back(_graph.addNode());
    }
  }

  ~MinimumSpanningTree() {}

  void addEdge(const Int u, const Int v, const CostType c) {
    Edge edge      = _graph.addEdge(_vNodes[u], _vNodes[v]);
    _edgeMap[edge] = c;
  }

  // return the cost of the mst
  CostType solve(Vector<Pair<Int, Int>>& vEdges) {
    Vector<Edge> vRes;
    const Int cost = lemon::kruskal(_graph, _edgeMap, std::back_inserter(vRes));

    for (size_t i = 0; i < vRes.size(); ++i) {
      const Edge& edge = vRes[i];
      vEdges.emplace_back(_graph.id(_graph.u(edge)), _graph.id(_graph.v(edge)));
    }
    return cost;
  }

private:
  Graph          _graph;
  EdgeMap        _edgeMap;
  Vector<Node>   _vNodes;
};

PROJECT_NAMESPACE_END
