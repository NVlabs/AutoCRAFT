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

#define USE_BIDIR_SEARCH

#include "routeDrPS.hpp"
#include "ds/disjointSet.hpp"
#include "ds/hash.hpp"
#include "drc/drcMgr.hpp"

PROJECT_NAMESPACE_START

struct AstarNode {
  
  AstarNode() {}
  AstarNode(const Point3d<Int>& p)
    : loc(p) {}
  ~AstarNode() {}

  Point3d<Int>         loc;
  Array<Real, 2>       costG     = {MAX_REAL, MAX_REAL};
  Array<Real, 2>       costF     = {MAX_REAL, MAX_REAL};
  Array<Int, 2>        bendCnt   = {MAX_INT, MAX_INT};
  Array<char, 2>       visited   = {false, false};
  Array<AstarNode*, 2> pParent   = {nullptr, nullptr};
  Vector<AstarNode*>   vpNeighbors;

  void reset() {
    costG   = {MAX_REAL, MAX_REAL};
    costF   = {MAX_REAL, MAX_REAL};
    bendCnt = {MAX_INT, MAX_INT};
    visited = {false, false};
    pParent = {nullptr, nullptr};
  }

  template <size_t D> struct Cmp {
    constexpr bool operator()(const AstarNode* n0, const AstarNode* n1) {
      if (n0->costF[D] != n1->costF[D])
        return n0->costF[D] > n1->costF[D];
      else if (n0->bendCnt[D] != n1->bendCnt[D])
        return n0->bendCnt[D] > n1->bendCnt[D];
      //else if (n0->costG[D] != n1->costG[D])
        //return n0->costG[D] > n1->costG[D];
      else
        return false;
    }
    constexpr bool operator()(const AstarNode& n0, const AstarNode& n1) {
      if (n0.costF[D] != n1.costF[D])
        return n0.costF[D] > n1.costF[D];
      else if (n0.bendCnt[D] != n1.bendCnt[D])
        return n0.bendCnt[D] > n1.bendCnt[D];
      //else if (n0.costG[D] != n1.costG[D])
        //return n0.costG[D] > n1.costG[D];
      else
        return false;
    }
  };
};

class AstarCore {
public:
  using GridNode = DrPsRouter::GridNode;
  using GridEdge = DrPsRouter::GridEdge;

  AstarCore(CirDB& cir, Net& net, Routable& ro, DrPsRouter& ps,
            const bool isSym, const bool isSelfSym,
            const bool useStrictDRC)
    : _cir(cir),
      _net(net),
      _region(net.region()),
      _ro(ro),
      _ps(ps),
      _drc(cir),
      _isSym(isSym),
      _isSelfSym(isSelfSym),
      _useStrictDRC(useStrictDRC)
      //_vTmpSpatialNets(_cir.numLayers())
  {}
  ~AstarCore() {}

  bool solve();

private:
  CirDB&      _cir;
  Net&        _net;
  Region&     _region;
  Routable&   _ro;
  DrPsRouter& _ps;
  DrcMgr      _drc;

  // params
  const bool _isSym;
  const bool _isSelfSym;
  const bool _useStrictDRC;
  const Int  _maxExplore            = 100000;
  const Real _fcG                   = 1;
  const Real _fcH                   = 1;
  const Real _fcOV                  = 5;
  const Real _fcPenalty             = 0.1;
  const Real _fcHistory             = 0.05;
  const Real _fcVirtualPinStepX     = 0.05;
  const Real _fcVirtualPinStepY     = 0.05;
  const Int  _virtualPinExpandStepX = 10;
  const Int  _virtualPinExpandStepY = 10;

  // connecting comps
  DisjointSet                                       _compDS;
  Vector<Vector<Pair<Box<Int>, Int>>>               _vCompBoxes;
  Vector<FlatHashSet<Point3d<Int>>>                 _vCompAcsPts;
  Vector<FlatHashMap<Int, Spatial<Int>>>            _vCompSpatialBoxes;
  Vector<Pair<Int, Int>>                            _vSubNets;
  FlatHashMap<Point3d<Int>, UniquePtr<AstarNode>>   _allNodesMap;
  
  Vector<Pin*>                                      _vpPins;
  Vector<Int>                                       _vRoutableIds; // routableIdx in _net
  Vector<Int>                                       _vTopoCompIds; // compIdx in _ro.topo()
  
  // self-sym
  bool                                              _hasCrossAxisPin  = false;
  bool                                              _hasCrossAxisComp = false;
  
  // tmp spatial
  //Vector<SpatialMap<Int, Net*>>                     _vTmpSpatialNets; // for same net drc

  // result
  Vector<Vector<Pair<Point3d<Int>, Point3d<Int>>>>  _vvRoutePaths;
  Vector<Vector<const Via*>>                        _vvRoutePathVias;
  Vector<Vector<Orient2dE>>                         _vvRoutePathViaOrients;
  Vector<Vector<Pair<Int, Int>>>                    _vvRoutePathWidthExts;
  Vector<Vector<Pair<Box<Int>, Int>>>               _vvRouteWires;


  ////////////////////////////////////
  //            funcs               //
  ////////////////////////////////////
  void init();
  void initFromPartial();
  void initFromScratch();
  void initFromScratchSelfSym();
  void initRL(); // used for RL-based routing
  bool splitSubNet();
  bool splitSubNetFromScratch();
  bool splitSubNetFromPartial();
  bool route();
  void ripup();
  void saveResult();

  bool routeSubNet(Int srcIdx, Int tarIdx);
  void resetAllNodes();
  bool pathSearch(const Int srcIdx, const Int tarIdx);
  bool bTerminate(AstarNode* pU, const Int tarIdx, const bool isReverse);
  void mergeComp(const Int srcIdx, const Int tarIdx);
  void backTrack(const AstarNode* pU, const Int srcIdx, const Int tarIdx);
  void mergePath(const List<Point3d<Int>>& lPathPts, List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs) const;
  bool bNeedMergePath(const Point3d<Int>& u1, const Point3d<Int>& v1, const Point3d<Int>& u2, const Point3d<Int>& v2) const;
  void path2Wires(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs, Vector<Pair<Box<Int>, Int>>& vWires) const;
  void savePath(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs);
  void saveWires(const Vector<Pair<Box<Int>, Int>>& vWires);
  void findNeighbors(AstarNode* pU);
  bool isValidNode(const AstarNode* pV) const;
  bool bViolateDRC(const AstarNode* pU, const AstarNode* pV, const Int srcIdx, const Int tarIdx, const bool isReverse) const;
  bool bNeedUpdate(const AstarNode* pV, const Int costG, const Int bendCnt, const bool isReverse) const;
  void addHistory(const List<Point3d<Int>>& lPathPts);
  void addHistory(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs);

  // helper funcs
  void          addAcsPts(const Int idx, const Box<Int>& box, const Int z);
  bool          hasBend(const AstarNode* pU, const AstarNode* pV, const bool isReverse) const;
  Box<Int>      curLayerCheckWire(const AstarNode* pU, const AstarNode* pV, const bool i) const;
  Box<Int>      prevLayerCheckWire(const AstarNode* pU, const bool i) const; // for lower layers
  Box<Int>      bfsNetWire(const Box<Int>& wire, const Int layerIdx) const; // for lower layers
};


PROJECT_NAMESPACE_END
