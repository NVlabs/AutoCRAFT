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

#include "ds/pqueue.hpp"
#include "routeDrMgr.hpp"
#include "drc/drcMgr.hpp"

PROJECT_NAMESPACE_START

class DrPsRouter {
public:
  using GridNode = RouteDrMgr::GridNode;
  using GridEdge = RouteDrMgr::GridEdge;

  // cmp structs for pqueue
  struct NetCmp {
    bool operator() (const Net* p1, const Net* p2) {
      if (p1->priority() != p2->priority()) {
        return p1->priority() < p2->priority();
      }
      else if (p1->numFails() != p2->numFails()) {
        return p1->numFails() < p2->numFails();
      }
      else if (p1->isSelfSym() != p2->isSelfSym()) {
        return p1->isSelfSym() < p2->isSelfSym();
      }
      else if (p1->hasSymNet() != p2->hasSymNet()) {
        return p1->hasSymNet() < p2->hasSymNet();
      }
      else if (p1->bbox().hpwl() != p2->bbox().hpwl()) {
        return p1->bbox().hpwl() > p2->bbox().hpwl();
      }
      else if (p1->numPins() != p2->numPins()) {
        return p1->numPins() < p2->numPins();
      }
      return false;
      //return (p1->numFails()    * fcFail +
              //p1->isSelfSym()   * fcSelfSym +
              //p1->hasSymNet()   * fcSym +
              //p1->numPins()     * fcPin +
              //p1->bbox().hpwl() * fcBbox) < 
             //(p2->numFails()    * fcFail +
              //p2->isSelfSym()   * fcSelfSym +
              //p2->hasSymNet()   * fcSym +
              //p2->numPins()     * fcPin +
              //p2->bbox().hpwl() * fcBbox); 
    }
    //Real fcFail = 1.0;
    //Real fcSelfSym = 1.0;
    //Real fcSym = 1.0;
    //Real fcPin = 1.0;
    //Real fcBbox = 1.0;
  };

  struct RipupNetCmp {
    bool operator() (const Net* p1, const Net* p2) {
      if (p1->priority() != p2->priority()) {
        return p1->priority() < p2->priority();
      }
      else if (p1->numFails() != p2->numFails()) {
        return p1->numFails() < p2->numFails();
      }
      else if (p1->isSelfSym() != p2->isSelfSym()) {
        return p1->isSelfSym() < p2->isSelfSym();
      }
      else if (p1->hasSymNet() != p2->hasSymNet()) {
        return p1->hasSymNet() < p2->hasSymNet();
      }
      else if (p1->bbox().hpwl() != p2->bbox().hpwl()) {
        return p1->bbox().hpwl() > p2->bbox().hpwl();
      }
      else if (p1->numPins() != p2->numPins()) {
        return p1->numPins() < p2->numPins();
      }
      return false;
      //return (p1->numFails()    * fcFail +
              //p1->isSelfSym()   * fcSelfSym +
              //p1->hasSymNet()   * fcSym +
              //p1->numPins()     * fcPin +
              //p1->bbox().hpwl() * fcBbox +
              //p1->length()      * fcLength) < 
             //(p2->numFails()    * fcFail +
              //p2->isSelfSym()   * fcSelfSym +
              //p2->hasSymNet()   * fcSym +
              //p2->numPins()     * fcPin +
              //p2->bbox().hpwl() * fcBbox +
              //p2->length()      * fcLength); 
    }
    //Real fcFail = 1.0;
    //Real fcSelfSym = 1.0;
    //Real fcSym = 1.0;
    //Real fcPin = 1.0;
    //Real fcBbox = 1.0;
    //Real fcLength = 1.0;
  };

  DrPsRouter(RouteDrMgr& drMgr, const Int numThreads = 1)
    : _drMgr(drMgr),
      _cir(drMgr._cir),
      _vGridNodes(drMgr._vGridNodes),
      _vGridEdges(drMgr._vGridEdges),
      _vvAdjEdgeIds(drMgr._vvAdjEdgeIds),
      _mGridNode2Idx(drMgr._mGridNode2Idx),
      _mGridEdge2Idx(drMgr._mGridEdge2Idx),
      _numThreads(numThreads), 
      _vHistory(drMgr._vGridEdges.size())
  {}
  ~DrPsRouter() {}

  RouteDrMgr&       drMgr()       { return _drMgr; }
  const RouteDrMgr& drMgr() const { return _drMgr; }
  CirDB&            cir()         { return _cir; }
  const CirDB&      cir()   const { return _cir; }

  bool              solve(const bool useIO = false, const bool bRouteAll = false);
  
  // history
  Real history(const Int eIdx)                                                    const { return _vHistory.at(eIdx); }
  Real history(const Segment3d<Int>& seg)                                         const ;
  Real history(const Segment3d<Int>& seg, const Int n, const Real decay = 1)      const ; // expand n tracks near by
  Real history(const Point3d<Int>& u, const Point3d<Int>& v)                      const ;
  Real neighborHistory(const Segment3d<Int>& seg, const Int n, const bool isPrev) const ;
  void addHistory(const Int eIdx, const Real cost)                                      ;
  void addHistory(const Segment3d<Int>& seg, const Real cost)                           ;
  void decayHistory(const Int eIdx, const Real cost)                                    ;
  void decayHistory(const Segment3d<Int>& seg, const Real cost)                         ;
  void clearHistory(const Int eIdx)                                                     ;
  void clearHistory(const Segment3d<Int>& seg)                                          ;
  void resetHistory()                                                                   ;

  // for routables
  void constructNetRoutables(Net& net, const bool useIO);
  void constructNormalNetRoutables(Net& net, const bool useIO);
  void constructSelfSymNetRoutables(Net& net, const bool useIO);
  void constructSymNetRoutables(Net& net, const bool useIO);
  void constructCrossSymNetRoutables(Net& net, const bool useIO);


  void addUnroutedNet(Net& net);
  void addUnroutedNets(const bool useIO, const NetTypeE netType = NetTypeE::critical);
  bool runNrr(const Vector<Int>& vNetIds, const bool useIO);
  bool runNrr(const bool useIO, const NetTypeE netType = NetTypeE::critical);
  bool runNrrCore(const NetTypeE netType, const bool useIO, const bool useStrictDRC, const Int drcCost, const Int historyCost);
  bool runNrrCore(const Vector<Int>& vNetIds, const bool useIO, const bool useStrictDRC, const Int drcCost, const Int historyCost);
  bool hasUnRoutedNet();
  Int  nextUnRoutedNetIdx();
  bool route(Net& net, const bool useStrictDRC, const Int drcCost = 0, const Int historyCost = 0);
  bool checkDRC(const Vector<Int>& vNetIds, const bool useIO) const;
  bool checkDRC(const bool useIO, const NetTypeE netType) const;
  bool checkNetDRC(const Net& net) const;
  void ripup(Net& net) const;
  void ripupSegment(Net& net, const Segment3d<Int>& seg) const;
  void ripupPartial(Net& net, const Vector<Segment3d<Int>>& vSegs) const;
  void ripupSegmentNRefine(Net& net, const Segment3d<Int>& seg) const;
  void ripupPartialNRefine(Net& net, const Vector<Segment3d<Int>>& vSegs) const;
  void ripupSegmentNFloating(Net& net, const Segment3d<Int>& seg) const;
  void ripupPartialNFloating(Net& net, const Vector<Segment3d<Int>>& vSegs) const;
  void ripupFloating(Net& net) const;
  void refineNet(Net& net) const;
  void resetNetWires(Net& net) const;
  void resetNetStatus(Net& net) const;

  void segs2Wires(const Vector<Segment3d<Int>>& vSegs, Vector<Pair<Box<Int>, Int>>& vWires) const;
  void segs2Wires(const Vector<Segment3d<Int>>& vSegs, const Vector<TopoTree::SegData>& vSds, Vector<Pair<Box<Int>, Int>>& vWires) const;
  void seg2Wires(const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const;
  void seg2Wires(const Segment3d<Int>& seg, const TopoTree::SegData& sd, Vector<Pair<Box<Int>, Int>>& vWires) const;
  void mergeWires(const Vector<Pair<Box<Int>, Int>>& vWires, Vector<Pair<Box<Int>, Int>>& vMergedWires) const;
  void queryNetSegWires(const Net& net, const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const;
  
  void queryTopoWires(const Net& net, const TopoTree& topo, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const;
  void queryTopoWires(const Net& net, const TopoTree& topo, Vector<Pair<Box<Int>, Int>>& vWires) const;

  void queryTopoCompWires(const Net& net, const TopoTree& topo, const Int compIdx, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const;
  void queryTopoCompWires(const Net& net, const TopoTree& topo, const Int compIdx, Vector<Pair<Box<Int>, Int>>& vWires) const;

  void queryTopoSegsWires(const Net& net, const Vector<Segment3d<Int>>& vSegs, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const;
  void queryTopoSegsWires(const Net& net, const Vector<Segment3d<Int>>& vSegs, Vector<Pair<Box<Int>, Int>>& vWires) const;

  void queryTopoPtsWires(const Net& net, const FlatHashSet<Point3d<Int>>& sPts, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const;

  void topoComp2AcsPts(const TopoTree& topo, const Int compIdx, FlatHashSet<Point3d<Int>>& sAcs) const;
  void topoComp2AcsPts(const TopoTree& topo, const Int compIdx, Vector<Point3d<Int>>& vAcs) const;
  //void querySegWires(const Net& net, const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const;
  //
  void checkFailed();

  // helper
  Int           wireWidth(const Point3d<Int>& u) const;
  Box<Int>      toWire(const Point3d<Int>& u, const Point3d<Int>& v, const Int width, const Int ext) const;
  Int           scaledMdist(const Point3d<Int>& u, const Point3d<Int>& v) const;
  Int           scaledMdist(const Box<Int>& u, const Box<Int>& v) const;
  Int           scaledMdist(const Point3d<Int>& u, const Pair<Box<Int>, Int>& v) const;
  Int           scaledMdist(const Pair<Box<Int>, Int>& u, const Pair<Box<Int>, Int>& v) const;
  bool          hasBend(const Point3d<Int>& u, const Point3d<Int>& r, const Point3d<Int>& v) const;

  Int           drcCost() const { return _drcCost; }
  Int           hisCost() const { return _hisCost; }
  Int           horCost() const { return _horCost; }
  Int           verCost() const { return _verCost; }
  Int           viaCost() const { return _viaCost; }
  Int           viaCost(const Int z) const;
  Int           viaCost(Int zl, Int zh) const;

private:
  RouteDrMgr&                       _drMgr;
  CirDB&                            _cir;
  const Vector<GridNode>&           _vGridNodes;
  const Vector<GridEdge>&           _vGridEdges;
  const Vector<Vector<Int>>&        _vvAdjEdgeIds;
  const FlatHashMap<GridNode, Int>& _mGridNode2Idx;
  const FlatHashMap<GridEdge, Int>& _mGridEdge2Idx;
  const Int                         _numThreads;


  // for rr
  Vector<Real>            _vHistory; // same size as _vGridEdges
  Int                     _drcCost      = 12000;
  Int                     _hisCost      = 600;

  // for path finding
  const Int               _horCost      = 1;
  const Int               _verCost      = 1;
  const Int               _viaCost      = 3000;

  PairingHeap<Net*, NetCmp> _pqc; // pq for critical nets



};

PROJECT_NAMESPACE_END
