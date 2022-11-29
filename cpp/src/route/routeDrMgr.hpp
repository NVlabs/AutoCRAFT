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

#include <parallel_hashmap/phmap_utils.h>

#include "db/dbCir.hpp"

PROJECT_NAMESPACE_START

class RouteDrMgr {
  friend class SmtRouter;
  friend class DrPsRouter;

public:
  
  using GridNode = Point3d<Int>;

  struct GridEdge {
    GridEdge(const Int i, const Int j) : u(i), v(j)
    {
      assert(u < v);
    }
    
    Int u, v; // node ids
    Int adjNodeIdx(const Int i) const {
      assert(i == u or i == v);
      return i == u ? v : u;
    }
    bool operator == (const GridEdge &e) const { 
      return u == e.u and v == e.v;
    }
    friend size_t hash_value(const GridEdge &e) {
      return phmap::HashState().combine(0, e.u, e.v);
    }
  };

  RouteDrMgr(CirDB& cir)
    : _cir(cir)
  {
  }
  ~RouteDrMgr() {}
  
  ////////////////////////////////////
  //              DR                //
  ////////////////////////////////////
  // top-level funcs
  bool                solvePS(const Int numThreads);
  // init
  void                init(const bool useM1 = false, const Int maxRgIdx = 14);
  void                computePinAcs();
  void                computeBoxAcs(const Box<Int>& box, const Int z, Vector<Point3d<Int>>& vAcs);
  void                computeNetBbox();
  void                setNetSymCstrs();
  void                setNetPathMatchingCstrs();
  Int                 findSymAxis(const Net& na, const Net& nb, const bool isVer) const;
  Int                 findSelfSymAxis(const Net& n, const bool isVer) const;
  Real                degPinSym(const Net& na, const Net& nb, const bool isVer, const Int symAxis) const;
  Real                degPinSelfSym(const Net& n, const bool isVer, const Int symAxis) const;
  bool                isCrossSym(const Net& net) const;
  bool                isCrossNet(const Net& net) const;
  bool                isPartAPin(const Pin& pin) const;
  bool                isPartBPin(const Pin& pin) const;
  bool                hasSymPin(const Pin& pin, const bool isVer, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const;
  bool                hasSymPinX(const Pin& pin, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const;
  bool                hasSymPinY(const Pin& pin, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const;
  Pin&                symPin(const Pin& pin, const Vector<Vector<Box<Int>>>& vvSymBoxes) const;
  Pin*                pSymPin(const Pin& pin, const Vector<Vector<Box<Int>>>& vvSymBoxes) const;
  void                addPinShapes(const Net& n, Vector<Vector<Box<Int>>>& vvBoxes) const;
  
  // io pin
  void                genIOPinShapes(); 

  // grid graph
  const GridNode&     gridNode(const Int i) const;
  const GridEdge&     gridEdge(const Int i) const;
  Int                 gridNodeIdx(const GridNode& n) const;
  Int                 gridEdgeIdx(const GridEdge& e) const;
  Int                 gridEdgeIdx(Int u, Int v) const;
  void                vGridEdgeIds(const Box<Int>& box, const Int layerIdx, Vector<Int>& vIds) const;
  void                vGridEdgeIds(const Segment3d<Int>& seg, Vector<Int>& vIds) const;
  Segment3d<Int>      gridEdge2Seg(const GridEdge& e) const;
  Segment3d<Int>      gridEdge2Seg(const Int eIdx) const;
  const GridNode&     adjNode(const GridEdge& e, const GridNode& n) const;
  const Vector<Int>&  vAdjEdgeIds(const GridNode& n) const;
  const Vector<Int>&  vAdjEdgeIds(const Int i) const;
  bool                isValidEdge(const Int i) const;
  bool                hasNode(const GridNode& n) const;
  bool                hasEdge(const GridEdge& n) const;
  Int                 numGridNodes() const { return _vGridNodes.size(); }
  Int                 numGridEdges() const { return _vGridEdges.size(); }
  
  // rl routing funcs
  //Pin&                rlPin(const Int i)                   { return *_vpRLPins.at(i); }
  //Pin&                rlPin(const String& n)               { return *_vpRLPins.at(name2RLPinIdx(n)); }
  //const Pin&          rlPin(const Int i)             const { return *_vpRLPins.at(i); }
  //const Pin&          rlPin(const String& n)         const { return *_vpRLPins.at(name2RLPinIdx(n)); }
  //Pin*                pRLPin(const Int i)                  { return _vpRLPins.at(i).get(); }
  //Pin*                pRLPin(const String& n)              { return _vpRLPins.at(name2RLPinIdx(n)).get(); }
  //const Pin*          pRLPin(const Int i)            const { return _vpRLPins.at(i).get(); }
  //const Pin*          pRLPin(const String& n)        const { return _vpRLPins.at(name2RLPinIdx(n)).get(); }
  //Int                 name2RLPinIdx(const String& n) const { return _mRLPinName2Idx.at(n); }
  //Int                 numRLPins()                    const { return _vpRLPins.size(); }
  //bool                hasRLPin(const String& n)      const { return _mRLPinName2Idx.find(n) != _mRLPinName2Idx.end(); }

  Box<Int>            pathMatchBbox() const;
  Box<Int>            pathMatchBbox(const RoutePathMatchCstr& cstr) const;
  //bool                insertRLVia(Net& net, const Box<Int>& bbox, const Int metalLayerIdx, const Int xSignalTrack, const Int ySignalTrack, const Orient2dE orient = Orient2dE::n);
  //bool                insertRLVia(Net& net, const Point3d<Int>& u, const Orient2dE orient = Orient2dE::n);
  //void                saveRLVias2NetResults();
  //void                clearRLPins();
  Point3d<Int>        pinSignalTrackPt(const Net& net, const Int pinIdx, const Box<Int>& bbox) const; 
  Point3d<Int>        signalTrackPt(Int px, Int py, const Int layerIdx, const Box<Int>& bbox) const;
  Pair<Int, Int>      numSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const;
  Int                 numXSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const;
  Int                 numYSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const;

  void                resetRouting();
  
  // for rl points
  //Vector<String>                     rlDataNetNames() const;
  //Vector<Point3d<Int>>               rlData(const Box<Int>& bbox, const Net& net) const;
  //Vector<Pair<String, Point3d<Int>>> rlData(const Box<Int>& bbox) const;

private:
  CirDB&    _cir;
  
  Vector<GridNode>             _vGridNodes;
  Vector<GridEdge>             _vGridEdges;
  Vector<Vector<Int>>          _vvAdjEdgeIds; // node id -> adj edges [pre, next, down, up]
  FlatHashMap<GridNode, Int>   _mGridNode2Idx;
  FlatHashMap<GridEdge, Int>   _mGridEdge2Idx;
  SpatialMap3d<Int, Int>       _spatialEdges;
  Vector<spatial3d::b_value<Int, Int>>  _vSpatialEdgeVals;
  
  void buildGrids(const bool useM1, const Int maxRgIdx);
  Int  addGridNode(const Int x, const Int y, const Int z);
  Int  addGridEdge(const Int u, const Int v);
  void addAdjEdgeIds(const Int eId);

  // for RL-based routing
  //Vector<UniquePtr<Pin>>                _vpRLPins; // for RL-based routing
  //Vector<Point3d<Int>>                  _vRLPinCenters;
  //Vector<const Via*>                    _vpRLViaRefs;
  //Vector<Orient2dE>                     _vRLViaOrients;
  //FlatHashMap<String, Int>              _mRLPinName2Idx;

};

PROJECT_NAMESPACE_END
