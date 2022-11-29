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

#include "dbCell.hpp"
#include "dbGrid.hpp"
#include "dbVia.hpp"
#include "geo/point3d.hpp"
#include "geo/segment3d.hpp"
#include "geo/spatial3d.hpp"
#include "ds/hash.hpp"
#include "ds/disjointSet.hpp"
#include "util/util.hpp"
#include "db/cstr/dbRoutePathMatchCstr.hpp"


PROJECT_NAMESPACE_START

class Net;
class Routable;
class Pin;

class TopoTree {
  using SpatialSegCmp = spatial3d::B_value_box_equal<Int, Int>;
  
public:
  
  using Wire = Pair<Box<Int>, Int>;

  struct SegData {
    SegData(const Via* p = nullptr, Orient2dE o = Orient2dE::undef, Int w = 0, Int e = 0, Int r = -1)
      : pVia(p), orient(o), width(w), ext(w), roIdx(r) {}

    ~SegData() {}

    const Via& via() const { assert(pVia); return *pVia; }

    const Via*    pVia;
    Orient2dE     orient;
    Int           width;
    Int           ext;
    Int           roIdx;

    bool operator == (const SegData& s) const {
      return std::tie(pVia, orient, width, ext) == std::tie(s.pVia, s.orient, s.width, s.ext);
    }
    bool operator != (const SegData& s) const {
      return !(*this == s);
    }
    friend std::ostream& operator << (std::ostream& os, const SegData& s) {
      os << (s.pVia ? s.pVia->name() : "null") << " "
         << util::enumUtil::val2Str(Orient2dEStr, s.orient) << " " << s.width << " " << s.ext << " " << s.roIdx << std::endl;
      return os;
    }
  };

  TopoTree()
    : _pNet(nullptr), _pRo(nullptr), _length(0), _length2d(0) {}
  ~TopoTree() {}
 
  Net&                              net()                                           { return *_pNet; }
  const Net&                        net()                                     const { return *_pNet; }
  Net*                              pNet()                                          { return _pNet; }
  const Net*                        pNet()                                    const { return _pNet; }
  
  Routable&                         ro()                                            { return *_pRo; }
  const Routable&                   ro()                                      const { return *_pRo; }
  Routable*                         pRo()                                           { return _pRo; }
  const Routable*                   pRo()                                     const { return _pRo; }

  Pin&                              pin(const Int i)                                { return *_vpPins.at(i); }
  const Pin&                        pin(const Int i)                          const { return *_vpPins.at(i); }
  Pin*                              pPin(const Int i)                               { return _vpPins.at(i); }
  const Pin*                        pPin(const Int i)                         const { return _vpPins.at(i); }
  Vector<Pin*>&                     vpPins()                                        { return _vpPins; }
  const Vector<Pin*>&               vpPins()                                  const { return _vpPins; }
  Int                               numPins()                                 const { return _vpPins.size(); }
  void                              addPinPtr(Pin* p)                               { _vpPins.emplace_back(p); }
  void                              setNetPtr(Net* p)                               { _pNet = p; }
  void                              setRoPtr(Routable* p)                           { _pRo = p; }

  bool                              hasSeg()                                  const { return _spatialSegs.size() > 0; }
  Int                               numSegs()                                 const { return _spatialSegs.size(); }
  Int                               length()                                  const { return _length; }
  Int                               length2d()                                const { return _length2d; }
  Int                               numVias()                                 const { return (_length - _length2d) / 2; }
  Box<Int>&                         bbox()                                          { return _bbox; }
  const Box<Int>&                   bbox()                                    const { return _bbox; }

  void                              init();
  void                              vRoutes(Vector<Segment3d<Int>>& vSegs) const;
  void                              vRoutes(Vector<Segment3d<Int>>& vSegs, Vector<SegData>& vSegData) const;
  void                              querySegs(const Point3d<Int>& minCorner,
                                              const Point3d<Int>& maxCorner,
                                              Vector<Segment3d<Int>>& vSegs,
                                              Vector<SegData>& vSegData,
                                              spatial3d::QueryType qt = spatial3d::QueryType::intersects) const;
  void                              vAdjSegs(const Segment3d<Int>& seg,
                                             Vector<Segment3d<Int>>& vAdjs) const;
  void                              buildSpatialPins();
  void                              addRoute(const Point3d<Int>& u, const Point3d<Int>& v, const SegData& segData = SegData());
  void                              addRoute(const Segment3d<Int>& seg, const SegData& segData = SegData());
  void                              addRoute(const Point3d<Int>& u, const Point3d<Int>& v, const SegData& segData, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs);
  void                              addRoute(const Segment3d<Int>& seg, const SegData& segData, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs);
  bool                              removeRoute(const Point3d<Int>& u, const Point3d<Int>& v, const bool bMerge = true);
  bool                              removeRoute(const Segment3d<Int>& seg, const bool bMerge = true);
  bool                              removeRoute(const Point3d<Int>& u, const Point3d<Int>& v, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool bMerge = true);
  bool                              removeRoute(const Segment3d<Int>& seg, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool bMerge = true);
  void                              removeAdjSegs(const Segment3d<Int>& seg);
  void                              removeFloating();
  void                              removeFloating(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs);
  void                              removeRedundant();
  void                              removeRedundant(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs);
  void                              refineDangling(const bool removeFloating = true, const bool removeRedundant = true);
  void                              refineDangling(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool removeFloating = true, const bool removeRedundant = true);
  void                              refineSelfLoop(); 
  void                              refineSelfLoop(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs); // remove loops, return removed segs

  bool                              isConnected()                             const ;
  Int                               numComps()                                const { return _vsCompPts.size(); }
  Int                               numCompPts(const Int i)                   const { return _vsCompPts.at(i).size(); }
  Int                               numCompSegs(const Int i)                  const { return _vsCompSegs.at(i).size(); }
  FlatHashSet<Point3d<Int>>&        compPts(const Int i)                            { return _vsCompPts.at(i); }
  const FlatHashSet<Point3d<Int>>&  compPts(const Int i)                      const { return _vsCompPts.at(i); }

  void                              vCompSegs(const Int compIdx, Vector<Segment3d<Int>>& vSegs) const;
  void                              vCompSegs(const Int compIdx, Vector<Segment3d<Int>>& vSegs, Vector<SegData>& vSegData) const;
  void                              vCompPts(const Int compIdx, Vector<Point3d<Int>>& vPts) const;
  void                              addPinAcs();
  void                              groupComps();
  void                              showComps() const;

  void                              vPaths(const Point3d<Int>& s, const Point3d<Int>& t, Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  //void                              vPaths(const Point3d<Int>& s, const Point3d<Int>& t, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const;
  void                              vPaths(const Vector<Point3d<Int>>& vs, const Vector<Point3d<Int>>& vt, Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  //void                              vPaths(const Vector<Point3d<Int>>& vs, const Vector<Point3d<Int>>& vt, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const;
  void                              vPaths(const Pin& sPin, const Pin& tPin, Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  //void                              vPaths(const Pin& sPin, const Pin& tPin, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const;


  void                              clear();

private:
  Net*                                      _pNet;
  Routable*                                 _pRo;
  Vector<Pin*>                              _vpPins;
  Vector<Point3d<Int>>                      _vPoints;
  Vector<Vector<Int>>                       _vvAdjPtIds;

  Vector<Pair<Int, Int>>                    _vSegs;
  Vector<SegData>                           _vSegData;
  Vector<FlatHashSet<Point3d<Int>>>         _vsCompPts; // net components: sets of pt ids in _vPoints
  Vector<FlatHashSet<Int>>                  _vsCompSegs; // net components: sets of seg ids in _vSegs
  //Vector<FlatHashSet<Int>>                  _vsCompBoxes; // net components: sets of box ids in _net._vWires
  FlatHashMap<Point3d<Int>, Int>            _mPt2Idx; // pt -> idx in _vPoints
  FlatHashMap<Pair<Int, Int>, Int>          _mSegIds; // seg = {ptIdx0, ptIdx1}, -> idx in _vSegs
  SpatialMap3d<Int, Int, SpatialSegCmp>     _spatialSegs; // loc -> segIdx in _vSegs
  SpatialMap3d<Int, Int>                    _spatialPins; // loc -> pinIdx in _vpPins of nets
  Int                                       _length;
  Int                                       _length2d;
  Box<Int>                                  _bbox;
  
  ///////////////////////////////////////////////
  //            helper funcs                   //
  ///////////////////////////////////////////////
  Int   addPoint(const Point3d<Int>& pt);
  Int   addSeg(const Pair<Int, Int>& p, const SegData& segData);
  void  insertSeg(const Int p0Idx, const Int p1Idx, const SegData& segData);
  bool  removeSeg(const Point3d<Int>& p0, const Point3d<Int>& p1, const Int segIdx);
  bool  bTouchPin(const Point3d<Int>& pt) const;
  bool  bTouchPin(const Point3d<Int>& pt, const Pin& pin) const;
  void  dfsDeg(const Int u, const Int p, Vector<Int>& par, Vector<char>& vis, Vector<char>& deg, Queue<Int>& q) const;
  void  dfsCycle(const Int u, const Int p, Vector<Int>& par, Vector<char>& color, Vector<Vector<Pair<Int, Int>>>& vvEdges) const;
  void  dfsPaths(const Int u, const Int t, Vector<Int>& prefix, Vector<char>& vis, Vector<Vector<Int>>& paths) const;
  void  dfsPins(const Int u, Vector<char>& vis, FlatHashSet<Pin*>& pins) const;
  void  calcConnectivity(DisjointSet& ds, Vector<Vector<Int>>& vvPinAdjs) const;
};

class Routable {
public:

  Routable (const Int idx = -1,
            Net*  pNet = nullptr,
            Net*  pSymNet = nullptr,
            const Int symNetRoutableIdx = -1,
            const bool isSelfSym = false,
            const bool isRouted = false)
    : _idx(idx),
      _pNet(pNet),
      _pSymNet(pSymNet),
      _symNetRoutableIdx(symNetRoutableIdx),
      _isSelfSym(isSelfSym),
      _isRouted(isRouted)
  {}
  ~Routable () {}

  using Wire = Pair<Box<Int>, Int>;
  using Arc = Segment3d<Int>;
  using ArcData = TopoTree::SegData;


  Int                       idx()                       const { return _idx; }
  
  Net&                      net()                             { return *_pNet; }
  const Net&                net()                       const { return *_pNet; }
  Net*                      pNet()                            { return _pNet; }
  const Net*                pNet()                      const { return _pNet; }
  Net&                      symNet()                          { return *_pSymNet; }
  const Net&                symNet()                    const { return *_pSymNet; }
  Net*                      pSymNet()                         { return _pSymNet; }
  const Net*                pSymNet()                   const { return _pSymNet; }

  Pin&                      pin(const Int i)                  { return *_vpPins.at(i); }
  const Pin&                pin(const Int i)            const { return *_vpPins.at(i); }
  Pin*                      pPin(const Int i)                 { return _vpPins.at(i); }
  const Pin*                pPin(const Int i)           const { return _vpPins.at(i); }
  Vector<Pin*>&             vpPins()                          { return _vpPins; }
  const Vector<Pin*>&       vpPins()                    const { return _vpPins; }
  Int                       numPins()                   const { return _vpPins.size(); }

  Int                       routableIdx(const Int i)    const { return _vRoutableIds.at(i); }
  Vector<Int>&              vRoutableIds()                    { return _vRoutableIds; }
  const Vector<Int>&        vRoutableIds()              const { return _vRoutableIds; }
  Int                       numRoutables()              const { return _vRoutableIds.size(); }

  //Int                       wireIdx(const Int i)        const { return _vWireIds.at(i); }
  //Vector<Int>&              vWireIds()                        { return _vWireIds; }
  //const Vector<Int>&        vWireIds()                  const { return _vWireIds; }
  //Int                       numWires()                  const { return _vWireIds.size(); }
  FlatHashSet<Wire>&        sWires()                          { return _sWires; }
  const FlatHashSet<Wire>&  sWires()                    const { return _sWires; }
  Int                       numWires()                  const { return _sWires.size(); }
  bool                      hasWire(const Wire& w)      const { return _sWires.find(w) != _sWires.end(); }
  //void                      vWires(Vector<Wire>& v)     const { std::copy(_sWires.begin(), _sWires.end(), std::back_inserter(v)); }
  Vector<Wire>              vWires()                    const { Vector<Wire> v; std::copy(_sWires.begin(), _sWires.end(), std::back_inserter(v)); return v; }
  
  //Int                       vArcIdx(const Int i)        const { return _vArcIds.at(i); }
  //Vector<Int>&              vArcIds()                         { return _vArcIds; }
  //const Vector<Int>&        vArcIds()                   const { return _vArcIds; }
  //Int                       numArcs()                   const { return _vArcIds.size(); }
  //bool                      hasArc()                    const { return _vArcIds.size() > 0; }
  FlatHashSet<Arc>&         arcs()                            { return _sArcs; }
  const FlatHashSet<Arc>&   arcs()                      const { return _sArcs; }
  Int                       numArcs()                   const { return _sArcs.size(); }
  bool                      hasArc(const Arc& a)        const { return _sArcs.find(a) != _sArcs.end(); }
  void                      vArcs(Vector<Arc>& v)       const { std::copy(_sArcs.begin(), _sArcs.end(), std::back_inserter(v)); }
  
  
  Int                       symNetRoutableIdx()         const { return _symNetRoutableIdx; }
  bool                      hasSymRoutable()            const { return _symNetRoutableIdx != -1; }
  bool                      isSelfSym()                 const { return _isSelfSym; }
  bool                      isRouted()                  const { return _isRouted; }

  TopoTree&                 topo()                            { return _topo; }
  const TopoTree&           topo()                      const { return _topo; }

  void                      init();
  void                      setIdx(const Int i)               { _idx = i; }
  void                      setNet(Net& n)                    { _pNet = &n; }
  void                      setNetPtr(Net* n)                 { _pNet = n; }
  void                      setSymNet(Net& n)                 { _pSymNet = &n; }
  void                      setSymNetPtr(Net* n)              { _pSymNet = n; }
  void                      setSymNetRoutableIdx(const Int i) { _symNetRoutableIdx = i; }
  void                      setSelfSym(const bool b = true)   { _isSelfSym = b; }
  void                      setRouted(const bool b = true)    { _isRouted = b; }
  void                      addPin(Pin& p)                    { _vpPins.emplace_back(&p); }
  void                      addPinPtr(Pin* p)                 { _vpPins.emplace_back(p); }
  void                      addRoutableIdx(const Int i)       { _vRoutableIds.emplace_back(i); }
  //void                      addWireIdx(const Int i)           { _vWireIds.emplace_back(i); }
  void                      addWire(const Wire& w)            { _sWires.emplace(w); }
  void                      clearRouting();
  void                      addRoute(const Arc& arc, const ArcData& arcData);
  bool                      removeRoute(const Arc& arc);
  bool                      removeFloatingRoutes();
  bool                      removeFloatingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs);
  bool                      refineDanglingRoutes();
  bool                      refineDanglingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs);

private:
  Int               _idx;
  Net*              _pNet;
  Net*              _pSymNet;
  Int               _symNetRoutableIdx;
  bool              _isSelfSym;
  bool              _isRouted;
  Vector<Pin*>      _vpPins;
  Vector<Int>       _vRoutableIds; // idx of the containing routables in _vRoutables in net
  //Vector<Int>       _vWireIds; // idx of _vWires in net
  FlatHashSet<Wire> _sWires;
  //Vector<Int>   _vArcIds; // idx of _vArcs in net
  FlatHashSet<Arc>  _sArcs;
  TopoTree          _topo;

  //Vector<Vector<Arc>>           _vCompArcs;
  //Vector<Vector<Point3d<Int>>>  _vCompPts;
};

//class CirDB;
class Net {
  friend class Parser;
  
  //using SpatialSegCmp = spatial3d::B_value_box_equal<Int, Int>;

public:

  Net() :
    //_pCir(nullptr),
    _idx(-1), _type(NetTypeE::signal), _pRegion(nullptr), _priority(1),
    _ioLoc(NetIOLocE::undef), _pIORouteGrid(nullptr),
    _minPinLayerIdx(MAX_INT), _maxPinLayerIdx(0), 
    _minWireLayerIdx(MAX_INT), _maxWireLayerIdx(0), 
    _pSymNet(nullptr), _isSelfSym(false),
    _symAxis(SymAxisE::undef), _symAxisX(0), _symAxisY(0),
    _pRoutePathMatchCstr(nullptr),
    _isRouted(false), _numFails(0)
  {}
  ~Net() {}
  
  //using Arc = Pair<Point3d<Int>, Point3d<Int>>;
  using Arc = Segment3d<Int>;
  using ArcData = TopoTree::SegData;

  using Wire = Pair<Box<Int>, Int>;

  //CirDB&                                                cir()                                             { return *_pCir; }
  //const CirDB&                                          cir()                                       const { return *_pCir; }
  //CirDB*                                                pCir()                                            { return _pCir; }
  //const CirDB*                                          pCir()                                      const { return _pCir; }

  const String&                                         name()                                      const { return _name; }
  Int                                                   idx()                                       const { return _idx; }

  Pin&                                                  pin(const Int i)                                  { return *_vpPins.at(i); }
  Pin&                                                  pin(const String& n)                              { return *_vpPins.at(name2PinIdx(n)); }
  const Pin&                                            pin(const Int i)                            const { return *_vpPins.at(i); }
  const Pin&                                            pin(const String& n)                        const { return *_vpPins.at(name2PinIdx(n)); }
  Pin*                                                  pPin(const Int i)                                 { return _vpPins.at(i); }
  Pin*                                                  pPin(const String& n)                             { return _vpPins.at(name2PinIdx(n)); }
  const Pin*                                            pPin(const Int i)                           const { return _vpPins.at(i); }
  const Pin*                                            pPin(const String& n)                       const { return _vpPins.at(name2PinIdx(n)); }
  Int                                                   name2PinIdx(const String& n)                const { return _mPinName2Idx.at(n); }
  Int                                                   numPins()                                   const { return _vpPins.size(); }
  const Vector<Pin*>&                                   vpPins()                                    const { return _vpPins; }
  bool                                                  hasPin(const String& n)                     const { return _mPinName2Idx.find(n) != _mPinName2Idx.end(); }

  NetTypeE                                              type()                                      const { return _type; }
  bool                                                  isPower()                                   const { return isVdd() or isVss(); }
  bool                                                  isVdd()                                     const { return _type == NetTypeE::vdd; }
  bool                                                  isVss()                                     const { return _type == NetTypeE::vss; }
  bool                                                  isSignal()                                  const { return _type == NetTypeE::signal; }
  bool                                                  isClock()                                   const { return _type == NetTypeE::clock; }
  bool                                                  isCritical()                                const { return _type == NetTypeE::critical; }

  // FIXME: need to remove the region concept from net
  Region&                                               region()                                          { return *_pRegion; }
  const Region&                                         region()                                    const { return *_pRegion; }
  Region*                                               pRegion()                                         { return _pRegion; }
  const Region*                                         pRegion()                                   const { return _pRegion; }

  Int                                                   priority()                                  const { return _priority; }
  
  bool                                                  isLayerValid(const Int i)                   const { return i < static_cast<Int>(_vLayerValids.size()) ? _vLayerValids.at(i) : false; }
  Int                                                   horTracks(const Int i)                      const { return _vLayerHorTracks.at(i); }
  Int                                                   verTracks(const Int i)                      const { return _vLayerVerTracks.at(i); }
  Int                                                   layerPriority(const Int i)                  const { return _vLayerPriorities.at(i); }


  // for constraint: pin assignment
  bool                                                  isIO()                                      const { return _ioLoc != NetIOLocE::undef; }
  NetIOLocE                                             ioLoc()                                     const { return _ioLoc; }
  const RouteGrid&                                      ioRouteGrid()                               const { return *_pIORouteGrid; }
  const RouteGrid*                                      pIORouteGrid()                              const { return _pIORouteGrid; }
  Pin&                                                  ioPin()                                           { assert(isIO()); return *_vpPins.back(); }
  const Pin&                                            ioPin()                                     const { assert(isIO()); return *_vpPins.back(); }
  Pin*                                                  pIOPin()                                          { assert(isIO()); return _vpPins.back(); }
  const Pin*                                            pIOPin()                                    const { assert(isIO()); return _vpPins.back(); }

  Box<Int>&                                             bbox()                                            { return _bbox; }
  const Box<Int>&                                       bbox()                                      const { return _bbox; }
  Int                                                   minPinLayerIdx()                            const { return _minPinLayerIdx; }
  Int                                                   maxPinLayerIdx()                            const { return _maxPinLayerIdx; }
  Box<Int>&                                             wireBbox()                                        { return _wireBbox; }
  const Box<Int>&                                       wireBbox()                                  const { return _wireBbox; }
  Int                                                   minWireLayerIdx()                           const { return _minWireLayerIdx; }
  Int                                                   maxWireLayerIdx()                           const { return _maxWireLayerIdx; }

  // route
  Int                                                   numArcs()                                   const { return _mArcs.size(); }
  bool                                                  hasArc(const Arc& a)                        const { return _mArcs.find(a) != _mArcs.end(); }
  FlatHashMap<Arc, ArcData>&                            mArcs()                                           { return _mArcs; }
  const FlatHashMap<Arc, ArcData>&                      mArcs()                                     const { return _mArcs; }
  void                                                  vArcs(Vector<Arc>& v)                       const ;
  void                                                  vArcs(Vector<Arc>& v, Vector<ArcData>& vd)  const ;

  const Via&                                            arcVia(const Arc& a)                        const { return *_mArcs.at(a).pVia; }
  const Via*                                            pArcVia(const Arc& a)                       const { return _mArcs.at(a).pVia; }
  Orient2dE                                             arcViaOrient(const Arc& a)                  const { return _mArcs.at(a).orient; }
  Int                                                   arcWidth(const Arc& a)                      const { return _mArcs.at(a).width; }
  Int                                                   arcExt(const Arc& a)                        const { return _mArcs.at(a).ext; }
  ArcData&                                              arcData(const Arc& a)                             { return _mArcs.at(a); }


  Int                                                   numWires()                                  const { return _sWires.size(); }
  bool                                                  hasWire(const Wire& w)                      const { return _sWires.find(w) != _sWires.end(); }
  FlatHashSet<Wire>&                                    sWires()                                          { return _sWires; }
  const FlatHashSet<Wire>&                              sWires()                                    const { return _sWires; }
  //void                                                  vWires(Vector<Wire>& v)                     const { std::copy(_sWires.begin(), _sWires.end(), std::back_inserter(v)); }
  Vector<Wire>                                          vWires()                                    const { Vector<Wire> v; std::copy(_sWires.begin(), _sWires.end(), std::back_inserter(v)); return v; }

  Int                                                   numPatchWires()                             const { return _sPatchWires.size(); }
  bool                                                  hasPatchWire(const Wire& w)                 const { return _sPatchWires.find(w) != _sPatchWires.end(); }
  FlatHashSet<Wire>&                                    sPatchWires()                                     { return _sPatchWires; }
  const FlatHashSet<Wire>&                              sPatchWires()                               const { return _sPatchWires; }
  //void                                                  vPatchWires(Vector<Wire>& v)                const { std::copy(_sPatchWires.begin(), _sPatchWires.end(), v.begin()); }
  Vector<Wire>                                          vPatchWires()                               const { Vector<Wire> v; std::copy(_sPatchWires.begin(), _sPatchWires.end(), v.begin()); return v; }

  Routable&                                             routable(const Int i)                             { return _vRoutables.at(i); }
  const Routable&                                       routable(const Int i)                       const { return _vRoutables.at(i); }
  Routable*                                             pRoutable(const Int i)                            { return &_vRoutables.at(i); }
  const Routable*                                       pRoutable(const Int i)                      const { return &_vRoutables.at(i); }
  Vector<Routable>&                                     vRoutables()                                      { return _vRoutables; }
  const Vector<Routable>&                               vRoutables()                                const { return _vRoutables; }
  Int                                                   numRoutables()                              const { return _vRoutables.size(); }
  bool                                                  hasRoutable()                               const { return _vRoutables.size() > 0; }
  
  Int                                                   routableOrder(const Int i)                  const { return _vRoutableOrder.at(i); }
  Vector<Int>&                                          vRoutableOrder()                                  { return _vRoutableOrder; }
  const Vector<Int>&                                    vRoutableOrder()                            const { return _vRoutableOrder; } 


  // symmetry
  Net&                                                  symNet()                                          { return *_pSymNet; }
  const Net&                                            symNet()                                    const { return *_pSymNet; }
  Net*                                                  pSymNet()                                         { return _pSymNet; }
  const Net*                                            pSymNet()                                   const { return _pSymNet; }
  bool                                                  hasSymNet()                                 const { return _pSymNet != nullptr; }
  bool                                                  isSelfSym()                                 const { return _isSelfSym; }
  SymAxisE                                              symAxis()                                   const { return _symAxis; }
  Int                                                   symAxisX()                                  const { return _symAxisX; }
  Int                                                   symAxisY()                                  const { return _symAxisY; }
  bool                                                  isVerSym()                                  const { return _symAxis == SymAxisE::ver; }
  bool                                                  isHorSym()                                  const { return _symAxis == SymAxisE::hor; }

  // path matching
  RoutePathMatchCstr&                                             routePathMatchCstr()                              { return *_pRoutePathMatchCstr; }
  const RoutePathMatchCstr&                                       routePathMatchCstr()                        const { return *_pRoutePathMatchCstr; }
  RoutePathMatchCstr*                                             pRoutePathMatchCstr()                             { return _pRoutePathMatchCstr; }
  const RoutePathMatchCstr*                                       pRoutePathMatchCstr()                       const { return _pRoutePathMatchCstr; }
  bool                                                            hasPathMatch()                              const { return _pRoutePathMatchCstr != 0; }
  Pin&                                                            connSrcPin(const Int i)                           { return _vConns.at(i).first->srcPin(_vConns.at(i).second); }
  const Pin&                                                      connSrcPin(const Int i)                     const { return _vConns.at(i).first->srcPin(_vConns.at(i).second); }
  Pin&                                                            connTarPin(const Int i)                           { return _vConns.at(i).first->tarPin(_vConns.at(i).second); }
  const Pin&                                                      connTarPin(const Int i)                     const { return _vConns.at(i).first->tarPin(_vConns.at(i).second); }
  RoutePathMatchCstr::PathCstr&                                   connCstr(const Int i)                             { return *_vConns.at(i).first; }
  const RoutePathMatchCstr::PathCstr&                             connCstr(const Int i)                       const { return *_vConns.at(i).first; }
  Pair<RoutePathMatchCstr::PathCstr*, Int>&                       conn(const Int i)                                 { return _vConns.at(i); }
  const Pair<RoutePathMatchCstr::PathCstr*, Int>&                 conn(const Int i)                           const { return _vConns.at(i); }
  Vector<Pair<RoutePathMatchCstr::PathCstr*, Int>>&               vConns()                                          { return _vConns; }
  const Vector<Pair<RoutePathMatchCstr::PathCstr*, Int>>&         vConns()                                    const { return _vConns; }
  Int                                                             numConns()                                  const { return _vConns.size(); }


  bool                                                  isRouted()                                  const { return _isRouted; }
  Int                                                   numFails()                                  const { return _numFails; }

  // spatial and component structure                    
  TopoTree&                                             topo()                                            { return _topo; }
  const TopoTree&                                       topo()                                      const { return _topo; }

  // rr
  void                                                  init();
  void                                                  clearRouting();
  void                                                  addRoute(const Arc& arc, const ArcData& arcData);
  bool                                                  removeRoute(const Arc& arc);
  bool                                                  removeFloatingRoutes();
  bool                                                  removeFloatingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs);
  bool                                                  refineDanglingRoutes();
  bool                                                  refineDanglingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs);


  // other
  void                                                  setPriority(const Int p)                                       { _priority = p; }
  void                                                  setType(const NetTypeE t)                                      { _type = t; }
  void                                                  setSymAxis(const SymAxisE s)                                   { _symAxis = s; }
  void                                                  setSymNet(Net& n)                                              { _pSymNet = &n; }
  void                                                  setSymNet(Net* p)                                              { _pSymNet = p; }
  void                                                  setSelfSym(const bool b = true)                                { _isSelfSym = b; }
  void                                                  setSymAxisX(const Int x)                                       { _symAxisX = x; }
  void                                                  setSymAxisY(const Int y)                                       { _symAxisY = y; }
  void                                                  setRoutePathMatchCstr(RoutePathMatchCstr* pCstr)               { _pRoutePathMatchCstr = pCstr; }
  void                                                  resetBbox()                                                    { _bbox.set(0, 0, 0, 0); }
  void                                                  setBbox(const Box<Int>& b)                                     { _bbox = b; }
  void                                                  setMinPinLayerIdx(const Int z)                                 { _minPinLayerIdx = z; }
  void                                                  setMaxPinLayerIdx(const Int z)                                 { _maxPinLayerIdx = z; }
  void                                                  resetWireBbox()                                                { _wireBbox.set(0, 0, 0, 0); }
  void                                                  setWireBbox(const Box<Int>& b)                                 { _wireBbox = b; }
  void                                                  updateWireBbox()                                               ;
  void                                                  setMinWireLayerIdx(const Int z)                                { _minWireLayerIdx = z; }
  void                                                  setMaxWireLayerIdx(const Int z)                                { _maxWireLayerIdx = z; }
  void                                                  setRouted(const bool b = true)                                 { _isRouted = b; }
  void                                                  addFail(const Int i = 1)                                       { _numFails += i; }
  void                                                  addPin(Pin& pin)                                               { _vpPins.emplace_back(&pin); }
  void                                                  addPin(Pin* pPin)                                              { _vpPins.emplace_back(pPin); }

  // visualize

private:
  //CirDB*                                          _pCir;
  String                                          _name;
  Int                                             _idx;
  Vector<Pin*>                                    _vpPins;
  FlatHashMap<String, Int>                        _mPinName2Idx;
  NetTypeE                                        _type;
  Region*                                         _pRegion;
  Int                                             _priority;
  
  // net assignment cstr
  Vector<Byte>                                    _vLayerValids;
  Vector<Int>                                     _vLayerHorTracks;
  Vector<Int>                                     _vLayerVerTracks;
  Vector<Int>                                     _vLayerPriorities;

  // io pin assignment                            
  NetIOLocE                                       _ioLoc;
  const RouteGrid*                                _pIORouteGrid;

  Box<Int>                                        _bbox; // for net ordering (exclude io pin)
  Int                                             _minPinLayerIdx;
  Int                                             _maxPinLayerIdx;
  Box<Int>                                        _wireBbox; // bbox of real shapes
  Int                                             _minWireLayerIdx;
  Int                                             _maxWireLayerIdx;

  // route
  FlatHashMap<Arc, ArcData>                       _mArcs;
  //FlatHashMap<Arc, Int>                           _mArc2RoutableIdx;
  //Vector<Arc>                                     _vArcs;
  //Vector<const Via*>                              _vpArcVias;
  //Vector<Orient2dE>                               _vArcViaOrients;
  //Vector<Pair<Int, Int>>                          _vArcWidthExts;
  FlatHashSet<Wire>                               _sWires;
  FlatHashSet<Wire>                               _sPatchWires;
  //Vector<Wire>                                    _vWires;
  //Vector<Wire>                                    _vPatchWires;

  // for RL-based matching routing
  //Vector<Int>                                     _vRLViaIds;

  Vector<Routable>                                _vRoutables;
  Vector<Int>                                     _vRoutableOrder;

  // sym cstr
  Net*                                            _pSymNet;
  bool                                            _isSelfSym;
  SymAxisE                                        _symAxis;
  Int                                             _symAxisX;
  Int                                             _symAxisY;

  // path match cstr
  RoutePathMatchCstr*                               _pRoutePathMatchCstr;
  Vector<Pair<RoutePathMatchCstr::PathCstr*, Int>>  _vConns;

  bool                                            _isRouted;
  Int                                             _numFails;

  // spatial and component tructure
  TopoTree                                        _topo;
  
  ///////////////////////////////////////////////
  //            helper funcs                   //
  ///////////////////////////////////////////////
};

#define Routable_ForEachPin(ro, pPin_, i) \
  for (i = 0; i < ro.numPins() and (pPin_ = ro.pPin(i)); ++i)

#define Net_ForEachPin(net, pPin_, i) \
  for (i = 0; i < net.numPins() and (pPin_ = net.pPin(i)); ++i)

#define Net_ForEachRoutable(net, pRoutable_, i) \
  for (i = 0; i < net.numRoutables() and (pRoutable_ = net.pRoutable(i)); ++i)




PROJECT_NAMESPACE_END
