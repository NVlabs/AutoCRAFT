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

#include "rl/rlUtil.hpp"

#include <limbo/parsers/gdsii/stream/GdsWriter.h>

PROJECT_NAMESPACE_START

void RlUtils::genFeatsEdgesMasksImpl(const CirDB& cir, const Net& net, const Vector<Segment3d<Int>>& vSegs,
                                     const DrPsRouter& ps, const DrcMgr& drcMgr, 
                                     Vector<Vector<Real>>& feats, Vector<Vector<Int>>& edges, Vector<Real>& masks,
                                     const Real normConstWL, const Real normConstHis) const
{
  // gen feats
  feats.clear();
  feats.resize(vSegs.size());
  
  const auto& bbox = net.topo().bbox();

  //const Real rangeX = bbox.width() + 1;
  //const Real rangeY = bbox.height() + 1;
  //const Real rangeZ = cir.routeGrid(cir.numRouteGrids() - 1).upperLayerIdx();
  //const Real rangeZ = 20;
  const Real rangeZ = cir.numLayers();

  const Real originX = bbox.xl();
  const Real originY = bbox.yl();

  for (size_t i = 0; i < vSegs.size(); ++i) {
    const auto& seg = vSegs.at(i);
    const auto& u = seg.p0();
    const auto& v = seg.p1();

    assert(u.z() % 2 == 0 and v.z() % 2 == 0);
    assert(u.x() <= v.x() and u.y() <= v.y() and u.z() <= v.z());

    assert(bbox.xl() <= u.x() and u.x() <= bbox.xh());
    assert(bbox.yl() <= u.y() and u.y() <= bbox.yh());
    assert(bbox.xl() <= v.x() and v.x() <= bbox.xh());
    assert(bbox.yl() <= v.y() and v.y() <= bbox.yh());

    //const Real ux = static_cast<Real>(u.x() - originX) / rangeX;
    //const Real uy = static_cast<Real>(u.y() - originY) / rangeY;
    //const Real vx = static_cast<Real>(v.x() - originX) / rangeX;
    //const Real vy = static_cast<Real>(v.y() - originY) / rangeY;
    //const Real len = static_cast<Real>(seg.length()) / bbox.hpwl();
    const Real ux = static_cast<Real>(u.x() - originX) / normConstWL;
    const Real uy = static_cast<Real>(u.y() - originY) / normConstWL;
    const Real vx = static_cast<Real>(v.x() - originX) / normConstWL;
    const Real vy = static_cast<Real>(v.y() - originY) / normConstWL;
    const Real len = static_cast<Real>(seg.length()) / normConstWL;
    const Real layerIdx = seg.bVia() ? static_cast<Real>(seg.zl() + 1) / rangeZ : static_cast<Real>(u.z()) / rangeZ;
    const Real via = seg.bVia() ? 1. : 0.;

    // history
    //const Real his = static_cast<Real>(ps.history(seg, 2, 0.8)) / ps.hisCost() / bbox.hpwl();
    Vector<Real> vHis(11, 10);
    vHis[5] = static_cast<Real>(ps.history(seg)) / ps.hisCost() / normConstHis;

    const Int rgIdx = (u.z() / 2 < cir.numRouteGrids()) ? u.z() / 2 : u.z() / 2 - 1;
    const RouteGrid& rg = cir.routeGrid(rgIdx);
    if (seg.bHor()) {
      Int y = u.y();
      for (Int i = 1; i <= 5; ++i) {
        y = rg.prevSignalY(y);
        if (cir.routingBbox().yl() <= y and y <= cir.routingBbox().yh()) {
          vHis[5 - i] = static_cast<Real>(ps.history(Segment3d<Int>(u.x(), y, u.z(), v.x(), y, v.z()))) / ps.hisCost() / normConstHis;
        }
        else break;
      }
      y = u.y();
      for (Int i = 1; i <= 5; ++i) {
        y = rg.nextSignalY(y);
        if (cir.routingBbox().yl() <= y and y <= cir.routingBbox().yh()) {
          vHis[5 + i] = static_cast<Real>(ps.history(Segment3d<Int>(u.x(), y, u.z(), v.x(), y, v.z()))) / ps.hisCost() / normConstHis;
        }
        else break;
      }
    }
    else if (seg.bVer()) {
      Int x = u.x();
      for (Int i = 1; i <= 5; ++i) {
        x = rg.prevSignalX(x);
        if (cir.routingBbox().xl() <= x and x <= cir.routingBbox().xh()) {
          vHis[5 - i] = static_cast<Real>(ps.history(Segment3d<Int>(x, u.y(), u.z(), x, v.y(), v.z()))) / ps.hisCost() / normConstHis;
        }
        else break;
      }
      x = u.x();
      for (Int i = 1; i <= 5; ++i) {
        x = rg.nextSignalX(x);
        if (cir.routingBbox().xl() <= x and x <= cir.routingBbox().xh()) {
          vHis[5 + i] = static_cast<Real>(ps.history(Segment3d<Int>(x, u.y(), u.z(), x, v.y(), v.z()))) / ps.hisCost() / normConstHis;
        }
        else break;
      }
    }


    Real isConnected2Pin = 0;
    const Point<Int> fu(u.x(), u.y());
    const Point<Int> fv(v.x(), v.y());
    Vector<Pin*> vTouchedPins;
    cir.spatialPins(u.z()).query(fu, fu, vTouchedPins);
    cir.spatialPins(v.z()).query(fv, fv, vTouchedPins);
    for (const Pin* pPin : vTouchedPins) {
      if (pPin->pNet() == &net) {
        isConnected2Pin = 1;
        break;
      }
    }


    Vector<Pair<Box<Int>, Int>> vWires;
    ps.queryNetSegWires(net, seg, vWires);

    Real drv = 0;
    for (const auto& wire : vWires) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (cir.layer(layerIdx).isMetal()) {
        if (!drcMgr.checkWireValidLength(layerIdx, box) or
            !drcMgr.checkWireMetalPrlSpacing(net, layerIdx, box, 0) or 
            !drcMgr.checkWireMetalEolSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
      else {
        if (!drcMgr.checkWireCutSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
    }
    drv /= 10;

    //feats[i] = {ux, uy, vx, vy, len, layerIdx, via, isConnected2Pin, his, drv};
    feats[i] = {ux, uy, vx, vy, len, layerIdx, via, isConnected2Pin, drv};
    feats[i].insert(feats[i].end(), vHis.begin(), vHis.end());
  }

  // gen edges
  edges.clear();
  for (Int i = 0; i < static_cast<Int>(vSegs.size()); ++i) {
    const auto& si = vSegs.at(i);
    assert(si.b90());
    for (Int j = i + 1; j < static_cast<Int>(vSegs.size()); ++j) {
      const auto& sj = vSegs.at(j);
      assert(sj.b90());
      if (Segment3d<Int>::bConnect(si, sj)) {
        edges.emplace_back((std::initializer_list<Int>){i, j});
        edges.emplace_back((std::initializer_list<Int>){j, i});
      }
    }
  }

  // gen masks
  masks.clear();
  masks.assign(vSegs.size(), 1.);
  for (size_t i = 0; i < vSegs.size(); ++i) {
    const auto& seg = vSegs.at(i);
    if (seg.bHor() or seg.bVer()) { // consider hor and ver segs only (without vias)
      masks[i] = 0.;
    }
  }
}

void RlUtils::genFeatsEdgesMasks(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                 Vector<Vector<Real>>& feats, Vector<Vector<Int>>& edges, Vector<Real>& masks,
                                 const Real normConstWL, const Real normConstHis) const
{
  Vector<Segment3d<Int>> vSegs;
  net.vArcs(vSegs);
  genFeatsEdgesMasksImpl(cir, net, vSegs, ps, drcMgr, feats, edges, masks, normConstWL, normConstHis);

}

void RlUtils::genFeatsEdgesMasksWithPathInfo(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                             Vector<Vector<Real>>& segFeats, Vector<Vector<Real>>& pathFeats,
                                             Vector<Vector<Int>>& seg2SegEdges, Vector<Vector<Int>>& path2SegEdges,
                                             Vector<Real>& segMasks,
                                             const Real normConstPathSegs,
                                             const Real normConstWL, const Real normConstVia, const Real normConstRes,
                                             const Real normConstHis) const
{
  Vector<Segment3d<Int>> vSegs;
  net.vArcs(vSegs);

  genFeatsEdgesMasksImpl(cir, net, vSegs, ps, drcMgr, segFeats, seg2SegEdges, segMasks, normConstWL, normConstHis);
  genPathFeatsEdgesImpl(cir, net, vSegs, pathFeats, path2SegEdges, normConstPathSegs, normConstWL, normConstVia, normConstRes); 
}

void RlUtils::genFeatsEdgesMasksWithSuperPathInfo(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                                  Vector<Vector<Real>>& segFeats, Vector<Vector<Real>>& pathFeats,
                                                  Vector<Vector<Int>>& seg2SegEdges, Vector<Vector<Int>>& path2SegEdges,
                                                  Vector<Real>& segMasks,
                                                  Vector<Vector<Real>>& superPathFeats, Vector<Vector<Int>>& path2SuperEdges,
                                                  const Real normConstPathSegs, 
                                                  const Real normConstWL, const Real normConstVia, const Real normConstRes,
                                                  const Real normConstHis) const
{
  Vector<Segment3d<Int>> vSegs;
  net.vArcs(vSegs);

  genFeatsEdgesMasksImpl(cir, net, vSegs, ps, drcMgr, segFeats, seg2SegEdges, segMasks, normConstWL, normConstHis);
  genPathFeatsEdgesImpl(cir, net, vSegs, pathFeats, path2SegEdges, normConstPathSegs, normConstWL, normConstVia, normConstRes); 
  genSuperPathFeatsEdgesImpl(cir, net, superPathFeats, path2SuperEdges);
}

void RlUtils::genPathFeatsEdges(const CirDB& cir, const Net& net, 
                                Vector<Vector<Real>>& pathFeats, Vector<Vector<Int>>& pathEdges,
                                const Real normConstPathSegs, const Real normConstWL, const Real normConstVia, const Real normConstRes) const
{
  Vector<Segment3d<Int>> vSegs;
  net.vArcs(vSegs);
  genPathFeatsEdgesImpl(cir, net, vSegs, pathFeats, pathEdges, normConstPathSegs, normConstWL, normConstVia, normConstRes);
}

void RlUtils::genPathFeatsEdgesImpl(const CirDB& cir, const Net& net, const Vector<Segment3d<Int>>& vSegs, 
                                    Vector<Vector<Real>>& pathFeats, Vector<Vector<Int>>& pathEdges,
                                    const Real normConstPathSegs, const Real normConstWL, const Real normConstVia, const Real normConstRes) const
{
  assert(net.hasPathMatch());
  pathFeats.clear();
  pathFeats.resize(net.numConns());
  pathEdges.clear();

  FlatHashMap<Segment3d<Int>, Int> mSegIds;
  for (const auto& seg : vSegs) {
    mSegIds.emplace(seg, mSegIds.size());
  }

  for (Int i = 0; i < net.numConns(); ++i) {
    const auto& conn = net.conn(i);
    const Pin& src = conn.first->srcPin(conn.second);
    const Pin& tar = conn.first->tarPin(conn.second);

    Vector<Vector<Segment3d<Int>>> vPaths;
    net.topo().vPaths(src, tar, vPaths);

    const Int numPaths = vPaths.size();


    Real numPathSegs = 0, len = 0, via = 0, res = 0;
    if (numPaths > 0) {
      Int pathIdx = 0;
      if (numPaths > 1) {
        Int minPathRes = cir.pathRes(vPaths.at(pathIdx));
        for (Int j = 1; j < numPaths; ++j) {
          const Int r = cir.pathRes(vPaths.at(j));
          if (r < minPathRes) {
            minPathRes = r;
            pathIdx = j;
          }
        }
      }
      const auto& path = vPaths.at(pathIdx);

      numPathSegs = path.size();
      for (const auto& seg : path) {
        if (mSegIds.find(seg) != mSegIds.end()) {
          const Int segIdx = mSegIds.at(seg);
          pathEdges.emplace_back();
          pathEdges.back() = {i, segIdx};
        }
      }
      std::tie(len, via, res) = cir.pathStats(path);
    }
    else {
    }

    pathFeats[i] = {numPathSegs / normConstPathSegs,
                    //len / net.topo().bbox().hpwl(),
                    len / normConstWL,
                    via / normConstVia,
                    res / normConstRes};
  }
  //std::cerr << net.name() << " " << net.isRouted() << " " << pathEdges.size() << std::endl;
  //if (pathEdges.size() and net.name() == "ph13b")
  //toVisGds(cir, net, "test.gds");
}

void RlUtils::genSuperPathFeatsEdgesImpl(const CirDB& cir, const Net& net, 
                                         Vector<Vector<Real>>& superFeats,
                                         Vector<Vector<Int>>& path2SuperEdges) const
{
  superFeats.clear();
  path2SuperEdges.clear();
 
  for (Int i = 0; i < net.numConns(); ++i) {
    const auto& pathCstr = net.connCstr(i);
    path2SuperEdges.emplace_back();
    path2SuperEdges.back() = {i, pathCstr.idx()};
  }
  superFeats.resize(net.routePathMatchCstr().numPathCstrs());
}
  
  
Pair<Real, Real> RlUtils::getWlVia(const Net& net, const bool isScaled) const
{
  Real wl = 0, via = 0;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    assert(arc.b90());
    if (arc.bVia()) {
      via += 1;
    }
    else {
      wl += static_cast<Real>(arc.length());
    }
  }
  if (isScaled) {
    wl /= net.bbox().hpwl();
    //via /= 20;
  }
  return std::make_pair(wl, via);
}

Tuple<Real, Real, Int> RlUtils::getWlViaDrv(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr, const bool isScaled) const
{
  Real wl = 0, via = 0;
  Int drv = 0;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    assert(arc.b90());
    if (arc.bVia()) {
      via += 1;
    }
    else {
      wl += static_cast<Real>(arc.length());
    }

    Vector<Pair<Box<Int>, Int>> vWires;
    ps.queryNetSegWires(net, arc, vWires);

    for (const auto& wire : vWires) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (cir.layer(layerIdx).isMetal()) {
        if (!drcMgr.checkWireValidLength(layerIdx, box) or
            !drcMgr.checkWireMetalPrlSpacing(net, layerIdx, box, 0) or 
            !drcMgr.checkWireMetalEolSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
      else {
        if (!drcMgr.checkWireCutSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
    }
  }

  if (isScaled) {
    wl /= net.bbox().hpwl();
    //via /= 20;
  }

  return std::make_tuple(wl, via, drv);
}
  
void RlUtils::addCurSol2History(const Net& net, DrPsRouter& ps, const Int cost) const
{
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    ps.addHistory(arc, cost);
  }

}


Pair<Real, Real> RlUtils::getMinWlVia(Net& net, DrPsRouter& ps, const bool isScaled) const
{
  bool done = false;

  ps.resetHistory();
  if (!done) {
    ps.addUnroutedNet(net);
    done = ps.runNrrCore(NetTypeE::critical, false, false, ps.drcCost(), ps.hisCost()); 
  }
  if (!done) {
    ps.addUnroutedNet(net);
    done = ps.runNrrCore(NetTypeE::critical, false, false, ps.drcCost(), ps.hisCost()); 
  }
  if (!done) {
    ps.addUnroutedNet(net);
    done = ps.runNrrCore(NetTypeE::critical, false, false, ps.drcCost(), ps.hisCost()); 
  }
  if (!done) {
    ps.addUnroutedNet(net);
    done = ps.runNrrCore(NetTypeE::critical, false, false, ps.drcCost(), ps.hisCost()); 
  }
  if (!done) {
    ps.addUnroutedNet(net);
    done = ps.runNrrCore(NetTypeE::critical, false, false, ps.drcCost(), ps.hisCost()); 
  }

  auto minWlVia = getWlVia(net, isScaled); 

  ps.ripup(net);
  ps.resetHistory();

  return minWlVia;
}

Pair<Real, Real> RlUtils::getLayersWlVia(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                         Vector<Real>& vWls, Vector<Real>& vVias, const bool isScaled) const
{
  vWls.assign(cir.numMetalLayers(), 0);
  vVias.assign(cir.numCutLayers(), 0);
 
  Real wl = 0, via = 0;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    assert(arc.b90());
    if (arc.bVia()) {
      const Int cutLayerIdx = cir.layer(arc.zl() + 1).selfIdx();
      vVias.at(cutLayerIdx) += 1;
      ++via;
    }
    else {
      const Int metalLayerIdx = cir.layer(arc.p0().z()).selfIdx();
      vWls.at(metalLayerIdx) += static_cast<Real>(arc.length());
      wl += static_cast<Real>(arc.length());
    }
  }
  if (isScaled) {
    for (auto& wl : vWls) {
      wl /= net.bbox().hpwl();
    }
    //via /= 20;
  }
  return std::make_pair(wl, via);
}


Tuple<Real, Real, Int> RlUtils::getLayersWlViaDrv(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                                  Vector<Real>& vWls, Vector<Real>& vVias, const bool isScaled) const
{
  vWls.assign(cir.numMetalLayers(), 0);
  vVias.assign(cir.numCutLayers(), 0);

  Real wl = 0, via = 0;
  Int drv = 0;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    assert(arc.b90());
    if (arc.bVia()) {
      const Int cutLayerIdx = cir.layer(arc.zl() + 1).selfIdx();
      vVias.at(cutLayerIdx) += 1;
      ++via;
    }
    else {
      const Int metalLayerIdx = cir.layer(arc.p0().z()).selfIdx();
      vWls.at(metalLayerIdx) += static_cast<Real>(arc.length());
      wl += static_cast<Real>(arc.length());
    }
    
    Vector<Pair<Box<Int>, Int>> vWires;
    ps.queryNetSegWires(net, arc, vWires);
    for (const auto& wire : vWires) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (cir.layer(layerIdx).isMetal()) {
        if (!drcMgr.checkWireValidLength(layerIdx, box) or
            !drcMgr.checkWireMetalPrlSpacing(net, layerIdx, box, 0) or 
            !drcMgr.checkWireMetalEolSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
      else {
        if (!drcMgr.checkWireCutSpacing(net, layerIdx, box)) {
          ++drv;
        }
      }
    }
  }

  if (isScaled) {
    for (auto& wl : vWls) {
      wl /= net.bbox().hpwl();
    }
    //via /= 20;
  }
  return std::make_tuple(wl, via, drv);
}

Vector<Real> RlUtils::getPathRes(const CirDB& cir, const Net& net) const
{
  Vector<Real> vRes(net.numConns(), 0);
  for (Int i = 0; i < net.numConns(); ++i) {
    const auto& conn = net.conn(i);
    const Pin& src = conn.first->srcPin(conn.second);
    const Pin& tar = conn.first->tarPin(conn.second);
    vRes[i] = cir.pathRes(src, tar);
  }
  return vRes;
}

void RlUtils::getSurroundingImage(const CirDB& cir, const Net& net, const DrPsRouter& ps,
                                  const Int xSize, const Int ySize,
                                  const Int xStep, const Int yStep,
                                  const Real expandFc, Vector<Vector<Vector<Vector<Real>>>>& image) const
{
  image.assign(xSize, Vector<Vector<Vector<Real>>>(ySize, Vector<Vector<Real>>(cir.numMetalLayers(), Vector<Real>(3, 0))));
  // net: channel 0; obstacle: channel 1; history: channel 2

  auto bbox = net.wireBbox();
  bbox.expandX(bbox.width() * expandFc);
  bbox.expandY(bbox.height() * expandFc);
  bbox.set(std::max(bbox.xl(), cir.routingBbox().xl()),
           std::max(bbox.yl(), cir.routingBbox().yl()),
           std::min(bbox.xh(), cir.routingBbox().xh()),
           std::min(bbox.yh(), cir.routingBbox().yh()));

  auto add = [&] (const Int channel, const Box<Int>& b, const Int z, const Real val, const bool isIncre) {
    const Box<Int> box(std::max(0,             b.xl() - bbox.xl()),
                       std::max(0,             b.yl() - bbox.yl()),
                       std::min(bbox.width(),  b.xh() - bbox.xl()),
                       std::min(bbox.height(), b.yh() - bbox.yl()));

    //if (box.width() == 0 or box.height() == 0) {
      //return;
    //}

    const Int xl = box.xl() / xStep, xh = (box.xh() % xStep == 0) ? box.xh() / xStep - 1 : box.xh() / xStep;
    const Int yl = box.yl() / yStep, yh = (box.yh() % yStep == 0) ? box.yh() / yStep - 1 : box.yh() / yStep;
    assert(0 <= xl and xh < xSize);
    assert(0 <= yl and yh < ySize);
    if (isIncre) {
      for (Int x = xl; x <= xh; ++x) {
        for (Int y = yl; y <= yh; ++y) {
          image[x][y][z][channel] += val;
        }
      }
    }
    else {
      for (Int x = xl; x <= xh; ++x) {
        for (Int y = yl; y <= yh; ++y) {
          image[x][y][z][channel] = val;
        }
      }
    }
  };

  for (Int i = 0; i < cir.numMetalLayers(); ++i) {
    const Int layerIdx = cir.metalLayer(i).idx();
    // net
    Vector<Pair<Box<Int>, Pin*>> vpPins;
    cir.spatialPins(layerIdx).queryBoth(bbox, vpPins);
    for (const auto& p : vpPins) {
      add(p.second->pNet() != &net, p.first, i, 1, false);
    }
    Vector<Pair<Box<Int>, Net*>> vpNets;
    cir.spatialNets(layerIdx).queryBoth(bbox, vpNets);
    for (const auto& p : vpNets) {
      add(p.second != &net, p.first, i, 1, false);
    }

    // obs
    Vector<Box<Int>> v;
    cir.spatialObs(layerIdx).queryBox(bbox, v);
    for (const auto& b : v) {
      add(1, b, i, 1, false);
    }

    // his
    Vector<Int> eIds;
    ps.drMgr().vGridEdgeIds(bbox, layerIdx, eIds);
    for (const Int eIdx : eIds) {
      const auto& e = ps.drMgr().gridEdge(eIdx);
      const auto& u = ps.drMgr().gridNode(e.u);
      const auto& v = ps.drMgr().gridNode(e.v);
      const Real history = ps.history(eIdx);
      assert(u.z() == v.z() and u.z() == layerIdx);
      assert(u.x() <= v.x() and u.y() <= v.y());
      
      const Box<Int> b(u.x(), u.y(), v.x(), v.y());
      add(2, b, i, history / ps.hisCost(), true);
    }
  }
}

void RlUtils::toVisGds(const CirDB& cir, const Vector<Int>& vNetIds, const String& fileName) const
{
  GdsParser::GdsWriter gw(fileName.c_str());

  const String libName = cir.name() + "_lib";
  gw.create_lib(libName.c_str(), 0.0005, 5e-10);

  gw.gds_write_bgnstr();
  gw.gds_write_strname(cir.name().c_str());
 

  Int xl = MAX_INT, yl = MAX_INT;
  Int xh = MIN_INT, yh = MIN_INT;
  for (const Int netIdx : vNetIds) {
    const Net& net = cir.net(netIdx);
    xl = std::min(xl, net.wireBbox().xl());
    yl = std::min(yl, net.wireBbox().yl());
    xh = std::max(xh, net.wireBbox().xh());
    yh = std::max(yh, net.wireBbox().yh());

    for (const Pin* pPin : net.vpPins()) {
      const Pin& pin = *pPin;
      for (Int i = 0; i < pin.numBoxes(); ++i) {
        const auto& box = pin.box(i);
        const Int l = pin.boxLayerIdx(i);
        gw.write_box(netIdx + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
      }
    }
    for (const auto& wire : net.sWires()) { 
      const auto& box = wire.first;
      const Int z = wire.second;
      gw.write_box(netIdx + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
    }
  }

  const Box<Int> qBox(xl, yl, xh, yh);
  for (Int i = 0; i < cir.numLayers(); ++i) {
    Vector<Box<Int>> vBoxes;
    Vector<Pair<Box<Int>, Pin*>> vp;
    cir.spatialObs(i).queryBox(qBox, vBoxes);
    cir.spatialPins(i).queryBoth(qBox, vp);
    //std::sort(vBoxes.begin(), vBoxes.end());
    //vBoxes.resize(std::unique(vBoxes.begin(), vBoxes.end()) - vBoxes.begin());
    for (const auto& box : vBoxes) {
      gw.write_box(i + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
    }
    for (const auto& p : vp) {
      const Int idx = p.second->net().idx();
      if (std::find(vNetIds.begin(), vNetIds.end(), idx) == vNetIds.end()) {
        const auto& box = p.first;
        gw.write_box(i + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
      }
    }
  }

  gw.gds_write_endstr();
  gw.gds_write_endlib();
}

void RlUtils::toVisGds(const CirDB& cir, const Net& net, const String& fileName) const
{
  GdsParser::GdsWriter gw(fileName.c_str());

  const String libName = cir.name() + "_" + net.name() + "_lib";
  gw.create_lib(libName.c_str(), 0.0005, 5e-10);

  gw.gds_write_bgnstr();
  gw.gds_write_strname(net.name().c_str());

  for (const Pin* pPin : net.vpPins()) {
    const Pin& pin = *pPin;
    for (Int i = 0; i < pin.numBoxes(); ++i) {
      const auto& box = pin.box(i);
      const Int l = pin.boxLayerIdx(i);
      gw.write_box(l, 0, box.xl(), box.yl(), box.xh(), box.yh());
      gw.gds_create_text(pin.name().c_str(), box.centerX(), box.centerY(), l, 500);
    }
  }

  constexpr Int e = 5;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    const auto& u = arc.p0();
    const auto& v = arc.p1();
    const Int xl = std::min(u.x(), v.x());
    const Int xh = std::max(u.x(), v.x());
    const Int yl = std::min(u.y(), v.y());
    const Int yh = std::max(u.y(), v.y());
    assert(arc.b90());
    if (arc.bVia()) {
      const Int zl = std::min(u.z(), v.z());
      const Int zh = std::max(u.z(), v.z());
      assert(zl + 2 == zh);
      gw.write_box(zl, 0, xl - e, yl - e, xh + e, yh + e);
      gw.write_box(zl + 1, 0, xl - e, yl - e, xh + e, yh + e);
      gw.write_box(zh, 0, xl - e, yl - e, xh + e, yh + e);

    }
    else {
      gw.write_box(u.z(), 0, xl - e, yl - e, xh + e, yh + e);
    }
  }
  //for (const auto& wire : net.sWires()) { 
    //const auto& box = wire.first;
    //const Int z = wire.second;
    //gw.write_box(z, 0, box.xl(), box.yl(), box.xh(), box.yh());
  //}

  //if (net.name() == "ph13b") {

  //const Pin& sPin = net.pin("xmux13<1>/y");
  //const Pin& tPin = net.pin("xinv13/a");
  //std::cerr << "src ";
  for (const auto& pPin : net.vpPins()) {
    for (const auto& acs : pPin->vAcs()) {
      //std::cerr << acs << " ";
      gw.write_box(acs.z(),0, acs.x()-30, acs.y()-30, acs.x()+30, acs.y()+30);
    }
  }
  //std::cerr << std::endl;
  //std::cerr << "tar ";
  //for (const auto& acs : tPin.vAcs()) {
    //std::cerr << acs << " ";
    //gw.write_box(acs.z(), 0, acs.x()-30, acs.y()-30, acs.x()+30, acs.y()+30);
  //}
  //std::cerr << std::endl;
  //Vector<Vector<Segment3d<Int>>> vvSegs;
  //net.topo().vPaths(sPin, tPin, vvSegs);
  //for (const auto& vSegs : vvSegs) {
    //for (const auto& seg: vSegs) {
      //std::cerr << seg << std::endl;
    //}
  //}
  //for (const auto& vSegs : vvSegs) {
    //for (const auto& seg: vSegs) {
      //if (seg.bVia()) {
        //gw.write_box(seg.zl(), 0, seg.xl()-20, seg.yl()-20, seg.xh()+20, seg.yh()+20);
        //gw.write_box(seg.zl() + 1, 0, seg.xl()-20, seg.yl()-20, seg.xh()+20, seg.yh()+20);
        //gw.write_box(seg.zh(), 0, seg.xl()-20, seg.yl()-20, seg.xh()+20, seg.yh()+20);

      //}
      //else {
        //gw.write_box(seg.zl(), 0, seg.xl()-20, seg.yl()-20, seg.xh()+20, seg.yh()+20);
      //}
    //}
  //}
  //}

  // nearby obs
  const auto& qBox = net.wireBbox();
  const Int zl = std::max(0, net.minWireLayerIdx() - 2);
  const Int zh = std::min(cir.numLayers(), net.maxWireLayerIdx() + 2);
  for (Int i = zl; i <= zh; ++i) {
    Vector<Box<Int>> vBoxes;
    Vector<Pair<Box<Int>, Pin*>> vp;
    cir.spatialObs(i).queryBox(qBox, vBoxes);
    cir.spatialPins(i).queryBoth(qBox, vp);
    //std::sort(vBoxes.begin(), vBoxes.end());
    //vBoxes.resize(std::unique(vBoxes.begin(), vBoxes.end()) - vBoxes.begin());
    for (const auto& box : vBoxes) {
      gw.write_box(i + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
    }
    for (const auto& p : vp) {
      if (p.second->pNet() != &net) {
        const auto& box = p.first;
        gw.write_box(i + 10000, 0, box.xl(), box.yl(), box.xh(), box.yh());
      }
    }
  }
  // bbox
  //gw.write_box(99999, 0, net.bbox().xl(), net.bbox().yl(), net.bbox().xh(), net.bbox().yh());
  


  gw.gds_write_endstr();
  gw.gds_write_endlib();

}


PROJECT_NAMESPACE_END
