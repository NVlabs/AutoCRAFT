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

#include "dbCir.hpp"
#include "util/util.hpp"
#include "graph/dijkstra.hpp"

PROJECT_NAMESPACE_START

CirDB::~CirDB() 
{
  for (auto& p : _vpPlaceGrids) delete p;
  for (auto& p : _vpRouteGrids) delete p;
  for (auto& p : _vpPowerRouteGrids) delete p;
  for (auto& p : _vpPrims) delete p;
  for (auto& p : _vpDevMaps) delete p;
  for (auto& p : _vpCells) delete p;
  for (auto& p : _vpIOPins) delete p;
  for (auto& p : _vpLayers) delete p;
  for (auto& p : _vpVias) delete p;
  for (auto& p : _vpNets) delete p;
  for (auto& p : _vpRegions) delete p;
  for (auto& p : _vpPrePlaceCstrs) delete p;
  for (auto& p : _vpPlaceSymCstrs) delete p;
  for (auto& p : _vpPlaceArrayCstrs) delete p;
  for (auto& p : _vpPlaceClusterCstrs) delete p;
  for (auto& p : _vpPlaceExtCstrs) delete p;
  for (auto& p : _vpPlaceEdgeDistCstrs) delete p;
  for (auto& p : _vpPlaceOrderCstrs) delete p;
  for (auto& p : _vpPlaceAlignCstrs) delete p;
  for (auto& p : _vpPlaceDisjointCstrs) delete p;
  for (auto& p : _vpPlaceRowCstrs) delete p;
  for (auto& p : _vpRouteSymCstrs) delete p;
  for (auto& p : _vpRoutePathMatchCstrs) delete p;
  for (auto& p : _vpNetPrioCstrs) delete p;

}

void CirDB::updateBbox()
{
  Int xl = MAX_INT, yl = MAX_INT, xh = MIN_INT, yh = MIN_INT;
  for (Int i = 0; i < this->numRegions(); ++i) {
    const Region& reg = this->region(i);
    xl = std::min(xl, reg.bboxOuter().xl());
    yl = std::min(yl, reg.bboxOuter().yl());
    xh = std::max(xh, reg.bboxOuter().xh());
    yh = std::max(yh, reg.bboxOuter().yh());
  }
  _bbox.set(xl, yl, xh, yh);
}

void CirDB::groupCells()
{
  for (Int i = 0; i < this->numRegions(); ++i) {
    Region& reg = this->region(i);
    for (Int j = 0; j < reg.numCells(); ++j) {
      Cell& c = reg.cell(j);
      Net* pVddNet = c.hasVddPin() ? c.vddPin().pNet() : nullptr;
      Net* pVssNet = c.hasVssPin() ? c.vssPin().pNet() : nullptr;
      assert(!pVddNet or pVddNet->isVdd());
      assert(!pVssNet or pVssNet->isVss());
      const Pair<Net*, Net*> p = std::make_pair(pVddNet, pVssNet);

      auto& mPowerNets2CellGroupIdx = reg.mPowerNets2CellGroupIdx();
      auto& vvpCellGroups = reg.vvpCellGroups();
      auto it = mPowerNets2CellGroupIdx.find(p);
      if (it == mPowerNets2CellGroupIdx.end()) {
        c.setPowerGroupIdx(vvpCellGroups.size());
        mPowerNets2CellGroupIdx.emplace(p, vvpCellGroups.size());
        vvpCellGroups.emplace_back(Vector<Cell*>{&c});
      }
      else {
        c.setPowerGroupIdx(it->second);
        vvpCellGroups[it->second].emplace_back(&c);
      }
    }
    //for (const auto& v : reg.vvpCellGroups()) {
      //std::cerr << reg.name() << " group: " << std::endl;
      //for (const auto& p : v) {
        //std::cerr << p->name() << std::endl;
      //}
      //std::cerr << std::endl;
    //}
  }
#ifndef NDEBUG
  for (Int i = 0; i < this->numRegions(); ++i) {
    const Region& reg = this->region(i);
    for (const Cell* pCell : reg.vpCells()) {
      assert(pCell->powerGroupIdx() != -1);
    }
  }

#endif

}

void CirDB::initSpatials()
{
  _vvpPins.resize(_vpLayers.size());
  _vSpatialPins.resize(_vpLayers.size());
  _vvpObs.resize(_vpLayers.size());
  _vSpatialObs.resize(_vpLayers.size());
  _vSpatialNets.resize(_vpLayers.size());
}

void CirDB::buildSpatialPinsNObs()
{
  Vector<Vector<Pair<Box<Int>, Pin*>>> vvBoxPins(_vpLayers.size());
  Vector<Vector<Pair<Box<Int>, Obs*>>> vvBoxObs(_vpLayers.size());

  Int i, j;
  const Cell* pCell;
  Cir_ForEachCell((*this), pCell, i) {
    const Cell& c = *pCell;
    for (const Pin& pin : c.vPins()) {
      for (Int i = 0; i < pin.numBoxes(); ++i) {
        const Box<Int>& box = pin.box(i);
        const Int layerIdx = pin.boxLayerIdx(i);

        _vvpPins[layerIdx].emplace_back(const_cast<Pin*>(&pin));
        vvBoxPins[layerIdx].emplace_back(box, const_cast<Pin*>(&pin));
      }
    }
    for (const Obs& obs : c.vObs()) {
      const Box<Int>& box = obs.box();
      const Int layerIdx = obs.layerIdx();

      _vvpObs[layerIdx].emplace_back(const_cast<Obs*>(&obs));
      vvBoxObs[layerIdx].emplace_back(box, const_cast<Obs*>(&obs));
    }
  }
  // edge and dummy cells
  const Region* pReg;
  Cir_ForEachRegion((*this), pReg, i) {
    Reg_ForEachEdgeCell((*pReg), pCell, j) {
      const Cell& c = *pCell;
      assert(c.numPins() == 0);
      for (const Obs& obs : c.vObs()) {
        const Box<Int>& box = obs.box();
        const Int layerIdx = obs.layerIdx();
        _vvpObs[layerIdx].emplace_back(const_cast<Obs*>(&obs));
        vvBoxObs[layerIdx].emplace_back(box, const_cast<Obs*>(&obs));
      }
    }
    Reg_ForEachDummyCell((*pReg), pCell, j) {
      const Cell& c = *pCell;
      assert(c.numPins() == 0);
      for (const Obs& obs : c.vObs()) {
        const Box<Int>& box = obs.box();
        const Int layerIdx = obs.layerIdx();
        _vvpObs[layerIdx].emplace_back(const_cast<Obs*>(&obs));
        vvBoxObs[layerIdx].emplace_back(box, const_cast<Obs*>(&obs));
      }
    }
  }

  for (size_t i = 0; i < _vpLayers.size(); ++i) {
    _vSpatialPins[i] = SpatialMap<Int, Pin*>(vvBoxPins.at(i));
    _vSpatialObs[i] = SpatialMap<Int, Obs*>(vvBoxObs.at(i));
  }
#ifndef NDEBUG
  for (const auto& cellPtr : _vpCells) {
    const Cell& c = *cellPtr;
    for (const Pin& pin : c.vPins()) {
      for (Int i = 0; i < pin.numBoxes(); ++i) {
        const Box<Int>& box = pin.box(i);
        const Int layerIdx = pin.boxLayerIdx(i);
        auto& sp = _vSpatialPins.at(layerIdx);
        Vector<Pin*> vpTouchPins;
        sp.query(box, vpTouchPins);
        for (Pin* pPin : vpTouchPins) {
          if (pPin->pNet() != pin.pNet()) {
            spdlog::error("{} {}", pin.name(), pPin->name());
            spdlog::error("{} {}", pPin->pNet()->name(), pin.pNet()->name());
          }
          assert(pPin->pNet() == pin.pNet());
        }
      }
    }
  }
#endif
}

void CirDB::buildSpatialNets()
{
  Int i;
  const Net* pNet;
  Cir_ForEachNet((*this), pNet, i) {
    buildSpatialNet(*pNet);
  }
}

void CirDB::buildSpatialNet(const Net& net)
{
  Int i, j, layerIdx;
  const Pin* pPin;
  const Box<Int>* pBox;
  Net_ForEachPin(net, pPin, i) {
    Pin_ForEachLayerIdx((*pPin), layerIdx) {
      Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
        _vSpatialNets[layerIdx].insert(*pBox, const_cast<Net*>(&net));
      }
    }
  }
  for (const auto& wire : net.sWires()) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    _vSpatialNets[layerIdx].insert(box, const_cast<Net*>(&net));
  }
}

void CirDB::setIOPinBbox(const Int i, const Box<Int>& b)
{
  _vIOPinBboxes[i] = b;
}

void CirDB::genRoutingBbox() 
{
  spdlog::info("[CirDB] Generate routing boundaries");

  Int xl = MAX_INT, xh = MIN_INT;
  Int yl = MAX_INT, yh = MIN_INT;

  Int i;
  const Region* pReg;
  Cir_ForEachRegion((*this), pReg, i) {
    xl = std::min(pReg->bboxInner().xl(), xl);
    xh = std::max(pReg->bboxInner().xh(), xh);
    yl = std::min(pReg->bboxInner().yl(), yl);
    yh = std::max(pReg->bboxInner().yh(), yh);
  }

  _routingBbox.set(xl, yl, xh, yh);
}

void CirDB::genWsp()
{
  spdlog::info("[CirDB] Generate Wsp regions");

  assert(this->metalLayer(1).isHor());
  assert(this->metalLayer(2).isVer());
  assert(this->metalLayer(3).isHor());
  assert(this->metalLayer(4).isVer());

  auto addWsp = [&] (Vector<Pair<Box<Int>, Int>>& vWsps, Vector<Pair<Box<Int>, Int>>& vFillDrws, const RouteGrid& rg, Int xl, Int yl, Int xh, Int yh, const bool isVer) {
    const Int layerIdx = rg.upperLayerIdx();
    Int t, hw;
    if (isVer) {
      t = rg.floorX(xl);
      hw = rg.xLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(t - hw, yl, t + hw, yh), layerIdx);
      t = rg.floorX(t - 1);
      xl = t - hw;
      hw = rg.xLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(t - hw, yl, t + hw, yh), layerIdx);
      t = rg.ceilX(xh);
      hw = rg.xLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(t - hw, yl, t + hw, yh), layerIdx);
      t = rg.ceilX(t + 1);
      xh = t + hw;
      hw = rg.xLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(t - hw, yl, t + hw, yh), layerIdx);
    }
    else {
      t = rg.floorY(yl);
      hw = rg.yLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(xl, t - hw, xh, t + hw), layerIdx);
      t = rg.floorY(t - 1);
      yl = t - hw;
      hw = rg.yLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(xl, t - hw, xh, t + hw), layerIdx);
      t = rg.ceilY(yh);
      hw = rg.yLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(xl, t - hw, xh, t + hw), layerIdx);
      t = rg.ceilY(t + 1);
      yh = t + hw;
      hw = rg.yLocWidth(t) / 2;
      vFillDrws.emplace_back(Box<Int>(xl, t - hw, xh, t + hw), layerIdx);
    }
    vWsps.emplace_back(Box<Int>(xl, yl, xh, yh), layerIdx);
  };
  

  auto calcBound = [&] (Region& reg, const RouteGrid& rg) -> Box<Int> {
    const Int layerIdx = rg.upperLayerIdx();
    Int xl = reg.bboxInner().xl();
    Int yl = reg.bboxInner().yl();
    Int xh = reg.bboxInner().xh();
    Int yh = reg.bboxInner().yh();

    Int i, j, k;
    const Cell* pCell;
    const Pin* pPin;
    const Obs* pObs;
    const Box<Int>* pBox;
    const Net* pNet;
    //const Pair<Box<Int>, Int>* pWire;
    Reg_ForEachCell(reg, pCell, i) {
      Cell_ForEachPin((*pCell), pPin, j) {
        Pin_ForEachLayerBox((*pPin), layerIdx, pBox, k) {
          xl = std::min(xl, pBox->xl());
          yl = std::min(yl, pBox->yl());
          xh = std::max(xh, pBox->xh());
          yh = std::max(yh, pBox->yh());

        }
      }
      Cell_ForEachObs((*pCell), pObs, j) {
        if (pObs->layerIdx() == layerIdx) {
          xl = std::min(xl, pObs->box().xl());
          yl = std::min(yl, pObs->box().yl());
          xh = std::max(xh, pObs->box().xh());
          yh = std::max(yh, pObs->box().yh());
        }
      }
    }
    Reg_ForEachDummyCell(reg, pCell, i) {
      Cell_ForEachObs((*pCell), pObs, j) {
        if (pObs->layerIdx() == layerIdx) {
          xl = std::min(xl, pObs->box().xl());
          yl = std::min(yl, pObs->box().yl());
          xh = std::max(xh, pObs->box().xh());
          yh = std::max(yh, pObs->box().yh());
        }
      }
    }
    Reg_ForEachEdgeCell(reg, pCell, i) {
      Cell_ForEachObs((*pCell), pObs, j) {
        if (pObs->layerIdx() == layerIdx) {
          xl = std::min(xl, pObs->box().xl());
          yl = std::min(yl, pObs->box().yl());
          xh = std::max(xh, pObs->box().xh());
          yh = std::max(yh, pObs->box().yh());
        }
      }
    }
    Reg_ForEachNet(reg, pNet, i) {
      Net_ForEachPin((*pNet), pPin, j) {
        if (pPin->minLayerIdx() <= layerIdx and layerIdx <= pPin->maxLayerIdx()) {
          Pin_ForEachLayerBox((*pPin), layerIdx, pBox, k) {
            xl = std::min(xl, pBox->xl());
            yl = std::min(yl, pBox->yl());
            xh = std::max(xh, pBox->xh());
            yh = std::max(yh, pBox->yh());
          }
        }
      }
      for (const auto& wire : pNet->sWires()) {
        if (wire.second == layerIdx) {
          xl = std::min(xl, wire.first.xl());
          yl = std::min(yl, wire.first.yl());
          xh = std::max(xh, wire.first.xh());
          yh = std::max(yh, wire.first.yh());
        }
      }
      for (const auto& wire : pNet->sPatchWires()) {
        if (wire.second == layerIdx) {
          xl = std::min(xl, wire.first.xl());
          yl = std::min(yl, wire.first.yl());
          xh = std::max(xh, wire.first.xh());
          yh = std::max(yh, wire.first.yh());
        }

      }
    }
    //spdlog::info("{} {} {} {} {}", layerIdx, xl, yl, xh, yh);
    return Box<Int>(xl, yl, xh, yh);

  };

  Int i;
  Region* pReg;

  Int xl = MAX_INT, xh = MIN_INT;
  Int yl = MAX_INT, yh = MIN_INT;
  
  const RouteGrid& m12 = this->routeGrid(0);
  const RouteGrid& m23 = this->routeGrid(1);
  const RouteGrid& m34 = this->routeGrid(2);
  const RouteGrid& m45 = this->routeGrid(3);

  Cir_ForEachRegion((*this), pReg, i) {
    const Box<Int> b12 = calcBound((*pReg), m12);
    xl = std::min(xl, b12.xl());
    xh = std::max(xh, b12.xh());
    yl = std::min(yl, b12.yl());
    yh = std::max(yh, b12.yh());
  }
  addWsp(this->vWspBboxes(), this->vWspFillDrws(), m12, xl, yl, xh, yh, false);
  addWsp(this->vWspBboxes(), this->vWspFillDrws(), m23, xl, yl, xh, yh, true);
  addWsp(this->vWspBboxes(), this->vWspFillDrws(), m34, xl, yl, xh, yh, false);
  addWsp(this->vWspBboxes(), this->vWspFillDrws(), m45, xl, yl, xh, yh, true);

  //Cir_ForEachRegion((*this), pReg, i) {

    //const RouteGrid& m12 = pReg->routeGrid(0);
    //const RouteGrid& m23 = pReg->routeGrid(1);
    //const RouteGrid& m34 = pReg->routeGrid(2);
    //const RouteGrid& m45 = pReg->routeGrid(3);

    //const Box<Int> b12 = calcBound((*pReg), m12);
    //const Box<Int> b23 = calcBound((*pReg), m23);
    //const Box<Int> b34 = calcBound((*pReg), m34);
    //const Box<Int> b45 = calcBound((*pReg), m45);


    //addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m12, b12.xl(), b12.yl(), b12.xh(), b12.yh(), false);
    //addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m23, b23.xl(), b23.yl(), b23.xh(), b23.yh(), true);
    //addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m34, b34.xl(), b34.yl(), b34.xh(), b34.yh(), false);
    //addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m45, b45.xl(), b45.yl(), b45.xh(), b45.yh(), true);
    
    ////addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m12, b12.xl(), b12.yl(), b12.xh(), b12.yh(), false);
    ////addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m23, b12.xl(), b12.yl(), b12.xh(), b12.yh(), true);
    ////addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m34, b12.xl(), b12.yl(), b12.xh(), b12.yh(), false);
    ////addWsp(pReg->vWspBboxes(), pReg->vWspFillDrws(), m45, b12.xl(), b12.yl(), b12.xh(), b12.yh(), true);
    
  //}
}

Real CirDB::segRes(const Segment3d<Int>& seg, const TopoTree::SegData& sd) const
{
  Real res = 0;
  if (seg.bVia()) {
    const Int layerIdx = seg.zl() + 1;
    const CutLayer& layer = *static_cast<const CutLayer*>(this->pLayer(layerIdx));
    const Via& via = *sd.pVia;
    res = layer.rMean() / via.numBoxes(layerIdx);
    assert(via.numBoxes(layerIdx) == 1);
    //if (via.numBoxes(layerIdx) == 0) {
      //std::cerr << seg << std::endl;
      //std::cerr << via.name() << " "<<  layerIdx << std::endl;
      //std::cerr << "via " << res << std::endl;
    //}
  }
  else {
    const MetalLayer& layer = *static_cast<const MetalLayer*>(this->pLayer(seg.p0().z()));
    res = layer.unitRMean(sd.width) * seg.length();
  }
  return res;
}
  
Real CirDB::pathRes(const Vector<Segment3d<Int>>& vSegs) const
{
  Real res = 0;
  for (size_t i = 0; i < vSegs.size(); ++i) {
    const auto& seg = vSegs.at(i);
    if (seg.bVia()) {
      const Int layerIdx = seg.zl() + 1;
      const CutLayer& layer = *static_cast<const CutLayer*>(this->pLayer(layerIdx));
      //const auto fu = seg.p0().to2d();
      //Vector<Box<Int>> vBoxes;
      //_vSpatialNets.at(layerIdx).queryBox(fu, fu, vBoxes);
      //assert(vBoxes.size() == 1); // should only have 1 via cut
      //res += layer.rMean() / vBoxes.size();
      res += layer.rMean();
    }
    else {
      const Int layerIdx = seg.p0().z();
      const MetalLayer& layer = *static_cast<const MetalLayer*>(this->pLayer(layerIdx));
      //Vector<Box<Int>> vBoxes;
      //_vSpatialNets.at(layerIdx).queryBox(seg.min_corner().to2d(), seg.max_corner().to2d(), vBoxes);
      //assert(vBoxes.size() > 0);
      //const Box<Int> mergedBox = intersection(vBoxes);
      //if (layer.isHor()) {
        //res += layer.unitRMean(mergedBox.height()) * mergedBox.width();
      //}
      //else {
        //res += layer.unitRMean(mergedBox.width()) * mergedBox.height();
      //
      const Int rgIdx = layer.selfIdx() < this->numRouteGrids() ? layer.selfIdx() : layer.selfIdx() - 1;
      assert(0 <= rgIdx and rgIdx < this->numRouteGrids());
      const RouteGrid& rg = this->routeGrid(rgIdx);
      res += layer.unitRMean(rg.wireWidth(seg.p0())) * seg.length();
    }
  }
  return res;
}

Real CirDB::pathRes(const Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  //auto intersection = [] (const Vector<Box<Int>>& v) -> Box<Int> {
    //Int xl = MAX_INT, yl = MAX_INT;
    //Int xh = MIN_INT, yh = MIN_INT;
    //for (const auto& b : v) {
      //xl = std::min(xl, b.xl());
      //yl = std::min(yl, b.yl());
      //xh = std::max(xh, b.xh());
      //yh = std::max(yh, b.yh());
    //}
    //return Box<Int>(xl, yl, xh, yh);
  //};
  Real res = 0;
  for (size_t i = 0; i < vvSegs.size(); ++i) {
    const auto& vSegs = vvSegs.at(i);
    res += pathRes(vSegs);
  }
  return res;
}

//Real CirDB::pathRes(const Vector<Vector<Segment3d<Int>>>& vvSegs, const Vector<Vector<TopoTree::SegData>>& vvSegData) const
//{
  //assert(vvSegs.size() == vvSegData.size());

  //Real res = 0;
  //for (size_t i = 0; i < vvSegs.size(); ++i) {
    //const auto& vSegs = vvSegs.at(i);
    //const auto& vSegData = vvSegData.at(i);
    //assert(vSegs.size() == vSegData.size());

    //for (size_t j = 0; j < vSegs.size(); ++j) {
      //const auto& seg = vSegs.at(j);
      //const auto& segData = vSegData.at(j);
      //if (seg.bVia()) {
        //const Int layerIdx = seg.zl() + 1;
        //const CutLayer& layer = *static_cast<const CutLayer*>(this->pLayer(layerIdx));
        //res += layer.rMean() / segData.pVia->numBoxes(layerIdx);
      //}
      //else {
        //const MetalLayer& layer = *static_cast<const MetalLayer*>(this->pLayer(seg.p0().z()));
        //res += layer.unitRMean(segData.width) * seg.length();
      //}
    //}
  //}
  //return res;
//}

Int CirDB::pathLength(const Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  Int len = 0;
  for (const auto& vSegs : vvSegs) {
    for (const auto& seg : vSegs) {
      if (!seg.bVia()) {
        len += seg.length();
      }
    }
  }
  return len;
}

Int CirDB::pathVias(const Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  Int via = 0;
  for (const auto& vSegs : vvSegs) {
    for (const auto& seg : vSegs) {
      if (seg.bVia()) {
        ++via;
      }
    }
  }
  return via;
}

Pair<Int, Int> CirDB::pathLengthVias(const Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  Int len = 0, via = 0;
  for (const auto& vSegs : vvSegs) {
    for (const auto& seg : vSegs) {
      if (seg.bVia()) {
        ++via;
      }
      else {
        len += seg.length();
      }
    }
  }
  return std::make_pair(len, via);
}

Tuple<Int, Int, Real> CirDB::pathStats(const Vector<Segment3d<Int>>& vSegs) const
{
  Int len = 0, via = 0;
  Real res = 0;

  for (size_t i = 0; i < vSegs.size(); ++i) {
    const auto& seg = vSegs.at(i);
    if (seg.bVia()) {
      const Int layerIdx = seg.zl() + 1;
      const CutLayer& layer = *static_cast<const CutLayer*>(this->pLayer(layerIdx));
      ++via;
      res += layer.rMean();
    }
    else {
      const Int layerIdx = seg.p0().z();
      const MetalLayer& layer = *static_cast<const MetalLayer*>(this->pLayer(layerIdx));
      len += seg.length();
      const Int rgIdx = layer.selfIdx() < this->numRouteGrids() ? layer.selfIdx() : layer.selfIdx() - 1;
      assert(0 <= rgIdx and rgIdx < this->numRouteGrids());
      const RouteGrid& rg = this->routeGrid(rgIdx);
      res += layer.unitRMean(rg.wireWidth(seg.p0())) * seg.length();
    }
  }

  return std::make_tuple(len, via, res);

}

Tuple<Int, Int, Real> CirDB::pathStats(const Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
   
  Int len = 0, via = 0;
  Real res = 0;
  for (const auto& vSegs : vvSegs) {
    const auto t = this->pathStats(vSegs);
    len += std::get<0>(t);
    via += std::get<1>(t);
    res += std::get<2>(t);
  }
  return std::make_tuple(len, via, res);
}

//Tuple<Int, Int, Real> CirDB::pathStats(const Vector<Vector<Segment3d<Int>>>& vvSegs, const Vector<Vector<TopoTree::SegData>>& vvSegData) const
//{
  //assert(vvSegs.size() == vvSegData.size());

  //Int len = 0, via = 0;
  //Real res = 0;
  //for (size_t i = 0; i < vvSegs.size(); ++i) {
    //const auto& vSegs = vvSegs.at(i);
    //const auto& vSegData = vvSegData.at(i);
    //assert(vSegs.size() == vSegData.size());

    //for (size_t j = 0; j < vSegs.size(); ++j) {
      //const auto& seg = vSegs.at(j);
      //const auto& segData = vSegData.at(j);
      //if (seg.bVia()) {
        //const Int layerIdx = seg.zl() + 1;
        //const CutLayer& layer = *static_cast<const CutLayer*>(this->pLayer(layerIdx));
        //++via;
        //res += layer.rMean() / segData.pVia->numBoxes(layerIdx);
      //}
      //else {
        //const MetalLayer& layer = *static_cast<const MetalLayer*>(this->pLayer(seg.p0().z()));
        //len += seg.length();
        //res += layer.unitRMean(segData.width) * seg.length();
      //}
    //}
  //}
  //return std::make_tuple(len, via, res);
//}

Real CirDB::netRes(const Net& net) const
{
  // FIXME: parallel routes not supported
  Real res = 0;
  for (const auto& arcEntry : net.mArcs()) {
    const auto& arc = arcEntry.first;
    const auto& arcData = arcEntry.second;
    res += segRes(arc, arcData);
  }
  return res;
}


Real CirDB::pathRes(const Pin& sPin, const Pin& tPin) const
{
  // FIXME: parallel routes not supported
  
  assert(sPin.pNet() == tPin.pNet());
  const Net& net = sPin.net();

  Vector<Vector<Segment3d<Int>>> vPaths;
  net.topo().vPaths(sPin, tPin, vPaths);

  //if (vPaths.empty()) {
    //return {MAX_REAL, false};
  //}
  
  const Int numPaths = vPaths.size();

  Real res = MAX_REAL;
  if (numPaths > 0) {

    Int pathIdx = 0;
    if (numPaths > 1) {
      Int minPathRes = this->pathRes(vPaths.at(pathIdx));
      for (Int i = 1; i < numPaths; ++i) {
        const Int r = this->pathRes(vPaths.at(i));
        if (r < minPathRes) {
          minPathRes = r;
          pathIdx = i;
        }
      }
    }
    const auto& path = vPaths.at(pathIdx);

    res = pathRes(path);
  }
  else {
    //Vector<Vector<Pair<Box<Int>, Int>>> vWirePaths;
    //Vector<Pair<Box<Int>, Int>> prefix;
    //FlatHashSet<Pair<Box<Int>, Int>> vis;

    //std::function<void(const Pair<Box<Int>, Int>&, const Pair<Box<Int>, Int>&, Vector<Pair<Box<Int>, Int>>& prefix)>
    //dfsWirePaths = [&] (const Pair<Box<Int>, Int>& p, const Pair<Box<Int>, Int>& t, Vector<Pair<Box<Int>, Int>>& prefix) {
    
      //vis.emplace(p);
      //prefix.emplace_back(p);
      //for (const auto& ss : prefix) {
        //std::cerr << ss.first << " " << ss.second << std::endl;
      //}
      //std::cerr << std::endl;
      

      //const Box<Int>& box = p.first;
      //const Int z = p.second;

      //if (p == t) {
        //vWirePaths.emplace_back(prefix);
      //}
      //else {
        //Vector<Box<Int>> vc, vl, vh;
        //this->spatialNets(z).queryBox(box, vc);
        //for (const auto& b : vc) {
          //if (vis.find({b, z}) == vis.end()) {
            //dfsWirePaths({b, z}, t, prefix);
          //}
        //}
        //if (z > 0) {
          //this->spatialNets(z - 1).queryBox(box, vl);
          //for (const auto& b : vl) {
            //if (vis.find({b, z - 1}) == vis.end()) {
              //dfsWirePaths({b, z - 1}, t, prefix);
            //}
          //}
        //} 
        //if (z < this->numLayers() - 1) {
          //this->spatialNets(z + 1).queryBox(box, vh);
          //for (const auto& b : vh) {
            //if (vis.find({b, z + 1}) == vis.end()) {
              //dfsWirePaths({b, z + 1}, t, prefix);
            //}
          //}
        //}
      //}
      //prefix.pop_back();
      //vis.erase(p);
    //};

    //Pair<Box<Int>, Int> ws, wt;
    //for (const auto& item : net.mArcs()) {
      //const auto& arc = item.first;
      //const Point3d<Int>& u = arc.p0();
      //const Point3d<Int>& v = arc.p1();
      //const Point<Int>& u2d = u.to2d();
      //const Point<Int>& v2d = v.to2d();

      //Vector<Pair<Box<Int>, Pin*>> vPBs;

      //this->spatialPins(u.z()).queryBoth(u2d, u2d, vPBs);
      //assert(vPBs.size() <= 1);
      //if (vPBs.size()) {
        //const Pin* touchedPin = vPBs[0].second;
        //if (touchedPin == const_cast<Pin*>(&sPin)) {
          //ws = {vPBs[0].first, u.z()};
        //}
        //else if (touchedPin == const_cast<Pin*>(&tPin)) {
          //wt = {vPBs[0].first, u.z()};
        //}
      //}

      //vPBs.clear();
      //this->spatialPins(v.z()).queryBoth(v2d, v2d, vPBs);
      //assert(vPBs.size() <= 1);
      //if (vPBs.size()) {
        //const Pin* touchedPin = vPBs[0].second;
        //if (touchedPin == const_cast<Pin*>(&sPin)) {
          //ws = {vPBs[0].first, v.z()};
        //}
        //else if (touchedPin == const_cast<Pin*>(&tPin)) {
          //wt = {vPBs[0].first, v.z()};
        //}
      //}

    //}
    //dfsWirePaths(ws, wt, prefix);



  }
  return res;
}


void CirDB::showInfo() const {
  spdlog::info("Cell: {}", this->numCells());
  
  Vector<Int> vNumNets(200, 0);
  for (const auto& pNet : _vpNets) {
    ++vNumNets[pNet->numPins()];
  }
  spdlog::info("Net: {}", this->numNets());
  for (size_t i = 0; i < vNumNets.size(); ++i) {
    if (vNumNets[i] > 0) {
      spdlog::info("  {}-pin: {}", i, vNumNets[i]);
    }
  }

  //for (size_t i = 0; i < _vpNets.size(); ++i) {
    //const Net& net = *_vpNets.at(i);
    //spdlog::info("{}", net.name());
    //for (const auto& p : net.vpPins()) {
      //spdlog::info("{}", p->name());
    //}
    //spdlog::info("");
  //}
  //exit(1);

  //for (auto l : _vpLayers) {
    //spdlog::info("{} {} {} {}", l->name(), l->idx(), l->selfIdx(), l->typeStr());
  //}
  //for (const auto& pDevMap : _vpDevMaps) {
    //spdlog::info("DevMap {}", pDevMap->name());
    //for (auto pPrim : pDevMap->vpPrims()) {
      //spdlog::info("  Prim {}", pPrim->name());
      ////for (const auto& pin : pPrim->vPins()) {
        ////spdlog::info("    Pin {}", pin.name());
      ////}
    //}
  //}
  //for (const auto& pReg : _vpRegions) {
    //spdlog::info("Region {}", pReg->name());
    //for (size_t i = 1; i < 9; ++i) {
      //if (&pReg->edgePrim(i))
        //spdlog::info("  EdgeCell {} ", pReg->edgePrim(i).name());
    //}
    //for (auto pPrim : pReg->vpDummyPrims()) {
      //spdlog::info("  DummyCell {}", pPrim->name());
    //}
    //for (const auto& pRg : pReg->vpRouteGrids()) {
      //spdlog::info("{} ", pRg->name());
    //}
    //for (const auto& pRg : pReg->vpPowerRouteGrids()) {
      //spdlog::info("{} ", pRg->name());
    //}
  //}
  //for (const auto& v : _vpVias) {
    //const Via& via = *v;
    //spdlog::info("{}", via.name());
    //spdlog::info("{} {}", via.minLayerIdx(), via.maxLayerIdx());
    //spdlog::info("{}", via.numBoxes());
  //}
}

void CirDB::showPlaceHpwl(const bool useIO) const 
{
  Real hpwl = 0;
  Int i, j;
  const Net* pNet;
  const Pin* pPin;
  Cir_ForEachNet((*this), pNet, i) {
    if (pNet->isPower()) {
      continue;
    }
    if (pNet->isIO() and !useIO) {
      continue;
    }
    if (pNet->numPins() <= 1) {
      continue;
    }
    Int xl = MAX_INT, xh = MIN_INT;
    Int yl = MAX_INT, yh = MIN_INT;
    Net_ForEachPin((*pNet), pPin, j) {
      const Cell& c = pPin->cell();
      xl = std::min(xl, c.loc().x());
      xh = std::max(xh, c.loc().x() + c.width());
      yl = std::min(yl, c.loc().y());
      yh = std::max(yh, c.loc().y() + c.height());
    }
    assert(xl != MAX_INT);
    assert(xh != MIN_INT);
    assert(yl != MAX_INT);
    assert(yh != MIN_INT);
    Real netHpwl = (xh - xl + yh - yl);
    //spdlog::info("Net {}: {}", pNet->name(), netHpwl);
    hpwl += netHpwl;
  }
  spdlog::info("[CirDB] Total placement HPWL: {:.3f} um", hpwl * this->physRes());
}

void CirDB::showRouteWL() const 
{
  Int numVias = 0;
  Real wl = 0;

  Int i;
  const Net* pNet;
  //const Net::Arc* pArc;
  Cir_ForEachNet((*this), pNet, i) {
    if (pNet->isPower()) {
      continue;
    }
    if (pNet->numPins() <= 1) {
      continue;
    }
    if (!pNet->isRouted()) {
      continue;
    }
    for (const auto& arcEntry : pNet->mArcs()) {
      const auto& arc = arcEntry.first;
      const auto& u = arc.p0();
      const auto& v = arc.p1();
      if (u.z() == v.z()) {
        wl += std::abs(u.x() - v.x()) + std::abs(u.y() - v.y());
      }
      else {
        ++numVias;
      }
    }
  }
  spdlog::info("[CirDB] Total routed WL: {:.3f}, #Via: {}", wl * this->physRes(), numVias);

  numVias = 0;
  wl = 0;
  Cir_ForEachNet((*this), pNet, i) {
    if (pNet->isPower()) {
      for (const auto& arcEntry : pNet->mArcs()) {
        const auto& arc = arcEntry.first;
        const auto& u = arc.p0();
        const auto& v = arc.p1();
        if (u.z() == v.z()) {
          wl += std::abs(u.x() - v.x()) + std::abs(u.y() - v.y());
        }
        else {
          ++numVias;
        }
      }
    }
  }
  spdlog::info("[CirDB] Total routed WL (power): {:.3f}, #Via: {}", wl * this->physRes(), numVias);
}

void CirDB::showNetSegs() const
{
  for (Int i = 0; i < this->numNets(); ++i) {
    const Net& n = this->net(i);
    //if (n.name() == "ckint000") {

    std::cerr << n.name() << " " << n.topo().numComps() << " " << n.topo().numSegs() << std::endl;
    //n.topo().showComps();
    //}
  }
}

void CirDB::debug()
{

  //const TopoTree::SegData d(nullptr, Orient2dE::undef, 0, 0);

  //Net& n = this->net("dmid01");
  //std::cerr << n.name() << " " << n.numRoutables() << std::endl;
  //auto& ro = n.routable(0);
  //n.addRoute(Segment3d<Int>(0, 0, 0, 4, 0, 0), d);
  //n.addRoute(Segment3d<Int>(0, 0, 0, 4, 0, 0), d);
  //n.addRoute(Segment3d<Int>(1, 1, 0, 4, 1, 0), d);
  //n.addRoute(Segment3d<Int>(1, 0, 0, 1, 1, 0), d);
  //ro.addRoute(Segment3d<Int>(0, 0, 0, 4, 0, 0), d);
  //ro.addRoute(Segment3d<Int>(0, 0, 0, 4, 0, 0), d);
  //ro.addRoute(Segment3d<Int>(1, 1, 0, 4, 1, 0), d);
  //ro.addRoute(Segment3d<Int>(1, 0, 0, 1, 1, 0), d);

  //n.addRoute(Segment3d<Int>(2, 1, 0, 2, 2, 0), d);
  //n.addRoute(Segment3d<Int>(3, 1, 0, 3, 2, 0), d);
  //n.removeRoute(Segment3d<Int>(2, 1, 0, 3, 1, 0));

  ////n.removeRoute(Segment3d<Int>(1, 0, 0, 1, 1, 0));
  ////n.addRoute(Segment3d<Int>(1, 0, 0, 1, 1, 0), d);
  ////n.addRoute(Segment3d<Int>(2, 0, 0, 2, 1, 0), d);
  ////n.removeRoute(Segment3d<Int>(1, 0, 0, 1, 1, 0));

  //n.topo().groupComps();
  //std::cerr << "Comps: " << n.topo().numComps() << std::endl;
  //std::cerr << "Segs: " << n.topo().numSegs() << std::endl;
  ////n.topo().showComps();

  //Vector<Segment3d<Int>> vSegs;
  //Vector<TopoTree::SegData> vSegData;
  //n.topo().vRoutes(vSegs, vSegData);
  //assert(static_cast<Int>(vSegs.size()) == n.topo().numSegs());
  //assert(vSegData.size() == vSegs.size());
  //for (Int i = 0; i < n.topo().numSegs(); ++i) {
    //const auto& s = vSegs.at(i);
    //const auto& sData = vSegData.at(i);
    //std::cerr << s <<  " " << (sData.pVia ? sData.pVia->name() : "null") << " " << util::enumUtil::val2Str(Orient2dEStr, sData.orient) << " " << sData.width << " " << sData.ext << std::endl;
  //}

  //ro.topo().groupComps();
  //std::cerr << "Comps: " << ro.topo().numComps() << std::endl;
  //std::cerr << "Segs: " << ro.topo().numSegs() << std::endl;
  ////ro.topo().showComps();

  //vSegs.clear();
  //vSegData.clear();
  //ro.topo().vRoutes(vSegs, vSegData);
  //assert(static_cast<Int>(vSegs.size()) == ro.topo().numSegs());
  //assert(vSegData.size() == vSegs.size());
  //for (Int i = 0; i < ro.topo().numSegs(); ++i) {
    //const auto& s = vSegs.at(i);
    //const auto& sData = vSegData.at(i);
    //std::cerr << s <<  " " << (sData.pVia ? sData.pVia->name() : "null") << " " << util::enumUtil::val2Str(Orient2dEStr, sData.orient) << " " << sData.width << " " << sData.ext << std::endl;
  //}

}

PROJECT_NAMESPACE_END
