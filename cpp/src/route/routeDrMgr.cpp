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

#include "routeDrMgr.hpp"
#include "routeDrPS.hpp"
#include "routeDrPost.hpp"
#include "drc/drcMgr.hpp"

PROJECT_NAMESPACE_START

bool RouteDrMgr::solvePS(const Int numThreads)
{
  spdlog::stopwatch sw;
 
  //init(); 
  
  //genPgMesh();
  //genIOPinShapes();
  

  DrPsRouter r(*this, numThreads);
  bool b = r.solve(false, false);

  //PostRoute post(_cir);
  //post.patch();

  spdlog::info("[RouteDrMgr] {:<30} Elapsed: {:.3f} s", "DR (PS)", sw);
  return b;
}

void RouteDrMgr::init(const bool useM1, const Int maxRgIdx)
{
  Int i;
  Net* pNet;

  computePinAcs();
  computeNetBbox();
  buildGrids(useM1, maxRgIdx);

  setNetSymCstrs();
  setNetPathMatchingCstrs();

  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isPower()) {
      pNet->init();
    }
  }
  //Cir_ForEachNet(_cir, pNet, i) {
    //if (pNet->hasSymNet()) {
      //spdlog::info("sym {} {} {}", pNet->name(), pNet->symNet().name(), (pNet->isVerSym() ? pNet->symAxisX() : pNet->symAxisY()));
    //}
    //else if (pNet->isSelfSym()) {
      //spdlog::info("selfsym {}", pNet->name(), (pNet->isVerSym() ? pNet->symAxisX() : pNet->symAxisY()));
    //}
  //}
}

void RouteDrMgr::computePinAcs()
{
  DrcMgr drc(_cir);

  Int i;
  Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (pNet->isPower()) {
      continue;
    }
    for (Pin* pPin : pNet->vpPins()) {
      Pin& pin = *pPin;
      if (pin.isIO()) {
        continue;
      }
      Vector<Point3d<Int>> vAcs;
      for (Int j = 0; j < pin.numBoxes(); ++j) {
        const Int z = pin.boxLayerIdx(j);
        const Box<Int>& box = pin.box(j);
        computeBoxAcs(box, z, vAcs);
      }
      
      std::sort(vAcs.begin(), vAcs.end());
      vAcs.resize(std::unique(vAcs.begin(), vAcs.end()) - vAcs.begin());

      for (const Point3d<Int>& acs : vAcs) {
        const RouteGrid& rg = _cir.routeGrid(_cir.layer(acs.z()).selfIdx());
        if (rg.hasValidVias(acs.x(), acs.y())) {
          const Via& via = *rg.vpValidVias(acs.x(), acs.y()).at(0);
          if (drc.checkViaSpacing(pin.net(), acs.x(), acs.y(), via, Orient2dE::n)) {
            Int j, layerIdx;
            const Box<Int>* pBox;
            bool isValid = true;
            Via_ForEachMetalLayerIdx(via, layerIdx) {
              Via_ForEachLayerBox(via, layerIdx, pBox, j) {
                Box<Int> b(*pBox);
                b.shift(acs.x(), acs.y());
                if (!drc.checkWireMetalEolSpacing(pin.net(), layerIdx, b)) {
                  isValid = false;
                }
              }
            }
            if (isValid) {
              pin.vAcs().emplace_back(acs);
            }
          }
        }
      }
      if (pin.vAcs().empty()) {
        //std::cerr << pin.net().name() << std::endl;
        //for (Int i = 0; i < pin.numBoxes(); ++i) {
          //const Int z = pin.boxLayerIdx(i);
          //const Box<Int>& box = pin.box(i);
          //std::cerr << _cir.layer(z).name() << " " << box << std::endl;
        //}
        spdlog::warn("Pin {} has no access point", pin.name());
      }
      else {
        //assert(pin.vAcs().size() > 0);
        pin.setSuperAcsIdx(pin.numAcs() / 2);
        //if (pin.net().name() == "dmid5") {
          //for (const auto& pt : vAcs) {
            //std::cerr << (Real)(pt.x()) / 2000 << " " << (Real)(pt.y()) / 2000 << std::endl;
          //}
        //}
      }
    }
  }

}

void RouteDrMgr::computeBoxAcs(const Box<Int>& box, const Int z, Vector<Point3d<Int>>& vAcs)
{
        
  const Layer& layer = _cir.layer(z);
  if (layer.type() != LayerTypeE::metal) {
    return;
  }
  assert(layer.type() == LayerTypeE::metal);

  const Int metalIdx = layer.selfIdx();

  assert(0 <= metalIdx and metalIdx <= _cir.numRouteGrids());
  const RouteGrid* pPreRg = metalIdx == 0 ? nullptr : _cir.pRouteGrid(metalIdx - 1);
  const RouteGrid* pCurRg = metalIdx == _cir.numRouteGrids() ? nullptr : _cir.pRouteGrid(metalIdx);
  
  Vector<Int> vXs, vYs;
  const Int xl = box.xl(), xh = box.xh();
  const Int yl = box.yl(), yh = box.yh();

  auto addXYs = [&] (const RouteGrid* pRg) {
    if (pRg) {
      const Int stepX = pRg->stepX();
      const Int stepY = pRg->stepY();
      const Int numXGrids = pRg->numXGrids();
      const Int numYGrids = pRg->numYGrids();

      const Int sxl = xl - pRg->origin().x();
      const Int syl = yl - pRg->origin().y();
      const Int sxh = xh - pRg->origin().x();
      const Int syh = yh - pRg->origin().y();
      
      const Int startX = sxl - (sxl % stepX), endX = sxh - (sxh % stepX) + stepX;
      const Int startY = syl - (syl % stepY), endY = syh - (syh % stepY) + stepY;
      for (Int x = startX; x < endX; x += stepX) {
        for (Int i = 0; i < numXGrids; ++i) {
          const Int xLoc = x + pRg->xGrid(i);
          if (xl < xLoc and xLoc < xh) {
            if (pRg->xUse(i) == TrackUseE::signal) {
              vXs.emplace_back(xLoc);
            }
          }
        }
      }
      for (Int y = startY; y < endY; y += stepY) {
        for (Int i = 0; i < numYGrids; ++i) {
          const Int yLoc = y + pRg->yGrid(i);
          if (yl < yLoc and yLoc < yh) {
            if (pRg->yUse(i) == TrackUseE::signal) {
              vYs.emplace_back(yLoc);
            }
          }
        }
      }
    }
  };
  addXYs(pPreRg);
  addXYs(pCurRg);
  
  std::sort(vXs.begin(), vXs.end());
  std::sort(vYs.begin(), vYs.end());
  vXs.resize(std::unique(vXs.begin(), vXs.end()) - vXs.begin());
  vYs.resize(std::unique(vYs.begin(), vYs.end()) - vYs.begin());
  
  for (const Int x : vXs) {
    for (const Int y : vYs) {
      Point<Int> pt(x, y);
      if (Box<Int>::bConnect(box, pt)) {
        vAcs.emplace_back(pt.x(), pt.y(), z);
      }
    }
  }
  std::sort(vAcs.begin(), vAcs.end());
  vAcs.resize(std::unique(vAcs.begin(), vAcs.end()) - vAcs.begin());
}

void RouteDrMgr::computeNetBbox()
{
  Int i, j;
  Net* pNet;
  Pin* pPin;
  Cir_ForEachNet(_cir, pNet, i) {
    if (pNet->numPins() > 1) {
      Int mnx = MAX_INT, mxx = MIN_INT;
      Int mny = MAX_INT, mxy = MIN_INT;
      Net_ForEachPin((*pNet), pPin, j) {
        if (pPin->isIO()) {
        }
        else if (pPin->pCell()) {
          for (const Box<Int>& box : pPin->vBoxes()) {
            mnx = std::min(mnx, box.xl());
            mxx = std::max(mxx, box.xh());
            mny = std::min(mny, box.yl());
            mxy = std::max(mxy, box.yh());

          }
          //mnx = std::min(mnx, pPin->cell().loc().x());
          //mxx = std::max(mxx, pPin->cell().loc().x() + pPin->cell().width());
          //mny = std::min(mny, pPin->cell().loc().y());
          //mxy = std::max(mxy, pPin->cell().loc().y() + pPin->cell().height());
        }
        else { // rl pin
          for (const Box<Int>& box : pPin->vBoxes()) {
            mnx = std::min(mnx, box.xl());
            mxx = std::max(mxx, box.xh());
            mny = std::min(mny, box.yl());
            mxy = std::max(mxy, box.yh());
          }
        }
        pNet->setMinPinLayerIdx(std::min(pNet->minPinLayerIdx(), pPin->minLayerIdx()));
        pNet->setMaxPinLayerIdx(std::max(pNet->maxPinLayerIdx(), pPin->maxLayerIdx()));
      }
      pNet->setBbox(Box<Int>(mnx, mny, mxx, mxy));
    }
  }
}

void RouteDrMgr::buildGrids(const bool useM1, const Int maxRgIdx)
{
  const Int xl = _cir.routingBbox().xl();
  const Int yl = _cir.routingBbox().yl();
  const Int xh = _cir.routingBbox().xh();
  const Int yh = _cir.routingBbox().yh();

  for (Int i = 0; i <= _cir.numRouteGrids() and i <= maxRgIdx; ++i) { // 14 (lower than TM10) FIXME in the future
    const RouteGrid* pPreRg = i == 0 ? nullptr : _cir.pRouteGrid(i - 1);
    const RouteGrid* pCurRg = i == _cir.numRouteGrids() ? nullptr : _cir.pRouteGrid(i);

    assert(pPreRg or pCurRg);

    auto addXYs = [&xl, &yl, &xh, &yh] (const RouteGrid* pRg, Vector<Int>& vXs, Vector<Int>& vYs) {
      if (pRg) {
        const Int stepX = pRg->stepX();
        const Int stepY = pRg->stepY();

        const Int sxl = xl - pRg->origin().x();
        const Int syl = yl - pRg->origin().y();
        const Int sxh = xh - pRg->origin().x();
        const Int syh = yh - pRg->origin().y();

        const Int startX = sxl - (sxl % stepX), endX = sxh - (sxh % stepX) + stepX;
        const Int startY = syl - (syl % stepY), endY = syh - (syh % stepY) + stepY;
        for (Int x = startX; x < endX; x += stepX) {
          for (Int j = 0; j < pRg->numXGrids(); ++j) {
            const Int xLoc = x + pRg->xGrid(j);
            if (xl <= xLoc and xLoc <= xh) {
              if (pRg->xUse(j) == TrackUseE::signal) {
                vXs.emplace_back(xLoc);
              }
            }
          }
        }
        for (Int y = startY; y < endY; y += stepY) {
          for (Int j = 0; j < pRg->numYGrids(); ++j) {
            const Int yLoc = y + pRg->yGrid(j);
            if (yl <= yLoc and yLoc <= yh) {
              if (pRg->yUse(j) == TrackUseE::signal) {
                vYs.emplace_back(yLoc);
              }
            }
          }
        }
      }
    };
    Vector<Int> vPreXs, vPreYs;
    Vector<Int> vXs, vYs;
    addXYs(pPreRg, vPreXs, vPreYs);
    addXYs(pCurRg, vXs, vYs);
    vXs.insert(vXs.end(), vPreXs.begin(), vPreXs.end());
    vYs.insert(vYs.end(), vPreYs.begin(), vPreYs.end());
    std::sort(vXs.begin(), vXs.end());
    std::sort(vYs.begin(), vYs.end());
    vXs.resize(std::unique(vXs.begin(), vXs.end()) - vXs.begin());
    vYs.resize(std::unique(vYs.begin(), vYs.end()) - vYs.begin());
    
    const Int z = _cir.metalLayer(i).idx(); // metalLayerIdx -> layerIdx
    assert(_cir.layer(z).type() == LayerTypeE::metal);

    const bool isVer = pCurRg ? pCurRg->isLowerLayerVer() : pPreRg->isUpperLayerVer();
    
    if (isVer) {
      for (size_t j = 0; j < vXs.size(); ++j) {
        const Int x = vXs.at(j);
        for (size_t k = 1; k < vYs.size(); ++k) {
          const Int yl = vYs.at(k - 1);
          const Int yh = vYs.at(k);
          if (k == 1) {
            addGridNode(x, yl, z);
          }
          const Int nId = addGridNode(x, yh, z);
          if (z > _cir.layer(0).idx() or useM1) {
            const Int eId = addGridEdge(nId - 1, nId);
            addAdjEdgeIds(eId);
          }
        }
      }
    }
    else {
      for (size_t j = 0; j < vYs.size(); ++j) {
        const Int y = vYs.at(j);
        for (size_t k = 1; k < vXs.size(); ++k) {
          const Int xl = vXs.at(k - 1);
          const Int xh = vXs.at(k);
          if (k == 1) {
            addGridNode(xl, y, z);
          }
          const Int nId = addGridNode(xh, y, z);
          if (z > _cir.layer(0).idx() or useM1) {
            const Int eId = addGridEdge(nId - 1, nId);
            addAdjEdgeIds(eId);
          }
        }
      }
    }
    // add via edges
    if (pPreRg) {
      assert(z == pPreRg->upperLayerIdx());
      const Int zl = pPreRg->lowerLayerIdx(), zh = z;
      for (const Int x : vPreXs) {
        if (!std::binary_search(vXs.begin(), vXs.end(), x))
          continue;
        for (const Int y : vPreYs) {
          if (!std::binary_search(vYs.begin(), vYs.end(), y))
            continue;
          if (pPreRg->hasValidVias(x, y)) { // only add edges with valid vias
            const Int uId = addGridNode(x, y, zl);
            const Int vId = addGridNode(x, y, zh);
            const Int eId = addGridEdge(uId, vId);
            addAdjEdgeIds(eId);
          }
        }
      }
    }
  }
  _spatialEdges = SpatialMap3d<Int, Int>(_vSpatialEdgeVals);
  //for (Int i = 0; i < _vvGridNodes.at(reg.idx()).size(); ++i) {
    ////std::cerr << _vvGridNodes.at(reg.idx()).at(i) << " ";
    //Int cnt = 0;
    //const GridNode& u = gridNode(reg, i);
    //for (Int eId : _vvvAdjEdgeIds.at(reg.idx()).at(i)) {
      ////std::cerr << eId << " ";
      //if (eId == -1)
        //++cnt;
      //else {
        //const GridEdge& e = gridEdge(reg, eId);
        //const GridNode& v = adjNode(reg, e, u);
        //assert(Segment3d<Int>(u, v).b90());
      //}
    //}
    //assert(cnt <= 2);
    ////std::cerr << std::endl;
  //}

}

void RouteDrMgr::setNetSymCstrs()
{
  Int i;
  const RouteSymCstr* pCstr;
  Cir_ForEachRouteSymCstr(_cir, pCstr, i) {
    const RouteSymCstr& sc = *pCstr;
    const SymAxisE axis = sc.axis();
    
    for (Int j = 0; j < sc.numNets(); ++j) {
      Net& na = const_cast<Net&>(sc.net(j));
      na.setSymAxis(axis);
      const SymPartE sp = sc.symPart(j);
      if (sp == SymPartE::a) {
        Net& nb = const_cast<Net&>(sc.symNet(j));
        na.setSymNet(&nb);
        nb.setSymNet(&na);
        nb.setSymAxis(axis);
        const Int symAxis = findSymAxis(na, nb, na.isVerSym());
        if (na.isVerSym()) {
          na.setSymAxisX(symAxis);
          nb.setSymAxisX(symAxis);
        }
        else {
          na.setSymAxisY(symAxis);
          nb.setSymAxisY(symAxis);
        }
        na.setType(NetTypeE::critical);
        nb.setType(NetTypeE::critical);
      }
      else if (sp == SymPartE::self) {
        na.setSelfSym(true); 
        const Int symAxis = findSelfSymAxis(na, na.isVerSym());
        if (na.isVerSym()) {
          na.setSymAxisX(symAxis);
        }
        else {
          na.setSymAxisY(symAxis);
        }
        na.setType(NetTypeE::critical);
      }
    }

  }
  
}

void RouteDrMgr::setNetPathMatchingCstrs()
{
  Int i;
  RoutePathMatchCstr* pCstr;
  Cir_ForEachRoutePathMatchCstr(_cir, pCstr, i) {
    const RoutePathMatchCstr& sc = *pCstr;
    for (Net* pNet : sc.vpNets()) {
      pNet->setRoutePathMatchCstr(pCstr);
      pNet->setType(NetTypeE::critical);
    }
    
  }
}

Int RouteDrMgr::findSymAxis(const Net& na, const Net& nb, const bool isVer) const
{
  auto accum = [&] (const Pin& pin) -> Pair<Int, Int> {
    Int i, layerIdx;
    const Box<Int>* pBox;
    Int sum = 0, boxCnt = 0;
    if (isVer) {
      Pin_ForEachLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
          sum += pBox->centerX();
          ++boxCnt;
        }
      }
    }
    else {
      Pin_ForEachLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
          sum += pBox->centerY();
          ++boxCnt;
        }
      }
    }
    return std::make_pair(sum, boxCnt);
  };

  Int maxDeg = 0;
  Int axis = 0;
  for (const Pin* pa : na.vpPins()) {
    if (pa->isIO()) {
      continue;
    }
    const auto wa = accum(*pa);
    for (const Pin* pb : nb.vpPins()) {
      if (pb->isIO()) {
        continue;
      }
      const auto wb = accum(*pb);
      const Int symAxis = (wa.first + wb.first) / (wa.second + wb.second);
      const Real deg = degPinSym(na, nb, isVer, symAxis);
      if (deg > maxDeg) {
        maxDeg = deg;
        axis = symAxis;
      }
    }
  }
  return axis;
}

Int RouteDrMgr::findSelfSymAxis(const Net& n, const bool isVer) const
{
  auto accum = [&] (const Pin& pin) -> Pair<Int, Int> {
    Int i, layerIdx;
    const Box<Int>* pBox;
    Int sum = 0, boxCnt = 0;
    if (isVer) {
      Pin_ForEachLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
          sum += pBox->centerX();
          ++boxCnt;
        }
      }
    }
    else {
      Pin_ForEachLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
          sum += pBox->centerY();
          ++boxCnt;
        }
      }
    }
    return std::make_pair(sum, boxCnt);
  };

  Int maxDeg = 0;
  Int axis = 0;
  for (Int i = 0; i < n.numPins() - 1; ++i) {
    const Pin& pi = n.pin(i);
    if (pi.isIO()) {
      continue;
    }
    const auto wi = accum(pi);
    for (Int j = i + 1; j < n.numPins(); ++j) {
      const Pin& pj = n.pin(j);
      if (pj.isIO()) {
        continue;
      }
      const auto wj = accum(pj);
      const Int symAxis = (wi.first + wj.first) / (wi.second + wj.second);
      const Real deg = degPinSelfSym(n, isVer, symAxis);
      if (deg > maxDeg) {
        maxDeg = deg;
        axis = symAxis;
      }
    }
  }
  return axis;
}

Real RouteDrMgr::degPinSym(const Net& na, const Net& nb, const bool isVer, const Int symAxis) const
{
  if (&na == &nb) {
    return degPinSelfSym(na, isVer, symAxis);
  }

  Vector<Vector<Box<Int>>> vvBoxes;
  addPinShapes(nb, vvBoxes);

  // count matched pins
  Int i, match = 0;
  const Pin* pPin;
   
  if (isVer) {
    Net_ForEachPin(na, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      if (hasSymPinX(*pPin, symAxis, vvBoxes)) {
        ++match;
      }
    }
  }
  else {
    Net_ForEachPin(na, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      if (hasSymPinY(*pPin, symAxis, vvBoxes)) {
        ++match;
      }
    }
  }
  const Int numPins = std::max(na.numPins(), nb.numPins());
  return static_cast<Real>(match) / numPins * 2;
} 

Real RouteDrMgr::degPinSelfSym(const Net& n, const bool isVer, const Int symAxis) const
{ 
  Vector<Vector<Box<Int>>> vvBoxes;
  addPinShapes(n, vvBoxes);

  // count matched pins
  Int i, match = 0;
  const Pin* pPin;

  if (isVer) {
    Net_ForEachPin(n, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      if (hasSymPinX(*pPin, symAxis, vvBoxes)) {
        ++match;
      }
    }
  }
  else {
    Net_ForEachPin(n, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      if (hasSymPinY(*pPin, symAxis, vvBoxes)) {
        ++match;
      }
    }
  }
  return static_cast<Real>(match) / n.numPins();
}

bool RouteDrMgr::isCrossSym(const Net& net) const
{
  if (!net.hasSymNet()) {
     return false;
  }
  const Net& symNet = net.symNet();
  return isCrossNet(net) and isCrossNet(symNet);
}

bool RouteDrMgr::isCrossNet(const Net& net) const 
{
  bool hasPartA = false, hasPartB = false;
  Int i, j, layerIdx;
  const Pin* pPin; 
  const Box<Int>* pBox;
  if (net.isVerSym()) {
    Net_ForEachPin(net, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      const Pin& pin = *pPin;
      Pin_ForEachMetalLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
          if (pBox->xh() < net.symAxisX()) {
            hasPartA = true;
          }
          if (pBox->xl() > net.symAxisX()) {
            hasPartB = true;
          }
        }
      }
    }
  }
  else if (net.isHorSym()) {
    Net_ForEachPin(net, pPin, i) {
      if (pPin->isIO()) {
        continue;
      }
      const Pin& pin = *pPin;
      Pin_ForEachMetalLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
          if (pBox->yh() < net.symAxisY()) {
            hasPartA = true;
          }
          if (pBox->yl() > net.symAxisY()) {
            hasPartB = true;
          }
        }
      }
    }
  }
  return hasPartA and hasPartB;
}

bool RouteDrMgr::isPartAPin(const Pin& pin) const
{
  const Net& net = pin.net();
  Int i, layerIdx;
  const Box<Int>* pBox;
  if (net.isVerSym()) {
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
        if (pBox->xh() >= net.symAxisX()) {
          return false;
        }
      }
    }
  }
  else if (net.isHorSym()) {
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
        if (pBox->yh() >= net.symAxisY()) {
          return false;
        }
      }
    }
  }
  return true;
}

bool RouteDrMgr::isPartBPin(const Pin& pin) const
{
  const Net& net = pin.net();
  Int i, layerIdx;
  const Box<Int>* pBox;
  if (net.isVerSym()) {
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
        if (pBox->xl() <= net.symAxisX()) {
          return false;
        }
      }
    }
  }
  else if (net.isHorSym()) {
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
        if (pBox->yl() <= net.symAxisY()) {
          return false;
        }
      }
    }
  }
  return true;
}

bool RouteDrMgr::hasSymPin(const Pin& pin, const bool isVer, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const
{
  if (isVer) {
    return hasSymPinX(pin, symAxis, vvSymBoxes);
  }
  return hasSymPinY(pin, symAxis, vvSymBoxes);
}

bool RouteDrMgr::hasSymPinX(const Pin& pin, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const
{
  Int i, layerIdx;
  const Box<Int>* pBox;
  Pin_ForEachMetalLayerIdx(pin, layerIdx) {
    Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
      Box<Int> symBox(*pBox);
      symBox.flipX(symAxis);
      if (!std::binary_search(vvSymBoxes[layerIdx].begin(), vvSymBoxes[layerIdx].end(), symBox)) {
        return false;
      }
    }
  }
  return true;
}

bool RouteDrMgr::hasSymPinY(const Pin& pin, const Int symAxis, const Vector<Vector<Box<Int>>>& vvSymBoxes) const
{
  Int i, layerIdx;
  const Box<Int>* pBox;
  Pin_ForEachMetalLayerIdx(pin, layerIdx) {
    Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
      Box<Int> symBox(*pBox);
      symBox.flipY(symAxis);
      if (!std::binary_search(vvSymBoxes[layerIdx].begin(), vvSymBoxes[layerIdx].end(), symBox)) {
        return false;
      }
    }
  }
  return true;
}

Pin& RouteDrMgr::symPin(const Pin& pin, const Vector<Vector<Box<Int>>>& vvSymBoxes) const
{
  return *pSymPin(pin, vvSymBoxes);
}

Pin* RouteDrMgr::pSymPin(const Pin& pin, const Vector<Vector<Box<Int>>>& vvSymBoxes) const
{
  const Net& net = pin.net();
  assert(hasSymPin(pin, net.isVerSym(), (net.isVerSym() ? net.symAxisX() : net.symAxisY()), vvSymBoxes));
   
  Box<Int> symBox = pin.box(pin.minLayerIdx(), 0);
  if (net.isVerSym()) {
    symBox.flipX(net.symAxisX());
  }
  else if (net.isHorSym()) {
    symBox.flipY(net.symAxisY());
  }
  Vector<Pin*> vpPins;
  _cir.spatialPins(pin.minLayerIdx()).query(symBox, vpPins);
  //assert(vpPins.size() == 1);
  return vpPins.empty() ? nullptr : vpPins.at(0);
}


void RouteDrMgr::addPinShapes(const Net& n, Vector<Vector<Box<Int>>>& vvBoxes) const
{
  vvBoxes.resize(_cir.numLayers());

  Int i, j, layerIdx;
  const Pin* pPin;
  const Box<Int>* pBox;
  Net_ForEachPin(n, pPin, i) {
    if (pPin->isIO()) {
      continue;
    }
    Pin_ForEachLayerIdx((*pPin), layerIdx) {
      Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
        vvBoxes[layerIdx].emplace_back(*pBox);
      }
    }
  }
  for (auto& vBoxes : vvBoxes) {
    std::sort(vBoxes.begin(), vBoxes.end());
  }

}

Int RouteDrMgr::addGridNode(const Int x, const Int y, const Int z)
{
  const GridNode n(x, y, z);
  
  auto it = _mGridNode2Idx.find(n);
  if (it == _mGridNode2Idx.end()) {
    const Int idx = _vGridNodes.size();
    _mGridNode2Idx.emplace(n, idx);
    _vGridNodes.emplace_back(n);
    _vvAdjEdgeIds.emplace_back(Vector<Int>(4, -1));
    return idx;
  }
  return it->second;
}

Int RouteDrMgr::addGridEdge(const Int u, const Int v)
{
  const GridEdge e(u, v);
  const GridNode& nu = _vGridNodes.at(u);
  const GridNode& nv = _vGridNodes.at(v);
  assert(nu.x() <= nv.x() and nu.y() <= nv.y() and nu.z() <= nv.z());

  auto it = _mGridEdge2Idx.find(e);
  if (it == _mGridEdge2Idx.end()) {
    const Int idx = _vGridEdges.size();
    _mGridEdge2Idx.emplace(e, idx);
    _vGridEdges.emplace_back(e);

    _vSpatialEdgeVals.emplace_back(spatial3d::b_box<Int>(nu, nv), idx);
    return idx;
  }
  return it->second;
}

void RouteDrMgr::addAdjEdgeIds(const Int eId)
{
  const GridEdge& e = _vGridEdges.at(eId);
  const GridNode& u = _vGridNodes.at(e.u);
  const GridNode& v = _vGridNodes.at(e.v);
  const Int uId = e.u;
  const Int vId = e.v;
  if (u.z() == v.z()) {
    assert(u.x() <= v.x() and u.y() <= v.y());
    _vvAdjEdgeIds[uId][1] = eId;
    _vvAdjEdgeIds[vId][0] = eId;
  }
  else {
    assert(u.x() == v.x() and u.y() == v.y());
    _vvAdjEdgeIds[uId][3] = eId;
    _vvAdjEdgeIds[vId][2] = eId;
  }

}

const RouteDrMgr::GridNode& RouteDrMgr::gridNode(const Int i) const
{
  return _vGridNodes.at(i);
}

const RouteDrMgr::GridEdge& RouteDrMgr::gridEdge(const Int i) const
{
  return _vGridEdges.at(i);
}

Int RouteDrMgr::gridNodeIdx(const GridNode& n) const
{
  return _mGridNode2Idx.at(n);
}

Int RouteDrMgr::gridEdgeIdx(const GridEdge& e) const
{
  return _mGridEdge2Idx.at(e);
}

Int RouteDrMgr::gridEdgeIdx(Int u, Int v) const
{
  if (u > v) {
    std::swap(u, v);
  }
  const GridEdge e(u, v);
  auto it = _mGridEdge2Idx.find(e);
  assert(it != _mGridEdge2Idx.end());
  return it->second;
}

void RouteDrMgr::vGridEdgeIds(const Box<Int>& box, const Int layerIdx, Vector<Int>& vIds) const
{
  const Point3d<Int> min_corner(box.xl(), box.yl(), layerIdx);
  const Point3d<Int> max_corner(box.xh(), box.yh(), layerIdx);
  _spatialEdges.query(min_corner, max_corner, vIds, spatial3d::QueryType::covered_by);
  //for (const Int eIdx : vIds) {
    //const auto& e = _vGridEdges.at(eIdx);
    //const auto& u = _vGridNodes.at(e.u);
    //const auto& v = _vGridNodes.at(e.v);
    //const Segment3d<Int> s(u, v);
    //assert(!s.bVia());
    //assert(Box<Int>::bConnect(box, u.to2d()));
    //assert(Box<Int>::bConnect(box, v.to2d()));
  //}
}

void RouteDrMgr::vGridEdgeIds(const Segment3d<Int>& seg, Vector<Int>& vIds) const
{
  assert(seg.b90());
  //_spatialEdges.query(seg.min_corner(), seg.max_corner(), v, spatial3d::QueryType::covered_by);
  
  const auto& p0 = seg.min_corner();
  const auto& p1 = seg.max_corner();
  
  //if (!hasNode(p0)) {
    //std::cerr << p0 << std::endl;
  //}
  //if (!hasNode(p1)) {
    //std::cerr << p1 << std::endl;
  //}
  assert(hasNode(p0));
  assert(hasNode(p1));

  auto getDim = [] (const auto& p0, const auto& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  const auto segDim = getDim(p0, p1);

  std::function<void(const Int nodeIdx)>
    bfs = [&] (const Int nodeIdx) {

      const auto& vAdjEdgeIds = _vvAdjEdgeIds.at(nodeIdx);

      switch (segDim) {
        case 0: { // x direction
                  const Int nxtEdgeIdx  = vAdjEdgeIds.at(1);
                  if (isValidEdge(nxtEdgeIdx)) {
                    const auto& nxtEdge = gridEdge(nxtEdgeIdx);
                    assert(nxtEdge.u == nodeIdx);
                    const auto& nxtNode = gridNode(nxtEdge.v);

                    assert(gridNode(nxtEdge.v).x() >  gridNode(nodeIdx).x());
                    assert(gridNode(nxtEdge.v).y() == gridNode(nodeIdx).y());
                    assert(gridNode(nxtEdge.v).z() == gridNode(nodeIdx).z());
                    if (nxtNode.x() <= p1.x()) {
                      vIds.emplace_back(nxtEdgeIdx);
                      bfs(nxtEdge.v);
                    }
                  }
                  break;
                }
        case 1: { // y direction
                  const Int nxtEdgeIdx  = vAdjEdgeIds.at(1);
                  if (isValidEdge(nxtEdgeIdx)) {
                    const auto& nxtEdge = gridEdge(nxtEdgeIdx);
                    assert(nxtEdge.u == nodeIdx);
                    const auto& nxtNode = gridNode(nxtEdge.v);

                    assert(gridNode(nxtEdge.v).x() == gridNode(nodeIdx).x());
                    assert(gridNode(nxtEdge.v).y() >  gridNode(nodeIdx).y());
                    assert(gridNode(nxtEdge.v).z() == gridNode(nodeIdx).z());
                    if (nxtNode.y() <= p1.y()) {
                      vIds.emplace_back(nxtEdgeIdx);
                      bfs(nxtEdge.v);
                    }
                  }
                  break;
                }
        case 2: { // z direction
                  const Int nxtEdgeIdx  = vAdjEdgeIds.at(3);
                  if (isValidEdge(nxtEdgeIdx)) {
                    const auto& nxtEdge = gridEdge(nxtEdgeIdx);
                    assert(nxtEdge.u == nodeIdx);
                    const auto& nxtNode = gridNode(nxtEdge.v);

                    assert(gridNode(nxtEdge.v).x() == gridNode(nodeIdx).x());
                    assert(gridNode(nxtEdge.v).y() == gridNode(nodeIdx).y());
                    assert(gridNode(nxtEdge.v).z() >  gridNode(nodeIdx).z());
                    if (nxtNode.z() <= p1.z()) {
                      vIds.emplace_back(nxtEdgeIdx);
                      bfs(nxtEdge.v);
                    }
                  }
                  break;
                }
      }
    };
  const Int p0Idx = gridNodeIdx(p0);
  bfs(p0Idx);

  assert(_vGridNodes.at(_vGridEdges.at(vIds.front()).u) == p0);
  assert(_vGridNodes.at(_vGridEdges.at(vIds.back()).v) == p1);
}

Segment3d<Int> RouteDrMgr::gridEdge2Seg(const GridEdge& e) const
{
  const GridNode& u = _vGridNodes.at(e.u);
  const GridNode& v = _vGridNodes.at(e.v);
  return Segment3d<Int>(u, v);
}

Segment3d<Int> RouteDrMgr::gridEdge2Seg(const Int eIdx) const
{
  const GridEdge& e = _vGridEdges.at(eIdx);
  return gridEdge2Seg(e);
}

const RouteDrMgr::GridNode& RouteDrMgr::adjNode(const GridEdge& e, const GridNode& n) const
{
  const GridNode& eu = _vGridNodes.at(e.u);
  const GridNode& ev = _vGridNodes.at(e.v);
  assert(eu == n or ev == n);
  return eu == n ? ev : eu;
}

const Vector<Int>& RouteDrMgr::vAdjEdgeIds(const GridNode& n) const
{
  return _vvAdjEdgeIds.at(gridNodeIdx(n));
}

const Vector<Int>& RouteDrMgr::vAdjEdgeIds(const Int i) const
{
  return _vvAdjEdgeIds.at(i);
}

bool RouteDrMgr::isValidEdge(const Int i) const
{
  return i >= 0;
}

bool RouteDrMgr::hasNode(const GridNode& n) const
{
  return _mGridNode2Idx.find(n) != _mGridNode2Idx.end();
}

bool RouteDrMgr::hasEdge(const GridEdge& e) const
{
  return _mGridEdge2Idx.find(e) != _mGridEdge2Idx.end();
}


void RouteDrMgr::genIOPinShapes() 
{
  spdlog::info("[RouteDrMgr] Generate IO pin shapes");

  Int i;
  Pin* pPin;

  Cir_ForEachIOPin(_cir, pPin, i) {
    const Region& reg = pPin->net().region();
    const IOPinLocE ioPinLoc = _cir.ioPinLoc(i);
    const Box<Int>& box = _cir.ioPinBbox(i);
    //if (mBox2IOPinIds.find(bbox) == mBox2IOPinIds.end()) {
      //mBox2IOPinIds.emplace(bbox, Vector<Int>());
    //}
    //mBox2IOPinIds[bbox].emplace_back(i);
    const RouteGrid& rg = pPin->net().ioRouteGrid();

    const Int xLayerIdx = rg.xLayerIdx();
    const Int yLayerIdx = rg.yLayerIdx();
    const MetalLayer& xLayer = *static_cast<const MetalLayer*>(_cir.pLayer(rg.xLayerIdx()));
    const MetalLayer& yLayer = *static_cast<const MetalLayer*>(_cir.pLayer(rg.yLayerIdx()));


    switch (ioPinLoc) {
      case IOPinLocE::left: {
        const Int x = box.xh();
        Int y = rg.ceilY(box.yl());
        while (true) {
          if (y > reg.bboxInner().yh()) {
            spdlog::warn("[RouteDrMgr] IO pin {} out of boundaries", pPin->name());
          }
          const Int w = rg.wireWidth(Point3d<Int>(x, y, yLayerIdx));
          const Int hw = w / 2;
          const Int ext = yLayer.length2ValidLength(yLayer.area() / w) / 2;
          const Box<Int> b(x - ext, y - hw, x + ext, y + hw);
          if (rg.isYLocPowerTrack(y) or _cir.spatialNets(yLayerIdx).exist(b)) {
            y = rg.ceilY(y + 1);
          }
          else {
            pPin->addBox(yLayerIdx, b);
            _cir.spatialNets(yLayerIdx).insert(b, pPin->pNet());
            break;
          }
        }
        break;
      }
      case IOPinLocE::right: {
        const Int x = box.xl();
        Int y = rg.ceilY(box.yl());
        while (true) {
          if (y > reg.bboxInner().yh()) {
            spdlog::warn("[RouteDrMgr] IO pin {} out of boundaries", pPin->name());
          }
          const Int w = rg.wireWidth(Point3d<Int>(x, y, yLayerIdx));
          const Int hw = w / 2;
          const Int ext = yLayer.length2ValidLength(yLayer.area() / w) / 2;
          const Box<Int> b(x - ext, y - hw, x + ext, y + hw);
          if (rg.isYLocPowerTrack(y) or _cir.spatialNets(yLayerIdx).exist(b)) {
            y = rg.ceilY(y + 1);
          }
          else {
            pPin->addBox(yLayerIdx, b);
            _cir.spatialNets(yLayerIdx).insert(b, pPin->pNet());
            break;
          }
        }
        break;
      }
      case IOPinLocE::bottom: {
        const Int y = box.yh();
        Int x = rg.ceilX(box.xl());
        while (true) {
          if (x > reg.bboxInner().xh()) {
            spdlog::warn("[RouteDrMgr] IO pin {} out of boundaries", pPin->name());
          }
          const Int w = rg.wireWidth(Point3d<Int>(x, y, xLayerIdx));
          const Int hw = w / 2;
          const Int ext = xLayer.length2ValidLength(xLayer.area() / w) / 2;
          const Box<Int> b(x - hw, y - ext, x + hw, y + ext);
          if (rg.isXLocPowerTrack(x) or _cir.spatialNets(xLayerIdx).exist(b)) {
            x = rg.ceilX(x + 1);
          }
          else {
            pPin->addBox(xLayerIdx, b);
            _cir.spatialNets(xLayerIdx).insert(b, pPin->pNet());
            break;
          }
        }
        break;
      }
      case IOPinLocE::top: {
        const Int y = box.yl();
        Int x = rg.ceilX(box.xl());
        while (true) {
          if (x > reg.bboxInner().xh()) {
            spdlog::warn("[RouteDrMgr] IO pin {} out of boundaries", pPin->name());
          }
          const Int w = rg.wireWidth(Point3d<Int>(x, y, xLayerIdx));
          const Int hw = w / 2;
          const Int ext = xLayer.length2ValidLength(xLayer.area() / w) / 2;
          const Box<Int> b(x - hw, y - ext, x + hw, y + ext);
          if (rg.isXLocPowerTrack(x) or _cir.spatialNets(xLayerIdx).exist(b)) {
            x = rg.ceilX(x + 1);
          }
          else {
            pPin->addBox(xLayerIdx, b);
            _cir.spatialNets(xLayerIdx).insert(b, pPin->pNet());
            break;
          }
        }
        break;
      }
      default:
        assert(false);
    }
  }
}

Box<Int> RouteDrMgr::pathMatchBbox() const
{
  Int xl = MAX_INT, xh = MIN_INT;
  Int yl = MAX_INT, yh = MIN_INT;

  Int i;
  const RoutePathMatchCstr* pCstr; 
  Cir_ForEachRoutePathMatchCstr(_cir, pCstr, i) {
    const Box<Int>& b = pathMatchBbox(*pCstr);
    xl = std::min(b.xl(), xl);
    xh = std::max(b.xh(), xh);
    yl = std::min(b.yl(), yl);
    yh = std::max(b.yh(), yh);
  }
  return Box<Int>(xl, yl, xh, yh);
}

Box<Int> RouteDrMgr::pathMatchBbox(const RoutePathMatchCstr& cstr) const
{
  Int xl = MAX_INT, xh = MIN_INT;
  Int yl = MAX_INT, yh = MIN_INT;

  //Int i, layerIdx;
  //const Box<Int>* pBox;
  for (const auto& pathCstr : cstr.vPathCstrs()) {
    for (const auto& conn : pathCstr.vConns()) {
      const Pin& src = *conn.first;
      const Pin& tar = *conn.second;
      assert(src.pNet() == tar.pNet());

      //Pin_ForEachLayerIdx(src, layerIdx) {
        //Pin_ForEachLayerBox(src, layerIdx, pBox, i) {
          //xl = std::min(pBox->xl(), xl);
          //xh = std::max(pBox->xh(), xh);
          //yl = std::min(pBox->yl(), yl);
          //yh = std::max(pBox->yh(), yh);
        //}
      //}
      //Pin_ForEachLayerIdx(tar, layerIdx) {
        //Pin_ForEachLayerBox(tar, layerIdx, pBox, i) {
          //xl = std::min(pBox->xl(), xl);
          //xh = std::max(pBox->xh(), xh);
          //yl = std::min(pBox->yl(), yl);
          //yh = std::max(pBox->yh(), yh);
        //}
      //}
      xl = std::min(src.cell().xl(), xl);
      xh = std::max(src.cell().xh(), xh);
      yl = std::min(src.cell().yl(), yl);
      yh = std::max(src.cell().yh(), yh);
      xl = std::min(tar.cell().xl(), xl);
      xh = std::max(tar.cell().xh(), xh);
      yl = std::min(tar.cell().yl(), yl);
      yh = std::max(tar.cell().yh(), yh);
    }
  }

  Box<Int> box(xl, yl, xh, yh);
  box.expandX(0.1 * box.width());
  box.expandY(0.1 * box.height());
  return box;
}

//bool RouteDrMgr::insertRLVia(Net& net, const Box<Int>& bbox, const Int metalLayerIdx, const Int xSignalTrack, const Int ySignalTrack, const Orient2dE orient)
//{
  //const MetalLayer& layer = _cir.metalLayer(metalLayerIdx);
  //const RouteGrid& rg = _cir.routeGrid(layer.selfIdx());

  //const Int xl = rg.ceilX(bbox.xl());
  //const Int yl = rg.ceilY(bbox.yl());
  //const Int xh = rg.floorX(bbox.xh());
  //const Int yh = rg.floorY(bbox.yh());

  //const Int x = rg.nextSignalX(xl, xSignalTrack);
  //const Int y = rg.nextSignalY(yl, ySignalTrack);

  //assert(x <= xh and y <= yh);

  //const Point3d<Int> u(x, y, layer.idx());
  
  //return insertRLVia(net, u, orient);
//}

//bool RouteDrMgr::insertRLVia(Net& net, const Point3d<Int>& u, const Orient2dE orient)
//{
  //assert(_cir.layer(u.z()).isMetal());
  //const MetalLayer& lowerLayer = *static_cast<const MetalLayer*>(_cir.pLayer(u.z()));
  //const RouteGrid& lowerRg = _cir.routeGrid(lowerLayer.selfIdx());

  //const Via& via = *lowerRg.vpValidVias(u.x(), u.y()).at(0);
  
  //const Point3d<Int> v(u.x(), u.y(), via.maxLayerIdx());
  //const MetalLayer& upperLayer = *static_cast<const MetalLayer*>(_cir.pLayer(v.z()));
  //const RouteGrid& upperRg = _cir.routeGrid(upperLayer.selfIdx());

  //assert(lowerRg.isOnGrid(u.x(), u.y()));
  //assert(upperRg.isOnGrid(v.x(), v.y()));

  //// check DRC
  //DrcMgr drc(_cir);
  //if (drc.checkViaSpacing(net, u.x(), u.y(), via, orient)) {
    //return false;
  //}


  //const Int rlPinIdx = _vpRLPins.size();
  //const String name = "rl_pin_" + std::to_string(rlPinIdx) + "_" + net.name();
  //_vpRLPins.emplace_back(std::make_unique<Pin>());
  //_vRLPinCenters.emplace_back(u);
  //_vpRLViaRefs.emplace_back(&via);
  //_vRLViaOrients.emplace_back(orient);
  //_mRLPinName2Idx.emplace(name, rlPinIdx);

  //Pin& pin = *_vpRLPins.back();
  //pin.setName(name);
  //pin.setNet(&net);

  //Int i, layerIdx;
  //const Box<Int>* pBox;
  //Via_ForEachLayerIdx(via, layerIdx) {
    //Via_ForEachLayerBox(via, layerIdx, pBox, i) {
      //const Int xl = pBox->xl() + u.x();
      //const Int yl = pBox->yl() + u.y();
      //const Int xh = pBox->xh() + u.x();
      //const Int yh = pBox->yh() + u.y();
      //Box<Int> b(xl, yl, xh, yh);
      //switch (orient) {
        //case Orient2dE::n:                                                                      break;
        //case Orient2dE::w:  b.rotate90(b.centerX(), b.centerY(), false);                        break;
        //case Orient2dE::s:  b.rotate180(b.centerX(), b.centerY());                              break;
        //case Orient2dE::e:  b.rotate90(b.centerX(), b.centerY(), true);                         break;
        //case Orient2dE::fn:                                               b.flipX(b.centerX()); break;
        //case Orient2dE::fw: b.rotate90(b.centerX(), b.centerY(), false);  b.flipX(b.centerX()); break;
        //case Orient2dE::fs:                                               b.flipY(b.centerY()); break;
        //case Orient2dE::fe: b.rotate90(b.centerX(), b.centerY(), true);   b.flipX(b.centerX()); break;
        //default: assert(false);
      //}
      //pin.addBox(layerIdx, b);
    //}
  //}
  //pin.setRL(true);

  //net.addPin(&pin);
  

  //// add to cir spatial
  //Pin_ForEachLayerIdx(pin, layerIdx) {
    //Pin_ForEachLayerBox(pin, layerIdx, pBox, i) {
      //const Box<Int>& b = *pBox;
      //_cir.spatialNets(layerIdx).insert(b, &net);
    //}
  //}

  //return true;
//}

//void RouteDrMgr::saveRLVias2NetResults()
//{
  //Int i, j, layerIdx;
  //const Box<Int>* pBox;
  //for (i = 0; i < static_cast<Int>(_vpRLPins.size()); ++i) {
    //Pin& pin = *_vpRLPins.at(i);
    //Net& net = pin.net();
    //Pin_ForEachLayerIdx(pin, layerIdx) {
      //Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
        //const Box<Int>& b = *pBox;
        //net.sWires().emplace(b, layerIdx);
      //}
    //}
    //const Point3d<Int>& u = _vRLPinCenters.at(i);
    //const Point3d<Int> v(u.x(), u.y(), pin.maxLayerIdx());
    //net.vArcs().emplace_back(u, v);
    //net.vpArcVias().emplace_back(_vpRLViaRefs.at(i));
    //net.vArcViaOrients().emplace_back(_vRLViaOrients.at(i));
    //net.vArcWidthExts().emplace_back(0, 0);
  //}
//}

//void RouteDrMgr::clearRLPins()
//{

  //Int i;
  //Net* pNet;
  ////Cir_ForEachNet(_cir, pNet, i) {
    //////std::cerr << pNet->name() << " " << pNet->numRealPins() << " " << pNet->numPins() << std::endl;
    ////pNet->clearRLPins();
  ////}
  //_vpRLPins.clear();
  //_vRLPinCenters.clear();
  //_vpRLViaRefs.clear();
  //_vRLViaOrients.clear();
  //_mRLPinName2Idx.clear();
//}

Point3d<Int> RouteDrMgr::pinSignalTrackPt(const Net& net, const Int pinIdx, const Box<Int>& bbox) const
{
  assert(0 <= pinIdx and pinIdx < net.numPins());
  
  const Pin& pin = net.pin(pinIdx);
  assert(pin.numBoxes() == 1);
  assert(pin.minLayerIdx() == pin.maxLayerIdx());

  const Int z = pin.minLayerIdx();

  return signalTrackPt(pin.box(0).centerX(), pin.box(0).centerY(), z, bbox);
}

Point3d<Int> RouteDrMgr::signalTrackPt(Int px, Int py, const Int layerIdx, const Box<Int>& bbox) const
{
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));
  const RouteGrid& rg = _cir.routeGrid(layer.selfIdx());

  px = rg.floorX(px);
  py = rg.floorY(py);

  const Int xl = rg.ceilX(bbox.xl());
  const Int yl = rg.ceilY(bbox.yl());
  //const Int xh = rg.floorX(bbox.xh());
  //const Int yh = rg.floorY(bbox.yh());

  const Int xlh = (xl - rg.originX()) / rg.stepX() * rg.stepX() + rg.stepX();
  const Int ylh = (yl - rg.originY()) / rg.stepY() * rg.stepY() + rg.stepY();
  
  Int nx = 0;
  Int x = xl;
  while (x <= xlh) {
    if (rg.isXLocSignalTrack(x)) {
      ++nx;
    }
    x = rg.nextSignalX(x);
  }
  const Int pxl = (px - rg.originX()) / rg.stepX() * rg.stepX();
  x = pxl;
  while (x <= px) {
    if (rg.isXLocSignalTrack(x)) {
      ++nx;
    }
    x = rg.nextSignalX(x);
  }
  
  Int nxPerStep = 0;
  for (Int i = 0; i < rg.numXGrids(); ++i) {
    if (rg.xUse(i) != TrackUseE::power) {
      ++nxPerStep;
    }
  }
  nx += (nxPerStep * (pxl - xlh) / rg.stepX());

  Int ny = 0;
  Int y = yl;
  while (y <= ylh) {
    if (rg.isYLocSignalTrack(y)) {
      ++ny;
    }
    y = rg.nextSignalY(y);
  }
  const Int pyl = (py - rg.originY()) / rg.stepY() * rg.stepY();
  y = pyl;
  while (y <= py) {
    if (rg.isYLocSignalTrack(y)) {
      ++ny;
    }
    y = rg.nextSignalY(y);
  }
  
  Int nyPerStep = 0;
  for (Int i = 0; i < rg.numYGrids(); ++i) {
    if (rg.yUse(i) != TrackUseE::power) {
      ++nyPerStep;
    }
  }
  ny += (nyPerStep * (pyl - ylh) / rg.stepY());

  return Point3d<Int>(nx, ny, layer.selfIdx());
}

Pair<Int, Int> RouteDrMgr::numSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const
{
  const MetalLayer& layer = _cir.metalLayer(metalLayerIdx);
  const RouteGrid& rg = _cir.routeGrid(layer.selfIdx());

  const Int xl = rg.ceilX(bbox.xl());
  const Int yl = rg.ceilY(bbox.yl());
  const Int xh = rg.floorX(bbox.xh());
  const Int yh = rg.floorY(bbox.yh());

  Int x = xl, y = yl, nx = 0, ny = 0;
  while (x <= xh) {
    if (rg.isXLocSignalTrack(x)) {
      ++nx;
    }
    x = rg.nextSignalX(x);
  }
  while (y <= yh) {
    if (rg.isYLocSignalTrack(y)) {
      ++ny;
    }
    y = rg.nextSignalY(y);
  }

  return std::make_pair(nx, ny);
}

Int RouteDrMgr::numXSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const
{
  const MetalLayer& layer = _cir.metalLayer(metalLayerIdx);
  const RouteGrid& rg = _cir.routeGrid(layer.selfIdx());
  
  const Int xl = rg.ceilX(bbox.xl());
  const Int xh = rg.floorX(bbox.xh());
  
  Int x = xl, nx = 0;
  while (x <= xh) {
    if (rg.isXLocSignalTrack(x)) {
      ++nx;
    }
    x = rg.nextSignalX(x);
  }
  return nx;
}

Int RouteDrMgr::numYSignalTracks(const Int metalLayerIdx, const Box<Int>& bbox) const
{
  const MetalLayer& layer = _cir.metalLayer(metalLayerIdx);
  const RouteGrid& rg = _cir.routeGrid(layer.selfIdx());
  
  const Int yl = rg.ceilY(bbox.yl());
  const Int yh = rg.floorY(bbox.yh());

  Int y = yl, ny = 0;
  while (y <= yh) {
    if (rg.isYLocSignalTrack(y)) {
      ++ny;
    }
    y = rg.nextSignalY(y);
  }
  return ny;
}

//Vector<String> RouteDrMgr::rlDataNetNames() const
//{
  //Vector<String> v;

  //Int i;
  //const RoutePathMatchCstr* pCstr;
  //Cir_ForEachRoutePathMatchCstr(_cir, pCstr, i) {
    //const auto& cstr = *pCstr;
    //for (Int j = 0; j < cstr.numConns(); ++j) {
      //const Pin& srcPin = cstr.srcPin(j);
      //const Pin& tarPin = cstr.tarPin(j);
      //assert(srcPin.pNet() == tarPin.pNet());
      //v.emplace_back(srcPin.net().name());
    //}
  //}
  //std::sort(v.begin(), v.end());
  //v.resize(std::unique(v.begin(), v.end()) - v.begin());
  //return v;
//}

//Vector<Point3d<Int>> RouteDrMgr::rlData(const Box<Int>& bbox, const Net& net) const
//{
  //assert(bbox.xl() < bbox.xh() and bbox.yl() < bbox.yh());

  //Vector<Point3d<Int>> v;
  //for (Int i = 0; i < net.numPins(); ++i) {
    //v.emplace_back(pinSignalTrackPt(net, i, bbox));
  //}
  //return v;

//}

//Vector<Pair<String, Point3d<Int>>> RouteDrMgr::rlData(const Box<Int>& bbox) const
//{
  //assert(bbox.xl() < bbox.xh() and bbox.yl() < bbox.yh());

  //FlatHashSet<const Net*> spNets;
  //Int i;
  //const RoutePathMatchCstr* pCstr;
  //Cir_ForEachRoutePathMatchCstr(_cir, pCstr, i) {
    //const auto& cstr = *pCstr;
    //for (Int j = 0; j < cstr.numConns(); ++j) {
      //const Pin& srcPin = cstr.srcPin(j);
      //const Pin& tarPin = cstr.tarPin(j);
      //assert(srcPin.pNet() == tarPin.pNet());
      //spNets.emplace(srcPin.pNet());
    //}
  //}

  //Vector<Pair<String, Point3d<Int>>> v;
  //for (const auto& pNet : spNets) {
    //const Net& net = *pNet;
    //for (Int j = 0; j < net.numPins(); ++j) {
      //v.emplace_back(net.name(), pinSignalTrackPt(net, j, bbox));
    //}
  //}
  //return v;
//}

void RouteDrMgr::resetRouting()
{
  // clear nets
  Int i;
  Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    pNet->vRoutables().clear();
    pNet->vRoutableOrder().clear();
    pNet->clearRouting();
  }
  
  // clear spatial
  Cir_ForEachLayerIdx(_cir, i) {
    _cir.spatialNets(i).clear();
  }
  _cir.buildSpatialNets();
  
}

PROJECT_NAMESPACE_END
