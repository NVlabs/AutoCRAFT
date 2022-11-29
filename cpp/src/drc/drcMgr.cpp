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
#include "drcMgr.hpp"
#include "geo/box2polygon.hpp"

PROJECT_NAMESPACE_START

bool DrcMgr::checkWireValidWidth(const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));
  
  // assume only have right way wires in every layer
  const Int w = b.width() < b.height() ? b.width() : b.height();
  
  if (w < layer.minWidth() or w > layer.maxWidth()) {
    return false;
  }
  return layer.isValidWidth(w);
}

bool DrcMgr::checkWireValidLength(const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));
  
  // assume only have right way wires in every layer
  const Int l = b.width() > b.height() ? b.width() : b.height();

  return layer.isValidLength(l);
}


bool DrcMgr::checkWireMetalSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));

  Box<Int> checkBox(b);
  checkBox.expand(layer.spacing() - 1);

  // obs
  Vector<Obs*> vpTouchedObs;
  _cir.spatialObs(layerIdx).query(checkBox, vpTouchedObs);
  if (!vpTouchedObs.empty()) {
    return false;
  }

  // pin
  Vector<Pair<Box<Int>, Pin*>> vpTouchedPins;
  _cir.spatialPins(layerIdx).queryBoth(checkBox, vpTouchedPins);
  for (const auto& item : vpTouchedPins) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second->pNet();
    if (pTouchedNet != &net) {
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }

  // nets
  Vector<Pair<Box<Int>, Net*>> vpTouchedNets;
  _cir.spatialNets(layerIdx).queryBoth(checkBox, vpTouchedNets);
  for (const auto& item : vpTouchedNets) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second;
    if (pTouchedNet != &net) {
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }

  return true;
}

bool DrcMgr::checkWireMetalPrlSpacing(const Net& net, const Int layerIdx, const Box<Int>& b, const Int prl) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));

  const Int width = std::min(b.width(), b.height());
  const Int spacing = layer.prlSpacing(width, prl);

  Box<Int> checkBox(b);
  if (layer.isVer()) {
    checkBox.expandX(spacing - 1);
  }
  else {
    checkBox.expandY(spacing - 1);
  }

  // obs
  Vector<Obs*> vpTouchedObs;
  _cir.spatialObs(layerIdx).query(checkBox, vpTouchedObs);
  if (!vpTouchedObs.empty()) {
    //const Obs& obs = *vpTouchedObs[0];
    //std::cerr << "Obs" << std::endl;
    //std::cerr << layerIdx << " "<< b << " " << obs.box() << std::endl;
    return false;
  }
  // pins
  Vector<Pair<Box<Int>, Pin*>> vpTouchedPins;
  _cir.spatialPins(layerIdx).queryBoth(checkBox, vpTouchedPins);
  for (const auto& item : vpTouchedPins) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second->pNet();
    if (pTouchedNet != &net) {
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }

  // nets
  Vector<Pair<Box<Int>, Net*>> vpTouchedNets;
  _cir.spatialNets(layerIdx).queryBoth(checkBox, vpTouchedNets);
  for (const auto& item : vpTouchedNets) {
    const Box<Int>& box = item.first;
    const Net* pTouchedNet = item.second;
    if (pTouchedNet != &net) {
    //spdlog::info("Net");
      //std::cerr << layerIdx << " " << b  << " " << box << " " << pTouchedNet->name() << std::endl;
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
    //spdlog::info("SameNet");
        return false;
      }
    }
  }

  return true;

}

bool DrcMgr::checkWireMetalEolSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const
{
  // according to simplilifed yaml rules
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));

  const Int w = std::min(b.width(), b.height());
  const Int eolIdx = layer.eolIdx(w);
  const Int eolSpace = layer.eolSpacing(eolIdx);
  const Int eolWithin = layer.eolWithin(eolIdx);

  Box<Int> checkBox0, checkBox1;
  if (b.width() > b.height()) {
    checkBox0.set(b.xl() - eolSpace + 1,
                  b.yl() - eolWithin + 1,
                  b.xl() - 1,
                  b.yh() + eolWithin - 1);
    checkBox1.set(b.xh() + 1,
                  b.yl() - eolWithin + 1,
                  b.xh() + eolSpace - 1,
                  b.yh() + eolWithin - 1);
  }
  else {
    checkBox0.set(b.xl() - eolWithin + 1,
                  b.yl() - eolSpace + 1,
                  b.xh() + eolWithin - 1,
                  b.yl() - 1);
    checkBox1.set(b.xl() - eolWithin + 1,
                  b.yh() + 1,
                  b.xh() + eolWithin - 1,
                  b.yh() + eolSpace - 1);
  }

  // obs
  Vector<Obs*> vpTouchedObs;
  _cir.spatialObs(layerIdx).query(checkBox0, vpTouchedObs);
  _cir.spatialObs(layerIdx).query(checkBox1, vpTouchedObs);
  if (!vpTouchedObs.empty()) {
    //const Obs& obs = *vpTouchedObs[0];
    //std::cerr << "Obs" << std::endl;
    //std::cerr << layerIdx << " "<< b << " " << obs.box() << std::endl;
    return false;
  }

  // pins
  Vector<Pair<Box<Int>, Pin*>> vpTouchedPins;
  _cir.spatialPins(layerIdx).queryBoth(checkBox0, vpTouchedPins);
  _cir.spatialPins(layerIdx).queryBoth(checkBox1, vpTouchedPins);
  for (const auto& item : vpTouchedPins) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second->pNet();
    if (pTouchedNet != &net) {
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }

  // nets
  // assume only have right way wires in every layer
  Vector<Pair<Box<Int>, Net*>> vpTouchedNets;
  _cir.spatialNets(layerIdx).queryBoth(checkBox0, vpTouchedNets);
  _cir.spatialNets(layerIdx).queryBoth(checkBox1, vpTouchedNets);
  for (const auto& item : vpTouchedNets) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second;
    if (pTouchedNet != &net) {
      //std::cerr << layer.name() << " " <<net.name() << " " << b << " " << pTouchedNet->name() <<" " << box << std::endl;
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }
  //if (!vpTouchedNets.empty()) {
    ////std::cerr << net.name() << " " << vpTouchedNets[0].second->name() << std::endl;;
    ////std::cerr << layerIdx << " "<< b << " " << vpTouchedNets[0].first << std::endl;
    //return false;
  //}

  
  return true;
}

bool DrcMgr::checkWireCutSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isCut());
  const CutLayer& layer = *static_cast<const CutLayer*>(_cir.pLayer(layerIdx));
  
  const Int spaceX = layer.spaceX();
  const Int spaceY = layer.spaceY();

  //spdlog::info("{} {} {}", layer.name(), spaceX, spaceY);

  Box<Int> checkBox0(b);
  Box<Int> checkBox1(b);
  checkBox0.expandX(spaceX - 1);
  checkBox1.expandY(spaceY - 1);

  // obs
  Vector<Obs*> vpTouchedObs;
  _cir.spatialObs(layerIdx).query(checkBox0, vpTouchedObs);
  _cir.spatialObs(layerIdx).query(checkBox1, vpTouchedObs);
  if (!vpTouchedObs.empty()) {
    //const Obs& obs = *vpTouchedObs[0];
    //std::cerr << "Obs" << std::endl;
    //std::cerr << layerIdx << " "<< b << " " << obs.box() << std::endl;
    return false;
  }

  // pins
  Vector<Pair<Box<Int>, Pin*>> vpTouchedPins;
  _cir.spatialPins(layerIdx).queryBoth(checkBox0, vpTouchedPins);
  _cir.spatialPins(layerIdx).queryBoth(checkBox1, vpTouchedPins);
  for (const auto& item : vpTouchedPins) {
    const auto& box = item.first;
    const Net* pTouchedNet = item.second->pNet();
    if (pTouchedNet != &net) {
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        return false;
      }
    }
  }


  // nets
  Vector<Pair<Box<Int>, Net*>> vpTouchedNets;
  _cir.spatialNets(layerIdx).queryBoth(checkBox0, vpTouchedNets);
  _cir.spatialNets(layerIdx).queryBoth(checkBox1, vpTouchedNets);
  for (const auto& item : vpTouchedNets) {
    const Box<Int>& box = item.first;
    const Net* pTouchedNet = item.second;
    if (pTouchedNet != &net) {
      //std::cerr << layerIdx << " " << b << " " << box << std::endl;
      return false;
    }
    else { // same net
      if (!Box<Int>::bConnect(box, b)) {
        //std::cerr << layerIdx << " " << b << " " << box << std::endl;
        return false;
      }
    }
  }

  return true;
}

bool DrcMgr::checkWireMinArea(const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));

  return b.area() >= layer.area();
}

bool DrcMgr::checkViaSpacing(const Net& net, const Int x, const Int y, const Via& via, const Orient2dE orient) const
{
  Int i, layerIdx;
  const Box<Int>* pBox;

  Via_ForEachLayerIdx(via, layerIdx) {
    Via_ForEachLayerBox(via, layerIdx, pBox, i) {
      Box<Int> b(pBox->xl() + x,
                 pBox->yl() + y,
                 pBox->xh() + x,
                 pBox->yh() + y);
      switch (orient) {
        case Orient2dE::n:                                                                      break;
        case Orient2dE::w:  b.rotate90(b.centerX(), b.centerY(), false);                        break;
        case Orient2dE::s:  b.rotate180(b.centerX(), b.centerY());                              break;
        case Orient2dE::e:  b.rotate90(b.centerX(), b.centerY(), true);                         break;
        case Orient2dE::fn:                                               b.flipX(b.centerX()); break;
        case Orient2dE::fw: b.rotate90(b.centerX(), b.centerY(), false);  b.flipX(b.centerX()); break;
        case Orient2dE::fs:                                               b.flipY(b.centerY()); break;
        case Orient2dE::fe: b.rotate90(b.centerX(), b.centerY(), true);   b.flipX(b.centerX()); break;
        default: assert(false);
      }
      if (_cir.layer(layerIdx).isMetal()) {
        if (!checkWireMetalPrlSpacing(net, layerIdx, b)) {
          return false;
        }
        // check eol
        // TODO
      }
      else {
        if (!checkWireCutSpacing(net, layerIdx, b)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool DrcMgr::checkWireMinStep(const Int layerIdx, const Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));
  
  Int edgeCnt = 0;
  if (b.width() < layer.minStep()) {
    ++edgeCnt;
  }
  if (b.height() < layer.minStep()) {
    ++edgeCnt;
  }
  return edgeCnt <= layer.maxEdges();
}

bool DrcMgr::checkWireMinStep(const Int layerIdx, const Polygon<Int>& p) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));

  const Ring<Int>& ring = p.outer();
  assert(ring.size());

  Int edgeCnt = 0;
  for (size_t i = 1; i < ring.size(); ++i) {
    const Point<Int>& u = ring.at(i);
    const Point<Int>& v = ring.at(i);
    assert(u.x() == v.x() or u.y() == v.y());
    if (u.x() == v.x()) {
      if (std::abs(u.y() - v.y()) < layer.minStep()) {
        ++edgeCnt;
      }
      else {
        edgeCnt = 0;
      }
    }
    else {
      if (std::abs(u.x() - v.x()) < layer.minStep()) {
        ++edgeCnt;
      }
      else {
        edgeCnt = 0;
      }
    }
    if (edgeCnt > layer.maxEdges()) {
      return false;
    }
  }
  return true;
}

bool DrcMgr::checkWire(const Net& net, const Pair<Box<Int>, Int>& wire) const
{
  const auto& box = wire.first;
  const Int layerIdx = wire.second;


  if (_cir.layer(layerIdx).isMetal()) {
    if (!checkWireValidWidth(layerIdx, box)) {
      return false;
    }
    if (!checkWireValidLength(layerIdx, box)) {
      return false;
    }
    if (!checkWireMetalPrlSpacing(net, layerIdx, box)) {
      return false;
    }
    if (!checkWireMetalEolSpacing(net, layerIdx, box)) {
      return false;
    }
    if (!checkWireMinArea(layerIdx, box)) {
      return false;
    }
    return true;
  }
  else {
    return checkWireCutSpacing(net, layerIdx, box);
  }
}

DrvE DrcMgr::checkNet(const Net& net) const
{
  Int i, j, layerIdx;
  const Pin* pPin;
  const Box<Int>* pBox;

  Vector<Vector<Box<Int>>> vvBoxes(_cir.numLayers());
  Vector<Vector<Polygon<Int>>> vvPolygons(_cir.numLayers());

  Net_ForEachPin(net, pPin, i) {
    Pin_ForEachLayerIdx((*pPin), layerIdx) {
      Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
        vvBoxes[layerIdx].emplace_back(*pBox);
      }
    }
  }
  for (const auto& wire : net.sWires()) {
    vvBoxes[wire.second].emplace_back(wire.first);
  }

  for (layerIdx = 0; layerIdx < static_cast<Int>(vvBoxes.size()); ++layerIdx) {
    const Layer& layer = _cir.layer(layerIdx);
    const auto& vBoxes = vvBoxes.at(layerIdx);
    auto& vPolygons = vvPolygons.at(layerIdx);
    if (!vBoxes.empty()) {
      geo::box2Polygon<Int>(vBoxes, vPolygons);

      //if (layerIdx <= _cir.layer("m5t").idx()) { // m5t and below
      for (const Polygon<Int>& polygon : vPolygons) {
        const Ring<Int>& ring = polygon.outer();
        //std::cerr << ring[0] << " " << ring[1] << " " << ring[2] << " " << ring[3] << std::endl;
        assert(ring.size() == 4); // must be rectangular shape // ring[0] is lower right cornera // clockwise
        const Box<Int> b(ring[1].x(), ring[1].y(), ring[3].x(), ring[3].y());
        if (layer.isMetal()) {
          if (!checkWireValidWidth(layerIdx, b)) {
            //spdlog::warn("Valid width");
            return DrvE::valid_width;
          }
          if (!checkWireValidLength(layerIdx, b)) {
            //spdlog::warn("Valid length");
            return DrvE::valid_length;
          }
          //if (!checkWireMinArea(layerIdx, b)) {
            ////spdlog::warn("Min area");
            //return DrvE::min_area;
          //}
          //if (!checkWireMinStep(layerIdx, b)) {
            //return DrvE::min_step;
          //}
          if (!checkWireMetalPrlSpacing(net, layerIdx, b)) {
            //spdlog::warn("PRL spacing");
            return DrvE::spc_prl;
          }
          if (!checkWireMetalEolSpacing(net, layerIdx, b)) {
            //spdlog::warn("EOL spacing");
            return DrvE::spc_eol;
          }
        }
        else {
          assert(layer.isCut());
          if (!checkWireCutSpacing(net, layerIdx, b)) {
            //std::cerr << b << std::endl;
            //spdlog::warn("Cut spacing");
            return DrvE::spc_cut;
          }
        }
      }
    }
    else { // upper layers

    }
    //}
  }
  return DrvE::undef;
}

void DrcMgr::fitWire2ValidLength(const Int layerIdx, Box<Int>& b) const
{
  assert(_cir.layer(layerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));


  if (layer.isHor()) { // horizontal wire
    if (b.area() < layer.area()) {
      const Int minL = std::ceil(layer.area() / b.height());
      b.expandX((minL - b.width()) / 2);
    }
    //if (!checkWireMinStep(layerIdx, b)) {
      //const Int minL = layer.minStep();
      ////spdlog::info("MINSTEP EXT");
      //b.expandX((minL - b.width()) / 2);
    //}
    if (b.width() < layer.maxValidLength()) {
      const Int minL = layer.length2ValidLength(b.width());
      b.expandX((minL - b.width()) / 2);
    }
  }
  else {
    assert(layer.isVer());
    if (b.area() < layer.area()) {
      const Int minL = std::ceil(layer.area() / b.width());
      b.expandY((minL - b.height()) / 2);
    }
    //if (!checkWireMinStep(layerIdx, b)) {
      //const Int minL = layer.minStep();
      ////spdlog::info("MINSTEP EXT");
      //b.expandY((minL - b.height()) / 2);
    //}
    if (b.height() < layer.maxValidLength()) {
      const Int minL = layer.length2ValidLength(b.height());
      b.expandY((minL - b.height()) / 2);
    }
  }

}

PROJECT_NAMESPACE_END
