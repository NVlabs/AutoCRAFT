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

#include "routeDrPost.hpp"
#include "geo/box2polygon.hpp"
#include "drc/drcMgr.hpp"

PROJECT_NAMESPACE_START

void PostRoute::patch()
{
  spdlog::info("[PostRoute] Patch wires");
  Int i;
  Region* pReg;

  Cir_ForEachRegion(_cir, pReg, i) {
    patch(*pReg);
  }
}

void PostRoute::patch(Region& reg)
{
  Int i;
  Net* pNet;

  Reg_ForEachNet(reg, pNet, i) {
    Vector<Vector<Box<Int>>> vvBoxes(_cir.numLayers());
    Vector<Vector<Box<Int>>> vvPatches(_cir.numLayers());
    initShapes(*pNet, vvBoxes);
    checkShapes(vvBoxes, vvPatches);
    addPatches2Nets(*pNet, vvPatches);
  }

  
}

void PostRoute::initShapes(const Net& net, Vector<Vector<Box<Int>>>& vvBoxes)
{

  Int i, j, layerIdx;
  const Pin* pPin;
  const Box<Int>* pBox;

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

}

void PostRoute::checkShapes(const Vector<Vector<Box<Int>>>& vvBoxes, Vector<Vector<Box<Int>>>& vvPatches)
{

  DrcMgr drc(_cir);


  Vector<Vector<Polygon<Int>>> vvPolygons(_cir.numLayers());
  for (Int layerIdx = 0; layerIdx < _cir.numLayers(); ++layerIdx) {
    const Layer& layer = _cir.layer(layerIdx);
    auto& vBoxes = vvBoxes.at(layerIdx);
    auto& vPolygons = vvPolygons.at(layerIdx);
    if (vBoxes.size()) {
      geo::box2Polygon<Int>(vBoxes, vPolygons);
      //if (layerIdx <= _cir.layer("m5t").idx()) { // lower layers
      for (const Polygon<Int>& polygon : vPolygons)  {
        const Ring<Int>& ring = polygon.outer();
        assert(ring.size() == 4); // must be rectangular shape // ring[0] is lower right cornera // clockwise
        const Box<Int> b(ring[1].x(), ring[1].y(), ring[3].x(), ring[3].y());
        

        Box<Int> extBox(b);
        if (layer.isMetal()) {
          const MetalLayer& metalLayer = *static_cast<const MetalLayer*>(_cir.pLayer(layerIdx));
          if (metalLayer.isHor()) {
            if (!drc.checkWireMinArea(layerIdx, extBox)) {
              const Int minL = std::ceil(metalLayer.area() / extBox.height());
              extBox.expandX((minL - extBox.width()) / 2);
            }
            if (!drc.checkWireMinStep(layerIdx, extBox)) {
              const Int minL = metalLayer.minStep();
              extBox.expandX((minL - extBox.width()) / 2);
            }
            if (!drc.checkWireValidLength(layerIdx, extBox)) {
              const Int minL = metalLayer.length2ValidLength(extBox.width());
              extBox.expandX((minL - extBox.width()) / 2);
            }
            if (extBox.xl() < b.xl()) {
              vvPatches[layerIdx].emplace_back(extBox.xl(), b.yl(), b.xl(), b.yh());
            }
            if (extBox.xh() > b.xh()){
              vvPatches[layerIdx].emplace_back(b.xh(), b.yl(), extBox.xh(), b.yh());
            }
          }
          else {
            if (!drc.checkWireMinArea(layerIdx, extBox)) {
              const Int minL = std::ceil(metalLayer.area() / extBox.width());
              extBox.expandY((minL - extBox.height()) / 2);
            }
            if (!drc.checkWireMinStep(layerIdx, extBox)) {
              const Int minL = metalLayer.minStep();
              extBox.expandY((minL - extBox.height()) / 2);
            }
            if (!drc.checkWireValidLength(layerIdx, extBox)) {
              const Int minL = metalLayer.length2ValidLength(extBox.height());
              extBox.expandY((minL - extBox.height()) / 2);
            }
            if (extBox.yl() < b.yl()) {
              vvPatches[layerIdx].emplace_back(b.xl(), extBox.yl(), b.xh(), b.yl());
            }
            if (extBox.yh() > b.yh()){
              vvPatches[layerIdx].emplace_back(b.xl(), b.yh(), b.xh(), extBox.yh());
            }

          }

        }
      }
      //}
      //else { // upper layers

      //}
    }
  }

}

void PostRoute::addPatches2Nets(Net& net, const Vector<Vector<Box<Int>>>& vvPatches)
{
  for (size_t layerIdx = 0; layerIdx < vvPatches.size(); ++layerIdx) {
    for (const auto& box : vvPatches.at(layerIdx)) {
      //std::cerr << box << " " << layerIdx << std::endl;
      net.sPatchWires().emplace(box, layerIdx);
    }
  }

}

PROJECT_NAMESPACE_END
