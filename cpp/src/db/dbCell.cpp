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

#include "dbCell.hpp"

PROJECT_NAMESPACE_START


void Cell::setPrim(const Primitive& prim)
{
  _pPrim = &prim;
  _width = prim.sizeX();
  _height = prim.sizeY();

  // upd pins
  //assert(static_cast<Int>(_vPins.size()) == prim.numPins());
  if (static_cast<Int>(_vPins.size()) != prim.numPins()) {
    spdlog::warn("{} pin count mismatch (cell: {}, prim:{})", _name, _vPins.size(), prim.numPins());
    _vPins.resize(std::max(prim.numPins(), static_cast<Int>(_vPins.size())));
    for (Int i = 0; i < prim.numPins(); ++i) {
      Pin& pin = _vPins.at(i);
      pin._devNetName = _pDevMap->primNet2DevNet(prim.pin(i).name());
      pin._name = _name + "/" + pin._devNetName;
      pin._pCell = this;
    }
  }
  //else {
    for (Int i = 0; i < prim.numPins(); ++i) {
      Pin& pin = _vPins.at(i);
      
      const String& primPinName = _pDevMap->devNet2PrimNet(pin.devNetName());
      if (primPinName != "NC") {
        const Pin& primPin = prim.pin(primPinName);

        pin._dir = primPin._dir;
        pin._use = primPin._use;
        pin._vBoxes.clear();
        pin._vBoxes.reserve(primPin._vBoxes.size());
        for (const Box<Int>& box : primPin._vBoxes) {
          const Int xl = box.xl() + prim.originX() + prim.foreignX();
          const Int yl = box.yl() + prim.originY() + prim.foreignY();
          const Int xh = box.xh() + prim.originX() + prim.foreignX();
          const Int yh = box.yh() + prim.originY() + prim.foreignY();
          pin._vBoxes.emplace_back(xl, yl, xh, yh);
        }
        pin._vBoxLayerIds = primPin._vBoxLayerIds;
        pin._vvBoxIds = primPin._vvBoxIds;
        pin._minLayerIdx = primPin._minLayerIdx;
        pin._maxLayerIdx = primPin._maxLayerIdx;
      }
    }
    // upd obs
    _vObs.clear();
    _vObs.reserve(prim.numObs());
    for (const Obs& obs : prim.vObs()) {
      const Box<Int>& box = obs.box();
      const Int layerIdx = obs.layerIdx();
      const Int xl = box.xl() + prim.originX() + prim.foreignX();
      const Int yl = box.yl() + prim.originY() + prim.foreignY();
      const Int xh = box.xh() + prim.originX() + prim.foreignX();
      const Int yh = box.yh() + prim.originY() + prim.foreignY();
      _vObs.emplace_back(Box<Int>(xl, yl, xh, yh), layerIdx);
      _vObs.back()._layerIdx = layerIdx;
      _vObs.back()._pCell = this;
    }
  //}
}

//void Cell::setPrim(const Int i)
//{
  //assert(0 <= i and i <= _pDevMap->numPrims());
  //_curPrimIdx = i; 
  //setPrim(curPrim());
//}

//void Cell::nextPrim()
//{
  //_curPrimIdx = (_curPrimIdx == _pDevMap->numPrims() - 1) ? 0 : _curPrimIdx + 1;
  //setPrim(curPrim());
//}

void Cell::setLoc(const Int x, const Int y, const Int gx, const Int gy)
{
  const Int oriX = _loc.x();
  const Int oriY = _loc.y();
  _loc.setXY(x, y);
  _gridLoc.setXY(gx, gy);

  const Int sx = x - oriX;
  const Int sy = y - oriY;
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.shift(sx, sy);
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().shift(sx, sy);
  }
}


void Cell::setOrient(const Orient2dE o)
{
  switch (_orient) {
    case Orient2dE::n:
      switch (o) {
        case Orient2dE::n:                        break;
        case Orient2dE::w:  rotate90();           break;
        case Orient2dE::s:  rotate180();          break;
        case Orient2dE::e:  rotate270();          break;
        case Orient2dE::fn:              flipX(); break;
        case Orient2dE::fw: rotate90();  flipX(); break;
        case Orient2dE::fs:              flipY(); break;
        case Orient2dE::fe: rotate270(); flipX(); break;
        default: assert(false);
      }
      break; 
    case Orient2dE::w:
      switch (o) {
        case Orient2dE::n:  rotate270();          break;
        case Orient2dE::w:                        break;
        case Orient2dE::s:  rotate90();           break;
        case Orient2dE::e:  rotate180();          break;
        case Orient2dE::fn: rotate270(); flipX(); break;
        case Orient2dE::fw:              flipX(); break;
        case Orient2dE::fs: rotate90();  flipX(); break;
        case Orient2dE::fe:              flipY(); break;
        default: assert(false);
      }
      break; 
    case Orient2dE::s:
      switch (o) {
        case Orient2dE::n:  rotate180();          break;
        case Orient2dE::w:  rotate270();          break;
        case Orient2dE::s:                        break;
        case Orient2dE::e:  rotate90();           break;
        case Orient2dE::fn:              flipY(); break;
        case Orient2dE::fw: rotate270(); flipX(); break;
        case Orient2dE::fs:              flipX(); break;
        case Orient2dE::fe: rotate90();  flipX(); break;
        default: assert(false);
      }
      break; 
    case Orient2dE::e:
      switch (o) {
        case Orient2dE::n:  rotate90();           break;
        case Orient2dE::w:  rotate180();          break;
        case Orient2dE::s:  rotate270();          break;
        case Orient2dE::e:                        break;
        case Orient2dE::fn: rotate90();  flipX(); break;
        case Orient2dE::fw:              flipY(); break;
        case Orient2dE::fs: rotate270(); flipX(); break;
        case Orient2dE::fe:              flipX(); break;
        default: assert(false);
      }
      break; 
    case Orient2dE::fn:
      switch (o) {
        case Orient2dE::n:  flipX();              break;
        case Orient2dE::w:  flipX(); rotate90();  break;
        case Orient2dE::s:  flipY();              break;
        case Orient2dE::e:  flipX(); rotate270(); break;
        case Orient2dE::fn:                       break;
        case Orient2dE::fw: rotate270();          break;
        case Orient2dE::fs: rotate180();          break;
        case Orient2dE::fe: rotate90();           break;
        default: assert(false);
      }
      break; 
    case Orient2dE::fw:
      switch (o) {
        case Orient2dE::n:  flipX(); rotate270(); break;
        case Orient2dE::w:  flipX();              break;
        case Orient2dE::s:  flipX(); rotate90();  break;
        case Orient2dE::e:  flipY();              break;
        case Orient2dE::fn: rotate90();           break;
        case Orient2dE::fw:                       break;
        case Orient2dE::fs: rotate270();          break;
        case Orient2dE::fe: rotate180();          break;
        default: assert(false);
      }
      break; 
    case Orient2dE::fs:
      switch (o) {
        case Orient2dE::n:  flipY();              break;
        case Orient2dE::w:  flipX(); rotate270(); break;
        case Orient2dE::s:  flipX();              break;
        case Orient2dE::e:  flipX(); rotate90();  break;
        case Orient2dE::fn: rotate180();          break;
        case Orient2dE::fw: rotate90();           break;
        case Orient2dE::fs:                       break;
        case Orient2dE::fe: rotate270();          break;
        default: assert(false);
      }
      break; 
    case Orient2dE::fe:
      switch (o) {
        case Orient2dE::n:  flipX(); rotate90();  break;
        case Orient2dE::w:  flipY();              break;
        case Orient2dE::s:  flipX(); rotate270(); break;
        case Orient2dE::e:  flipX();              break;
        case Orient2dE::fn: rotate270();          break;
        case Orient2dE::fw: rotate180();          break;
        case Orient2dE::fs: rotate90();           break;
        case Orient2dE::fe:                       break;
        default: assert(false);
      }
      break; 
    default: assert(false);                 
  }
  assert(_orient == o);
}

void Cell::rotate90()
{
  _orient = orient2d::rotate90(_orient);
  std::swap(_width, _height);
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.rotate90(_loc.x(), _loc.y(), false);
      box.shiftX(_width);
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().rotate90(_loc.x(), _loc.y(), false);
    obs.box().shiftX(_width);
  }
}

void Cell::rotate180()
{
  _orient = orient2d::rotate180(_orient);
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.rotate180(_loc.x(), _loc.y());
      box.shift(_width, _height);
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().rotate180(_loc.x(), _loc.y());
    obs.box().shift(_width, _height);
  }
}

void Cell::rotate270()
{
  _orient = orient2d::rotate270(_orient);
  std::swap(_width, _height);
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.rotate90(_loc.x(), _loc.y(), true);
      box.shiftY(_height);
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().rotate90(_loc.x(), _loc.y(), true);
    obs.box().shiftY(_height);
  }
}

void Cell::flipX()
{
  _orient = orient2d::flipX(_orient);
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.flipX(centerX());
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().flipX(centerX());
  }
}

void Cell::flipY()
{
  _orient = orient2d::flipY(_orient);
  for (Pin& pin : _vPins) {
    for (Box<Int>& box : pin.vBoxes()) {
      box.flipY(centerY());
    }
  }
  for (Obs& obs : _vObs) {
    obs.box().flipY(centerY());
  }
}

void Cell::setPowerGroupIdx(const Int i)
{
  _powerGroupIdx = i;
}


PROJECT_NAMESPACE_END
