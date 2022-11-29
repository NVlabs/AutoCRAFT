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

#include "geo/point3d.hpp"
#include "geo/box.hpp"
#include "dbBasic.hpp"


PROJECT_NAMESPACE_START

class Net;
class Cell;
class RouteGrid;

class Pin {
  friend class Cell;
  friend class Parser;
  friend class LefReader;

public:


  Pin()
    : _idx(-1), _pCell(nullptr), _pNet(nullptr),
      _dir(PinDirE::undef), _use(PinUseE::undef), _shape(PinShapeE::undef),
      _ioIdx(-1), _minLayerIdx(MAX_INT), _maxLayerIdx(0), _superAcsIdx(-1), 
      _isIO(false)
  {}
  ~Pin() {}

  const String&               name()                          const { return _name; }
  const String&               devNetName()                    const { return _devNetName; }
  Int                         idx()                           const { return _idx; }

  Cell&                       cell()                                { return *_pCell; }
  const Cell&                 cell()                          const { return *_pCell; }
  Cell*                       pCell()                               { return _pCell; }
  const Cell*                 pCell()                         const { return _pCell; }

  Net&                        net()                                 { return *_pNet; }
  const Net&                  net()                           const { return *_pNet; }
  Net*                        pNet()                                { return _pNet; }
  const Net*                  pNet()                          const { return _pNet; }

  PinDirE                     dir()                           const { return _dir; }
  bool                        isDirInput()                    const { return _dir == PinDirE::input; }
  bool                        isDirOutput()                   const { return _dir == PinDirE::output; }
  bool                        isDirInout()                    const { return _dir == PinDirE::inout; }
  bool                        isDirFeedThru()                 const { return _dir == PinDirE::feedthru; }

  PinUseE                     use()                           const { return _use; }
  bool                        isUseSignal()                   const { return _use == PinUseE::signal; }
  bool                        isUseVdd()                      const { return _use == PinUseE::vdd; }
  bool                        isUseVss()                      const { return _use == PinUseE::vss; }
  bool                        isUsePG()                       const { return isUseVdd() or isUseVss(); }

  PinShapeE                   shape()                         const { return _shape; }
  bool                        isShapeAbutment()               const { return _shape == PinShapeE::abutment; }
  bool                        isShapeRing()                   const { return _shape == PinShapeE::ring; }
  bool                        isShapeFeedthru()               const { return _shape == PinShapeE::feedthru; }

  Int                         ioIdx()                         const { return _ioIdx; }

  Box<Int>&                   box(const Int i)                      { return _vBoxes.at(i); }
  const Box<Int>&             box(const Int i)                const { return _vBoxes.at(i); }
  Box<Int>&                   box(const Int l, const Int i)         { return _vBoxes.at(_vvBoxIds.at(l).at(i)); }
  const Box<Int>&             box(const Int l, const Int i)   const { return _vBoxes.at(_vvBoxIds.at(l).at(i)); }
  Vector<Box<Int>>&           vBoxes()                              { return _vBoxes; }
  const Vector<Box<Int>>&     vBoxes()                        const { return _vBoxes; }
  Int                         boxLayerIdx(const Int i)        const { return _vBoxLayerIds.at(i); }
  Int                         numBoxes()                      const { return _vBoxes.size(); }
  Int                         numBoxes(const Int l)           const { return _vvBoxIds.at(l).size(); }          

  Int                         minLayerIdx()                   const { return _minLayerIdx; }
  Int                         maxLayerIdx()                   const { return _maxLayerIdx; }

  Vector<Point3d<Int>>&       vAcs()                                { return _vAcs; }
  const Vector<Point3d<Int>>& vAcs()                          const { return _vAcs; }
  Int                         numAcs()                        const { return _vAcs.size(); }
  Point3d<Int>&               acs(const Int i)                      { return _vAcs.at(i); }
  const Point3d<Int>&         acs(const Int i)                const { return _vAcs.at(i); }
  Point3d<Int>&               superAcs()                            { return _vAcs.at(_superAcsIdx); }
  const Point3d<Int>&         superAcs()                      const { return _vAcs.at(_superAcsIdx); }
  Int                         superAcsIdx()                   const { return _superAcsIdx; }
  
  bool                        isIO()                          const { return _isIO; } 

  // set funcs
  void setName(const String& n) { _name = n; }
  void setNet(Net* p) { _pNet = p; }
  void setSuperAcsIdx(const Int i) { 
    _superAcsIdx = i;
  }
  void addBox(const Int l, const Box<Int>& b) {
    if (l >= static_cast<Int>(_vvBoxIds.size())) {
      _vvBoxIds.resize(l + 1);
    }
    _vvBoxIds[l].emplace_back(_vBoxes.size());
    _vBoxes.emplace_back(b);
    _vBoxLayerIds.emplace_back(l);
    _minLayerIdx = std::min(_minLayerIdx, l);
    _maxLayerIdx = std::max(_maxLayerIdx, l);
  }

private:
  String                _name;
  String                _devNetName; // for devMap
  Int                   _idx;
  Cell*                 _pCell; // nullptr if is IO pin  
  Net*                  _pNet;
  PinDirE               _dir;
  PinUseE               _use;
  PinShapeE             _shape;
  Int                   _ioIdx;

  Vector<Box<Int>>      _vBoxes;
  Vector<Int>           _vBoxLayerIds;
  Vector<Vector<Int>>   _vvBoxIds; // layer -> shapeIds;
  Int                   _minLayerIdx;
  Int                   _maxLayerIdx;
  
  Vector<Point3d<Int>>  _vAcs;
  Int                   _superAcsIdx;

  bool                  _isIO;
};

#define Pin_ForEachLayerIdx(pin, i) \
  for (i = pin.minLayerIdx(); i <= pin.maxLayerIdx(); ++i)

#define Pin_ForEachMetalLayerIdx(pin, i) \
  for (i = pin.minLayerIdx(); i <= pin.maxLayerIdx(); i += 2)

#define Pin_ForEachCutLayerIdx(pin, i) \
  for (i = pin.minLayerIdx() + 1; i < pin.maxLayerIdx(); i += 2)

#define Pin_ForEachLayerBox(pin, layerIdx, pBox_, i) \
  for (i = 0; i < pin.numBoxes(layerIdx) and (pBox_ = &pin.box(layerIdx, i)); ++i)

#define Pin_ForEachAcs(pin, pAcs_, i) \
  for (i = 0; i < pin.numAcs() and (pAcs_ = &pin.acs(i)); ++i)


PROJECT_NAMESPACE_END
