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

#include "geo/box.hpp"
#include "dbBasic.hpp"

PROJECT_NAMESPACE_START

class Via {
  friend class Parser;
  friend class LefReader;

public:
  Via()
    : _name(""), _idx(-1), _type(ViaTypeE::undef),
      _minLayerIdx(MAX_INT), _maxLayerIdx(0)
  {}
  ~Via() {}

  const String&               name()                          const { return _name; }
  Int                         idx()                           const { return _idx; }
  
  ViaTypeE                    type()                          const { return _type; }
  bool                        isDefault()                     const { return _type == ViaTypeE::def; }
  bool                        isGenerated()                   const { return _type == ViaTypeE::generated; }

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


private:
  String              _name;
  Int                 _idx;
  ViaTypeE            _type;

  Vector<Box<Int>>    _vBoxes;
  Vector<Int>         _vBoxLayerIds;
  Vector<Vector<Int>> _vvBoxIds;
  Int                 _minLayerIdx;
  Int                 _maxLayerIdx;

};

#define Via_ForEachLayerIdx(via, i) \
  for (i = via.minLayerIdx(); i <= via.maxLayerIdx(); ++i)

#define Via_ForEachMetalLayerIdx(via, i) \
  for (i = via.minLayerIdx(); i <= via.maxLayerIdx(); i += 2)

#define Via_ForEachCutLayerIdx(via, i) \
  for (i = via.minLayerIdx() + 1; i < via.maxLayerIdx(); i += 2)

#define Via_ForEachLayerBox(via, layerIdx, pBox_, i) \
  for (i = 0; i < via.numBoxes(layerIdx) and (pBox_ = &via.box(layerIdx, i)); ++i)

PROJECT_NAMESPACE_END
