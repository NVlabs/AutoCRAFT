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

#include "db/dbCir.hpp"
#include "geo/polygon.hpp"

PROJECT_NAMESPACE_START

class DrcMgr {
public:
  DrcMgr(CirDB& cir)
    : _cir(cir)
  {}
  ~DrcMgr() {}

  ////////////////////////////////////////
  //         Wire level checking        // 
  ////////////////////////////////////////

  // min/max width
  bool checkWireValidWidth(const Int layerIdx, const Box<Int>& b) const;

  // min/max length
  bool checkWireValidLength(const Int layerIdx, const Box<Int>& b) const;

  // spacing
  bool checkWireMetalSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const;
  bool checkWireMetalPrlSpacing(const Net& net, const Int layerIdx, const Box<Int>& b, const Int prl = 0) const;
  bool checkWireMetalEolSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const;
  bool checkWireCutSpacing(const Net& net, const Int layerIdx, const Box<Int>& b) const;

  // min area
  bool checkWireMinArea(const Int layerIdx, const Box<Int>& b) const;

  // via
  bool checkViaSpacing(const Net& net, const Int x, const Int y, const Via& via, const Orient2dE orient) const;
  
  // min step
  bool checkWireMinStep(const Int layerIdx, const Box<Int>& b) const;
  bool checkWireMinStep(const Int layerIdx, const Polygon<Int>& p) const;

  // total
  bool checkWire(const Net& net, const Pair<Box<Int>, Int>& wire) const;
  ////////////////////////////////////////
  //          Net level checking        // 
  ////////////////////////////////////////
  DrvE checkNet(const Net& net) const;

  ////////////////////////////////////////
  //          Fix Funcs                 // 
  ////////////////////////////////////////
  void fitWire2ValidLength(const Int layerIdx, Box<Int>& b) const;

private:
  CirDB& _cir;
};

PROJECT_NAMESPACE_END
