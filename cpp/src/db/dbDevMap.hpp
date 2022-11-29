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

#include "dbPrim.hpp"

PROJECT_NAMESPACE_START

class DevMap { // device to primitives mapping
  friend class Parser;

public:
  DevMap()
    : _name(""), _pPrim(nullptr), _multiplier(1), _bAbutment(false) {}
  ~DevMap() {}

  const String&                   name()                          const { return _name; }
  
  const Primitive&                prim()                          const { return *_pPrim; }
  const Primitive*                pPrim()                         const { return _pPrim; }
  Int                             multiplier()                    const { return _multiplier; }

  bool                            isAbutment()                    const { return _bAbutment; }
  
  const String&                   devNet(const Int i)             const { return _vNetMaps.at(i).first; }
  const String&                   primNet(const Int i)            const { return _vNetMaps.at(i).second; }
  const String&                   devNet2PrimNet(const String& n) const { return _vNetMaps.at(_mDevNetMap2Idx.at(n)).second; }
  const String&                   primNet2DevNet(const String& n) const { return _vNetMaps.at(_mPrimNetMap2Idx.at(n)).first; }
  Int                             numNets()                       const { return _vNetMaps.size(); }

private:
  String                        _name;
  const Primitive*              _pPrim;
  Int                           _multiplier; // number of prims to use
  bool                          _bAbutment;
  Vector<Pair<String, String>>  _vNetMaps;
  FlatHashMap<String, Int>      _mDevNetMap2Idx;
  FlatHashMap<String, Int>      _mPrimNetMap2Idx;
};

PROJECT_NAMESPACE_END
