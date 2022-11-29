/*
* SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "db/dbNet.hpp"

PROJECT_NAMESPACE_START

class RouteSymCstr {
  friend class Parser;

public:
  RouteSymCstr()
    : _idx(-1), _axis(SymAxisE::undef),
      _numPartAs(0), _numSelfSyms(0)
  {}
  ~RouteSymCstr() {}

  const String&       name()                        const { return _name; }
  Int                 idx()                         const { return _idx; }

  SymAxisE            axis()                        const { return _axis; }

  Net&                net(const Int i)                    { return *_vpNets.at(i); }
  const Net&          net(const Int i)              const { return *_vpNets.at(i); }
  Net*                pNet(const Int i)                   { return _vpNets.at(i); }
  const Net*          pNet(const Int i)             const { return _vpNets.at(i); }
  Vector<Net*>&       vpNets()                            { return _vpNets; }
  const Vector<Net*>& vpNets()                      const { return _vpNets; }
  Int                 numNets()                     const { return _vpNets.size(); }
  
  SymPartE            symPart(const Int i)          const { return _vSymPartEs.at(i); }
  SymPartE            symPart(const Net& n)         const { return _vSymPartEs.at(netIdx2Idx(n.idx())); }
  bool                isPartA(const Int i)          const { return symPart(i) == SymPartE::a; }
  bool                isPartA(const Net& n)         const { return symPart(n) == SymPartE::a; }
  bool                isPartB(const Int i)          const { return symPart(i) == SymPartE::b; }
  bool                isPartB(const Net& n)         const { return symPart(n) == SymPartE::b; }
  bool                isSelfSym(const Int i)        const { return symPart(i) == SymPartE::self; }
  bool                isSelfSym(const Net& n)       const { return symPart(n) == SymPartE::self; }
  Int                 numSelfSyms()                 const { return _numSelfSyms; }
  Int                 numPartAs()                   const { return _numPartAs; }
  bool                hasSelfSym()                  const { return _numSelfSyms > 0; }

  Int                 netIdx2Idx(const Int idx)     const { return _mNetIdx2Idx.at(idx); }
  Int                 netIdx2SymIdx(const Int idx)  const { return _vSymNetIds.at(netIdx2Idx(idx)); }

  Net&                symNet(const Int i)                 { return *_vpNets.at(_vSymNetIds.at(i)); }
  Net&                symNet(const Net& n)                { return *_vpNets.at(netIdx2SymIdx(n.idx())); }
  const Net&          symNet(const Int i)           const { return *_vpNets.at(_vSymNetIds.at(i)); }
  const Net&          symNet(const Net& n)          const { return *_vpNets.at(netIdx2SymIdx(n.idx())); }
  Net*                pSymNet(const Int i)                { return _vpNets.at(_vSymNetIds.at(i)); }
  Net*                pSymNet(const Net& n)               { return _vpNets.at(netIdx2SymIdx(n.idx())); }
  const Net*          pSymNet(const Int i)          const { return _vpNets.at(_vSymNetIds.at(i)); }
  const Net*          pSymNet(const Net& n)         const { return _vpNets.at(netIdx2SymIdx(n.idx())); }

  const Region&       region()                      const { return _vpNets.at(0)->region(); }
  const Region*       pRegion()                     const { return _vpNets.at(0)->pRegion(); }

private:
  String                _name;
  Int                   _idx;

  SymAxisE              _axis;

  Vector<Net*>          _vpNets;
  Vector<SymPartE>      _vSymPartEs;
  Vector<Int>           _vSymNetIds; 
  Int                   _numPartAs;
  Int                   _numSelfSyms; 

  FlatHashMap<Int, Int> _mNetIdx2Idx; // net.idx() -> idx in _vpNets;
};

PROJECT_NAMESPACE_END 
