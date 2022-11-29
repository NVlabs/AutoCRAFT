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

#include "db/dbBasic.hpp"
#include "ds/hash.hpp"

PROJECT_NAMESPACE_START

class Pin;
class Net;


class RoutePathMatchCstr {
  friend class Parser;
public:

  class PathCstr {
    friend class Parser;
  public:
    PathCstr() : _idx(-1) {}
    ~PathCstr() {}

    Int                             idx()                  const { return _idx; }

    Pin&                            srcPin(const Int i)          { return *_vConns.at(i).first; }
    const Pin&                      srcPin(const Int i)    const { return *_vConns.at(i).first; }
    Pin*                            pSrcPin(const Int i)         { return _vConns.at(i).first; }
    const Pin*                      pSrcPin(const Int i)   const { return _vConns.at(i).first; }

    Pin&                            tarPin(const Int i)          { return *_vConns.at(i).second; }
    const Pin&                      tarPin(const Int i)    const { return *_vConns.at(i).second; }
    Pin*                            pTarPin(const Int i)         { return _vConns.at(i).second; }
    const Pin*                      pTarPin(const Int i)   const { return _vConns.at(i).second; }

    Pair<Pin*, Pin*>&               conn(const Int i)            { return _vConns.at(i); }
    const Pair<Pin*, Pin*>&         conn(const Int i)      const { return _vConns.at(i); }

    Vector<Pair<Pin*, Pin*>>&       vConns()                     { return _vConns; }  
    const Vector<Pair<Pin*, Pin*>>& vConns()               const { return _vConns; }  
    Int                             numConns()             const { return _vConns.size(); }
    
    bool                            hasNet(const Net* p)   const { return _spNets.find(p) != _spNets.end(); }
    
  private:
    Int                      _idx; // idx in its parent cstr (the "idx"-th path cstr in the parent cstr)
    Vector<Pair<Pin*, Pin*>> _vConns;
    FlatHashSet<Net*>        _spNets;
  };

  RoutePathMatchCstr() 
    : _idx(-1) 
  {}
  ~RoutePathMatchCstr() {}

  const String&           name()                 const { return _name; }
  Int                     idx()                  const { return _idx; }
  

  Net&                    net(const Int i)             { return *_vpNets.at(i); }
  const Net&              net(const Int i)       const { return *_vpNets.at(i); }
  Net*                    pNet(const Int i)            { return _vpNets.at(i); }
  const Net*              pNet(const Int i)      const { return _vpNets.at(i); }
  Vector<Net*>&           vpNets()                     { return _vpNets; }
  const Vector<Net*>&     vpNets()               const { return _vpNets; }
  Int                     numNets()              const { return _vpNets.size(); }

  PathCstr&               pathCstr(const Int i)        { return _vPathCstrs.at(i); }
  const PathCstr&         pathCstr(const Int i)  const { return _vPathCstrs.at(i); }
  PathCstr&               pathCstr(const Net* p)       { return _vPathCstrs.at(netIdx(p)); }
  const PathCstr&         pathCstr(const Net* p) const { return _vPathCstrs.at(netIdx(p)); }
  Vector<PathCstr>&       vPathCstrs()                 { return _vPathCstrs; }
  const Vector<PathCstr>& vPathCstrs()           const { return _vPathCstrs; }
  Int                     numPathCstrs()         const { return _vPathCstrs.size(); }

  Int                     netIdx(const Net* p)   const { return _mNet2NetIdx.at(p); }
  bool                    hasNet(const Net* p)   const { return _mNet2NetIdx.find(p) != _mNet2NetIdx.end(); }

private:
  String                 _name;
  Int                    _idx;
  Vector<Net*>           _vpNets;
  Vector<PathCstr>       _vPathCstrs;
  FlatHashMap<Net*, Int> _mNet2NetIdx;
};

PROJECT_NAMESPACE_END
