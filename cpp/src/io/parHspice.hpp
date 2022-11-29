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

#include "global/global.hpp"
#include "ds/hash.hpp"

PROJECT_NAMESPACE_START

class HspiceReader {
public:
  enum class NodeType : Byte { undef = 0, device = 1, subckt = 2, topckt = 3 };

  class Node {
    friend class HspiceReader;

  public:
    Node(const String& n, const String& tn, const String& s,
         NodeType t = NodeType::undef, const Int np = 0)
        : _instName(n), _tmplName(tn), _spiceType(s), _type(t), _numPorts(np), _idx(-1)
    {
    }
    ~Node() {}

    const String&                       instName()                    const { return _instName; }
    const String&                       tmplName()                    const { return _tmplName; }
    const String&                       spiceType()                   const { return _spiceType; }
    NodeType                            type()                        const { return _type; }
    bool                                isDevice()                    const { return _type == NodeType::device; }
    bool                                isSubCkt()                    const { return _type == NodeType::subckt; }
    bool                                isTopCkt()                    const { return _type == NodeType::topckt; }
    
    const Node&                         child(const Int i)            const { return _vChilds.at(i); }
    const Vector<Node>&                 vChilds()                     const { return _vChilds; }
    Int                                 numChilds()                   const { return _vChilds.size(); }
    
    const String&                       net(const Int i)              const { return _vNets.at(i); }
    const Vector<String>&               vNets()                       const { return _vNets; }
    Int                                 netName2Idx(const String& n)  const { return _mNetName2Idx.at(n); }
    Int                                 numNets()                     const { return _vNets.size(); }
    Int                                 numPorts()                    const { return _numPorts; }

    const String&                       attrVal(const String& n)      const { return _mAttrs.at(n); }
    const FlatHashMap<String, String>&  mAttrs()                      const { return _mAttrs; }
    Int                                 numAttrs()                    const { return _mAttrs.size(); }
    bool                                hasAttrs()                    const { return _mAttrs.size() > 0; }

    Int                                 idx()                         const { return _idx; }

  private:
    String                          _instName;
    String                          _tmplName;
    String                          _spiceType; // type attr in hspice netlists: analog, digital...
    NodeType                        _type;
    Vector<Node>                    _vChilds;
    Int                             _numPorts; // [0, _numPorts) in _vNets are ports
    Vector<String>                  _vNets; // includes ports and internal nets
    FlatHashMap<String, Int>        _mNetName2Idx;

    FlatHashMap<String, String>     _mAttrs;
    // helper
    Int                             _idx;
  };

  HspiceReader() : _numSubCkts(0) {}
  ~HspiceReader() {}

  void parse(const String& fileName);


  const Node&           node(const Int i)     const { return _vNodes.at(i); }
  const Node&           node(const String& s) const { return _vNodes.at(_mNodeName2Idx.at(s)); }
  const Node&           topNode()             const { return _vNodes.at(_numSubCkts - 1); }
  Int                   numNodes()            const { return _vNodes.size(); }
  Int                   numSubCkts()          const { return _numSubCkts; }
  Int                   numDevices()          const { return numNodes() - numSubCkts(); }
  const Vector<Node>&   vNodes()              const { return _vNodes; }

private:
  Vector<Node>             _vNodes;
  FlatHashMap<String, Int> _mNodeName2Idx;
  Int                      _numSubCkts;
  
  bool hasTmpl(const String& s) const
  {
    return _mNodeName2Idx.find(s) != _mNodeName2Idx.end();
  }
};

PROJECT_NAMESPACE_END
