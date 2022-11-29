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
#include "parHspice.hpp"
#include "util/util.hpp"

#include <fstream>

PROJECT_NAMESPACE_START

void HspiceReader::parse(const String& fileName)
{
  std::ifstream ifs(fileName, std::ios::in);

  String         buf;
  Vector<String> vTokens;

  FlatHashMap<String, Node> mDevices;

  while (std::getline(ifs, buf)) {
    if (buf.empty() or buf[0] == '*') // comments
      continue;
    util::str::split(buf, " ", vTokens);
    if (vTokens.at(0) == ".subckt" or vTokens.at(0) == ".topckt") {
      String tok = vTokens.at(1);

      const bool hasSpiceType = (tok.size() > 5 and tok.substr(0, 5) == "type:");

      size_t         pos       = hasSpiceType ? 2 : 1;
      const String   name      = vTokens.at(pos);
      const String   spiceType = hasSpiceType ? tok.substr(5) : "";
      const NodeType type =
          vTokens.at(0) == ".subckt" ? NodeType::subckt : NodeType::topckt;

      const Int idx = _vNodes.size();
      _mNodeName2Idx.emplace(name, idx);
      _vNodes.emplace_back(name, name, spiceType, type);
      Node& node = _vNodes.back();

      ++pos;
      node._numPorts = vTokens.size() - pos;
      node._idx = idx;
      for (size_t i = pos; i < vTokens.size(); ++i) {
        const String& netName = vTokens.at(i);
        if (node._mNetName2Idx.find(netName) == node._mNetName2Idx.end()) {
          node._mNetName2Idx.emplace(netName, node._vNets.size());
          node._vNets.emplace_back(netName);
        }
      }
      // handle this node block
      FlatHashMap<String, String> mAttrVars;
      while (std::getline(ifs, buf)) {
        if (buf.empty() or buf[0] == '*') // comments
          continue;
        if (buf.find('$') != String::npos) // ignore $.model=....
          continue;

        util::str::split(buf, " ", vTokens);
        if (vTokens.at(0) == ".ends") {
          assert(vTokens.size() == 1 or (vTokens.size() == 2 and vTokens.at(1) == name));
          break;
        }
        if (vTokens.at(0) == "+") { // attr vals
          for (size_t i = 1; i < vTokens.size(); ++i) {
            const String& attr = vTokens.at(i);
            size_t eqPos = attr.find_first_of('=');
            const String& attrVarName = attr.substr(0, eqPos);
            const String& attrVarVal = attr.substr(eqPos + 1);
            mAttrVars.emplace(attrVarName, attrVarVal);
          }
        }

        size_t i = 1, j = i, k = 1;
        for (; j < vTokens.size() and vTokens.at(j).find('=') == String::npos; ++j)
          ;
        k = j;
        // construct a child node and set its connection
        const String&  instName = vTokens.at(0);
        const String&  tmplName = vTokens.at(--j);

        const bool hasTemplate = hasTmpl(tmplName);
        const String&  st = hasTemplate ? this->node(tmplName).spiceType() : "";
        const NodeType t  = hasTemplate ? NodeType::subckt : NodeType::device;

        if (mDevices.find(tmplName) == mDevices.end()) {
          Node devNode(tmplName, tmplName, st, t);
          mDevices.emplace(tmplName, devNode);
        }

        node._vChilds.emplace_back(instName, tmplName, st, t);
        Node& child = node._vChilds.back();
        for (; k < vTokens.size(); ++k) {
          const String& attr = vTokens.at(k);
          size_t eqPos = attr.find_first_of('=');
          const String& attrName = attr.substr(0, eqPos);
          const String& attrVal = attr.substr(eqPos + 1);
          if (attrVal.front() == '\'' and attrVal.back() == '\'') {
            const String& attrVar = attrVal.substr(1, attrVal.size() - 2);
            assert(mAttrVars.find(attrVar) != mAttrVars.end());
            child._mAttrs.emplace(attrName, mAttrVars.at(attrVar));
          }
          else {
            child._mAttrs.emplace(attrName, attrVal);
          }
        }
        
        for (; i < j; ++i) {
          const String& netName = vTokens.at(i);
          child._vNets.emplace_back(netName);
          if (node._mNetName2Idx.find(netName) == node._mNetName2Idx.end()) {
            node._mNetName2Idx.emplace(netName, node._vNets.size());
            node._vNets.emplace_back(netName);
          }
        }
        child._numPorts = child._vNets.size();
        // attributes: add something here if needed
        // for (; ++j < vTokens.size();) {
        //}
      }
    }
    else {
      assert(false);
    }
  }

  _numSubCkts = _vNodes.size();

  // add device nodes (must be transistors)
  for (auto& item : mDevices) {
    const String& tmplName = item.first;
    const Int idx = _vNodes.size();
    Node& node = item.second;
    node._idx = idx;
    node._numPorts = 4;
    node._mNetName2Idx.emplace("D", node._vNets.size());
    node._vNets.emplace_back("D");
    node._mNetName2Idx.emplace("G", node._vNets.size());
    node._vNets.emplace_back("G");
    node._mNetName2Idx.emplace("S", node._vNets.size());
    node._vNets.emplace_back("S");
    node._mNetName2Idx.emplace("B", node._vNets.size());
    node._vNets.emplace_back("B");
    _mNodeName2Idx.emplace(tmplName, idx);
    _vNodes.emplace_back(node);
  }

}

PROJECT_NAMESPACE_END
