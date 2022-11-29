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

#include "routeDrAstar.hpp"
#include "ds/pqueue.hpp"
#include "graph/mst.hpp"
#include "geo/kdTree.hpp"
#include "drc/drcMgr.hpp"
#include "geo/box2polygon.hpp"

PROJECT_NAMESPACE_START

bool AstarCore::solve()
{
  //spdlog::info("[AstarCore] Route net {}({}), Sym: {}, SelfSym: {}, Fail: {}, DRC cost: {}, History Cost: {}",
               //_net.name(),
               //_ro.idx(),
               //_isSym,
               //_isSelfSym,
               //_net.numFails(),
               //_useStrictDRC ? "inf" : std::to_string(_ps.drcCost()),
               //_ps.hisCost());

  //Int i, j, layerIdx;
  //const Pin* pPin;
  //const Box<Int>* pBox;
  //Net_ForEachPin(_net, pPin, i) {
    //Pin_ForEachLayerIdx((*pPin), layerIdx) {
      //Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
        //Vector<Obs*> vObs;
        //_cir.spatialObs(layerIdx).query(*pBox, vObs);
        //assert(vObs.empty());
        ////if (vObs.size()) {
          ////std::cerr << *pBox << " " << vObs[0]->box() << std::endl;
        ////}
      //}
    //}
  //}

  if (_net.isRouted()) {
    return true;
  }
  if (_ro.isRouted()) {
    return true;
  }

  init();

  //for (const auto& v : _vCompBoxes) {
    //for (const auto& p : v) {
      //Vector<Obs*> vObs;
      //_cir.spatialObs(p.second).query(p.first, vObs);
      //if (vObs.size()) {
        //std::cerr << p.first << " " << vObs[0]->box() << std::endl;
      //}
      //assert(vObs.empty());
    //}
  //}
  //for (const auto& v : _vCompAcsPts) {
    //for (const auto& p : v) {
      //Vector<Obs*> vObs;
      //Point<Int> p2d(p.x(), p.y());
      //_cir.spatialObs(p.z()).query(p2d, p2d, vObs);
      //if (vObs.size()) {
        //std::cerr << p2d << " " << vObs[0]->box() << std::endl;
      //}
      //assert(vObs.empty());
    //}
  //}

  if (!splitSubNet()) {
    spdlog::error("[AstarCore] Subnets splitting failed");
    return false;
  }
  //return true;
  if (!route()) {
    //spdlog::error("[AstarCore] Routing failed");
    ripup();
    return false;
  }

  saveResult();
  return true;
}

void AstarCore::init()
{
  if (_ro.numArcs() > 0) {
    initFromPartial();
  }
  else {
    initFromScratch();
  }
}

void AstarCore::initFromPartial()
{
  //std::cerr << "jasfldjsfkljsadklfjsla;kfjsla;jflsd" << std::endl;
  _vpPins.clear();
  _hasCrossAxisPin = false;

  const auto& topo = _ro.topo();
  //topo.showComps();

  Vector<FlatHashSet<Pair<Box<Int>, Int>>> vsCompWires(topo.numComps());
  for (Int i = 0; i < topo.numComps(); ++i) {
    //Vector<Segment3d<Int>> vSegs;
    //topo.vCompSegs(i, vSegs);
    //_ps.topoSegs2Wires(_net, vSegs, vsCompWires[i]);
    ////std::cerr << i << std::endl;
    ////for (const auto& r : vsCompWires[i]) {
      ////std::cerr << r.first << " " << r.second << std::endl;
    ////}
    //if (vSegs.empty()) {
      //_ps.topoPts2Wires(_net, topo.compPts(i), vsCompWires[i]);
    //}
    _ps.queryTopoCompWires(_net, topo, i, vsCompWires[i]);
  }

  if (_isSelfSym) {
    assert(_ro.numRoutables() == 0);
    assert(topo.numComps() > 1);
    //initFromPartialSelfSym();
    for (Int i = 0; i < topo.numComps(); ++i) {
      bool isPartA = false; 
      bool isPartB = false;

      const auto& compWires = vsCompWires.at(i);

      for (const auto& wire : compWires) {
        const auto& box = wire.first;
        //const Int layerIdx = wire.second;
        if (_net.isVerSym()) {
          if (box.xl() < _net.symAxisX()) {
            isPartA = true;
          }
          if (box.xh() > _net.symAxisX()) {
            isPartB = true;
          }
        }
        else if (_net.isHorSym()) {
          if (box.yl() < _net.symAxisY()) {
            isPartA = true;
          }
          if (box.yh() > _net.symAxisY()) {
            isPartB = true;
          }
        }
      }

      if (isPartA and isPartB) {
        _hasCrossAxisComp = true;
      }
      if (isPartA) {
        _vTopoCompIds.emplace_back(i);
      }
    }
    const Int halfNumComps = !_hasCrossAxisComp ? _vTopoCompIds.size() + 1 : _vTopoCompIds.size();
    _compDS.init(halfNumComps);
    _vCompBoxes.resize(halfNumComps);
    _vCompAcsPts.resize(halfNumComps);
    _vCompSpatialBoxes.resize(halfNumComps);
  }
  else {
    assert(!_ro.isSelfSym());
    for (Int i = 0; i < topo.numComps(); ++i) {
      _vTopoCompIds.emplace_back(i);
    }
    for (Int i = 0; i < _ro.numRoutables(); ++i) {
      _vRoutableIds.emplace_back(_ro.routableIdx(i));
    }
    const Int numComps = topo.numComps() + _ro.numRoutables();
    _compDS.init(numComps);
    _vCompBoxes.resize(numComps);
    _vCompAcsPts.resize(numComps);
    _vCompSpatialBoxes.resize(numComps);
  }


  Int i, j, k, layerIdx;
  Int rangeL = MAX_INT, rangeH = MIN_INT; // dummy pin range
  for (i = 0; i < static_cast<Int>(_vTopoCompIds.size()); ++i) {
    const Int topoCompIdx = _vTopoCompIds.at(i);

    const auto& compWires = vsCompWires.at(topoCompIdx);
    for (const auto& wire : compWires) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (_net.isVerSym()) {
        rangeL = std::min(rangeL, box.yl());
        rangeH = std::max(rangeH, box.yh());
      }
      else if (_net.isHorSym()) {
        rangeL = std::min(rangeL, box.xl());
        rangeH = std::max(rangeH, box.xh());
      }
      _vCompBoxes[i].emplace_back(box, layerIdx);
      _vCompSpatialBoxes[i][layerIdx].insert(box);
    }
    
    //const auto& compPts = topo.compPts(topoCompIdx);
    //for (const auto& pt : compPts) {
      //_vCompAcsPts[i].emplace(pt);
    //}
    _ps.topoComp2AcsPts(topo, topoCompIdx, _vCompAcsPts[i]);
  }
  //auto& src = _vCompAcsPts.at(0);
  //auto& tar = _vCompAcsPts.at(1);

  //std::cerr << "src:" << std::endl;
  //for (const auto& p : src) {
    //if (p.x() == 8263)
    //std::cerr << p << std::endl;
  //}
  //std::cerr << "tar:" << std::endl;
  //for (const auto& p : tar) {
    //if (p.x() == 8263)
    //std::cerr << p << std::endl;
  //}

  for (i = 0; i < static_cast<Int>(_vRoutableIds.size()); ++i) {
    const Int compIdx = i + _vTopoCompIds.size();
    const Int routableIdx = _vRoutableIds.at(i);
    const Routable& routable = _net.routable(routableIdx);
    
    for (j = 0; j < routable.numPins(); ++j) {
      const Pin& pin = routable.pin(j);

      const Box<Int>* pBox;
      Pin_ForEachLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, k) {
          const Box<Int>& box = *pBox;
          if (_net.isVerSym()) { 
            rangeL = std::min(rangeL, box.yl());
            rangeH = std::max(rangeH, box.yh());
          }
          else if (_net.isHorSym()) {
            rangeL = std::min(rangeL, box.xl());
            rangeH = std::max(rangeH, box.xh());
          }
          _vCompBoxes[compIdx].emplace_back(box, layerIdx);
          _vCompSpatialBoxes[compIdx][layerIdx].insert(box);
        }
      }
      const Point3d<Int>* pAcs;
      Pin_ForEachAcs(pin, pAcs, k) {
        _vCompAcsPts[compIdx].insert(*pAcs);
      }
    }

    //for (j = 0; j < static_cast<Int>(routable.sWires().size()); ++j) {
    for (const auto& wire : routable.sWires()) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (!_cir.layer(layerIdx).isMetal())
        continue;
      if (_net.isVerSym()) {
        rangeL = std::min(rangeL, box.yl());
        rangeH = std::max(rangeH, box.yh());
      }
      else if (_net.isHorSym()) {
        rangeL = std::min(rangeL, box.xl());
        rangeH = std::max(rangeH, box.xh());
      }
      _vCompBoxes[compIdx].emplace_back(box, layerIdx);
      _vCompSpatialBoxes[compIdx][layerIdx].insert(box);
      addAcsPts(compIdx, box, layerIdx);
    }

  }
  // add dummy pin at the sym axis for self-symmetric nets
  // init access points pointing to the left
  const Int expandStepX = _region.bboxInner().width() * _fcVirtualPinStepX;
  const Int expandStepY = _region.bboxInner().height() * _fcVirtualPinStepY;
  if (_net.isVerSym()) {
    const Int expand = expandStepY * _virtualPinExpandStepY;
    rangeL -= expand;
    rangeH += expand;
  }
  else if (_net.isHorSym()) {
    const Int expand = expandStepX * _virtualPinExpandStepX;
    rangeL -= expand;
    rangeH += expand;
  }
  
  if (_isSelfSym and !_hasCrossAxisPin) {
    assert(_ro.isSelfSym());

    const Int dummyIdx = _vTopoCompIds.size();
    _vTopoCompIds.emplace_back(-1);

    Box<Int> dummyCompBox;
    if (_net.isVerSym()) {
      dummyCompBox.set(_net.symAxisX() - expandStepX, rangeL,
                       _net.symAxisX() + expandStepX, rangeH);
    }
    else if (_net.isHorSym()) {
      dummyCompBox.set(rangeL, _net.symAxisY() - expandStepY,
                       rangeH, _net.symAxisY() + expandStepY);
    }

    const MetalLayer* pLayer;
    Cir_ForEachMetalLayer(_cir, pLayer, j) {
      const Int layerIdx = pLayer->idx();
      if (layerIdx > _region.numRouteGrids()) {
        break;
      }
      _vCompBoxes[dummyIdx].emplace_back(dummyCompBox, layerIdx);
      _vCompSpatialBoxes[dummyIdx][layerIdx].insert(dummyCompBox);

      Vector<Point3d<Int>> vDummyAcs;
      addAcsPts(dummyIdx, dummyCompBox, layerIdx);
      for (const Point3d<Int>& acs : vDummyAcs) {
        _vCompAcsPts[dummyIdx].insert(acs);
      }
    }
  }
}


void AstarCore::initFromScratch()
{
  _vpPins.clear();
  
  if (_isSelfSym) {
    initFromScratchSelfSym();
  }
  else {
    assert(!_ro.isSelfSym());
    for (Int i = 0; i < _ro.numPins(); ++i) {
      _vpPins.emplace_back(_ro.pPin(i));
    }
    for (Int i = 0; i < _ro.numRoutables(); ++i) {
      _vRoutableIds.emplace_back(_ro.routableIdx(i));
    }
    const Int numComps = _ro.numPins() + _ro.numRoutables();
    _compDS.init(numComps);
    _vCompBoxes.resize(numComps);
    _vCompAcsPts.resize(numComps);
    _vCompSpatialBoxes.resize(numComps);
  }

  Int i, j, k, layerIdx;
  Int rangeL = MAX_INT, rangeH = MIN_INT; // dummy pin range
  const Box<Int>* pBox;
  const Point3d<Int>* pAcs;
  for (i = 0; i < static_cast<Int>(_vpPins.size()); ++i) {
    const Pin& pin = *_vpPins.at(i);
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
        const Box<Int>& box = *pBox;
        if (_net.isVerSym()) {
          rangeL = std::min(rangeL, box.yl());
          rangeH = std::max(rangeH, box.yh());
        }
        else if (_net.isHorSym()) {
          rangeL = std::min(rangeL, box.xl());
          rangeH = std::max(rangeH, box.xh());
        }
        _vCompBoxes[i].emplace_back(box, layerIdx);
        _vCompSpatialBoxes[i][layerIdx].insert(box);
      }
    }

    Pin_ForEachAcs(pin, pAcs, j) {
      _vCompAcsPts[i].emplace(*pAcs);
    }
  }
  for (i = 0; i < static_cast<Int>(_vRoutableIds.size()); ++i) {
    const Int compIdx = i + _vpPins.size();
    const Int routableIdx = _vRoutableIds.at(i);
    const Routable& routable = _net.routable(routableIdx);
    
    for (j = 0; j < routable.numPins(); ++j) {
      const Pin& pin = routable.pin(j);
      Pin_ForEachMetalLayerIdx(pin, layerIdx) {
        Pin_ForEachLayerBox(pin, layerIdx, pBox, k) {
          const Box<Int>& box = *pBox;
          if (_net.isVerSym()) {
            rangeL = std::min(rangeL, box.yl());
            rangeH = std::max(rangeH, box.yh());
          }
          else if (_net.isHorSym()) {
            rangeL = std::min(rangeL, box.xl());
            rangeH = std::max(rangeH, box.xh());
          } 
          _vCompBoxes[compIdx].emplace_back(box, layerIdx);
          _vCompSpatialBoxes[compIdx][layerIdx].insert(box);
        }
      }
      Pin_ForEachAcs(pin, pAcs, k) {
        _vCompAcsPts[compIdx].insert(*pAcs);
      }

    }
    for (const auto& wire : routable.sWires()) {
      const auto& box = wire.first;
      const Int layerIdx = wire.second;
      if (!_cir.layer(layerIdx).isMetal())
        continue;
      if (_net.isVerSym()) {
        rangeL = std::min(rangeL, box.yl());
        rangeH = std::max(rangeH, box.yh());
      }
      else if (_net.isHorSym()) {
        rangeL = std::min(rangeL, box.xl());
        rangeH = std::max(rangeH, box.xh());
      }
      _vCompBoxes[compIdx].emplace_back(box, layerIdx);
      _vCompSpatialBoxes[compIdx][layerIdx].insert(box);
      addAcsPts(compIdx, box, layerIdx);
    }
    
  }
  // add dummy pin at the sym axis for self-symmetric nets
  // init access points pointing to the left
  const Int expandStepX = _region.bboxInner().width() * _fcVirtualPinStepX;
  const Int expandStepY = _region.bboxInner().height() * _fcVirtualPinStepY;
  if (_net.isVerSym()) {
    const Int expand = expandStepY * _virtualPinExpandStepY;
    rangeL -= expand;
    rangeH += expand;
  }
  else if (_net.isHorSym()) {
    const Int expand = expandStepX * _virtualPinExpandStepX;
    rangeL -= expand;
    rangeH += expand;
  }
  //spdlog::info("{} {}", _region.bboxInner().width() * _fcVirtualPinStepX, _region.bboxInner().height() * _fcVirtualPinStepY);
  if (_isSelfSym and !_hasCrossAxisPin) {
    assert(_ro.isSelfSym());
    
    const Int dummyIdx = _vpPins.size();
    _vpPins.emplace_back(nullptr);

    Box<Int> dummyPinBox;
    if (_net.isVerSym()) {
      dummyPinBox.set(_net.symAxisX() - expandStepX, rangeL,
                      _net.symAxisX() + expandStepX, rangeH);
    }
    else if (_net.isHorSym()) {
      dummyPinBox.set(rangeL, _net.symAxisY() - expandStepY,
                      rangeH, _net.symAxisY() + expandStepY);
    }
    
    const MetalLayer* pLayer;
    Cir_ForEachMetalLayer(_cir, pLayer, j) {
      const Int layerIdx = pLayer->idx();
      if (layerIdx > _region.numRouteGrids()) {
        break;
      }
      _vCompBoxes[dummyIdx].emplace_back(dummyPinBox, layerIdx);
      _vCompSpatialBoxes[dummyIdx][layerIdx].insert(dummyPinBox);

      Vector<Point3d<Int>> vDummyAcs;
      addAcsPts(dummyIdx, dummyPinBox, layerIdx);
      for (const Point3d<Int>& acs : vDummyAcs) {
        _vCompAcsPts[dummyIdx].insert(acs);
      }
    }
  }
}

void AstarCore::initFromScratchSelfSym()
{
  assert(_ro.numRoutables() == 0);
  
  Int i, j, layerIdx;
  const Pin* pPin;
  Routable_ForEachPin(_ro, pPin, i) {
    bool isPartA = false;
    bool isPartB = false;
    const Pin& pin = *pPin;
    const Box<Int>* pBox;
    Pin_ForEachMetalLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
        const Box<Int>& box = *pBox;
        if (_net.isVerSym()) {
          if (box.xl() < _net.symAxisX()) {
            isPartA = true;
          }
          if (box.xh() > _net.symAxisX()) {
            isPartB = true;
          }
        }
        else if (_net.isHorSym()) {
          if (box.yl() < _net.symAxisY()) {
            isPartA = true;
          }
          if (box.yh() > _net.symAxisY()) {
            isPartB = true;
          }
        }
      }
    }
    if (isPartA and isPartB) {
      _hasCrossAxisPin = true;
    }
    if (isPartA) {
      _vpPins.emplace_back(const_cast<Pin*>(pPin));
    }
  }

  const Int halfNumPins = !_hasCrossAxisPin ? _vpPins.size() + 1 : _vpPins.size();

  _compDS.init(halfNumPins);
  _vCompBoxes.resize(halfNumPins);
  _vCompAcsPts.resize(halfNumPins);
  _vCompSpatialBoxes.resize(halfNumPins);
}

void AstarCore::initRL()
{
  //assert(_ro.numRoutables() == 0);

  //for (Int i = 0; i < _ro.numPins(); ++i) {
    //_vpPins.emplace_back(_ro.pPin(i));
  //}
  //const Int numComps = _ro.numPins();

}

bool AstarCore::splitSubNet()
{
  if (_ro.numArcs() > 0) {
    return splitSubNetFromPartial();
  }
  return splitSubNetFromScratch();
}

bool AstarCore::splitSubNetFromScratch()
{
  const bool hasDummy = !_hasCrossAxisPin && _isSelfSym;
  const Int numRealPins = hasDummy ? _vCompBoxes.size() - 1 : _vCompBoxes.size();

  // compute cost values of edges
  MinimumSpanningTree<Int> mst(numRealPins);
  for (Int i = 0; i < numRealPins - 1; ++i) {
    for (Int j = i + 1; j < numRealPins; ++j) {
      Int minDist = MAX_INT;
      for (const auto& u : _vCompBoxes.at(i)) {
        for (const auto& v : _vCompBoxes.at(j)) {
          if (_cir.layer(u.second).isMetal() and _cir.layer(v.second).isMetal()) {
            const Int dist = _ps.scaledMdist(u, v);
            if (dist < minDist) {
              minDist = dist;
            }
          }
        }
      }
      assert(minDist != MAX_INT);
      if (minDist == MAX_INT) {
        return false;
      }
      mst.addEdge(i, j, minDist);
    }
  }

  mst.solve(_vSubNets);

  // handle the dummy pin
  if (hasDummy) {
    assert(_ro.isSelfSym() and _ro.numRoutables() == 0);
    assert(_vCompBoxes.size() == _vpPins.size());
    const Int dummyIdx = _vCompBoxes.size() - 1;
    Int luckyIdx = -1; // the comp to conenct to the dummy pin
    Int minDist = MAX_INT;
    for (Int i = 0; i < numRealPins; ++i) {
      for (const auto& u : _vCompBoxes.at(i)) {
        for (const auto& v : _vCompBoxes.at(dummyIdx)) {
          if (_cir.layer(u.second).isMetal() and _cir.layer(v.second).isMetal()) {
            Int dist = _ps.scaledMdist(u, v);
            if (dist < minDist) {
              minDist = dist;
              luckyIdx = i;
            }
          }
        }
      }
    }
    assert(luckyIdx != -1);
    _vSubNets.emplace_back(luckyIdx, dummyIdx);
  }

  return true;
}

bool AstarCore::splitSubNetFromPartial()
{
  const bool hasDummy = !_hasCrossAxisComp && _isSelfSym;
  const Int numRealComps = hasDummy ? _vCompBoxes.size() - 1 : _vCompBoxes.size();

  // compute cost values of edges
  //std::cerr << "asdfjklasjf " << numRealComps << std::endl;
  MinimumSpanningTree<Int> mst(numRealComps);
  for (Int i = 0; i < numRealComps - 1; ++i) {
    for (Int j = i + 1; j < numRealComps; ++j) {
      Int minDist = MAX_INT;
      for (const auto& u : _vCompBoxes.at(i)) {
        for (const auto& v : _vCompBoxes.at(j)) {
          if (_cir.layer(u.second).isMetal() and _cir.layer(v.second).isMetal()) {
            const Int dist = _ps.scaledMdist(u, v);
            if (dist < minDist) {
              minDist = dist;
            }
          }
        }
      }
      //if (minDist == MAX_INT) {
        //for (const auto& v : _vCompBoxes) {
          //std::cerr << "comp " << std::endl;
          //for (const auto& u : v) {
            //std::cerr << u.first << " " << u.second << std::endl;
          //}
        //}
        //std::cerr << i << " " << j << std::endl;
        //_ro.topo().showComps();
        //std::cerr << _ro.numRoutables() << std::endl;
        //std::cerr << _net.numRoutables() << std::endl;
        //std::cerr << std::endl;
        //std::cerr << "ro wires " << _ro.sWires().size() << std::endl;
        //Vector<Pair<Box<Int>, Int>> vRo(_ro.sWires().begin(), _ro.sWires().end());
        //std::sort(vRo.begin(), vRo.end(), [](auto& a, auto&b) {return std::tie(a.second, a.first) < std::tie(b.second, b.first);});
        //for (const auto& w : vRo) {
          //std::cerr << w.first << " " << w.second << std::endl;
        //}

        //_net.topo().showComps();
        //std::cerr << "net wires " << _net.sWires().size() << std::endl;
        //Vector<Pair<Box<Int>, Int>> vn(_net.sWires().begin(), _net.sWires().end());
        //std::sort(vn.begin(), vn.end(), [](auto& a, auto&b) {return std::tie(a.second, a.first) < std::tie(b.second, b.first);});
        //for (const auto& w : vn) {
          //std::cerr << w.first << " " << w.second << std::endl;
        //}
        //std::cerr << std::endl;
        //std::cerr << "spatial" << std::endl;
        //for (Int i : {0, 1, 2, 3, 4, 5, 6, 7, 8}) {
          //const auto& sp = _cir.spatialNets(i);
          //for (const auto& e : sp) {
            //if (e.second->name() == _net.name()) {
              //std::cerr << e.first  << " " << i << std::endl;
            //}
          //}
        //}
      //}
      assert(minDist != MAX_INT);
      if (minDist == MAX_INT) {
        return false;
      }
      mst.addEdge(i, j, minDist);
    }
  }
  mst.solve(_vSubNets);

  // handle the dummy comp
  if (hasDummy) {
    assert(_ro.isSelfSym() and _ro.numRoutables() == 0);
    assert(_vCompBoxes.size() == _vTopoCompIds.size());
    const Int dummyIdx = _vCompBoxes.size() - 1;
    Int luckyIdx = -1;
    Int minDist = MAX_INT;
    for (Int i = 0; i < numRealComps; ++i) {
      for (const auto& u : _vCompBoxes.at(i)) {
        for (const auto& v : _vCompBoxes.at(dummyIdx)) {
          if (_cir.layer(u.second).isMetal() and _cir.layer(v.second).isMetal()) {
            Int dist = _ps.scaledMdist(u, v);
            if (dist < minDist) {
              minDist = dist;
              luckyIdx = -1;
            }
          }
        }
      }
    }
    assert(luckyIdx != -1);
    _vSubNets.emplace_back(luckyIdx, dummyIdx);
  }

  return true;
}

bool AstarCore::route()
{
  for (const auto& pair : _vSubNets) {
    const Int srcIdx = pair.first;
    const Int tarIdx = pair.second;
    //spdlog::info("{} {}", srcIdx, tarIdx);
    //std::cerr << "kasjf;lsdjf;s " << srcIdx << " " << tarIdx << std::endl;
    if (!routeSubNet(srcIdx, tarIdx)) {
      return false;
    }
  }
  return true;

}

bool AstarCore::routeSubNet(Int srcIdx, Int tarIdx)
{
  srcIdx = _compDS.find(srcIdx);
  tarIdx = _compDS.find(tarIdx);
  assert(srcIdx != tarIdx);
  auto& src = _vCompAcsPts.at(srcIdx);
  auto& tar = _vCompAcsPts.at(tarIdx);


  resetAllNodes();

  // init access pins to node maps
  auto initAcs = [&] (const auto& compPts) {
    for (const Point3d<Int>& p : compPts) {
      if (_allNodesMap.find(p) == _allNodesMap.end()) {
        _allNodesMap.emplace(p, std::make_unique<AstarNode>(p));
      }
    }
  };
  initAcs(src);
  initAcs(tar);
  return pathSearch(srcIdx, tarIdx);
}

void AstarCore::resetAllNodes()
{
  for (auto& item : _allNodesMap) {
    AstarNode* pNode = item.second.get();
    pNode->reset();
  }
}

bool AstarCore::pathSearch(const Int srcIdx, const Int tarIdx)
{
  auto& src = _vCompAcsPts.at(srcIdx);
  auto& tar = _vCompAcsPts.at(tarIdx);

  // init KD trees
  auto initKdTrees = [] (K3dTree<Int>& kd, const auto& tar) {
    for (const Point3d<Int>& p : tar) {
      kd.insert(p);
    }
    kd.buildIndex();
  };
  K3dTree<Int> kd0, kd1;
  initKdTrees(kd0, src);
  initKdTrees(kd1, tar);

  // init pqueue
  using Pqueue0 = PairingHeap<AstarNode*, AstarNode::Cmp<0>>;
  using Pqueue1 = PairingHeap<AstarNode*, AstarNode::Cmp<1>>;
  using IterMap0 = FlatHashMap<AstarNode*, Pqueue0::point_iterator>;
  using IterMap1 = FlatHashMap<AstarNode*, Pqueue1::point_iterator>;
  Pqueue0 pq0;
  Pqueue1 pq1;
  IterMap0 itMap0;
  IterMap1 itMap1;

  auto addPt2Pq = [&] (auto& pq, auto& itMap, const K3dTree<Int>& tarKd, const auto& tar, const bool i) {
    for (const Point3d<Int>& p : tar) {
      assert(_cir.layer(p.z()).isMetal());
      Point3d<Int> scaledPt(p.x() * _ps.horCost(),
                            p.y() * _ps.verCost(),
                            _ps.viaCost(0, p.z()));
                            //p.z() * _ps.viaCost());
      Point3d<Int> scaledNearestPt;
      Int costH = MAX_INT;
      tarKd.nearestSearch(p, scaledNearestPt, costH);
      
      AstarNode* pNode = _allNodesMap.at(p).get();
      pNode->costF[i] = _fcH * costH;
      pNode->costG[i] = 0;
      pNode->bendCnt[i] = 0;
      itMap[pNode] = pq.push(pNode);
    }
  };
  addPt2Pq(pq0, itMap0, kd1, src, 0);
  addPt2Pq(pq1, itMap1, kd0, tar, 1);

  // path search
  auto search = [&] (auto& pq, auto& itMap, const K3dTree<Int>& tarKd, const Int srcIdx, const Int tarIdx, const bool i) -> bool {
    AstarNode* pU = pq.top();
    if (bTerminate(pU, tarIdx, i)) {
      mergeComp(srcIdx, tarIdx);
      backTrack(pU, srcIdx, tarIdx);
      return true;
    }

    pq.pop();

    pU->visited[i] = true;
    if (pU->vpNeighbors.empty()) {
      findNeighbors(pU);
    }
    for (auto pV : pU->vpNeighbors) {
      if (pV->visited[i])
        continue;
      if (!isValidNode(pV))
        continue;

      Real costG = pU->costG[i] + _ps.scaledMdist(pU->loc, pV->loc) + _ps.history(pU->loc, pV->loc);
      if (bViolateDRC(pU, pV, srcIdx, tarIdx, i)) {
        if (_useStrictDRC) {
          continue;
        }
        else {
          costG += _ps.drcCost();
        }
      }
      const Int bendCnt = pU->bendCnt[i] + hasBend(pU, pV, i);
      if (bNeedUpdate(pV, costG, bendCnt, i)) {
        assert(_cir.layer(pV->loc.z()).isMetal());
        Point3d<Int> scaledPt(pV->loc.x() * _ps.horCost(),
                              pV->loc.y() * _ps.verCost(),
                              _ps.viaCost(0, pV->loc.z()));
                              //pV->loc.z() * _ps.viaCost());
        Point3d<Int> scaledNearestPt;
        Int costH;
        tarKd.nearestSearch(pV->loc, scaledNearestPt, costH);
        const Real costF = costG * _fcG + costH * _fcH;
        
        pV->pParent[i] = pU;
        pV->costG[i] = costG;
        pV->costF[i] = costF;
        pV->bendCnt[i] = bendCnt;
        auto it = itMap.find(pV);
        if (it != itMap.end())
          pq.modify(it->second, pV);
        else
          itMap[pV] = pq.push(pV);
      }
    }
    return false;
  };
  while (!pq0.empty() and !pq1.empty()) {
    //spdlog::info("{} {}", pq0.size(), pq1.size());
    if (search(pq0, itMap0, kd1, srcIdx, tarIdx, 0))
      return true;
    if (search(pq1, itMap1, kd0, tarIdx, srcIdx, 1))
      return true;
  }

  return false;
}

bool AstarCore::bTerminate(AstarNode* pU, const Int tarIdx, const bool isReverse)
{
  auto& tar = _vCompAcsPts.at(tarIdx);
  if (pU->visited[isReverse ^ 1]) {
    ////const Int width = _ps.wireWidth(pU->loc);
    ////const Int ext = width / 2;
    ////const Int layerIdx = pU->loc.z();
    //const Box<Int>& b0 = prevLayerCheckWire(pU, 0);
    //const Box<Int>& b1 = prevLayerCheckWire(pU, 1);
    //assert(Box<Int>::bConnect(b0, b1));
    //Box<Int> b(std::min(b0.xl(), b1.xl()),
               //std::min(b0.yl(), b1.yl()),
               //std::max(b0.xh(), b1.xh()),
               //std::max(b0.yh(), b1.yh()));

    //const Int layerIdx = pU->loc.z();
    //_drc.fitWire2ValidLength(layerIdx, b);

    ////if (!_drc.checkWireValidWidth(layerIdx, b)) {
      ////return false;
    ////}
    //////if (!_drc.checkWireValidLength(layerIdx, b)) {
      //////return false;
    //////}
    //if (!_drc.checkWireMinArea(layerIdx, b)) {
      //return false;
    //}
    //if (!_drc.checkWireMetalPrlSpacing(_net, layerIdx, b)) {
      //return false;
    //}
    //if (!_drc.checkWireMetalEolSpacing(_net, layerIdx, b)) {
      //return false;
    //}

    return true;
  }
  if (tar.find(pU->loc) != tar.end()) {
    return true;
  }
  return false;
}

void AstarCore::mergeComp(const Int srcIdx, const Int tarIdx)
{
  _compDS.merge(srcIdx, tarIdx);

  const Int rootIdx = _compDS.find(srcIdx);
  const Int childIdx = rootIdx == srcIdx ? tarIdx : srcIdx;

  if (_compDS.nSets() > 1) {
    // merge acs pts
    auto& rootCompAcsPts = _vCompAcsPts.at(rootIdx);
    auto& childCompAcsPts = _vCompAcsPts.at(childIdx);
    rootCompAcsPts.insert(childCompAcsPts.begin(), childCompAcsPts.end());

    // merge boxes
    auto& rootCompBoxes = _vCompBoxes.at(rootIdx);
    auto& childCompBoxes = _vCompBoxes.at(childIdx);
    rootCompBoxes.insert(rootCompBoxes.end(), childCompBoxes.begin(), childCompBoxes.end());
  }

}

void AstarCore::backTrack(const AstarNode* pU, const Int srcIdx, const Int tarIdx)
{
  assert(_compDS.find(srcIdx) == _compDS.find(tarIdx));
  const Int rootIdx = _compDS.find(srcIdx);


  auto add2Path = [&] (List<Point3d<Int>>& lPathPts, const Point3d<Int>& pt, const bool i) {
    if (i == 0)
      lPathPts.emplace_front(pt);
    else
      lPathPts.emplace_back(pt);
    _vCompAcsPts[rootIdx].emplace(pt);
  };

  List<Point3d<Int>> lPathPts;
  add2Path(lPathPts, pU->loc, 0);

  auto backT = [&] (const Int i) {
    const AstarNode* pPar = pU->pParent[i];
    if (pPar == nullptr)
      return;
    do {
      add2Path(lPathPts, pPar->loc, i);
      pPar = pPar->pParent[i];
    } while (pPar != nullptr);
  };
  backT(0);
  backT(1);

  List<Pair<Point3d<Int>, Point3d<Int>>> lPathArcs;

  // paths
  mergePath(lPathPts, lPathArcs);
  savePath(lPathArcs);
  //addHistory(lPathPts);
  addHistory(lPathArcs);

  // wires
  Vector<Pair<Box<Int>, Int>> vWires;
  path2Wires(lPathArcs, vWires);
  saveWires(vWires);


  // add wires to comp boxs
  // add wires to tmp spatial
  for (const auto& wire : vWires) {
    const Box<Int>& box = wire.first; 
    const Int layerIdx = wire.second; 
    _vCompBoxes[rootIdx].emplace_back(wire);
    _vCompSpatialBoxes[rootIdx][layerIdx].insert(box);
    //_vTmpSpatialNets[layerIdx].insert(box, &_net);
    _cir.spatialNets(layerIdx).insert(box, &_net);
    if (_isSym) {
      Box<Int> symBox(box);
      if (_net.isVerSym()) {
        symBox.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symBox.flipY(_net.symAxisY());
      }
      //_vTmpSpatialNets[layerIdx].insert(symBox, _net.pSymNet());
      _cir.spatialNets(layerIdx).insert(symBox, _net.pSymNet());
    }
    else if (_isSelfSym) {
      Box<Int> symBox(box);
      if (_net.isVerSym()) {
        symBox.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symBox.flipY(_net.symAxisY());
      }
      //_vTmpSpatialNets[layerIdx].insert(symBox, &_net);
      _cir.spatialNets(layerIdx).insert(symBox, &_net);
    }
  }


}

void AstarCore::mergePath(const List<Point3d<Int>>& lPathPts, List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs) const
{
  if (lPathPts.size() == 1) {
    //lPathArcs.emplace_back(lPathPts.front(), lPathPts.front());
    return;
  }
  auto u1 = lPathPts.begin();
  auto v1 = std::next(u1, 1);
  auto v2 = std::next(v1, 1);
  for (; v2 != lPathPts.end(); ++v2) {
    if (bNeedMergePath(*u1, *v1, *v1, *v2)) {
      v1 = v2;
    }
    else {
      lPathArcs.emplace_back(*u1, *v1);
      u1 = v1;
      v1 = v2;
    }
  }
  lPathArcs.emplace_back(*u1, *v1);
}

bool AstarCore::bNeedMergePath(const Point3d<Int>& u1, const Point3d<Int>& v1, const Point3d<Int>& u2, const Point3d<Int>& v2) const
{
  // path1: u1 -> v1, path2: u2 -> v2
  if (u1.z() != v1.z()) {
    assert(u1.x() == v1.x() and u1.y() == v1.y());
    return false;
  }
  return direction3d::findDir(u1, v1) == direction3d::findDir(u2, v2);
}

void AstarCore::findNeighbors(AstarNode* pU)
{
  assert(pU->vpNeighbors.empty());
  
  const Point3d<Int>& u = pU->loc;

  auto addNeighbor = [&] (const Point3d<Int>& v) {
    if (_allNodesMap.find(v) == _allNodesMap.end()) {
      _allNodesMap.emplace(v, std::make_unique<AstarNode>(v));
    }
    pU->vpNeighbors.emplace_back(_allNodesMap.at(v).get());
  };

  const Vector<Int>& vAdjEdgeIds = _ps.drMgr().vAdjEdgeIds(u);
  for (const Int eIdx : vAdjEdgeIds) {
    if (_ps.drMgr().isValidEdge(eIdx)) {
      const auto& edge = _ps.drMgr().gridEdge(eIdx);
      const auto& v = _ps.drMgr().adjNode(edge, u);
      assert(_cir.layer(v.z()).isMetal());
      addNeighbor(v);
    }
  }
  
}

void AstarCore::path2Wires(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  Vector<Vector<Box<Int>>> vvBoxes(_cir.numLayers());
  Vector<Vector<Polygon<Int>>> vvPolygons(_cir.numLayers());

  for (const auto& arc : lPathArcs) {
    const Point3d<Int>& u = arc.first;
    const Point3d<Int>& v = arc.second;
    if (u.z() == v.z()) {
      const Int width = _ps.wireWidth(u);
      const Int eolExtension = width / 2;
      const Box<Int>& wire = _ps.toWire(u, v, width, eolExtension);

      vvBoxes[u.z()].emplace_back(wire);
    }
    else {
      assert(u.x() == v.x() and u.y() == v.y());
      const Layer& lowerLayer = _cir.layer(std::min(u.z(), v.z()));
      assert(lowerLayer.isMetal());
      const RouteGrid& rg = _cir.routeGrid(lowerLayer.selfIdx());
     
      // pick via (pick first currently)
      assert(rg.hasValidVias(u.x(), u.y()));
      const Vector<const Via*>& vpValidVias = rg.vpValidVias(u.x(), u.y());
      const Via& via = *vpValidVias.at(0);
      
      Int i, layerIdx;
      const Box<Int>* pBox;
      Via_ForEachLayerIdx(via, layerIdx) {
        Via_ForEachLayerBox(via, layerIdx, pBox, i) {
          const Box<Int> b(pBox->xl() + u.x(),
                           pBox->yl() + u.y(),
                           pBox->xh() + u.x(),
                           pBox->yh() + u.y());
          vvBoxes[layerIdx].emplace_back(b);
        }
      }
    }
  }

  for (Int i = 0; i < _cir.numLayers(); ++i) {
    const auto& vBoxes = vvBoxes.at(i);
    auto& vPolygons = vvPolygons.at(i);
    const Layer& layer = _cir.layer(i);
    if (!vBoxes.empty()) {
      //if (layer.idx() <= _cir.layer("m5t").idx()) {
      if (layer.isMetal()) {
        geo::box2Polygon<Int>(vBoxes, vPolygons);
        for (const Polygon<Int>& polygon : vPolygons) {
          const Ring<Int>& ring = polygon.outer();

          //if (ring.size() != 4) {
            //for (const auto& arc : lPathArcs) {
              //std::cerr << arc.first << " " << arc.second << std::endl;
            //}
            //std::cerr << std::endl;
            //std::cerr << std::endl;
            //for (const auto& b : vBoxes) {
              //std::cerr << b << std::endl;
            //}
          //}
          assert(ring.size() == 4);
          Box<Int> b(ring[1].x(), ring[1].y(), ring[3].x(), ring[3].y());
          
          _drc.fitWire2ValidLength(i, b);

          vWires.emplace_back(b, i);
        }
      }
      else { // cut layers
        for (const Box<Int>& b : vBoxes) {
          vWires.emplace_back(b, i);
        }
      }
    }
  }
  

}

void AstarCore::savePath(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs)
{
  _vvRoutePaths.emplace_back();
  _vvRoutePathVias.emplace_back();
  _vvRoutePathViaOrients.emplace_back();
  _vvRoutePathWidthExts.emplace_back();

  _vvRoutePaths.back().assign(lPathArcs.begin(), lPathArcs.end());
  for (const auto& arc : _vvRoutePaths.back()) {
    const Point3d<Int>& u = arc.first;
    const Point3d<Int>& v = arc.second;
    if (u.z() == v.z()) {
      const Int width = _ps.wireWidth(u);
      const Int eolExtension = width / 2;
      _vvRoutePathVias.back().emplace_back(nullptr);
      _vvRoutePathViaOrients.back().emplace_back(Orient2dE::undef);
      _vvRoutePathWidthExts.back().emplace_back(width, eolExtension);
    }
    else {
      const Layer& lowerLayer = _cir.layer(std::min(u.z(), v.z()));
      const Layer& upperLayer = _cir.layer(std::max(u.z(), v.z()));
      assert(lowerLayer.isMetal());
      assert(upperLayer.isMetal());
      const RouteGrid& rg = _cir.routeGrid(lowerLayer.selfIdx());
      assert(rg.hasValidVias(u.x(), u.y()));
      const Vector<const Via*>& vpValidVias = rg.vpValidVias(u.x(), u.y());
      const Via* pVia = vpValidVias.at(0);
      _vvRoutePathVias.back().emplace_back(pVia);
      _vvRoutePathViaOrients.back().emplace_back(Orient2dE::n);
      _vvRoutePathWidthExts.back().emplace_back(0, 0);
    }
  }
}

void AstarCore::saveWires(const Vector<Pair<Box<Int>, Int>>& vWires)
{
  _vvRouteWires.emplace_back();
  _vvRouteWires.back().assign(vWires.begin(), vWires.end());
}

void AstarCore::addHistory(const List<Point3d<Int>>& lPathPts)
{
  auto u = lPathPts.begin();
  auto v = std::next(u, 1);
  for (; v != lPathPts.end(); u = v, ++v) {
    const Int uGridNodeIdx = _ps.drMgr().gridNodeIdx(*u);
    const Int vGridNodeIdx = _ps.drMgr().gridNodeIdx(*v);
    const Int eIdx = _ps.drMgr().gridEdgeIdx(uGridNodeIdx, vGridNodeIdx);
    _ps.addHistory(eIdx, _ps.hisCost());
  }
}

void AstarCore::addHistory(const List<Pair<Point3d<Int>, Point3d<Int>>>& lPathArcs)
{
  for (const auto& p : lPathArcs) {
    const auto& u = p.first;
    const auto& v = p.second;
    _ps.addHistory({u, v}, _ps.hisCost());
  }
}

bool AstarCore::isValidNode(const AstarNode* pV) const 
{
  const Point3d<Int>& v = pV->loc;
  if (_isSym or _isSelfSym) {
    Point3d<Int> symV(v);
    if (_net.isVerSym()) {
      symV.flipX(_net.symAxisX());
    }
    else if (_net.isHorSym()) {
      symV.flipY(_net.symAxisY());
    }
    if (!_ps.drMgr().hasNode(symV)) {
      return false;
    }
  } 
  return _ps.drMgr().hasNode(v);
}

bool AstarCore::bViolateDRC(const AstarNode* pU, const AstarNode* pV, const Int srcIdx, const Int tarIdx, const bool isReverse) const
{

  const Point3d<Int>& u = pU->loc;
  const Point3d<Int>& v = pV->loc;
  //std::cerr << u << " " << v << std::endl;
  if (u.z() == v.z()) {
    assert(u.x() == v.x() or u.y() == v.y());
    const Int layerIdx = u.z();

    //const Int width = _ps.wireWidth(u);
    //const Int eolExtension = width / 2;
    //const Box<Int>& wire = _ps.toWire(u, v, width, eolExtension);
    Box<Int> wire = curLayerCheckWire(pU, pV, isReverse);
    
    Int xl = wire.xl(), yl = wire.yl(), xh = wire.xh(), yh = wire.yh();
    const Box<Int> bfsWire = bfsNetWire(wire, u.z());
    xl = std::min(xl, bfsWire.xl());
    yl = std::min(yl, bfsWire.yl());
    xh = std::max(xh, bfsWire.xh());
    yh = std::max(yh, bfsWire.yh());
    wire.set(xl, yl, xh, yh);
    
    // fit to valid length
    _drc.fitWire2ValidLength(u.z(), wire);
    
    if (!_drc.checkWireMetalPrlSpacing(_net, layerIdx, wire)) {
      //spdlog::error("{} Metal Prl Spacing", _cir.layer(layerIdx).name());
      return true;
    }
    if (!_drc.checkWireMetalEolSpacing(_net, layerIdx, wire)) {
      //spdlog::error("{} Metal Prl Spacing", _cir.layer(layerIdx).name());
      return true;
    }
    if (_isSym) {
      Box<Int> symWire(wire);
      if (_net.isVerSym()) {
        symWire.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symWire.flipY(_net.symAxisY());
      }
      if (!_drc.checkWireMetalPrlSpacing(_net.symNet(), layerIdx, symWire)) {
        return true;
      }
      if (!_drc.checkWireMetalEolSpacing(_net.symNet(), layerIdx, symWire)) {
        return true;
      }
    }
    else if (_isSelfSym) {
      Box<Int> symWire(wire);
      if (_net.isVerSym()) {
        symWire.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symWire.flipY(_net.symAxisY());
      }
      if (!_drc.checkWireMetalPrlSpacing(_net, layerIdx, symWire)) {
        return true;
      }
      if (!_drc.checkWireMetalEolSpacing(_net, layerIdx, symWire)) {
        return true;
      }
    }
  }
  else { // via
    assert(u.x() == v.x() and u.y() == v.y());
    assert(_cir.layer(u.z()).isMetal());
    assert(_cir.layer(v.z()).isMetal());

    //const MetalLayer& preLayer = *static_cast<const MetalLayer*>(_cir.pLayer(u.z()));
    //const MetalLayer& curLayer = *static_cast<const MetalLayer*>(_cir.pLayer(v.z()));

    const RouteGrid& rg = _cir.routeGrid(_cir.layer(std::min(u.z(), v.z())).selfIdx());
    assert(rg.hasValidVias(u.x(), u.y()));
    const Vector<const Via*>& vpValidVias = rg.vpValidVias(u.x(), u.y());
    const Via& via = *vpValidVias.at(0);
    const Orient2dE orient = Orient2dE::n;
    
    //const Int width = _ps.wireWidth(u);
    //const Int eolExtension = width / 2;
    Box<Int> wire = prevLayerCheckWire(pU, isReverse);
    
    Int xl = wire.xl(), yl = wire.yl(), xh = wire.xh(), yh = wire.yh();
    Int i;
    const Box<Int>* pBox;
    Via_ForEachLayerBox(via, u.z(), pBox, i) {
      xl = std::min(xl, u.x() + pBox->xl());
      yl = std::min(yl, u.y() + pBox->yl());
      xh = std::max(xh, u.x() + pBox->xh());
      yh = std::max(yh, u.y() + pBox->yh());
    }
    wire.set(xl, yl, xh, yh);

    const Box<Int> bfsWire = bfsNetWire(wire, u.z());
    xl = std::min(xl, bfsWire.xl());
    yl = std::min(yl, bfsWire.yl());
    xh = std::max(xh, bfsWire.xh());
    yh = std::max(yh, bfsWire.yh());
    wire.set(xl, yl, xh, yh);
    
    // fit to valid length
    _drc.fitWire2ValidLength(u.z(), wire);
    
    //if (!_drc.checkWireValidWidth(u.z(), wire)) {
      ////spdlog::error("{} Valid Width", lowerLayer.name());
      //return true;
    //}
    //if (!_drc.checkWireValidLength(u.z(), wire)) {
      //return false;
    //}
    if (!_drc.checkViaSpacing(_net, u.x(), u.y(), via, orient)) {
      //spdlog::error("{} Via Spacing", lowerLayer.name());
      return true;
    }
    if (!_drc.checkWireMinArea(u.z(), wire)) {
      //spdlog::error("{} Min Area", lowerLayer.name());
      return true;
    }
    if (!_drc.checkWireMetalEolSpacing(_net, u.z(), wire)) {
      //std::cerr << u << " " << v << std::endl;
      //spdlog::error("{} Eol Spacing", lowerLayer.name());
      return true;
    }

    if (_isSym) {
      Point<Int> symPt(u.x(), u.y());
      Box<Int> symWire(wire);
      if (_net.isVerSym()) {
        symPt.flipX(_net.symAxisX());
        symWire.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symPt.flipY(_net.symAxisY());
        symWire.flipY(_net.symAxisY());
      }
      if (!_drc.checkViaSpacing(_net.symNet(), symPt.x(), symPt.y(), via, orient)) {
        return true;
      }
      if (!_drc.checkWireMetalEolSpacing(_net.symNet(), u.z(), symWire)) {
        return true;
      }
    }
    else if (_isSelfSym) {
      Point<Int> symPt(u.x(), u.y());
      Box<Int> symWire(wire);
      if (_net.isVerSym()) {
        symPt.flipX(_net.symAxisX());
        symWire.flipX(_net.symAxisX());
      }
      else if (_net.isHorSym()) {
        symPt.flipY(_net.symAxisY());
        symWire.flipY(_net.symAxisY());
      }
      if (!_drc.checkViaSpacing(_net, symPt.x(), symPt.y(), via, orient)) {
        return true;
      }
      if (!_drc.checkWireMetalEolSpacing(_net, u.z(), symWire)) {
        return true;
      }
    }

  }
  return false;
}

Box<Int> AstarCore::curLayerCheckWire(const AstarNode* pU, const AstarNode* pV, const bool i) const
{
  const AstarNode* pN = pU;
  while (pN->pParent[i] and pN->pParent[i]->loc.z() == pU->loc.z()) {
    pN = pN->pParent[i];
  }
  const Point3d<Int>& n = pN->loc;
  const Point3d<Int>& v = pV->loc;
  assert(n.x() == v.x() or n.y() == v.y());
  
  const Int xl = std::min(n.x(), v.x());
  const Int yl = std::min(n.y(), v.y());
  const Int xh = std::max(n.x(), v.x());
  const Int yh = std::max(n.y(), v.y());
  const Int hw = _ps.wireWidth(n) / 2;

  assert(_cir.layer(n.z()).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(n.z()));
  if (layer.isVer()) {
    assert(xl == xh);
    const Int l = yh - yl;
    const Int ext = l >= layer.maxValidLength() ? hw : (layer.validLength(layer.validLengthIdx(l)) - l) / 2;
    return Box<Int>(xl - hw,
                    yl - ext,
                    xh + hw,
                    yh + ext);
  }
  else {
    assert(layer.isHor());
    assert(yl == yh);
    const Int l = xh - xl;
    const Int ext = l >= layer.maxValidLength() ? hw : (layer.validLength(layer.validLengthIdx(l)) - l) / 2;
    return Box<Int>(xl - ext,
                    yl - hw,
                    xh + ext,
                    yh + hw);
  }
}

Box<Int> AstarCore::prevLayerCheckWire(const AstarNode* pU, const bool i) const
{
  const AstarNode* pN = pU;
  while (pN->pParent[i] and pN->pParent[i]->loc.z() == pU->loc.z()) {
    pN = pN->pParent[i];
  }
  const Point3d<Int>& n = pN->loc;
  const Point3d<Int>& u = pU->loc;
  assert(n.x() == u.x() or n.y() == u.y());
  const Int xl = std::min(n.x(), u.x());
  const Int yl = std::min(n.y(), u.y());
  const Int xh = std::max(n.x(), u.x());
  const Int yh = std::max(n.y(), u.y());
  const Int hw = _ps.wireWidth(n) / 2;

  assert(_cir.layer(u.z()).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(u.z()));
  if (layer.isVer()) {
    assert(xl == xh);
    const Int l = yh - yl;
    const Int ext = l >= layer.maxValidLength() ? hw : (layer.validLength(layer.validLengthIdx(l)) - l) / 2;
    return Box<Int>(xl - hw,
                    yl - ext,
                    xh + hw,
                    yh + ext);
  }
  else {
    assert(layer.isHor());
    assert(yl == yh);
    const Int l = xh - xl;
    const Int ext = l >= layer.maxValidLength() ? hw : (layer.validLength(layer.validLengthIdx(l)) - l) / 2;
    return Box<Int>(xl - ext,
                    yl - hw,
                    xh + ext,
                    yh + hw);
  }
}


Box<Int> AstarCore::bfsNetWire(const Box<Int>& wire, const Int layerIdx) const
{
  Queue<Box<Int>> qBoxes;
  FlatHashSet<Box<Int>> explored;
  
  Vector<Pair<Box<Int>, Net*>> vpTouchedNets;
  _cir.spatialNets(layerIdx).queryBoth(wire, vpTouchedNets);
  
  for (const auto& p : vpTouchedNets) {
    if (p.second == &_net) {
      qBoxes.push(p.first);
    }
    //assert(p.second == &_net);
  }

  if (qBoxes.empty()) {
    return wire;
  }

  Int xl = MAX_INT, yl = MAX_INT, xh = MIN_INT, yh = MIN_INT;

  while (!qBoxes.empty()) {
    const Box<Int> b = qBoxes.front();
    qBoxes.pop();

    xl = std::min(xl, b.xl());
    yl = std::min(yl, b.yl());
    xh = std::max(xh, b.xh());
    yh = std::max(yh, b.yh());

    explored.emplace(b);

    vpTouchedNets.clear();
    _cir.spatialNets(layerIdx).queryBoth(b, vpTouchedNets);
    for (const auto& p : vpTouchedNets) {
      //assert(p.second == &_net);
      if (p.second == &_net) {
        if (explored.find(p.first) == explored.end()) {
          qBoxes.emplace(p.first);
        }
      }
    }
  }

  return Box<Int>(xl, yl, xh, yh);
}

bool AstarCore::bNeedUpdate(const AstarNode* pV, const Int costG, const Int bendCnt, const bool isReverse) const
{
  if (pV->costG[isReverse] > costG)
    return true;
  else if (pV->bendCnt[isReverse] > bendCnt)
    return true;
  return false;
}

void AstarCore::ripup()
{
  for (const auto& vRouteWires : _vvRouteWires) {
    for (const auto& wire : vRouteWires) {
      const Box<Int>& box = wire.first;
      const Int layerIdx = wire.second;
      const bool bExist = _cir.spatialNets(layerIdx).erase(box, &_net);
      assert(bExist);
      if (_isSym) {
        Box<Int> symBox(box);
        if (_net.isVerSym()) {
          symBox.flipX(_net.symAxisX());
        }
        else if (_net.isHorSym()) {
          symBox.flipY(_net.symAxisY());
        }
        const bool bExistSym = _cir.spatialNets(layerIdx).erase(symBox, _net.pSymNet());
        assert(bExistSym);
      }
      else if (_isSelfSym) {
        Box<Int> symBox(box);
        if (_net.isVerSym()) {
          symBox.flipX(_net.symAxisX());
        }
        else if (_net.isHorSym()) {
          symBox.flipY(_net.symAxisY());
        }
        const bool bExistSym = _cir.spatialNets(layerIdx).erase(symBox, &_net);
        assert(bExistSym);
      }
    }
  }
}

void AstarCore::saveResult()
{
  assert(_vvRoutePaths.size() == _vvRoutePathVias.size());
  assert(_vvRoutePaths.size() == _vvRoutePathViaOrients.size());
  assert(_vvRoutePaths.size() == _vvRoutePathWidthExts.size());

  for (size_t i = 0; i < _vvRoutePaths.size(); ++i) {
    const auto& vRoutePath = _vvRoutePaths.at(i);
    const auto& vRoutePathVia = _vvRoutePathVias.at(i);
    const auto& vRoutePathViaOrient = _vvRoutePathViaOrients.at(i);
    const auto& vRoutePathWidthExts = _vvRoutePathWidthExts.at(i);
    for (size_t j = 0; j < vRoutePath.size(); ++j) {
      const auto& path = vRoutePath.at(j);
      const Point3d<Int>& u = path.first;
      const Point3d<Int>& v = path.second;
      //const Segment3d<Int> seg(u, v);
      //assert(seg.b90());
      //_ro.vArcIds().emplace_back(_net.numArcs());
      //_net.vArcs().emplace_back(u, v);
      //_net.vpArcVias().emplace_back(vRoutePathVia.at(j));
      //_net.vArcViaOrients().emplace_back(vRoutePathViaOrient.at(j));
      //_net.vArcWidthExts().emplace_back(vRoutePathWidthExts.at(j));
      const Segment3d<Int> arc(u, v);
      //_ro.arcs().emplace(arc);
      //_net.mArcs().emplace(arc, Net::ArcData(vRoutePathVia.at(j), vRoutePathViaOrient.at(j), vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second));
      const TopoTree::SegData arcData(vRoutePathVia.at(j), vRoutePathViaOrient.at(j), vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second, _ro.idx());
      //assert(_ps.drMgr().hasNode(arc.p0()));
      //assert(_ps.drMgr().hasNode(arc.p1()));
      //if (!arc.b90()) {
        //std::cerr << arc << std::endl;
        //for (const Pin* p : _ro.vpPins()) {
          //std::cerr << "pin" << std::endl;
          //Int i, layerIdx;
          //const Box<Int>* pBox;
          //Pin_ForEachLayerIdx((*p), layerIdx) {
            //Pin_ForEachLayerBox((*p), layerIdx, pBox, i) {
              //std::cerr << *pBox << " " << i << std::endl;
            //}
          //}
        //}
        //std::cerr << " " << std::endl;
        //for (const auto& v : _vvRoutePaths) {
          //std::cerr << "path: ";
          //for (const auto& path : v) {
            //std::cerr << Segment3d<Int>(path.first, path.second) << " ";
          //}
          //std::cerr << std::endl;
        //}
      //}
      _ro.addRoute(arc, arcData);
      _net.addRoute(arc, arcData);
      //std::cerr << "add: " << arc << std::endl;
    }
  }
  for (const auto& vRouteWires : _vvRouteWires) {
    for (const auto& wire : vRouteWires) {
      //_ro.vWireIds().emplace_back(_net.numWires());
      //_net.vWires().emplace_back(wire);
      _ro.sWires().emplace(wire);
      _net.sWires().emplace(wire);
    }
  }
  // sym & selfsym
  if (_isSym) {
    Net& symNet = _net.symNet();
    Routable& symRo = symNet.routable(_ro.symNetRoutableIdx());
    for (size_t i = 0; i < _vvRoutePaths.size(); ++i) {
      const auto& vRoutePath = _vvRoutePaths.at(i);
      const auto& vRoutePathVia = _vvRoutePathVias.at(i);
      const auto& vRoutePathViaOrient = _vvRoutePathViaOrients.at(i);
      const auto& vRoutePathWidthExts= _vvRoutePathWidthExts.at(i);
      for (size_t j = 0; j < vRoutePath.size(); ++j) {
        const auto& path = vRoutePath.at(j);
        const Point3d<Int>& u = path.first;
        const Point3d<Int>& v = path.second;
        Point3d<Int> symU(u), symV(v);
        Orient2dE ori = vRoutePathViaOrient.at(j);
        if (_net.isVerSym()) {
          symU.flipX(_net.symAxisX());
          symV.flipX(_net.symAxisX());
          if (symU.z() != symV.z()) {
            ori = orient2d::flipX(ori);
          }
        }
        else if (_net.isHorSym()) {
          symU.flipY(_net.symAxisY());
          symV.flipY(_net.symAxisY());
          if (symU.z() != symV.z()) {
            ori = orient2d::flipY(ori);
          }
        }
        //const Segment3d<Int> seg(symU, symV);
        //symRo.vArcIds().emplace_back(symNet.numArcs());
        //symNet.vArcs().emplace_back(symU, symV);
        //symNet.vpArcVias().emplace_back(vRoutePathVia.at(j));
        //symNet.vArcViaOrients().emplace_back(ori);
        //symNet.vArcWidthExts().emplace_back(vRoutePathWidthExts.at(j));
        const Segment3d<Int> arc(symU, symV);
        const TopoTree::SegData arcData(vRoutePathVia.at(j), vRoutePathViaOrient.at(j), vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second, _ro.idx());
        //symRo.arcs().emplace(arc);
        //symNet.mArcs().emplace(arc, Net::ArcData(vRoutePathVia.at(j), ori, vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second));
        symRo.addRoute(arc, arcData);
        symNet.addRoute(arc, arcData);
      }
    }
    for (const auto& vRouteWires: _vvRouteWires) {
      for (const auto& wire : vRouteWires) {
        Box<Int> b(wire.first);
        if (_net.isVerSym()) {
          b.flipX(_net.symAxisX());
        }
        else if (_net.isHorSym()) {
          b.flipY(_net.symAxisY());
        }
        //symRo.vWireIds().emplace_back(symNet.numWires());
        //symNet.vWires().emplace_back(b, wire.second);
        symRo.sWires().emplace(b, wire.second);
        symNet.sWires().emplace(b, wire.second);
      }
    }
    symRo.setRouted(true);
    bool isSymNetRouted = true;
    for (Int i = 0; i < symNet.numRoutables(); ++i) {
      isSymNetRouted &= symNet.routable(i).isRouted();
    }
    symNet.setRouted(isSymNetRouted);
  }
  else if (_isSelfSym) {
    for (size_t i = 0; i < _vvRoutePaths.size(); ++i) {
      const auto& vRoutePath = _vvRoutePaths.at(i);
      const auto& vRoutePathVia = _vvRoutePathVias.at(i);
      const auto& vRoutePathViaOrient = _vvRoutePathViaOrients.at(i);
      const auto& vRoutePathWidthExts = _vvRoutePathWidthExts.at(i);
      for (size_t j = 0; j < vRoutePath.size(); ++j) {
        const auto& path = vRoutePath.at(j);
        const Point3d<Int>& u = path.first;
        const Point3d<Int>& v = path.second;
        Point3d<Int> symU(u), symV(v);
        Orient2dE ori = vRoutePathViaOrient.at(j);
        if (_net.isVerSym()) {
          symU.flipX(_net.symAxisX());
          symV.flipX(_net.symAxisX());
          if (symU.z() != symV.z()) {
            ori = orient2d::flipX(ori);
          }
        }
        else if (_net.isHorSym()) {
          symU.flipY(_net.symAxisY());
          symV.flipY(_net.symAxisY());
          if (symU.z() != symV.z()) {
            ori = orient2d::flipY(ori);
          }
        }
        //const Segment3d<Int> seg(symU, symV);
        //_ro.vArcIds().emplace_back(_net.numArcs());
        //_net.vArcs().emplace_back(symU, symV);
        //_net.vpArcVias().emplace_back(vRoutePathVia.at(j));
        //_net.vArcViaOrients().emplace_back(ori);
        //_net.vArcWidthExts().emplace_back(vRoutePathWidthExts.at(j));
        const Segment3d<Int> arc(symU, symV);
        //_ro.arcs().emplace(arc);
        //_net.mArcs().emplace(arc, Net::ArcData(vRoutePathVia.at(j), ori, vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second));
        const TopoTree::SegData arcData(vRoutePathVia.at(j), ori, vRoutePathWidthExts.at(j).first, vRoutePathWidthExts.at(j).second, _ro.idx());
        _ro.addRoute(arc, arcData);
        //_net.addRoute(arc, arcData);
      }
    }
    for (const auto& vRouteWires : _vvRouteWires) {
      for (const auto& wire : vRouteWires) {
        Box<Int> b(wire.first);
        if (_net.isVerSym()) {
          b.flipX(_net.symAxisX());
        }
        else if (_net.isHorSym()) {
          b.flipY(_net.symAxisY());
        }
        //_ro.vWireIds().emplace_back(_net.numWires());
        //_net.vWires().emplace_back(b, wire.second);
        _ro.sWires().emplace(b, wire.second);
        _net.sWires().emplace(b, wire.second);
      }
    }
  }

  _ro.setRouted(true);
  bool isNetRouted = true;
  for (Int i = 0; i < _net.numRoutables(); ++i) {
    isNetRouted &= _net.routable(i).isRouted();
  }
  _net.setRouted(isNetRouted);

  // r-tree structure
  assert(_ro.numArcs() == _ro.topo().numSegs()); 
  assert(_net.numArcs() == _net.topo().numSegs()); 
  _ro.topo().groupComps();
  _net.topo().groupComps();

  if (_isSym) {
    Net& symNet = _net.symNet();
    Routable& symRo = symNet.routable(_ro.symNetRoutableIdx());
    symRo.topo().groupComps();
    symNet.topo().groupComps();
    
    assert(symRo.numArcs() == symRo.topo().numSegs());
    assert(symNet.numArcs() == symNet.topo().numSegs());
  }
  
}

void AstarCore::addAcsPts(const Int idx, const Box<Int>& box, const Int z)
{
  Vector<Point3d<Int>> vAcs;
  _ps.drMgr().computeBoxAcs(box, z, vAcs);
  _vCompAcsPts[idx].insert(vAcs.begin(), vAcs.end());
}


bool AstarCore::hasBend(const AstarNode* pU, const AstarNode* pV, const bool isReverse) const
{
  const AstarNode* parU = pU->pParent[isReverse];
  if (parU != nullptr) {
    return _ps.hasBend(parU->loc, pU->loc, pV->loc);
  }
  return false;
}


PROJECT_NAMESPACE_END
