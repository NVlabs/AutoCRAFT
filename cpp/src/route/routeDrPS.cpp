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

#include "routeDrPS.hpp"
#include "routeDrAstar.hpp"
#include "util/util.hpp"
#include "drc/drcMgr.hpp"
#include "geo/box2polygon.hpp"

PROJECT_NAMESPACE_START

bool DrPsRouter::solve(const bool useIO, const bool bRouteAll)
{

  Int i;
  Net* pNet;

  if (bRouteAll) {
    Cir_ForEachNet(_cir, pNet, i) {
      if (pNet->isSignal()) {
        pNet->setType(NetTypeE::critical);
      }
    }
  }

  // initialize routables
  Cir_ForEachNet(_cir, pNet, i) {
    if (pNet->isPower()) {
      assert(pNet->isRouted());
    }
    else if (pNet->isCritical()) {
      constructNetRoutables(*pNet, useIO);
    }
    else {
      //pNet->setRouted();
    }
    //spdlog::info("{}", util::enumUtil::val2Str(NetTypeEStr, pNet->type()));
  }
  //_cir.debug();
  //exit(1);

  // initialize history
  resetHistory();

  bool success = runNrr(useIO, NetTypeE::critical);

  if (!success) {
    checkFailed();
  }

  //std::cerr << history(Segment3d<Int>(2550, 6480, 4, 2550, 7600, 4), 2) << std::endl;;

  //Net& net = _cir.net("dmid815");
  //net.topo().showComps();
  //std::cerr << "Arcs: " << std::endl;
  //for (const auto& a : net.mArcs()) {
    //std::cerr << a.first << std::endl;
  //}
  //std::cerr << std::endl;
  //std::cerr << "Wires: " << std::endl;
  //for (const auto& a : net.sWires()) {
    //std::cerr << a.first << " " << a.second<< std::endl;
  //}
  //std::cerr << std::endl;
  //const Segment3d<Int> rip(10914, 4880, 4, 10914, 7440, 4);
  //ripupSegment(net, rip);
  //refineNet(net);
  //bool sss = runNrr(useIO, NetTypeE::critical);
  //if (!sss) {
    //checkFailed();
  //}
  //std::cerr << "Arcs: " << std::endl;
  //for (const auto& a : net.mArcs()) {
    //std::cerr << a.first << std::endl;
  //}
  //std::cerr << std::endl;
  //std::cerr << "Wires: " << std::endl;
  //for (const auto& a : net.sWires()) {
    //std::cerr << a.first << " " << a.second<< std::endl;
  //}
  //std::cerr << std::endl;


  String s;
  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isRouted()) {
      s += pNet->name() + " ";
    }
  }
  spdlog::warn("[DrPsRouter] Unrouted nets: {}", s);

  return success;
}

void DrPsRouter::constructNetRoutables(Net& net, const bool useIO)
{
  if (net.numRoutables() > 0) {
    assert(net.hasSymNet() and net.symNet().numRoutables() > 0);
    return;
  }

  const bool isSelfSym = net.isSelfSym();
  const bool isSym = net.hasSymNet();

  //net.vRoutables().clear();
  //net.vRoutableOrder().clear();

  if (isSelfSym) {
    constructSelfSymNetRoutables(net, useIO);
  }
  else if (isSym) {
    if (_drMgr.isCrossSym(net)) {
      constructCrossSymNetRoutables(net, useIO);
    }
    else {
      constructSymNetRoutables(net, useIO);
    }
  }
  else {
    constructNormalNetRoutables(net, useIO);
  }
}

void DrPsRouter::constructNormalNetRoutables(Net& net, const bool useIO)
{
  auto& vRoutables = net.vRoutables();
  auto& vRoutableOrder = net.vRoutableOrder();

  vRoutables.emplace_back(0, &net);
  Routable& ro = vRoutables.back();

  Int i;
  Pin* pPin;
  Net_ForEachPin(net, pPin, i) {
    if (pPin->isIO() and !useIO) {
      continue;
    }
    //std::cerr << pPin->name() << std::endl;
    ro.vpPins().emplace_back(pPin);
  }
  //ro.vpPins() = net.vpPins();
  //std::cerr << net.name() << std::endl;
  ro.init();

  vRoutableOrder.emplace_back(ro.idx());
}

void DrPsRouter::constructSelfSymNetRoutables(Net& net, const bool useIO)
{
  // init pin shapes
  Vector<Vector<Box<Int>>> vvBoxes;
  _drMgr.addPinShapes(net, vvBoxes);

  // construct routables
  auto& vRoutables = net.vRoutables();
  auto& vRoutableOrder = net.vRoutableOrder();
  Routable roSelfSym(0, &net, nullptr, -1, true);
  Routable roRest(0, &net, nullptr, -1, false);

  if (net.isVerSym()) {
    for (const Pin* pPin : net.vpPins()) {
      if (pPin->isIO() and !useIO) {
        continue;
      }
      if (_drMgr.hasSymPinX(*pPin, net.symAxisX(), vvBoxes)) {
        roSelfSym.addPinPtr(const_cast<Pin*>(pPin));
      }
      else {
        roRest.addPinPtr(const_cast<Pin*>(pPin));
      }
    }
  }
  else if (net.isHorSym()) {
    for (const Pin* pPin : net.vpPins()) {
      if (pPin->isIO() and !useIO) {
        continue;
      }
      if (_drMgr.hasSymPinY(*pPin, net.symAxisY(), vvBoxes)) {
        roSelfSym.addPinPtr(const_cast<Pin*>(pPin));
      }
      else {
        roRest.addPinPtr(const_cast<Pin*>(pPin));
      }
    }
  }


  if (roSelfSym.numPins() > 0) {
    roSelfSym.setIdx(vRoutables.size());
    roSelfSym.init();
    vRoutables.emplace_back(roSelfSym);
    vRoutableOrder.emplace_back(roSelfSym.idx());
  }
  if (roRest.numPins() > 0) {
    roRest.setIdx(vRoutables.size());
    if (roRest.idx() > 0) {
      assert(roSelfSym.numPins() > 0);
      roRest.addRoutableIdx(roSelfSym.idx());
    }
    roRest.init();
    vRoutables.emplace_back(roRest);
    vRoutableOrder.emplace_back(roRest.idx());
  }
}

void DrPsRouter::constructSymNetRoutables(Net& net, const bool useIO)
{
  assert(net.hasSymNet());
  Net& symNet = net.symNet();
  //std::cerr << net.name() << " " << net.numRoutables() << " " << symNet.name() << " " << symNet.numRoutables() << std::endl;

  // init pin shapes for both nets
  Vector<Vector<Box<Int>>> vvBoxes1, vvBoxes2;
  _drMgr.addPinShapes(net, vvBoxes1);
  _drMgr.addPinShapes(symNet, vvBoxes2);

  // construct routables
  Routable roSym1, roRest1;
  Routable roSym2, roRest2;

  auto buildRoutables = [&] (Routable& roSym, Routable& roRest, Net& net, Net& symNet, const auto& vvSymBoxes) {
    roSym.setNetPtr(&net);
    roSym.setSymNetPtr(&symNet);
    roRest.setNetPtr(&net);
    if (net.isVerSym()) {
      for (const Pin* pPin : net.vpPins()) {
        if (pPin->isIO() and !useIO) {
          continue;
        }
        if (_drMgr.hasSymPinX(*pPin, net.symAxisX(), vvSymBoxes)) {
          roSym.addPinPtr(const_cast<Pin*>(pPin));
        }
        else {
          roRest.addPinPtr(const_cast<Pin*>(pPin));
        }
      }
    }
    else if (net.isHorSym()) {
      for (const Pin* pPin : net.vpPins()) {
        if (pPin->isIO() and !useIO) {
          continue;
        }
        if (_drMgr.hasSymPinY(*pPin, net.symAxisY(), vvSymBoxes)) {
          roSym.addPinPtr(const_cast<Pin*>(pPin));
        }
        else {
          roRest.addPinPtr(const_cast<Pin*>(pPin));
        }
      }
    }
  };
  buildRoutables(roSym1, roRest1, net, symNet, vvBoxes2);
  buildRoutables(roSym2, roRest2, symNet, net, vvBoxes1);


  assert(roSym1.numPins() == roSym2.numPins());
  assert(roSym1.numRoutables() == roSym2.numRoutables());
  assert(roSym1.numRoutables() == 0);

  // add routables to nets
  auto& vRoutables1 = net.vRoutables();
  auto& vRoutables2 = symNet.vRoutables();
  auto& vRoutableOrder1 = net.vRoutableOrder();
  auto& vRoutableOrder2 = symNet.vRoutableOrder();

  if (roSym1.numPins() > 0) {
    roSym1.setIdx(vRoutables1.size());
    roSym2.setIdx(vRoutables2.size());
    roSym1.setSymNetRoutableIdx(roSym2.idx());
    roSym2.setSymNetRoutableIdx(roSym1.idx());
    roSym1.init();
    roSym2.init();
    vRoutables1.emplace_back(roSym1);
    vRoutables2.emplace_back(roSym2);
    vRoutableOrder1.emplace_back(roSym1.idx());
    vRoutableOrder2.emplace_back(roSym2.idx());
  }


  roRest1.setIdx(vRoutables1.size());
  if (roRest1.idx() != 0) {
    assert(roSym1.numPins() > 0);
    roRest1.addRoutableIdx(roSym1.idx());
  }
  //std::cerr << "rosym1" << std::endl;
  //for (const auto& pPin : roSym1.vpPins()) {
    //std::cerr << pPin->name() << std::endl;
  //}
  //for (const auto& roIdx : roSym1.vRoutableIds()) {
    //std::cerr << roIdx << std::endl;
  //}
  //std::cerr << std::endl;
  //std::cerr << "rosym2" << std::endl;
  //for (const auto& pPin : roSym2.vpPins()) {
    //std::cerr << pPin->name() << std::endl;
  //}
  //for (const auto& roIdx : roSym2.vRoutableIds()) {
    //std::cerr << roIdx << std::endl;
  //}
  //std::cerr << std::endl;
  //std::cerr << "rorest1" << std::endl;
  //for (const auto& pPin : roRest1.vpPins()) {
    //std::cerr << pPin->name() << std::endl;
  //}
  //for (const auto& roIdx : roRest1.vRoutableIds()) {
    //std::cerr << roIdx << std::endl;
  //}
  //std::cerr << std::endl;
  //std::cerr << "rorest2" << std::endl;
  //for (const auto& pPin : roRest2.vpPins()) {
    //std::cerr << pPin->name() << std::endl;
  //}
  //for (const auto& roIdx : roRest2.vRoutableIds()) {
    //std::cerr << roIdx << std::endl;
  //}
  //std::cerr << std::endl;
  if (roRest1.numPins() > 0 or roRest1.numRoutables() > 1) {
    roRest1.init();
    vRoutables1.emplace_back(roRest1);
    vRoutableOrder1.emplace_back(roRest1.idx());
  }

  roRest2.setIdx(vRoutables2.size());
  if (roRest2.idx() != 0) {
    assert(roSym2.numPins() > 0);
    roRest2.addRoutableIdx(roSym2.idx());
  }
  if (roRest2.numPins() > 0 or roRest2.numRoutables() > 1) {
    roRest2.init();
    vRoutables2.emplace_back(roRest2);
    vRoutableOrder2.emplace_back(roRest2.idx());
  }

}

void DrPsRouter::constructCrossSymNetRoutables(Net& net, const bool useIO)
{
  Net& symNet = net.symNet();
  if (net.numRoutables() > 0) {
    assert(symNet.numRoutables() > 0);
    return;
  }
  // init pin shapes for both nets
  Vector<Vector<Box<Int>>> vvBoxes1, vvBoxes2;
  _drMgr.addPinShapes(net, vvBoxes1);
  _drMgr.addPinShapes(symNet, vvBoxes2);

  // construct routables
  Array<Routable, 2> roSym1, roSym2; // 0: partA, 1: partB
  Routable roRest1, roRest2;

  auto buildRoutables = [&] (auto& roSym, Routable& roRest, Net& net, Net& symNet, const auto& vvSymBoxes) {
    for (auto& ro : roSym) {
      ro.setNetPtr(&net);
      ro.setSymNetPtr(&symNet);
      roRest.setNetPtr(&net);
    }
    if (net.isVerSym()) {
      for (const Pin* pPin : net.vpPins()) {
        if (pPin->isIO() and !useIO) {
          continue;
        }
        if (_drMgr.hasSymPinX(*pPin, net.symAxisX(), vvSymBoxes)) {
          if (_drMgr.isPartAPin(*pPin)) {
            roSym[0].addPinPtr(const_cast<Pin*>(pPin));
          }
          else if (_drMgr.isPartBPin(*pPin)) {
            roSym[1].addPinPtr(const_cast<Pin*>(pPin));
          }
          else {
            roRest.addPinPtr(const_cast<Pin*>(pPin));
          }
        }
        else {
          roRest.addPinPtr(const_cast<Pin*>(pPin));
        }
      }
    }
    else if (net.isHorSym()) {
      for (const Pin* pPin : net.vpPins()) {
        if (pPin->isIO() and !useIO) {
          continue;
        }
        if (_drMgr.hasSymPinY(*pPin, net.symAxisY(), vvSymBoxes)) {
          if (_drMgr.isPartAPin(*pPin)) {
            roSym[0].addPinPtr(const_cast<Pin*>(pPin));
          }
          else if (_drMgr.isPartBPin(*pPin)) {
            roSym[1].addPinPtr(const_cast<Pin*>(pPin));
          }
          else {
            roRest.addPinPtr(const_cast<Pin*>(pPin));
          }
        }
        else {
          roRest.addPinPtr(const_cast<Pin*>(pPin));
        }
      }

    }
  };
  buildRoutables(roSym1, roRest1, net, symNet, vvBoxes2);
  buildRoutables(roSym2, roRest2, symNet, net, vvBoxes1);
  assert(roSym1[0].numPins() == roSym2[1].numPins());
  assert(roSym1[1].numPins() == roSym2[0].numPins());
  assert(roSym1[0].numRoutables() == 0);
  assert(roSym1[1].numRoutables() == 0);
  assert(roSym2[0].numRoutables() == 0);
  assert(roSym2[1].numRoutables() == 0);

  // add routables to nets
  auto& vRoutables1 = net.vRoutables();
  auto& vRoutables2 = symNet.vRoutables();
  auto& vRoutableOrder1 = net.vRoutableOrder();
  auto& vRoutableOrder2 = symNet.vRoutableOrder();

  for (const Int i : {0, 1}) {
    Routable& ro1 = roSym1[i];
    Routable& ro2 = roSym2[i ^ 1];
    if (ro1.numPins() > 0) {
      ro1.setIdx(vRoutables1.size());
      ro2.setIdx(vRoutables2.size());
      ro1.setSymNetRoutableIdx(ro2.idx());
      ro2.setSymNetRoutableIdx(ro1.idx());
      ro1.init();
      ro2.init();
      vRoutables1.emplace_back(ro1);
      vRoutables2.emplace_back(ro2);
      vRoutableOrder1.emplace_back(ro1.idx());
      vRoutableOrder2.emplace_back(ro2.idx());
    }
  }
  // roRest must exist to connect the two routables
  auto addRest = [&] (auto& roSym, Routable& roRest, auto& vRoutables, auto& vRoutableOrder) {
    roRest.setIdx(vRoutables.size());
    if (roRest.idx() != 0) {
      for (const Int i : {0, 1}) {
        if (roSym[i].numPins() > 0) {
          roRest.addRoutableIdx(roSym[i].idx());
        }
      }
    }
    if (roRest.numPins() > 0 or roRest.numRoutables() > 1) {
      roRest.init();
      vRoutables.emplace_back(roRest);
      vRoutableOrder.emplace_back(roRest.idx());
    }
  };
  addRest(roSym1, roRest1, vRoutables1, vRoutableOrder1);
  addRest(roSym2, roRest2, vRoutables2, vRoutableOrder2);

}

void DrPsRouter::addUnroutedNet(Net& net)
{
  if (!net.isRouted()){
    _pqc.push(&net);
  }
}

void DrPsRouter::addUnroutedNets(const bool useIO, const NetTypeE netType)
{
  Int i;
  Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isRouted()) {
      if (pNet->type() == netType) {
        if (pNet->numPins() <= 1) {
          pNet->setRouted();
        }
        else if (pNet->isIO() and pNet->numPins() <= 2 and !useIO) {
          pNet->setRouted();
        }
        else {
          //if (
          //pNet->name() == "dmid0" or 
          //pNet->name() == "dmid1" or
          //pNet->name() == "dmid2" or
          //pNet->name() == "dmid3" or
          //pNet->name() == "dmid4" or
          //pNet->name() == "dmid5" or
          //pNet->name() == "dmid6" or
          //pNet->name() == "dmid7" or
          //pNet->name() == "dmid01" or
          //pNet->name() == "dmid23" or
          //pNet->name() == "dmid45" or
          //pNet->name() == "dmid67" or
          //pNet->name() == "dmid07" or
          //pNet->name() == "dmid815" or
          //pNet->name() == "sb<0>" or
          //pNet->name() == "sb<1>" or
          //pNet->name() == "sb<2>" or
          //pNet->name() == "sb<3>" or
          //pNet->name() == "sb<4>" or
          //pNet->name() == "sd<0>" 
          //pNet->name() == "sd<1>" or
          //pNet->name() == "sd<2>" or
          //pNet->name() == "sd<3>" or
          //pNet->name() == "odpre" or
          //pNet->name() == "net0105" 
          //)
          //if (pNet->name() == "dmid815") 
          //if (pNet->name() == "odpre") 
            _pqc.push(pNet);
        }
      }
    }
  }
}

bool DrPsRouter::runNrr(const Vector<Int>& vNetIds, const bool useIO)
{
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 4 * _hisCost)) return true;
  resetHistory();                                        
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 2 * _drcCost, 2 * _hisCost)) return true;
  resetHistory();
  if (runNrrCore(vNetIds, useIO, false, 1 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(vNetIds, useIO, false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  return false;
}

bool DrPsRouter::runNrr(const bool useIO, const NetTypeE netType)
{
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 4 * _hisCost)) return true;
  resetHistory();                                                 
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 4 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 2 * _drcCost, 2 * _hisCost)) return true;
  resetHistory();                                                 
  if (runNrrCore(netType, useIO,  false, 1 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 2 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 4 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 1 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  if (runNrrCore(netType, useIO,  false, 8 * _drcCost, 0.5 * _hisCost)) return true;
  return false;
}

bool DrPsRouter::hasUnRoutedNet()
{
  return _pqc.size() > 0;
}

Int DrPsRouter::nextUnRoutedNetIdx()
{
  const Int idx = _pqc.top()->idx();
  _pqc.pop();
  return idx;
}

bool DrPsRouter::runNrrCore(const NetTypeE netType, const bool useIO,
                            const bool useStrictDRC, const Int drcCost, const Int historyCost)
{
  addUnroutedNets(useIO, netType);

  while (!_pqc.empty()) {
    //Net* pNet = _pqc.top();
    //Net& net = *pNet; 
    //_pqc.pop();
    const Int netIdx = nextUnRoutedNetIdx();
    Net& net = _cir.net(netIdx);
    //std::cerr << net.name() << " " << net.numRoutables() << std::endl;

    bool success = route(net, useStrictDRC, drcCost, historyCost);
    //spdlog::info("Success {}", success);
    if (!success) {
      assert(useStrictDRC);
      //pNet->addFail();
      //if (pNet->hasSymNet()) {
      //pNet->symNet().addFail();
      //}
      route(net, false, drcCost, historyCost);
    }
    else {
      //if (!checkNetDRC(*pNet)) {
      //pNet->addFail();
      //if (pNet->hasSymNet()) {
      //pNet->symNet().addFail();
      //}
      //}
    }

  }

  // check DRC
  if (checkDRC(useIO, netType)) {
    //spdlog::info("[DrPsRouter] Success");
    return true;
  }
  return false;
}
  
bool DrPsRouter::runNrrCore(const Vector<Int>& vNetIds, const bool useIO, const bool useStrictDRC, const Int drcCost, const Int historyCost)
{
  for (const Int netIdx : vNetIds) {
    Net& net = _cir.net(netIdx);
    addUnroutedNet(net);
  }
  while (!_pqc.empty()) {
    const Int netIdx = nextUnRoutedNetIdx();
    Net& net = _cir.net(netIdx);

    bool success = route(net, useStrictDRC, drcCost, historyCost);
    if (!success) {
      assert(useStrictDRC);
      route(net, false, drcCost, historyCost);
    }
    else {
    }
  }
  if (checkDRC(vNetIds, useIO)) {
    return true;
  }
  return false;
}

bool DrPsRouter::route(Net& net, const bool useStrictDRC, const Int drcCost, const Int historyCost) 
{
  Int i;
  Routable* pRoutable;
  //const Pin* pPin;
  //Int layerIdx, j;
  //const Box<Int>* pBox;
  //Net_ForEachPin(net, pPin, i) {
  //Pin_ForEachMetalLayerIdx((*pPin), layerIdx){
  //Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
  //std::cerr << *pBox<< std::endl;
  //}
  //}
  //}

  //std::cerr << "sym: " << net.symAxisX() << std::endl;
  //Net_ForEachPin(net.symNet(), pPin, i) {
  //Pin_ForEachMetalLayerIdx((*pPin), layerIdx){
  //Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
  //std::cerr << *pBox<< std::endl;
  //}
  //}
  //}
  //exit(0);
  if (drcCost > 0) {
    _drcCost = drcCost;
  }
  if (historyCost > 0) {
    _hisCost = historyCost;
  }
  Net_ForEachRoutable(net, pRoutable, i) {
    Routable& ro = *pRoutable;
    AstarCore astar(_cir, net, ro, *this, ro.hasSymRoutable(), ro.isSelfSym(), useStrictDRC);
    if (!astar.solve()) {
      return false;
    }
  }
  //Box<Int> wireBbox = net.bbox();
  //Int minWireLayerIdx = net.minPinLayerIdx(), maxWireLayerIdx = net.maxPinLayerIdx();
  //for (const auto& wire : net.sWires()) {
    //const auto& box = wire.first;
    //const Int layerIdx = wire.second;
    //wireBbox.set(std::min(wireBbox.xl(), box.xl()),
                 //std::min(wireBbox.yl(), box.yl()),
                 //std::max(wireBbox.xh(), box.xh()),
                 //std::max(wireBbox.yh(), box.yh()));
    //minWireLayerIdx = std::min(minWireLayerIdx, layerIdx);
    //maxWireLayerIdx = std::max(maxWireLayerIdx, layerIdx);
  //}
  //net.setWireBbox(wireBbox);
  //net.setMinWireLayerIdx(minWireLayerIdx);
  //net.setMaxWireLayerIdx(maxWireLayerIdx);
  net.updateWireBbox();
  return true;
}

bool DrPsRouter::checkDRC(const Vector<Int>& vNetIds, const bool useIO) const
{
  Vector<Net*> vpNets;

  for (const Int netIdx : vNetIds) {
    Net* pNet = _cir.pNet(netIdx);
    if (pNet->numPins() <= 1) {
      assert(pNet->isRouted());
    }
    else if (pNet->isIO() and pNet->numPins() <= 2 and !useIO) {
      assert(pNet->isRouted());
    }
    else {
      vpNets.emplace_back(pNet);
    }
  }
  std::sort(vpNets.begin(), vpNets.end(), RipupNetCmp());
  //std::random_shuffle(vpNets.begin(), vpNets.end());

  bool valid = true;
  Int vioCnt = 0;
  for (Net* pNet : vpNets) {
    assert(pNet->numPins() > 1);
    if (!checkNetDRC(*pNet)) {
      ++vioCnt;
      ripup(*pNet);
      valid = false;
      pNet->addFail();

      if (pNet->hasSymNet()) {
        ripup(pNet->symNet());
        pNet->symNet().addFail();
      }
    }
  }
  if (vioCnt) {
    spdlog::warn("[DrPsRouter] Violate Nets: {}", vioCnt);
  }
  return valid;
}

bool DrPsRouter::checkDRC(const bool useIO, const NetTypeE netType) const
{
  Vector<Net*> vpNets;

  Int i;
  Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (pNet->type() == netType) {
      if (pNet->numPins() <= 1) {
        assert(pNet->isRouted());
      }
      else if (pNet->isIO() and pNet->numPins() <= 2 and !useIO) {
        assert(pNet->isRouted());
      }
      else {
        vpNets.emplace_back(pNet);
      }
    }
  }
  std::sort(vpNets.begin(), vpNets.end(), RipupNetCmp());
  //std::random_shuffle(vpNets.begin(), vpNets.end());

  bool valid = true;
  Int vioCnt = 0;
  for (Net* pNet : vpNets) {
    assert(pNet->numPins() > 1);
    if (!checkNetDRC(*pNet)) {
      ++vioCnt;
      ripup(*pNet);
      valid = false;
      pNet->addFail();

      if (pNet->hasSymNet()) {
        ripup(pNet->symNet());
        pNet->symNet().addFail();
      }
    }
  }
  if (vioCnt) {
    spdlog::warn("[DrPsRouter] Violate Nets: {}", vioCnt);
  }

  // decay history

  return valid;
}

bool DrPsRouter::checkNetDRC(const Net& net) const
{
  DrcMgr drc(_cir);

  switch (drc.checkNet(net)) {
    case DrvE::spc_prl:
      spdlog::warn("[DrPsRouter] Violate DRC: {} SPC PRL", net.name());
      return false;
    case DrvE::spc_eol:
      spdlog::warn("[DrPsRouter] Violate DRC: {} SPC EOL", net.name());
      return false;
    case DrvE::spc_cut:
      spdlog::warn("[DrPsRouter] Violate DRC: {} SPC CUT", net.name());
      return false;
    case DrvE::min_area:
      spdlog::warn("[DrPsRouter] Violate DRC: {} MIN AREA", net.name());
      return false;
    case DrvE::min_step:
      spdlog::warn("[DrPsRouter] Violate DRC: {} MIN STEP", net.name());
      return false;
    case DrvE::valid_width:
      spdlog::warn("[DrPsRouter] Violate DRC: {} VALID WIDTH", net.name());
      return false;
    case DrvE::valid_length:
      spdlog::warn("[DrPsRouter] Violate DRC: {} VALID LENGTH", net.name());
      return false;
    case DrvE::undef: // clean
      return true;
  }
  return true;
}

void DrPsRouter::ripup(Net& net) const
{
  //TopoTree topo = net.topo();
  //Vector<Pair<Box<Int>, Int>> v1(net.sWires().begin(), net.sWires().end());
  //std::sort(v1.begin(), v1.end());
  //Vector<SpatialMap<Int, Net*>> sp(_cir.numLayers());
  //for (Int i = 0; i < _cir.numLayers(); ++i) {
    //sp[i] = _cir.spatialNets(i);
  //}

  for (const auto& wire : net.sWires()) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    const bool bExist = _cir.spatialNets(layerIdx).deepErase(box, &net);
    assert(bExist);
  }
  net.clearRouting();

  //bool f = true;
  //Vector<Pair<Box<Int>, Int>> v2;
  //for (Int i = 0; i < _cir.numLayers(); ++i) {
    //for (const auto& e : _cir.spatialNets(i)) {
      //if (e.second == &net) {
        //v2.emplace_back(e.first, i);
        ////std::cerr << e.first << " " << i << std::endl;
        //f = false;
      //}
      ////assert(e.second != &net);
    //}
  //}
  //if (!f) {
    //std::cerr << "sp" << std::endl;
    //for (Int i = 0; i < _cir.numLayers(); ++i) {
      //for (const auto& e : sp.at(i)) {
        //if (e.second == &net) {
          //std::cerr << e.first << " " << i << std::endl;
        //}
      //}
    //}
    //std::sort(v2.begin(), v2.end());
    //std::cerr << "before" << std::endl;
    //for (const auto& w : v1) {
      //std::cerr << w.first << " " << w.second << std::endl;
    //}
    //topo.showComps();
    //std::cerr << "after" << std::endl;
    //for (const auto& w : v2) {
      //std::cerr << w.first << " " << w.second << std::endl;
    //}
    //net.topo().showComps();
    //assert(false);
  //}

}

void DrPsRouter::ripupSegment(Net& net, const Segment3d<Int>& seg) const
{
  //std::cerr << "ripup: " << seg << std::endl;
  const bool b = net.removeRoute(seg);
  assert(b);
  
  // group comps
  resetNetStatus(net);
  
  // rebuild wires
  resetNetWires(net);
}

void DrPsRouter::ripupPartial(Net& net, const Vector<Segment3d<Int>>& vSegs) const
{
  for (const auto& seg : vSegs) {
    const bool b = net.removeRoute(seg);
    assert(b);
  } 
  
  // group comps
  resetNetStatus(net);

  // rebuild wires
  resetNetWires(net);
  
}

void DrPsRouter::ripupSegmentNRefine(Net& net, const Segment3d<Int>& seg) const
{
  const bool b = net.removeRoute(seg);
  assert(b);
  //net.topo().groupComps();
  //for (auto& ro : net.vRoutables()) {
    //ro.topo().groupComps();
  //}
  net.refineDanglingRoutes();

  // group comps
  resetNetStatus(net);

  // rebuild wires
  resetNetWires(net);

}

void DrPsRouter::ripupPartialNRefine(Net& net, const Vector<Segment3d<Int>>& vSegs) const
{
  for (const auto& seg : vSegs) {
    const bool b = net.removeRoute(seg);
    assert(b);
  }
  //net.topo().groupComps();
  //for (auto& ro : net.vRoutables()) {
    //ro.topo().groupComps();
  //}
  net.refineDanglingRoutes();
  
  // group comps
  resetNetStatus(net);

  // rebuild wires
  resetNetWires(net);

}

void DrPsRouter::ripupSegmentNFloating(Net& net, const Segment3d<Int>& seg) const
{
  const bool b = net.removeRoute(seg);
  assert(b);
  net.removeFloatingRoutes();

  // group comps
  resetNetStatus(net);

  // rebuild wires
  resetNetWires(net);

}

void DrPsRouter::ripupPartialNFloating(Net& net, const Vector<Segment3d<Int>>& vSegs) const
{
  for (const auto& seg : vSegs) {
    const bool b = net.removeRoute(seg);
    assert(b);
  }
  net.removeFloatingRoutes();

  // group comps
  resetNetStatus(net);

  // rebuild wires
  resetNetWires(net);

}

void DrPsRouter::ripupFloating(Net& net) const
{
  net.removeFloatingRoutes();
  // group comps
  resetNetStatus(net);
  
  // rebuild wires
  resetNetWires(net);
}

void DrPsRouter::refineNet(Net& net) const
{
  net.refineDanglingRoutes();
  //Vector<Segment3d<Int>> vSegs;
  //net.refineDanglingRoutes(vSegs);


  // group comps
  resetNetStatus(net);
  
  // rebuild wires
  resetNetWires(net);
  
}

void DrPsRouter::resetNetStatus(Net& net) const
{
  net.topo().groupComps();
  net.setRouted(net.topo().isConnected());
  //if (!net.topo().isConnected()) {
    //net.setRouted(false);
  //}
  for (Routable& ro : net.vRoutables()) {
    ro.topo().groupComps();
    ro.setRouted(ro.topo().isConnected());
    //if (!ro.topo().isConnected()) {
      //ro.setRouted(false);
    //}
  }
}

void DrPsRouter::resetNetWires(Net& net) const 
{
  // clear all wires
  for (const auto& wire : net.sWires()) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    const bool bExist = _cir.spatialNets(layerIdx).deepErase(box, &net);
    assert(bExist);
  }
  net.sWires().clear();
  for (auto& ro : net.vRoutables()) {
    ro.sWires().clear();
  }
  // seg to wires
  auto topo2Wires = [&] (const TopoTree& topo, Vector<Pair<Box<Int>, Int>>& vWires) {
    Vector<Segment3d<Int>> vSegs;
    Vector<TopoTree::SegData> vSds;
    topo.vRoutes(vSegs, vSds);

    Vector<Pair<Box<Int>, Int>> v;
    segs2Wires(vSegs, vSds, v);
    mergeWires(v, vWires);
  };

  Vector<Pair<Box<Int>, Int>> vWires;
  topo2Wires(net.topo(), vWires);
  net.sWires().insert(vWires.begin(), vWires.end());

  for (auto& ro : net.vRoutables()) {
    vWires.clear();
    topo2Wires(ro.topo(), vWires);
    ro.sWires().insert(vWires.begin(), vWires.end());
  }

  // add back to spatial
  for (const auto& wire : net.sWires()) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    _cir.spatialNets(layerIdx).insert(box, &net);
  }

  // update wire bbox
  net.updateWireBbox();
}


void DrPsRouter::segs2Wires(const Vector<Segment3d<Int>>& vSegs, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  for (const auto& seg : vSegs) {
    seg2Wires(seg, vWires);
  }
}

void DrPsRouter::segs2Wires(const Vector<Segment3d<Int>>& vSegs, const Vector<TopoTree::SegData>& vSds, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  assert(vSegs.size() == vSds.size());
  for (size_t i = 0; i < vSegs.size(); ++i) {
    const auto& seg = vSegs.at(i);
    const auto& sd = vSds.at(i);
    seg2Wires(seg, sd, vWires);
  }
  //for (const auto& w : vWires) {
    //std::cerr << w.first << " " << w.second << std::endl;
  //}
}

void DrPsRouter::seg2Wires(const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  assert(seg.b90());
  const auto& u = seg.p0();
  const auto& v = seg.p1();
  if (seg.bVia()) {
    const Layer& lowerLayer = _cir.layer(seg.zl());
    assert(lowerLayer.isMetal());
    const RouteGrid& rg = _cir.routeGrid(lowerLayer.selfIdx());

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
        vWires.emplace_back(b, layerIdx);
      }
    }
  }
  else {
    const Int width = wireWidth(u);
    assert(wireWidth(v) == width);
    const Int ext = width / 2;
    const Box<Int>& b = toWire(u, v, width, ext);

    vWires.emplace_back(b, u.z());
  }

}

void DrPsRouter::seg2Wires(const Segment3d<Int>& seg, const TopoTree::SegData& sd, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  assert(seg.b90());
  const auto& u = seg.p0();
  const auto& v = seg.p1();
  if (seg.bVia()) {
    assert(sd.pVia != nullptr);
    assert(sd.orient != Orient2dE::undef);
    assert(sd.width == 0);
    assert(sd.ext == 0);

    Int i, layerIdx;
    const Box<Int>* pBox;
    Via_ForEachLayerIdx((*sd.pVia), layerIdx) {
      Via_ForEachLayerBox((*sd.pVia), layerIdx, pBox, i) {
        Box<Int> b(pBox->xl() + u.x(),
                   pBox->yl() + u.y(),
                   pBox->xh() + u.x(),
                   pBox->yh() + u.y());
        switch (sd.orient) {
          case Orient2dE::n: break;
          case Orient2dE::w: b.rotate90(b.xl(), b.yl(), false); b.shiftX(b.width()); break;
          case Orient2dE::s: b.rotate180(b.xl(), b.yl()); b.shift(b.width(), b.height()); break;
          case Orient2dE::e: b.rotate90(b.xl(), b.yl(), true); b.shiftY(b.height()); break;
          case Orient2dE::fn: b.flipX(b.centerX()); break;
          case Orient2dE::fw: b.rotate90(b.xl(), b.yl(), false); b.shiftX(b.width()); b.flipX(b.centerX()); break;
          case Orient2dE::fs: b.flipY(b.centerY()); break;
          case Orient2dE::fe: b.rotate90(b.xl(), b.yl(), true); b.shiftY(b.height()); b.flipY(b.centerY()); break;
          case Orient2dE::undef:
          default: assert(false);
        }
        vWires.emplace_back(b, layerIdx);
      }
    }
  }
  else {
    assert(sd.pVia == nullptr);
    assert(sd.orient == Orient2dE::undef);
    assert(sd.width > 0);
    assert(sd.ext > 0);
    const Box<Int>& b = toWire(u, v, sd.width, sd.ext);
    //std::cerr << u << v << " " << b << " " << u.z() << std::endl;
    
    vWires.emplace_back(b, u.z());
  }

}

void DrPsRouter::mergeWires(const Vector<Pair<Box<Int>, Int>>& vWires, Vector<Pair<Box<Int>, Int>>& vMergedWires) const
{
  Vector<Vector<Box<Int>>> vvBoxes(_cir.numLayers());
  Vector<Vector<Polygon<Int>>> vvPolygons(_cir.numLayers());
  DrcMgr drc(_cir);

  for (const auto& wire : vWires) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    vvBoxes[layerIdx].emplace_back(box);
  }

  for (Int i = 0; i < _cir.numLayers(); ++i) {
    const auto& vBoxes = vvBoxes.at(i);
    auto& vPolygons = vvPolygons.at(i);
    const Layer& layer = _cir.layer(i);
    if (vBoxes.size() > 0) {
      if (layer.isMetal()) {
        geo::box2Polygon<Int>(vBoxes, vPolygons);
        for (const auto& polygon : vPolygons) {
          const Ring<Int>& ring = polygon.outer();
          
          assert(ring.size() == 4);
          Box<Int> b(ring[1].x(), ring[1].y(), ring[3].x(), ring[3].y());
          
          drc.fitWire2ValidLength(i, b);
          //std::cerr << (Real)(b.xh() - b.xl()) / 2000 << " "
                    //<< (Real)(b.yh() - b.yl()) / 2000 << " "
                    //<< (Real)(b.area()) / 2000 / 2000 << " "
                    //<< i << std::endl;
          vMergedWires.emplace_back(b, i);
        }
      }
      else { // cut layers
        for (const auto& b : vBoxes) {
          vMergedWires.emplace_back(b, i);
        }
      }
    }
  }

}

void DrPsRouter::queryNetSegWires(const Net& net, const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  assert(seg.b90());
  const auto& u = seg.p0();
  const auto& v = seg.p1();

  const Point<Int> fu(u.x(), u.y());
  const Point<Int> fv(v.x(), v.y());

  if (u.z() == v.z()) {
    Vector<Pair<Box<Int>, Net*>> vTouchedNets;
    _cir.spatialNets(u.z()).queryBoth(fu, fv, vTouchedNets);

    for (const auto& p : vTouchedNets) {
      if (p.second == &net) {
        vWires.emplace_back(p.first, u.z());
      }
    }
  }
  else {
    const Int zl = seg.zl();
    const Int zh = seg.zh();
    assert(zl + 2 == zh);
    for (Int z = zl; z <= zh; ++z) {
      Vector<Pair<Box<Int>, Net*>> vTouchedNets;
      _cir.spatialNets(z).queryBoth(fu, fv, vTouchedNets);
      for (const auto& p : vTouchedNets) {
        if (p.second == &net) {
          vWires.emplace_back(p.first, z);
        }
      }
    }
  }
}

void DrPsRouter::queryTopoWires(const Net& net, const TopoTree& topo, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  FlatHashSet<Pair<Box<Int>, Int>> sWires;
  queryTopoWires(net, topo, sWires);
  std::copy(sWires.begin(), sWires.end(), std::back_inserter(vWires));
}

void DrPsRouter::queryTopoWires(const Net& net, const TopoTree& topo, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const
{
  for (Int i = 0; i < topo.numComps(); ++i) {
    queryTopoCompWires(net, topo, i, sWires);
  }
}

void DrPsRouter::queryTopoCompWires(const Net& net, const TopoTree& topo, const Int compIdx, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  FlatHashSet<Pair<Box<Int>, Int>> sWires;
  queryTopoCompWires(net, topo, compIdx, sWires);
  std::copy(sWires.begin(), sWires.end(), std::back_inserter(vWires));
}

void DrPsRouter::queryTopoCompWires(const Net& net, const TopoTree& topo, const Int compIdx, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const
{
  assert(0 <= compIdx and compIdx < topo.numComps());
  Vector<Segment3d<Int>> vSegs;
  topo.vCompSegs(compIdx, vSegs);

  queryTopoSegsWires(net, vSegs, sWires);

  if (vSegs.empty()) {
    queryTopoPtsWires(net, topo.compPts(compIdx), sWires);
  }

}

void DrPsRouter::queryTopoSegsWires(const Net& net, const Vector<Segment3d<Int>>& vSegs, Vector<Pair<Box<Int>, Int>>& vWires) const
{
  FlatHashSet<Pair<Box<Int>, Int>> sWires;
  queryTopoSegsWires(net, vSegs, sWires);
  std::copy(sWires.begin(), sWires.end(), std::back_inserter(vWires));
}

void DrPsRouter::queryTopoSegsWires(const Net& net, const Vector<Segment3d<Int>>& vSegs, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const
{
  for (const auto& seg : vSegs) {
    const auto& u = seg.p0();
    const auto& v = seg.p1();
    assert(u.x() <= v.x() and u.y() <= v.y() and u.z() <= v.z());
    if (u.z() == v.z()) {
      const Int layerIdx = u.z();
      const Point<Int> fu(u.x(), u.y());
      const Point<Int> fv(v.x(), v.y());
      
      Vector<Pair<Box<Int>, Net*>> vTouchedNets;
      //Vector<Pair<Box<Int>, Obs*>> vTouchedObs;
      Vector<Pair<Box<Int>, Pin*>> vTouchedPins;
      _cir.spatialNets(layerIdx).queryBoth(fu, fv, vTouchedNets);
      //_cir.spatialObs(layerIdx).queryBoth(fu, fv, vTouchedObs);
      _cir.spatialPins(layerIdx).queryBoth(fu, fv, vTouchedPins);

      //assert(vTouchedObs.empty());
      for (const auto& p : vTouchedNets) {
        //if (p.second != &net) {
          //std::cerr << p.second << " " << &net << std::endl;
          //std::cerr << net.name() << std::endl;
          //std::cerr << p.second->name() << std::endl;
        //}
        //assert(p.second == &net);
        if (p.second == &net) {
          sWires.emplace(p.first, layerIdx);
        }
      }

      FlatHashSet<Pin*> spPins;
      for (const auto& p : vTouchedPins) {
        //assert(p.second->pNet() == &net);
        if (p.second->pNet() == &net) {
          spPins.emplace(p.second);
        }
      }
      for (const Pin* pPin : spPins) {
        Int j, k;
        const Box<Int>* pBox;
        Pin_ForEachLayerIdx((*pPin), j) {
          Pin_ForEachLayerBox((*pPin), j, pBox, k) {
            sWires.emplace(*pBox, j);
          }
        }
      }
    }
    else {
      assert(u.x() == v.x() and u.y() == v.y());
      assert(u.z() + 2 == v.z());
      const Point<Int> fu(u.x(), u.y());
      const Point<Int> fv(v.x(), v.y());

      Vector<Pair<Box<Int>, Net*>> vTouchedNets;
      //Vector<Pair<Box<Int>, Obs*>> vTouchedObs;
      Vector<Pair<Box<Int>, Pin*>> vTouchedPins;
      _cir.spatialNets(u.z()).queryBoth(fu, fu, vTouchedNets);
      //_cir.spatialObs(u.z()).queryBoth(fu, fu, vTouchedObs);
      _cir.spatialPins(u.z()).queryBoth(fu, fu, vTouchedPins);
      for (const auto& p : vTouchedNets) {
        //assert(p.second == &net);
        if (p.second == &net) {
          sWires.emplace(p.first, u.z());
        }
      }

      vTouchedNets.clear();
      _cir.spatialNets(u.z() + 1).queryBoth(fu, fu, vTouchedNets);
      //_cir.spatialObs(u.z() + 1).queryBoth(fu, fu, vTouchedObs);
      _cir.spatialPins(u.z() + 1).queryBoth(fu, fu, vTouchedPins);
      for (const auto& p : vTouchedNets) {
        //assert(p.second == &net);
        if (p.second == &net) {
          sWires.emplace(p.first, u.z() + 1);
        }
      }

      vTouchedNets.clear();
      _cir.spatialNets(v.z()).queryBoth(fu, fu, vTouchedNets);
      //_cir.spatialObs(v.z()).queryBoth(fu, fu, vTouchedObs);
      _cir.spatialPins(v.z()).queryBoth(fu, fu, vTouchedPins);
      for (const auto& p : vTouchedNets) {
        //assert(p.second == &net);
        if (p.second == &net) {
          sWires.emplace(p.first, v.z());
        }
      }

      //assert(vTouchedObs.empty());
      FlatHashSet<Pin*> spPins;
      for (const auto& p : vTouchedPins) {
        //assert(p.second->pNet() == &net);
        if (p.second->pNet() == &net) {
          spPins.emplace(p.second);
        }
      }
      for (const Pin* pPin : spPins) {
        Int j, k;
        const Box<Int>* pBox;
        Pin_ForEachLayerIdx((*pPin), j) {
          Pin_ForEachLayerBox((*pPin), j, pBox, k) {
            sWires.emplace(*pBox, j);
          }
        }
      }

    }
  }

}

void DrPsRouter::queryTopoPtsWires(const Net& net, const FlatHashSet<Point3d<Int>>& sPts, FlatHashSet<Pair<Box<Int>, Int>>& sWires) const
{
  for (const auto& pt : sPts) {
    const Point<Int> p2d(pt.x(), pt.y());
    const Int layerIdx = pt.z();

    Vector<Pair<Box<Int>, Net*>> vTouchedNets;
    //Vector<Pair<Box<Int>, Obs*>> vTouchedObs;
    Vector<Pair<Box<Int>, Pin*>> vTouchedPins;
    _cir.spatialNets(layerIdx).queryBoth(p2d, p2d, vTouchedNets);
    //_cir.spatialObs(layerIdx).queryBoth(p2d, p2d, vTouchedObs);
    _cir.spatialPins(layerIdx).queryBoth(p2d, p2d, vTouchedPins);

    //assert(vTouchedObs.empty());
    for (const auto& p : vTouchedNets) {
      //assert(p.second == &net);
      if (p.second == &net) {
        sWires.emplace(p.first, layerIdx);
      }
    }
    FlatHashSet<Pin*> spPins;
    for (const auto& p : vTouchedPins) {
      //assert(p.second->pNet() == &net);
      if (p.second->pNet() == &net) {
        spPins.emplace(p.second);
      }
    }
    for (const Pin* pPin : spPins) {
      Int i, j;
      const Box<Int>* pBox;
      Pin_ForEachLayerIdx((*pPin), i) {
        Pin_ForEachLayerBox((*pPin), i, pBox, j) {
          sWires.emplace(*pBox, i);
        }
      }
    }
  }
}

void DrPsRouter::topoComp2AcsPts(const TopoTree& topo, const Int compIdx, Vector<Point3d<Int>>& vAcs) const
{
  FlatHashSet<Point3d<Int>> sAcs;
  topoComp2AcsPts(topo, compIdx, sAcs);
  std::copy(sAcs.begin(), sAcs.end(), std::back_inserter(vAcs));
}

void DrPsRouter::topoComp2AcsPts(const TopoTree& topo, const Int compIdx, FlatHashSet<Point3d<Int>>& sAcs) const
{
  assert(0 <= compIdx and compIdx < topo.numComps());
  const auto& compPts = topo.compPts(compIdx);
  if (topo.numCompSegs(compIdx) == 0) {
    assert(topo.numCompPts(compIdx) > 0);
    for (const auto& pt : compPts) {
      sAcs.emplace(pt);
    }
  }
  else {
    Vector<Segment3d<Int>> vSegs;
    topo.vCompSegs(compIdx, vSegs);
    for (const auto& seg : vSegs) {
      const auto& u = seg.min_corner();
      const auto& v = seg.max_corner();
      assert(_drMgr.hasNode(u));
      assert(_drMgr.hasNode(v));
      if (u.x() != v.x()) {
        Point3d<Int> curNode(u);
        while (curNode.x() < v.x()) {
          sAcs.emplace(curNode);

          const Int curNodeIdx = _drMgr.gridNodeIdx(curNode);
          const auto& vAdjEdgeIds = _drMgr.vAdjEdgeIds(curNodeIdx);
          const Int eIdx = vAdjEdgeIds[1]; // next
          assert(_drMgr.isValidEdge(eIdx));

          const auto& edge = _drMgr.gridEdge(eIdx);
          const Int nextNodeIdx = edge.adjNodeIdx(curNodeIdx);
          
          curNode = _drMgr.gridNode(nextNodeIdx);
        }
        sAcs.emplace(curNode);
        
      }
      else if (u.y() != v.y()) {
        Point3d<Int> curNode(u);
        while (curNode.y() < v.y()) {
          sAcs.emplace(curNode);

          const Int curNodeIdx = _drMgr.gridNodeIdx(curNode);
          const auto& vAdjEdgeIds = _drMgr.vAdjEdgeIds(curNodeIdx);
          const Int eIdx = vAdjEdgeIds[1]; // next
          assert(_drMgr.isValidEdge(eIdx));

          const auto& edge = _drMgr.gridEdge(eIdx);
          const Int nextNodeIdx = edge.adjNodeIdx(curNodeIdx);
          
          curNode = _drMgr.gridNode(nextNodeIdx);
        }
        sAcs.emplace(curNode);

      }
      else {
        Point3d<Int> curNode(u);
        while (curNode.z() < v.z()) {
          sAcs.emplace(curNode);

          const Int curNodeIdx = _drMgr.gridNodeIdx(curNode);
          const auto& vAdjEdgeIds = _drMgr.vAdjEdgeIds(curNodeIdx);
          const Int eIdx = vAdjEdgeIds[3]; // via up
          assert(_drMgr.isValidEdge(eIdx));

          const auto& edge = _drMgr.gridEdge(eIdx);
          const Int nextNodeIdx = edge.adjNodeIdx(curNodeIdx);
          
          curNode = _drMgr.gridNode(nextNodeIdx);
        }
        sAcs.emplace(curNode);

      }
    }
  }

}

//void DrPsRouter::querySegWires(const Net& net, const Segment3d<Int>& seg, Vector<Pair<Box<Int>, Int>>& vWires) const
//{
  //if (seg.bVia()) {
    //const Int layerIdx = seg.zl();
    //const Point<Int> pt(seg.p0().x(), seg.p0().y());

    //auto addWires = [&] (const Int l, Vector<Pair<Box<Int>, Int>>& v) {

      //Vector<Pair<Box<Int>, Net*>> vTouched;
      //_cir.spatialNets(l).queryBoth(pt, pt, vTouched);
      ////if (vTouched.size() == 0) {
        ////std::cerr << seg << std::endl;
        ////std::cerr << pt << " " << l << std::endl;

        ////std::cerr << "Arcs: " << std::endl;
        ////for (const auto& a : net.mArcs()) {
          ////std::cerr << a.first << std::endl;
        ////}
        ////std::cerr << std::endl;
        ////std::cerr << "Wires: " << std::endl;
        ////for (const auto& a : net.sWires()) {
          ////std::cerr << a.first << " " << a.second<< std::endl;
        ////}
        ////std::cerr << std::endl;
      ////}
      //assert(vTouched.size() > 0);

      //for (const auto& p : vTouched) {
        //const auto& box = p.first;
        //Net* pNet = p.second;
        //assert(pNet == &net);
        //v.emplace_back(box, l);
      //}
    //};
    //addWires(layerIdx, vWires);
    //addWires(layerIdx + 1, vWires);
    //addWires(layerIdx + 2, vWires);
  //}
  //else {
    //const Point<Int> p02d(seg.p0().x(), seg.p0().y());
    //const Point<Int> p12d(seg.p1().x(), seg.p1().y());
    //const Int layerIdx = seg.p0().z();

    //Vector<Pair<Box<Int>, Net*>> vTouched;
    //_cir.spatialNets(layerIdx).queryBoth(p02d, p12d, vTouched);
    //assert(vTouched.size() > 0);

    //for (const auto& p : vTouched) {
      //const auto& box = p.first;
      //Net* pNet = p.second;
      //assert(pNet == &net);
      //vWires.emplace_back(box, layerIdx);
    //}
  //}
//}


void DrPsRouter::checkFailed()
{
  spdlog::error("Routing failed");

  Int i, cnt = 0;
  const Net* pNet;
  Cir_ForEachNet(_cir, pNet, i) {
    if (!pNet->isRouted()) {
      //spdlog::error("{}", pNet->name());
      ++cnt;
    }
  }
  spdlog::error("Unrouted nets: {}", cnt);
}

Real DrPsRouter::history(const Segment3d<Int>& seg) const
{
  Vector<Int> vGridEdgeIds;
  _drMgr.vGridEdgeIds(seg, vGridEdgeIds);
  Real res = 0;
  for (const Int eIdx : vGridEdgeIds) {
    assert(eIdx >= 0);
    res += history(eIdx);
  }
  return res;

  //const auto& p0 = seg.min_corner();
  //const auto& p1 = seg.max_corner();

  //if (!_drMgr.hasNode(p0)) {
    //std::cerr << p0 << std::endl;
  //}
  //if (!_drMgr.hasNode(p1)) {
    //std::cerr << p1 << std::endl;
  //}
  //assert(_drMgr.hasNode(p0));
  //assert(_drMgr.hasNode(p1));

  //auto getDim = [] (const auto& p0, const auto& p1) -> Byte {
    //if (p0.x() != p1.x()) return 0;
    //if (p0.y() != p1.y()) return 1;
    //if (p0.z() != p1.z()) return 2;
    //assert(false);
  //};

  //const auto segDim = getDim(p0, p1);

  //std::function<Int(const Int nodeIdx)>
    //bfs = [&] (const Int nodeIdx) -> Int {
      ////std::cerr << _drMgr.gridNode(nodeIdx) << std::endl;

      //const auto& vAdjEdgeIds = _drMgr.vAdjEdgeIds(nodeIdx);

      //switch (segDim) {
        //case 0: { // x direction
                  //const Int nxtEdgeIdx  = vAdjEdgeIds.at(1);
                  //if (_drMgr.isValidEdge(nxtEdgeIdx)) {
                    //const auto& nxtEdge = _drMgr.gridEdge(nxtEdgeIdx);
                    //assert(nxtEdge.u == nodeIdx);
                    //const auto& nxtNode = _drMgr.gridNode(nxtEdge.v);

                    //assert(_drMgr.gridNode(nxtEdge.v).x() >  _drMgr.gridNode(nodeIdx).x());
                    //assert(_drMgr.gridNode(nxtEdge.v).y() == _drMgr.gridNode(nodeIdx).y());
                    //assert(_drMgr.gridNode(nxtEdge.v).z() == _drMgr.gridNode(nodeIdx).z());
                    //if (nxtNode.x() <= p1.x()) {
                      //return history(nxtEdgeIdx) + bfs(nxtEdge.v);
                    //}
                  //}
                  //break;
                //}
        //case 1: { // y direction
                  //const Int nxtEdgeIdx  = vAdjEdgeIds.at(1);
                  //if (_drMgr.isValidEdge(nxtEdgeIdx)) {
                    //const auto& nxtEdge = _drMgr.gridEdge(nxtEdgeIdx);
                    //assert(nxtEdge.u == nodeIdx);
                    //const auto& nxtNode = _drMgr.gridNode(nxtEdge.v);

                    //assert(_drMgr.gridNode(nxtEdge.v).x() == _drMgr.gridNode(nodeIdx).x());
                    //assert(_drMgr.gridNode(nxtEdge.v).y() >  _drMgr.gridNode(nodeIdx).y());
                    //assert(_drMgr.gridNode(nxtEdge.v).z() == _drMgr.gridNode(nodeIdx).z());
                    //if (nxtNode.y() <= p1.y()) {
                      //return history(nxtEdgeIdx) + bfs(nxtEdge.v);
                    //}
                  //}
                  //break;
                //}
        //case 2: { // z direction
                  //const Int nxtEdgeIdx  = vAdjEdgeIds.at(3);
                  //if (_drMgr.isValidEdge(nxtEdgeIdx)) {
                    //const auto& nxtEdge = _drMgr.gridEdge(nxtEdgeIdx);
                    //assert(nxtEdge.u == nodeIdx);
                    //const auto& nxtNode = _drMgr.gridNode(nxtEdge.v);

                    //assert(_drMgr.gridNode(nxtEdge.v).x() == _drMgr.gridNode(nodeIdx).x());
                    //assert(_drMgr.gridNode(nxtEdge.v).y() == _drMgr.gridNode(nodeIdx).y());
                    //assert(_drMgr.gridNode(nxtEdge.v).z() >  _drMgr.gridNode(nodeIdx).z());
                    //if (nxtNode.z() <= p1.z()) {
                      //return history(nxtEdgeIdx) + bfs(nxtEdge.v);
                    //}
                  //}
                  //break;
                //}
      //}
      //assert(_drMgr.gridNode(nodeIdx) == p1);
      //return 0;
    //};

  //const Int p0Idx = _drMgr.gridNodeIdx(p0);
  //return bfs(p0Idx);
}


Real DrPsRouter::history(const Segment3d<Int>& seg, const Int n, const Real decay) const
{
  const auto& p0 = seg.min_corner();
  const auto& p1 = seg.max_corner();


  auto getDim = [] (const auto& p0, const auto& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  const Int segDim = getDim(p0, p1);

  auto fastPow = [] (Real a, Real b) -> Real {
    union {
      double d;
      int x[2];
    } u = { a };
    u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;
    return u.d;
  };

  Real res = history(seg);
  Vector<Real> vFs;
  if ((1. - decay) < EPSILON) {
    vFs.assign(n, 1);
  }
  else {
    for (Int i = 1; i <= n; ++i) {
      vFs.emplace_back(fastPow(decay, i));
    }
  }
  switch (segDim) {
    case 0: {
              const Int rgIdx = (p0.z() / 2 < _cir.numRouteGrids()) ? p0.z() / 2 : p0.z() / 2 - 1;
              assert(rgIdx >= 0);
              const RouteGrid& rg = _cir.routeGrid(rgIdx);
              assert(rg.yLayerIdx() == p0.z());
              Int y = p0.y();
              for (Int i = 0; i < n; ++i) {
                y = rg.nextSignalY(y);
                if (_cir.routingBbox().yl() <= y and y <= _cir.routingBbox().yh()) {
                  res += vFs.at(i) * history(Segment3d<Int>(p0.x(), y, p0.z(), p1.x(), y, p1.z()));
                }
                else break;
              }
              y = p0.y();
              for (Int i = 0; i < n; ++i) {
                y = rg.prevSignalY(y);
                if (_cir.routingBbox().yl() <= y and y <= _cir.routingBbox().yh()) {
                  res += vFs.at(i) * history(Segment3d<Int>(p0.x(), y, p0.z(), p1.x(), y, p1.z()));
                }
                else break;
              }
              break;
            }
    case 1: {
              const Int rgIdx = (p0.z() / 2 < _cir.numRouteGrids()) ? p0.z() / 2 : p0.z() / 2 - 1;
              assert(rgIdx >= 0);
              const RouteGrid& rg = _cir.routeGrid(rgIdx);
              assert(rg.xLayerIdx() == p0.z());
              Int x = p0.x();
              for (Int i = 0; i < n; ++i) {
                x = rg.nextSignalX(x);
                if (_cir.routingBbox().xl() <= x and x <= _cir.routingBbox().xh()) {
                  res += vFs.at(i) * history(Segment3d<Int>(x, p0.y(), p0.z(), x, p1.y(), p1.z()));
                }
                else break;
              }
              x = p0.x();
              for (Int i = 0; i < n; ++i) {
                x = rg.prevSignalX(x);
                if (_cir.routingBbox().xl() <= x and x <= _cir.routingBbox().xh()) {
                  res += vFs.at(i) * history(Segment3d<Int>(x, p0.y(), p0.z(), x, p1.y(), p1.z()));
                }
                else break;
              }
              break;
            }
    case 2: break;
    default:
      assert(false);
  }
  return res;

  //const Int p0Idx = _drMgr.gridNodeIdx(p0);
  //if (segDim == 2) {
    //return history(seg);
  //}

  //auto nxtOrthoNode = [&] (const Int nodeIdx, const bool isPos) -> Int {

    //const auto& node = _drMgr.gridNode(nodeIdx);

    //auto addValidEdges = [] (Queue<Pair<Int, Int>>& q, const Int nIdx, const auto& vAdjEdgeIds, const auto& sVisitedEdgeIds) {
      //for (const Int eIdx : vAdjEdgeIds) {
        //if (eIdx >= 0 and sVisitedEdgeIds.find(eIdx) == sVisitedEdgeIds.end()) {
          //q.emplace(nIdx, eIdx);
        //}
      //}
    //};

    //FlatHashSet<Int> sVisitedEdgeIds;

    //Queue<Pair<Int, Int>> q;
    //addValidEdges(q, nodeIdx, _drMgr.vAdjEdgeIds(nodeIdx), sVisitedEdgeIds);

  
    //while (!q.empty()) {
      //const Int nIdx = q.front().first;
      //const Int edgeIdx = q.front().second;
      //q.pop();

      //if (sVisitedEdgeIds.find(edgeIdx) != sVisitedEdgeIds.end()) {
        //continue;
      //}
      //sVisitedEdgeIds.emplace(edgeIdx);


      //const auto& curNode = _drMgr.gridNode(nIdx);
      //if (curNode.z() == node.z()) {
        //if (isPos) {
          //if (segDim == 0 and curNode.y() > node.y()) {
            //return nIdx;
          //}
          //if (segDim == 1 and curNode.x() > node.x()) {
            //return nIdx;
          //}
        //}
        //else {
          //if (segDim == 0 and curNode.y() < node.y()) {
            //return nIdx;
          //}
          //if (segDim == 1 and curNode.x() < node.x()) {
            //return nIdx;
          //}
        //}
      //}

      //assert(_drMgr.isValidEdge(edgeIdx));
      //const auto& edge = _drMgr.gridEdge(edgeIdx);
      //const Int   adjNodeIdx = edge.adjNodeIdx(nIdx);

      //addValidEdges(q, adjNodeIdx, _drMgr.vAdjEdgeIds(adjNodeIdx), sVisitedEdgeIds);

    //}
    //return -1;
  //};

  //Int res = history(seg);

  //Int curNodeIdx = p0Idx;
  //for (Int i = 1; i < n; ++i) {
    //curNodeIdx = nxtOrthoNode(curNodeIdx, true);
    //if (curNodeIdx == -1) {
      //break;
    //}
    //else {
      //switch (segDim) {
        //case 0: {
          //const auto& newPt = _drMgr.gridNode(curNodeIdx);
          //assert(newPt.y() > p0.y());
          //const Segment3d<Int> newSeg(p0.x(), newPt.y(), p0.z(), p1.x(), newPt.y(), p1.z());
          //res += history(newSeg);
          //break;
        //}
        //case 1: {
          //const auto& newPt = _drMgr.gridNode(curNodeIdx);
          //assert(newPt.x() > p0.x());
          //const Segment3d<Int> newSeg(newPt.x(), p0.y(), p0.z(), newPt.x(), p1.y(), p1.z());
          //res += history(newSeg);
          //break;
        //}
        //default: assert(false);
      //}
    //}
  //}
  //curNodeIdx = p0Idx;
  //for (Int i = 1; i < n; ++i) {
    //curNodeIdx = nxtOrthoNode(curNodeIdx, false);
    //if (curNodeIdx == -1) {
      //break;
    //}
    //else {
      //switch (segDim) {
        //case 0: {
          //const auto& newPt = _drMgr.gridNode(curNodeIdx);
          //assert(newPt.y() < p0.y());
          //const Segment3d<Int> newSeg(p0.x(), newPt.y(), p0.z(), p1.x(), newPt.y(), p1.z());
          //res += history(newSeg);
          //break;
        //}
        //case 1: {
          //const auto& newPt = _drMgr.gridNode(curNodeIdx);
          //assert(newPt.x() < p0.x());
          //const Segment3d<Int> newSeg(newPt.x(), p0.y(), p0.z(), newPt.x(), p1.y(), p1.z());
          //res += history(newSeg);
          //break;
        //}
      //}
    //}
  //}

  //return res;
}

Real DrPsRouter::neighborHistory(const Segment3d<Int>& seg, const Int n, const bool isPrev) const
{
  const auto& p0 = seg.min_corner();
  const auto& p1 = seg.max_corner();


  auto getDim = [] (const auto& p0, const auto& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  Real his = 0;

  const Int segDim = getDim(p0, p1);
  switch (segDim) {
    case 0: {
              const Int rgIdx = (p0.z() / 2 < _cir.numRouteGrids()) ? p0.z() / 2 : p0.z() / 2 - 1;
              assert(rgIdx >= 0);
              const RouteGrid& rg = _cir.routeGrid(rgIdx);
              assert(rg.yLayerIdx() == p0.z());
              Int y = p0.y();
              if (isPrev) {
                y = rg.prevSignalY(y, n);
                if (y < _cir.routingBbox().yl()) {
                  y = rg.ceilY(_cir.routingBbox().yl());
                }
              }
              else {
                y = rg.nextSignalY(y, n);
                if (y > _cir.routingBbox().yh()) {
                  y = rg.floorY(_cir.routingBbox().yh());
                }
              }
              his = history(Segment3d<Int>(p0.x(), y, p0.z(), p1.x(), y, p1.z()));
              break;
            }
    case 1: {
              const Int rgIdx = (p0.z() / 2 < _cir.numRouteGrids()) ? p0.z() / 2 : p0.z() / 2 - 1;
              assert(rgIdx >= 0);
              const RouteGrid& rg = _cir.routeGrid(rgIdx);
              assert(rg.xLayerIdx() == p0.z());
              Int x = p0.x();
              if (isPrev) {
                x = rg.prevSignalX(x, n);
                if (x < _cir.routingBbox().xl()) {
                  x = rg.ceilX(_cir.routingBbox().xl());
                }
              }
              else {
                x = rg.nextSignalX(x, n);
                if (x > _cir.routingBbox().xh()) {
                  x = rg.floorX(_cir.routingBbox().xh());
                }
              }
              his = history(Segment3d<Int>(x, p0.y(), p0.z(), x, p1.z(), p1.z()));
              break;
            }
    case 2: break;
    default:
      assert(false);
  }
  return his;
}

Real DrPsRouter::history(const Point3d<Int>& u, const Point3d<Int>& v) const
{
  Int uGridNodeIdx = _drMgr.gridNodeIdx(u);
  Int vGridNodeIdx = _drMgr.gridNodeIdx(v);

  if (uGridNodeIdx > vGridNodeIdx) {
    std::swap(uGridNodeIdx, vGridNodeIdx);
  }

  const Int gridEdgeIdx = _drMgr.gridEdgeIdx(uGridNodeIdx, vGridNodeIdx);
  return history(gridEdgeIdx);
}

void DrPsRouter::addHistory(const Int eIdx, const Real cost) 
{
  _vHistory.at(eIdx) += cost;
  //const auto& e = _vGridEdges.at(eIdx);
  //const auto& u = _vGridNodes.at(e.u);
  //const auto& v = _vGridNodes.at(e.v);
}

void DrPsRouter::addHistory(const Segment3d<Int>& seg, const Real cost)
{
  Vector<Int> vGridEdgeIds;
  _drMgr.vGridEdgeIds(seg, vGridEdgeIds);
  for (const Int eIdx : vGridEdgeIds) {
    _vHistory.at(eIdx) += cost;
  }
}

void DrPsRouter::decayHistory(const Int eIdx, const Real cost) 
{
  addHistory(eIdx, -cost);
}

void DrPsRouter::decayHistory(const Segment3d<Int>& seg, const Real cost)   
{
  addHistory(seg, -cost);
}

void DrPsRouter::clearHistory(const Int eIdx)
{
  _vHistory.at(eIdx) = 0;
}

void DrPsRouter::clearHistory(const Segment3d<Int>& seg)
{
  Vector<Int> vGridEdgeIds;
  _drMgr.vGridEdgeIds(seg, vGridEdgeIds);
  for (const Int eIdx : vGridEdgeIds) {
    _vHistory.at(eIdx) = 0;
  }
}

void DrPsRouter::resetHistory() 
{
  _vHistory.assign(_vGridEdges.size(), 0);
}
  
Int DrPsRouter::wireWidth(const Point3d<Int>& u) const
{
  assert(_cir.layer(u.z()).isMetal());
  const Int metalIdx = _cir.layer(u.z()).selfIdx();
  
  assert(0 <= metalIdx and metalIdx <= _cir.numRouteGrids());
  const RouteGrid* pPreRg = metalIdx == 0 ? nullptr : _cir.pRouteGrid(metalIdx - 1);
  const RouteGrid* pCurRg = metalIdx == _cir.numRouteGrids() ? nullptr : _cir.pRouteGrid(metalIdx);

  assert(pPreRg or pCurRg);

//#ifndef NDEBUG
  //if (pPreRg and pCurRg) {
    //assert(pPreRg->wireWidth(u) == pCurRg->wireWidth(u));
  //}
//#endif
  return pCurRg ? pCurRg->wireWidth(u) : pPreRg->wireWidth(u);
}

Box<Int> DrPsRouter::toWire(const Point3d<Int>& u, const Point3d<Int>& v, const Int width, const Int ext) const
{
  assert(u.z() == v.z());
  const Int hw = width / 2;
  Int xl, yl, xh, yh;
  if (u.x() == v.x()) { // vertical
    xl = u.x() - hw;
    yl = std::min(u.y(), v.y()) - ext;
    xh = u.x() + hw;
    yh = std::max(u.y(), v.y()) + ext;
  }
  else { // horizontal
    assert(u.y() == v.y());
    xl = std::min(u.x(), v.x()) - ext;
    yl = u.y() - hw;
    xh = std::max(u.x(), v.x()) + ext;
    yh = u.y() + hw;
  }
  return Box<Int>(xl, yl, xh, yh);
}

Int DrPsRouter::scaledMdist(const Point3d<Int>& u, const Point3d<Int>& v) const
{
  assert(_cir.layer(u.z()).isMetal());
  assert(_cir.layer(v.z()).isMetal());

  Int dist = 0;
  dist += abs(u.x() - v.x()) * _horCost;
  dist += abs(u.y() - v.y()) * _verCost;
  //dist += abs(u.z() - v.z()) * _viaCost;
  dist += viaCost(u.z(), v.z());
  return dist;
}

Int DrPsRouter::scaledMdist(const Box<Int>& u, const Box<Int>& v) const
{
  Int dist = 0;
  dist += std::max({u.bl().x() - v.tr().x(), v.bl().x() - u.tr().x(), static_cast<Int>(0)}) * _horCost;
  dist += std::max({u.bl().y() - v.tr().y(), v.bl().y() - u.tr().y(), static_cast<Int>(0)}) * _verCost;
  return dist;
}

Int DrPsRouter::scaledMdist(const Point3d<Int>& u, const Pair<Box<Int>, Int>& v) const
{
  assert(_cir.layer(u.z()).isMetal());
  assert(_cir.layer(v.second).isMetal());

  Int dx = 0, dy = 0;
  //Int dz = 0;
  const Box<Int>& box = v.first;
  if (u.x() < box.xl())
    dx = box.xl() - u.x();
  else if (u.x() > box.xh())
    dx = u.x() - box.xh();
  if (u.y() < box.yl())
    dy = box.yl() - u.y();
  else if (u.y() > box.yh())
    dy = u.y() - box.yh();
  //dz = abs(u.z() - v.second);
  //return dx * _horCost + dy * _verCost + dz * _viaCost;
  return dx * _horCost + dy * _verCost + viaCost(u.z(), v.second);
}

Int DrPsRouter::scaledMdist(const Pair<Box<Int>, Int>& u, const Pair<Box<Int>, Int>& v) const
{
  assert(_cir.layer(u.second).isMetal());
  assert(_cir.layer(v.second).isMetal());
  Int dist = scaledMdist(u.first, v.first);
  //dist += abs(u.second - v.second) * _viaCost;
  dist += viaCost(u.second, v.second);
  return dist;
}

bool DrPsRouter::hasBend(const Point3d<Int>& u, const Point3d<Int>& r, const Point3d<Int>& v) const
{
  return direction3d::findDir(u, r) != direction3d::findDir(r, v);
}

Int DrPsRouter::viaCost(const Int z) const
{
  assert(z % 2 == 1);
  switch (z) {
    case 1:  return 7744;
    case 3:  return 2330;
    case 5:  return 3615;
    case 7:  return 3214;
    case 9:  return 3728;
    case 11: return 3444;
    case 13: return 4673;
    case 15: return 4673;
    case 17: return 4673;
    case 19: return 4673;
    case 21: return 4673;
    case 23: return 4673;
    case 25: return 4673;
    case 27: return 4673;
    case 29: return 4673;
    default: return 4000;
  } 
}

Int DrPsRouter::viaCost(Int zl, Int zh) const
{
  if (zl > zh) {
    std::swap(zl, zh);
  }
  Int cost = 0;
  for (Int z = zl + 1; z < zh; z += 2) {
    cost += viaCost(z);
  }
  return cost;
}


PROJECT_NAMESPACE_END
