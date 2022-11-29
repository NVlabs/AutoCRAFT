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

#include "routeMgr.hpp"

PROJECT_NAMESPACE_START

void RouteMgr::genPg()
{
  Int i, j, k;
  Region* pRegion;
  const RouteGrid* pRg;
  Net* pNet;

  //Cir_ForEachLayerIdx(_cir, i) {
    //const Layer& layer = _cir.layer(i);
    //spdlog::info("{} {}", layer.name(), layer.selfIdx());
  //}

  Cir_ForEachRegion(_cir, pRegion, i) {
    Region& reg = *pRegion;
    spdlog::info("[RouteMgr] Generate PG mesh for region {}", reg.name());
    
    Reg_ForEachRouteGrid(reg, pRg, j) {
      if (j < reg.numRouteGrids() - 1) {
        Reg_ForEachPowerNet(reg, pNet, k) {
          //if (pNet->name() != "gnd")
          genPgGrid(reg, *pRg, *pNet);
        }
      }
    }

    // connect mesh (use the last route grid upper layer)
    // the layer should be vertical
    Reg_ForEachPowerNet(reg, pNet, j) {
      connectPgGrid(reg, *pNet); 
    }

    Reg_ForEachPowerNet(reg, pNet, j) {
      pNet->setRouted();
    }
  }
  
}

void RouteMgr::genPgGrid(const Region& reg, const RouteGrid& rg, Net& net)
{
  assert(net.isPower());

  const Int xl = reg.bboxInner().xl();
  const Int yl = reg.bboxInner().yl();
  const Int xh = reg.bboxInner().xh();
  const Int yh = reg.bboxInner().yh();

  const Int stepX = rg.stepX();
  const Int stepY = rg.stepY();

  const Int sxl = xl - rg.origin().x();
  const Int syl = yl - rg.origin().y();
  const Int sxh = xh - rg.origin().x();
  const Int syh = yh - rg.origin().y();

  const Int startX = sxl - (sxl % stepX), endX = sxh - (sxh % stepX) + stepX;
  const Int startY = syl - (syl % stepY), endY = syh - (syh % stepY) + stepY;
  
  const Int queryLayerIdx = rg.lowerLayerIdx();
  const Int upperLayerIdx = rg.upperLayerIdx();
  //if (queryLayerIdx >= _cir.layer("m4s").idx()) {
    //return;
  //}

  Vector<Int> vXs, vYs, vAllYs;
  Vector<Int> vXWidths, vYWidths, vAllYWidths;
  for (Int x = startX; x < endX; x += stepX) {
    for (Int i = 0; i < rg.numXGrids(); ++i) {
      const Int xLoc = x + rg.xGrid(i);
      if (xl <= xLoc and xLoc <= xh) {
        if (rg.xUse(i) == TrackUseE::power) {
          vXs.emplace_back(xLoc);
          vXWidths.emplace_back(rg.xWidth(i));
        }
      }
    }
  }
  
  for (Int y = startY; y < endY; y += stepY) {
    for (Int i = 0; i < rg.numYGrids(); ++i) {
      const Int yLoc = y + rg.yGrid(i);
      if (yl <= yLoc and yLoc <= yh) {
        if (rg.yUse(i) == TrackUseE::power) {
          
          bool isValid = true;
          Int numNets = 0;
          if (_cir.layer(upperLayerIdx).selfIdx() < 17) { // below VM2
            for (const Int x : vXs) {
              const Point<Int> pt(x, yLoc);

              Vector<Net*> vpNets;
              _cir.spatialNets(queryLayerIdx).query(pt, pt, vpNets);
              numNets += vpNets.size();
              //Vector<Pair<Box<Int>, Net*>> v;
              //_cir.spatialNets(queryLayerIdx).queryBoth(pt, pt, v);
              //if (vpNets.size() != 1 or vpNets.size() != 2) {
                ////for (const Net* pNet : vpNets) {
                  ////std::cerr << queryLayerIdx << std::endl;
                  ////std::cerr << pNet->name() << std::endl;
                ////}
                //std::cerr << vpNets.size() << std::endl;
                //for (const auto& p : v) {
                  //const auto& box = p.first;
                  //const Net* pNet = p.second;
                  //std::cerr << pNet->name() << " " << box << " " << queryLayerIdx << std::endl;
                //}
              //}
              for (const Net* pNet : vpNets) {
                if (pNet != &net) {
                  isValid = false;
                  break;
                }
              }
#ifndef NDEBUG
              // must be the same net, otherwise shorted
              for (size_t j = 1; j < vpNets.size(); ++j) {
                const Net* p1 = vpNets.at(j - 1);
                const Net* p2 = vpNets.at(j);
                if (p1 != p2) {
                  std::cerr << _cir.layer(queryLayerIdx).name() << " " << pt << " " << p1->name() << " " << p2->name() << std::endl;
                }
                assert(p1 == p2);
              }
#endif
              if (!isValid) {
                break;
              }
            }
            if (numNets == 0) {
              isValid = false;
            }
            if (isValid) {
              vYs.emplace_back(yLoc);
              vYWidths.emplace_back(rg.yWidth(i));
            }
          }
          else { // VM and above
            //bool isValid = true;
            //Vector<Net*> vpNets;
            //for (const Int x : vXs) {
              //const Point<Int> pt(x, yLoc);
              //_cir.spatialNets(queryLayerIdx + 1).query(pt, pt, vpNets);
            //}
            //for (const Net* pNet : vpNets) {
              //if (pNet != &net) {
                //isValid = false;
                //break;
              //}
            //}
            //if (isValid) {
              vYs.emplace_back(yLoc);
              vYWidths.emplace_back(rg.yWidth(i));
            //}
          }
          vAllYs.emplace_back(yLoc);
          vAllYWidths.emplace_back(rg.yWidth(i));
        }
      }
    }
  }
  
  auto addShape = [&] (const Int layerIdx, const Box<Int>& b) {
    //net.vWires().emplace_back(b, layerIdx);
    net.sWires().emplace(b, layerIdx);
    _cir.spatialNets(layerIdx).insert(b, &net);
  };
  auto addArc = [&] (const Point3d<Int>& u, const Point3d<Int>& v, const Int w, const Int ext) {
    //net.vArcs().emplace_back(u, v);
    if (u.z() == v.z()) {
      //net.vpArcVias().emplace_back(nullptr);
      //net.vArcViaOrients().emplace_back(Orient2dE::undef);
      //net.vArcWidthExts().emplace_back(w, ext);
      //net.mArcs().emplace(Net::Arc(u, v), std::make_tuple(nullptr, Orient2dE::undef, w, ext));
      net.mArcs().emplace(Net::Arc(u, v), Net::ArcData(nullptr, Orient2dE::undef, w, ext));
    }
    else {
      const Int layerIdx = (u.z() + v.z()) / 2;
      const RouteGrid& rg = reg.routeGrid(_cir.layer(layerIdx).selfIdx());
      assert(rg.vpVias().size() > 0);
      const Via* pVia = rg.vpVias().at(0);
      //net.vpArcVias().emplace_back(pVia);
      //net.vArcViaOrients().emplace_back(Orient2dE::n);
      //net.vArcWidthExts().emplace_back(0, 0);
      //net.mArcs().emplace(Net::Arc(u, v), std::make_tuple(pVia, Orient2dE::n, 0, 0));
      net.mArcs().emplace(Net::Arc(u, v), Net::ArcData(pVia, Orient2dE::n, 0, 0));
    }
  };


  // upper layer
  const Int cutLayerIdx = rg.cutLayerIdx();
  const CutLayer& cutLayer = *static_cast<const CutLayer*>(_cir.pLayer(cutLayerIdx)); 
  const Int viaExtX = cutLayer.widthX() / 2;
  const Int viaExtY = cutLayer.widthY() / 2;
  const Int viaSpaceX = cutLayer.spaceX();
  const Int viaSpaceY = cutLayer.spaceY();
  const Int viaVencAH = cutLayer.vencAH();
  const Int viaVencPH = cutLayer.vencPH();
  const Int maxStackVias = 4; // start checking from V5 (M5 and above)

  assert(_cir.layer(upperLayerIdx).isMetal());
  const MetalLayer& layer = *static_cast<const MetalLayer*>(_cir.pLayer(upperLayerIdx));

  const CutLayer* pUpViaLayer = upperLayerIdx + 1 < _cir.numLayers() ? static_cast<const CutLayer*>(_cir.pLayer(upperLayerIdx + 1)) : nullptr;
  const Int upViaVencAL = pUpViaLayer ? pUpViaLayer->vencAL() : 0;
  const Int upViaVencPL = pUpViaLayer ? pUpViaLayer->vencPL() : 0;

  const Int vencA = std::max(viaVencAH, upViaVencAL);
  const Int vencP = std::max(viaVencPH, upViaVencPL);

  if (layer.isHor()) { // horizontal stripes
    assert(rg.yLayerIdx() == upperLayerIdx);
    if (layer.selfIdx() < 16) { // below VM
      for (size_t i = 0; i < vYs.size(); ++i) {
        const Int y = vYs.at(i);
        const Int hw = std::max(vYWidths.at(i) / 2, viaExtY + vencP);
        const Int ext = viaExtX + vencA;
        Box<Int> b(xl - ext, y - hw, xh + ext, y + hw);

        assert(rg.vpVias().size() > 0);
        const Via* pVia = rg.vpVias().at(0);
        const Via* pUpVia = pUpViaLayer ? _cir.routeGrid(pUpViaLayer->selfIdx()).vpVias().at(0) : nullptr;
        const Box<Int>* pBox;
        Int j;
        Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), xl + pBox->xl()));
          b.setYL(std::min(b.yl(), y  + pBox->yl()));
          b.setXH(std::max(b.xh(), xh + pBox->xh()));
          b.setYH(std::max(b.yh(), y  + pBox->yh()));
        }
        Via_ForEachLayerBox((*pUpVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), xl + pBox->xl()));
          b.setYL(std::min(b.yl(), y  + pBox->yl()));
          b.setXH(std::max(b.xh(), xh + pBox->xh()));
          b.setYH(std::max(b.yh(), y  + pBox->yh()));
        }

        _drc.fitWire2ValidLength(upperLayerIdx, b);
        addShape(upperLayerIdx, b); 
        addArc(Point3d<Int>(xl, y, upperLayerIdx), Point3d<Int>(xh, y, upperLayerIdx), hw * 2, ext);

        // fill more vias
        for (size_t j = 0; j < vXs.size(); ++j) {
          const Int x = vXs.at(j);
          Int vy = y + viaExtY + viaSpaceY + viaExtY;
          const Int minY = y - hw, maxY = y + hw;
          while (vy + viaExtY + vencP <= maxY) {
            const Box<Int> viaBox(x - viaExtX, vy - viaExtY, x + viaExtX, vy + viaExtY);
            bool isViolate = false;
            if (cutLayer.selfIdx() >= maxStackVias) { // M5 and above (for max stack via checking: 4 vias)
              isViolate = true;
              for (Int k = 1; k <= maxStackVias; ++k) {
                const CutLayer& qLayer = _cir.cutLayer(cutLayer.selfIdx() - k);
                bool hasVia = _cir.spatialNets(qLayer.idx()).exist(viaBox) |
                              _cir.spatialPins(qLayer.idx()).exist(viaBox) |
                              _cir.spatialObs(qLayer.idx()).exist(viaBox);
                if (!hasVia) {
                  isViolate = false;
                  break;
                }
              }
            }
            if (!isViolate) {
              addShape(cutLayerIdx, viaBox);
              addArc(Point3d<Int>(x, vy, queryLayerIdx), Point3d<Int>(x, vy, upperLayerIdx), 0, 0);
              vy += (viaExtY + viaSpaceY + viaExtY); 
            }
          }

          vy = y - viaExtY - viaSpaceY - viaExtY;
          while (vy - viaExtY - vencP >= minY) {
            const Box<Int> viaBox(x - viaExtX, vy - viaExtY, x + viaExtX, vy + viaExtY);
            bool isViolate = false;
            if (cutLayer.selfIdx() >= maxStackVias) { // M5 and above (for max stack via checking: 4 vias)
              isViolate = true;
              for (Int k = 1; k <= maxStackVias; ++k) {
                const CutLayer& qLayer = _cir.cutLayer(cutLayer.selfIdx() - k);
                //spdlog::info("{} {} {}", cutLayer.name(), cutLayer.selfIdx(), qLayer.name());
                bool hasVia = _cir.spatialNets(qLayer.idx()).exist(viaBox) |
                              _cir.spatialPins(qLayer.idx()).exist(viaBox) |
                              _cir.spatialObs(qLayer.idx()).exist(viaBox);
                if (!hasVia) {
                  isViolate = false;
                  break;
                }
              }
            }
            if (!isViolate) {
              addShape(cutLayerIdx, viaBox);
              addArc(Point3d<Int>(x, vy, queryLayerIdx), Point3d<Int>(x, vy, upperLayerIdx), 0, 0);
              vy -= (viaExtY + viaSpaceY + viaExtY); 
            }
          }
          
        }
      }
    }
    else { // VM and above
      Int s = 0;
      for (size_t i = 0; i < vYs.size(); ++i) {
        const Int yLoc = vYs.at(i);
        Vector<Net*> vpNets;
        _cir.spatialNets(upperLayerIdx).query(Box<Int>(xl, yLoc, xh, yLoc), vpNets);
        if (vpNets.empty()) {
          s = i;
          break;
        }
      }
      for (size_t i = s; i < vYs.size(); i += reg.numPowerNets()) {
        const Int y = vYs.at(i);
        const Int hw = std::max(vYWidths.at(i) / 2, viaExtY + vencP);
        const Int ext = viaExtX + vencA;
        Box<Int> b(xl - ext, y - hw, xh + ext, y + hw);

        assert(rg.vpVias().size() > 0);
        const Via* pVia = rg.vpVias().at(0);
        const Via* pUpVia = pUpViaLayer ? _cir.routeGrid(pUpViaLayer->selfIdx()).vpVias().at(0) : nullptr;
        const Box<Int>* pBox;
        Int j;
        Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), xl + pBox->xl()));
          b.setYL(std::min(b.yl(), y  + pBox->yl()));
          b.setXH(std::max(b.xh(), xh + pBox->xh()));
          b.setYH(std::max(b.yh(), y  + pBox->yh()));
        }
        Via_ForEachLayerBox((*pUpVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), xl + pBox->xl()));
          b.setYL(std::min(b.yl(), y  + pBox->yl()));
          b.setXH(std::max(b.xh(), xh + pBox->xh()));
          b.setYH(std::max(b.yh(), y  + pBox->yh()));
        }
        _drc.fitWire2ValidLength(upperLayerIdx, b);
        addShape(upperLayerIdx, b);
        addArc(Point3d<Int>(xl, y, upperLayerIdx), Point3d<Int>(xh, y, upperLayerIdx), hw * 2, ext);
      }
    }
  }
  else { // veritcal stripes
    assert(layer.isVer());
    if (layer.selfIdx() < 16) { // below VM
      //std::cerr << layer.name() << std::endl;
      for (size_t i = 0; i < vXs.size(); ++i) {
        const Int x = vXs.at(i);
        const Int hw = std::max(vXWidths.at(i) / 2, viaExtX + vencP);
        Int y, startY = 0, endY = 0;
        bool toAdd = false;
        for (size_t j = 0; j < vAllYs.size(); ++j) {
          y = vAllYs.at(j);
          if (std::binary_search(vYs.begin(), vYs.end(), y)) {
            if (!toAdd) {
              startY = y;
              toAdd = true;
            }
            endY = y;
            if (j == vAllYs.size() - 1) { // last one
              const Int ext = viaExtY + vencA;
              Box<Int> b(x - hw, startY - ext, x + hw, endY + ext);

              assert(rg.vpVias().size() > 0);
              const Via* pVia = rg.vpVias().at(0);
              const Via* pUpVia = pUpViaLayer ? _cir.routeGrid(pUpViaLayer->selfIdx()).vpVias().at(0) : nullptr;
              const Box<Int>* pBox;
              Int k;
              Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, k) {
                b.setXL(std::min(b.xl(), x + pBox->xl()));
                b.setYL(std::min(b.yl(), startY + pBox->yl()));
                b.setXH(std::max(b.xh(), x + pBox->xh()));
                b.setYH(std::max(b.yh(), endY + pBox->yh()));
              }
              Via_ForEachLayerBox((*pUpVia), upperLayerIdx, pBox, k) {
                b.setXL(std::min(b.xl(), x + pBox->xl()));
                b.setYL(std::min(b.yl(), startY + pBox->yl()));
                b.setXH(std::max(b.xh(), x + pBox->xh()));
                b.setYH(std::max(b.yh(), endY + pBox->yh()));
              }

              _drc.fitWire2ValidLength(upperLayerIdx, b);
              addShape(upperLayerIdx, b);
              addArc(Point3d<Int>(x, startY, upperLayerIdx), Point3d<Int>(x, endY, upperLayerIdx), hw * 2, ext);
            }
          }
          else {
            if (toAdd) {
              const Int ext = viaExtY + vencA;
              Box<Int> b(x - hw, startY - ext, x + hw, endY + ext);

              assert(rg.vpVias().size() > 0);
              const Via* pVia = rg.vpVias().at(0);
              const Via* pUpVia = pUpViaLayer ? _cir.routeGrid(pUpViaLayer->selfIdx()).vpVias().at(0) : nullptr;
              const Box<Int>* pBox;
              Int k;
              Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, k) {
                b.setXL(std::min(b.xl(), x + pBox->xl()));
                b.setYL(std::min(b.yl(), startY  + pBox->yl()));
                b.setXH(std::max(b.xh(), x + pBox->xh()));
                b.setYH(std::max(b.yh(), endY  + pBox->yh()));
              }
              Via_ForEachLayerBox((*pUpVia), upperLayerIdx, pBox, k) {
                b.setXL(std::min(b.xl(), x + pBox->xl()));
                b.setYL(std::min(b.yl(), startY  + pBox->yl()));
                b.setXH(std::max(b.xh(), x + pBox->xh()));
                b.setYH(std::max(b.yh(), endY  + pBox->yh()));
              }

              _drc.fitWire2ValidLength(upperLayerIdx, b);
              addShape(upperLayerIdx, b);
              addArc(Point3d<Int>(x, startY, upperLayerIdx), Point3d<Int>(x, endY, upperLayerIdx), hw * 2, ext);
              toAdd = false;
            }
            startY = endY = y;
          }
        }
      }
    }
    else { // VM and above
      Int s = 0;
      for (size_t i = 0; i < vXs.size(); ++i) {
        const Int xLoc = vXs.at(i);
        Vector<Net*> vpNets;
        _cir.spatialNets(upperLayerIdx).query(Box<Int>(xLoc, yl, xLoc, yh), vpNets);
        if (vpNets.empty()) {
          s = i;
          break;
        }
      }
      for (size_t i = s; i < vXs.size(); i += reg.numPowerNets()) {
        const Int x = vXs.at(i);
        const Int hw = std::max(vXWidths.at(i) / 2, viaExtX + vencP);
        const Int ext = viaExtY + vencA;
        Box<Int> b(x - hw, yl - ext, x + hw, yh + ext);

        assert(rg.vpVias().size() > 0);
        const Via* pVia = rg.vpVias().at(0);
        const Via* pUpVia = pUpViaLayer ? _cir.routeGrid(pUpViaLayer->selfIdx()).vpVias().at(0) : nullptr;
        const Box<Int>* pBox;
        Int j;
        Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), x  + pBox->xl()));
          b.setYL(std::min(b.yl(), yl + pBox->yl()));
          b.setXH(std::max(b.xh(), x  + pBox->xh()));
          b.setYH(std::max(b.yh(), yh + pBox->yh()));
        }
        Via_ForEachLayerBox((*pUpVia), upperLayerIdx, pBox, j) {
          b.setXL(std::min(b.xl(), x  + pBox->xl()));
          b.setYL(std::min(b.yl(), yl + pBox->yl()));
          b.setXH(std::max(b.xh(), x  + pBox->xh()));
          b.setYH(std::max(b.yh(), yh + pBox->yh()));
        }
        _drc.fitWire2ValidLength(upperLayerIdx, b);
        addShape(upperLayerIdx, b);
        addArc(Point3d<Int>(x, yl, upperLayerIdx), Point3d<Int>(x, yh, upperLayerIdx), hw * 2, ext);
      }     
    }

  }

  // vias
  for (size_t i = 0; i < vXs.size(); ++i) {
    const Int x = vXs.at(i);
    Vector<Int> v(vYs.size());
    if (cutLayer.selfIdx() % 2) {
      for (size_t j = 0; j < v.size(); ++j) {
        v[j] = j;
      }
    }
    else {
      for (size_t j = 0; j < v.size(); ++j) {
        v[j] = v.size() - 1 - j;
      }
    }
    if (x % 2) {
      std::reverse(v.begin(), v.end());
    }
    for (const Int j : v) {
      const Int y = vYs.at(j);
      const Box<Int> b(x - viaExtX, y - viaExtY, x + viaExtX, y + viaExtY);

      // check enclosure
      Vector<Box<Int>> vHNets, vLNets;
      _cir.spatialNets(queryLayerIdx).queryBox(b, vLNets);
      _cir.spatialNets(upperLayerIdx).queryBox(b, vHNets);
      Vector<Box<Int>> vHObs, vLObs;
      _cir.spatialObs(queryLayerIdx).queryBox(b, vLObs);
      _cir.spatialObs(upperLayerIdx).queryBox(b, vHObs);

      bool isHEnclosed = false;
      for (const auto& box : vHNets) {
        if (Box<Int>::bCover(box, b)) {
          isHEnclosed = true;
          break;
        }
      }
      for (const auto& box : vHObs) {
        if (Box<Int>::bCover(box, b)) {
          isHEnclosed = true;
          break;
        }
      }
      
      bool isLEnclosed = false;
      for (const auto& box : vLNets) {
        if (Box<Int>::bCover(box, b)) {
          isLEnclosed = true;
          break;
        }
      }
      for (const auto& box : vLObs) {
        if (Box<Int>::bCover(box, b)) {
          isLEnclosed = true;
          break;
        }
      }


      bool isStackViolation = false;
      if (cutLayer.selfIdx() < 15 and cutLayer.selfIdx() >= maxStackVias) {  // 15 <-> VV0
        isStackViolation = true;
        for (Int k = 1; k <= maxStackVias; ++k) {
          const CutLayer& qLayer = _cir.cutLayer(cutLayer.selfIdx() - k);
          bool hasVia = _cir.spatialNets(qLayer.idx()).exist(b) |
                        _cir.spatialPins(qLayer.idx()).exist(b) |
                        _cir.spatialObs(qLayer.idx()).exist(b);
          if (!hasVia) {
            isStackViolation = false;
            break;
          }
        }
      }
      //if (queryLayerIdx == 0 or queryLayerIdx == 2) {
        //std::cerr << queryLayerIdx<<  " " << isHEnclosed << " " << isLEnclosed << " " << isStackViolation << std::endl;
      //}

      if (isHEnclosed and isLEnclosed and !isStackViolation) {
        

        // check via spacing
        const Box<Int> vcb0(b.xl() - viaSpaceX + 1, b.yl(), b.xl(), b.yh());
        const Box<Int> vcb1(b.xh(), b.yl(), b.xh() + viaSpaceX - 1, b.yh());
        const Box<Int> vcb2(b.xl(), b.yl() - viaSpaceY + 1, b.xh(), b.yl());
        const Box<Int> vcb3(b.xl(), b.yh(), b.xl(), b.yh() + viaSpaceY - 1);
        Vector<Net*> vpNets;
        _cir.spatialNets(cutLayerIdx).query(vcb0, vpNets);
        _cir.spatialNets(cutLayerIdx).query(vcb1, vpNets);
        _cir.spatialNets(cutLayerIdx).query(vcb2, vpNets);
        _cir.spatialNets(cutLayerIdx).query(vcb3, vpNets);
        if (vpNets.empty()) {
          // check if enclosed by lower layer
          //_cir.spatialNets(queryLayerIdx).query(b, vpNets);
          //if (vpNets.size()) {
            //assert(vpNets.at(0) == &net);
          //}
          // check metal spacing
          const Via* pVia = rg.vpVias().at(0);
          const Box<Int>* pBox;
          Box<Int> hBox(b), lBox(b);
          Int k;
          Via_ForEachLayerBox((*pVia), queryLayerIdx, pBox, k) {
            lBox.setXL(std::min(lBox.xl(), x + pBox->xl()));
            lBox.setYL(std::min(lBox.yl(), y + pBox->yl()));
            lBox.setXH(std::max(lBox.xh(), x + pBox->xh()));
            lBox.setYH(std::max(lBox.yh(), y + pBox->yh()));
          }
          Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, k) {
            hBox.setXL(std::min(hBox.xl(), x + pBox->xl()));
            hBox.setYL(std::min(hBox.yl(), y + pBox->yl()));
            hBox.setXH(std::max(hBox.xh(), x + pBox->xh()));
            hBox.setYH(std::max(hBox.yh(), y + pBox->yh()));
          }
          Vector<Box<Int>> vLBoxes, vHBoxes;
          _cir.spatialNets(queryLayerIdx).queryBox(lBox, vLBoxes);
          _cir.spatialNets(upperLayerIdx).queryBox(hBox, vHBoxes);
          for (const Box<Int>& box : vLBoxes) {
            lBox.setXL(std::min(lBox.xl(), box.xl()));
            lBox.setYL(std::min(lBox.yl(), box.yl()));
            lBox.setXH(std::max(lBox.xh(), box.xh()));
            lBox.setYH(std::max(lBox.yh(), box.yh()));
          }
          for (const Box<Int>& box : vHBoxes) {
            hBox.setXL(std::min(hBox.xl(), box.xl()));
            hBox.setYL(std::min(hBox.yl(), box.yl()));
            hBox.setXH(std::max(hBox.xh(), box.xh()));
            hBox.setYH(std::max(hBox.yh(), box.yh()));
          }
          _drc.fitWire2ValidLength(queryLayerIdx, lBox);
          _drc.fitWire2ValidLength(upperLayerIdx, hBox);
          bool isLSpacingViolation = false;
          bool isHSpacingViolation = false;
          //if (queryLayerIdx > 2) {
            //isLSpacingViolation = !_drc.checkWireMetalEolSpacing(net, queryLayerIdx, lBox); 
            //isHSpacingViolation = !_drc.checkWireMetalEolSpacing(net, upperLayerIdx, hBox); 
          //}
          if (!isLSpacingViolation and !isHSpacingViolation) {
            addShape(cutLayerIdx, b);
            addArc(Point3d<Int>(x, y, queryLayerIdx), Point3d<Int>(x, y, upperLayerIdx), 0, 0);
          }
        }
      }
    }
  }

}

void RouteMgr::connectPgGrid(const Region& reg, Net& net) 
{
  const Int xl = reg.bboxInner().xl();
  const Int xh = reg.bboxInner().xh();

  //Int yl = MAX_INT;
  //Int yh = MIN_INT;
  Int yl = reg.bboxInner().yl();
  Int yh = reg.bboxInner().yh();
  Int i, j, layerIdx;
  const Pin* pPin;
  const Box<Int>* pBox;
  Net_ForEachPin(net, pPin, i) {
    Pin_ForEachLayerIdx((*pPin), layerIdx) {
      Pin_ForEachLayerBox((*pPin), layerIdx, pBox, j) {
        yl = std::min(yl, pBox->yl());
        yh = std::max(yh, pBox->yh());
      }
    }
  }


  const RouteGrid& rg = reg.routeGrid(reg.numRouteGrids() - 1);
  
  const Int cutLayerIdx = rg.cutLayerIdx();
  const Int upperLayerIdx = rg.upperLayerIdx();
  const Int queryLayerIdx = rg.lowerLayerIdx();


  assert(rg.xLayerIdx() == upperLayerIdx);
  
  const Int sXL = xl - rg.origin().x();
  const Int sYL = yl - rg.origin().y();
  const Int sXH = xh - rg.origin().x();
  const Int sYH = yh - rg.origin().y();

  const Int stepX = rg.stepX();
  const Int stepY = rg.stepY();
  const Int startX = sXL - (sXL % stepX), endX = sXH - (sXH % stepX) + stepX;
  const Int startY = sYL - (sYL % stepY), endY = sYH - (sYH % stepY) + stepY;

  Vector<Int> vXs, vYs;
  Vector<Int> vXWidths, vYWidths;
  for (Int x = startX; x < endX; x += stepX) {
    for (Int i = 0; i < rg.numXGrids(); ++i) {
      const Int xLoc = x + rg.xGrid(i);
      if (xl <= xLoc and xLoc <= xh) {
        if (rg.xUse(i) == TrackUseE::power) {
          Vector<Net*> vpNets;
          _cir.spatialNets(upperLayerIdx).query(Box<Int>(xLoc, yl, xLoc, yh), vpNets);
          if (vpNets.empty()) {
            vXs.emplace_back(xLoc);
            vXWidths.emplace_back(rg.xWidth(i));
            break;
          }
          else {
            bool isValid = true;
            for (const Net* pNet : vpNets) {
              if (pNet != &net) {
                isValid = false;
                break;
              }
            }
            if (isValid) {
              vXs.emplace_back(xLoc);
              vXWidths.emplace_back(rg.xWidth(i));
              break;
            }
          }
        }
      }
    }
  }
  for (Int y = startY; y < endY; y += stepY) {
    for (Int i = 0; i < rg.numYGrids(); ++i) {
      const Int yLoc = y + rg.yGrid(i);
      if (yl <= yLoc and yLoc <= yh) {
        if (rg.yUse(i) == TrackUseE::power) {
          
          bool isValid = true;
          Int numNets = 0;
          for (const Int x : vXs) {
            const Point<Int> pt(x, yLoc);

            Vector<Net*> vpNets;
            _cir.spatialNets(queryLayerIdx).query(pt, pt, vpNets);
            numNets += vpNets.size();
            for (const Net* pNet : vpNets) {
              if (pNet != &net) {
                isValid = false;
                break;
              }
            }
#ifndef NDEBUG
            // must be the same net, otherwise shorted
            for (size_t j = 1; j < vpNets.size(); ++j) {
              const Net* p1 = vpNets.at(j - 1);
              const Net* p2 = vpNets.at(j);
              if (p1 != p2) {
                std::cerr << _cir.layer(queryLayerIdx).name() << " " << pt << " " << p1->name() << " " << p2->name() << std::endl;
              }
              assert(p1 == p2);
            }
#endif
            if (!isValid) {
              break;
            }
          }
          if (numNets == 0) {
            isValid = false;
          }
          if (isValid) {
            vYs.emplace_back(yLoc);
            vYWidths.emplace_back(rg.yWidth(i));
          }
        }
      }
    }
  }

  auto addShape = [&] (const Int layerIdx, const Box<Int>& b) {
    //net.vWires().emplace_back(b, layerIdx);
    net.sWires().emplace(b, layerIdx);
    _cir.spatialNets(layerIdx).insert(b, &net);
  };
  auto addArc = [&] (const Point3d<Int>& u, const Point3d<Int>& v, const Int w, const Int ext) {
    //net.vArcs().emplace_back(u, v);
    if (u.z() == v.z()) {
      //net.vpArcVias().emplace_back(nullptr);
      //net.vArcViaOrients().emplace_back(Orient2dE::undef);
      //net.vArcWidthExts().emplace_back(w, ext);
      //net.mArcs().emplace(Net::Arc(u, v), std::make_tuple(nullptr, Orient2dE::undef, w, ext));
      net.mArcs().emplace(Net::Arc(u, v), Net::ArcData(nullptr, Orient2dE::undef, w, ext));
    }
    else {
      const Int layerIdx = (u.z() + v.z()) / 2;
      const RouteGrid& rg = reg.routeGrid(_cir.layer(layerIdx).selfIdx());
      assert(rg.vpVias().size() > 0);
      const Via* pVia = rg.vpVias().at(0);
      //net.vpArcVias().emplace_back(pVia);
      //net.vArcViaOrients().emplace_back(Orient2dE::n);
      //net.vArcWidthExts().emplace_back(0, 0);
      //net.mArcs().emplace(Net::Arc(u, v), std::make_tuple(pVia, Orient2dE::n, w, ext));
      net.mArcs().emplace(Net::Arc(u, v), Net::ArcData(pVia, Orient2dE::n, w, ext));
    }
  };

  const CutLayer& cutLayer = *static_cast<const CutLayer*>(_cir.pLayer(cutLayerIdx));
  const MetalLayer& upperLayer = *static_cast<const MetalLayer*>(_cir.pLayer(upperLayerIdx));
  
  assert(upperLayer.isVer());
  
  const Int viaExtX = cutLayer.widthX() / 2;
  const Int viaExtY = cutLayer.widthX() / 2;
  const Int viaSpaceX = cutLayer.spaceX();
  const Int viaSpaceY = cutLayer.spaceY();
  const Int viaVencAH = cutLayer.vencAH();
  const Int viaVencPH = cutLayer.vencPH();

  // vertical stripes
  for (size_t i = 0; i < vXs.size(); ++i) {
    const Int x = vXs.at(i);
    const Int hw = std::max(vXWidths.at(i) / 2, viaExtX + viaVencPH);
    const Int ext = viaExtY + viaVencAH;
    Box<Int> b(x - hw, yl - ext, x + hw, yh + ext);
    assert(rg.vpVias().size() > 0);
    const Via* pVia = rg.vpVias().at(0);
    const Box<Int>* pBox;
    Int k;
    Via_ForEachLayerBox((*pVia), upperLayerIdx, pBox, k) {
      b.setXL(std::min(b.xl(), x  + pBox->xl()));
      b.setYL(std::min(b.yl(), yl + pBox->yl()));
      b.setXH(std::max(b.xh(), x  + pBox->xh()));
      b.setYH(std::max(b.yh(), yh + pBox->yh()));
    }
    _drc.fitWire2ValidLength(upperLayerIdx, b);
    addShape(upperLayerIdx, b);   
    addArc(Point3d<Int>(x, yl, upperLayerIdx), Point3d<Int>(x, yh, upperLayerIdx), hw * 2, ext);
  }
  
  // vias
  for (size_t i = 0; i < vXs.size(); ++i) {
    const Int x = vXs.at(i);
    for (size_t j = 0; j < vYs.size(); ++j) {
      const Int y = vYs.at(j);
      const Box<Int> b(x - viaExtX, y - viaExtY, x + viaExtX, y + viaExtY);
  
      const Box<Int> cb0(b.xl() - viaSpaceX + 1, b.yl(), b.xl(), b.yh());
      const Box<Int> cb1(b.xh(), b.yl(), b.xh() + viaSpaceX - 1, b.yh());
      const Box<Int> cb2(b.xl(), b.yl() - viaSpaceY + 1, b.xh(), b.yl());
      const Box<Int> cb3(b.xl(), b.yh(), b.xl(), b.yh() + viaSpaceY - 1);

      Vector<Net*> vpNets;
      _cir.spatialNets(cutLayerIdx).query(cb0, vpNets);
      _cir.spatialNets(cutLayerIdx).query(cb1, vpNets);
      _cir.spatialNets(cutLayerIdx).query(cb2, vpNets);
      _cir.spatialNets(cutLayerIdx).query(cb3, vpNets);
      if (vpNets.empty()) {
        addShape(cutLayerIdx, b);   
        addArc(Point3d<Int>(x, y, queryLayerIdx), Point3d<Int>(x, y, upperLayerIdx), 0, 0);
      }
    }
  }

}


PROJECT_NAMESPACE_END
