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

#include "placeMgr.hpp"
#include "placeSMT.hpp"


PROJECT_NAMESPACE_START

bool PlaceMgr::solveSMT(const Int numThreads, 
                        const Real ar, 
                        const Real fcUtil,
                        const Vector<String>& vRegNames,
                        const Vector<Real>& vFcRegUtils,
                        const Vector<Real>& vBounds,
                        const Int iter)
{
  spdlog::stopwatch sw;

  assert(vRegNames.size() == vFcRegUtils.size());
  //FlatHashMap<const Region*, Real> mFcRegUtils; 
  Umap<String, Real> mFcRegUtils; 
  for (size_t i = 0; i < vRegNames.size(); ++i) {
    //mFcRegUtils.emplace(_cir.pRegion(vRegNames.at(i)), vFcRegUtils.at(i));
    mFcRegUtils.emplace(vRegNames.at(i), vFcRegUtils.at(i));
  }

  setSymCellOrients();
  
  SmtPlacer p(_cir, ar, fcUtil, mFcRegUtils, vBounds, numThreads, iter);
  bool b = p.solve();

  genEdgeCells();
  genFillCells();

  setRowCellOrients();

  spdlog::info("[PlaceMgr] {:<30} Elapsed: {:.3f} s", "Place (SMT)", sw);
  return b;
}

void PlaceMgr::genEdgeCells()
{

  spdlog::info("[PlaceMgr] Generate edge cells");

  Int i;
  Region* pReg;

  Cir_ForEachRegion(_cir, pReg, i) {
    Region& reg = *pReg;
    const Primitive& e = reg.edgePrimEast();
    const Primitive& w = reg.edgePrimWest();
    const Primitive& n = reg.edgePrimNorth();
    const Primitive& s = reg.edgePrimSouth();
    const Primitive& ne = reg.edgePrimNorthEast();
    const Primitive& nw = reg.edgePrimNorthWest();
    const Primitive& se = reg.edgePrimSouthEast();
    const Primitive& sw = reg.edgePrimSouthWest();

    assert(reg.bboxInner().width() % n.sizeX() == 0);
    assert(reg.bboxInner().width() % s.sizeX() == 0);
    assert(reg.bboxInner().height() % e.sizeY() == 0);
    assert(reg.bboxInner().height() % w.sizeY() == 0);
    //assert(ne.sizeX() == e.sizeX() and ne.sizeY() == n.sizeY());
    //assert(nw.sizeX() == w.sizeX() and nw.sizeY() == n.sizeY());
    //assert(se.sizeX() == e.sizeX() and se.sizeY() == s.sizeY());
    //assert(sw.sizeX() == w.sizeX() and sw.sizeY() == s.sizeY());

    const Int xl = reg.bboxInner().xl();
    const Int yl = reg.bboxInner().yl();
    const Int xh = reg.bboxInner().xh();
    const Int yh = reg.bboxInner().yh();

    assert(reg.placeGrid().isOnGridX(xl));
    assert(reg.placeGrid().isOnGridY(yl));
    assert(reg.placeGrid().isOnGridX(xh));
    assert(reg.placeGrid().isOnGridY(yh));

    const Int stepX = n.sizeX();
    const Int stepY = e.sizeY();

    assert(stepX == reg.placeGrid().stepX());
    assert(stepY == reg.placeGrid().stepY());

    // east
    for (Int y = yl; y < yh; y += stepY) {
      const Int idx = reg.numEdgeCells();
      reg.addEdgeCell();
      Cell& c = reg.edgeCell(idx);
      const String& name = "reg" + std::to_string(i) + "_edge_e_" + std::to_string(idx);
      c.setName(name);
      c.setInstName(name);
      c.setIdx(idx);
      c.setPrim(e);
      c.setRegion(pReg);
      c.setOrient(reg.edgePrimEastOrient());
      const Int gx = (xh - xl) / stepX;
      const Int gy = (y - yl) / stepY;
      c.setLoc(xh, y, gx, gy);
      if (gy % 2) {
        c.flipY();
      }
    }
    // west
    for (Int y = yl; y < yh; y += stepY) {
      const Int idx = reg.numEdgeCells();
      reg.addEdgeCell();
      Cell& c = reg.edgeCell(idx);
      const String& name = "reg" + std::to_string(i) + "_edge_w_" + std::to_string(idx);
      c.setName(name);
      c.setInstName(name);
      c.setIdx(idx);
      c.setPrim(w);
      c.setRegion(pReg);
      c.setOrient(reg.edgePrimWestOrient());
      const Int gx = -1;
      const Int gy = (y - yl) / stepY;
      c.setLoc(xl - c.width(), y, gx, gy);
      if (gy % 2) {
        c.flipY();
      }
    }
    // north 
    for (Int x = xl; x < xh; x += stepX) {
      const Int idx = reg.numEdgeCells();
      reg.addEdgeCell();
      Cell& c = reg.edgeCell(idx);
      const String& name = "reg" + std::to_string(i) + "_edge_n_" + std::to_string(idx);
      c.setName(name);
      c.setInstName(name);
      c.setIdx(idx);
      c.setPrim(n);
      c.setRegion(pReg);
      c.setOrient(reg.edgePrimNorthOrient());
      c.setLoc(x, yh, (x - xl) / stepX, (yh - yl) / stepY);
    }
    // south 
    for (Int x = xl; x < xh; x += stepX) {
      const Int idx = reg.numEdgeCells();
      reg.addEdgeCell();
      Cell& c = reg.edgeCell(idx);
      const String& name = "reg" + std::to_string(i) + "_edge_s_" + std::to_string(idx);
      c.setName(name);
      c.setInstName(name);
      c.setIdx(idx);
      c.setPrim(s);
      c.setRegion(pReg);
      c.setOrient(reg.edgePrimSouthOrient());
      c.setLoc(x, yl - c.height(), (x - xl) / stepX, -1);
    }
    // corners (north east)
    const Int neIdx = reg.numEdgeCells();
    reg.addEdgeCell();
    Cell& c0 = reg.edgeCell(neIdx);
    const String& neName = "reg" + std::to_string(i) + "_edge_ne_" + std::to_string(neIdx);
    c0.setName(neName);
    c0.setInstName(neName);
    c0.setIdx(neIdx);
    c0.setPrim(ne);
    c0.setRegion(pReg);
    c0.setOrient(reg.edgePrimNorthEastOrient());
    c0.setLoc(xh, yh, (xh - xl) / stepX, (yh - yl) / stepY);
    // corners (north west)
    const Int nwIdx = reg.numEdgeCells();
    reg.addEdgeCell();
    Cell& c1 = reg.edgeCell(nwIdx);
    const String& nwName = "reg" + std::to_string(i) + "_edge_nw_" + std::to_string(nwIdx);
    c1.setName(nwName);
    c1.setInstName(nwName);
    c1.setIdx(nwIdx);
    c1.setPrim(nw);
    c1.setRegion(pReg);
    c1.setOrient(reg.edgePrimNorthWestOrient());
    c1.setLoc(xl - c1.width(), yh, -1, (yh - yl) / stepY);
    // corners (south east)
    const Int seIdx = reg.numEdgeCells();
    reg.addEdgeCell();
    Cell& c2 = reg.edgeCell(seIdx);
    const String& seName = "reg" + std::to_string(i) + "_edge_se_" + std::to_string(seIdx);
    c2.setName(seName);
    c2.setInstName(seName);
    c2.setIdx(seIdx);
    c2.setPrim(se);
    c2.setRegion(pReg);
    c2.setOrient(reg.edgePrimSouthEastOrient());
    c2.setLoc(xh, yl - c2.height(), (xh - xl) / stepX, -1);
    // corners (south west)
    const Int swIdx = reg.numEdgeCells();
    reg.addEdgeCell();
    Cell& c3 = reg.edgeCell(swIdx);
    const String& swName = "reg" + std::to_string(i) + "_edge_sw_" + std::to_string(swIdx);
    c3.setName(swName);
    c3.setInstName(swName);
    c3.setIdx(swIdx);
    c3.setPrim(sw);
    c3.setRegion(pReg);
    c3.setOrient(reg.edgePrimSouthWestOrient());
    c3.setLoc(xl - c3.width(), yl - c3.height(), -1, -1);
    

    reg.setBboxOuter(c3.loc().x(), 
                     c3.loc().y(),
                     c0.loc().x() + c0.width(),
                     c0.loc().y() + c0.height());
    //Int j;
    //const Cell* pCell;
    //Reg_ForEachEdgeCell(reg, pCell, j) {
      //spdlog::info("{} {} {}", pCell->name(), pCell->loc().x(), pCell->loc().y());
    //}
  }
}

void PlaceMgr::genFillCells()
{
  // TODO: pick primitive for cells with extension constraints
  spdlog::info("[PlaceMgr] Generate fill cells");

  Int i, j;
  Region* pReg;
  const Cell* pCell;

  Cir_ForEachRegion(_cir, pReg, i) {
    Region& reg = *pReg;
    assert(reg.numDummyPrims() > 0);

    const Primitive& dp = reg.dummyPrim(0);
    const DevMap* pDevMap = _cir.pDevMap(dp.name());
    assert(dp.sizeX() == reg.placeGrid().stepX());
    assert(dp.sizeY() == reg.placeGrid().stepY());

    const Int xl = reg.bboxInner().xl();
    const Int yl = reg.bboxInner().yl();
    const Int xh = reg.bboxInner().xh();
    const Int yh = reg.bboxInner().yh();
    assert(reg.placeGrid().isOnGridX(xl));
    assert(reg.placeGrid().isOnGridY(yl));
    assert(reg.placeGrid().isOnGridX(xh));
    assert(reg.placeGrid().isOnGridY(yh));

    const Int stepX = reg.placeGrid().stepX();
    const Int stepY = reg.placeGrid().stepY();

    FlatHashSet<Point<Int>> sCellLocs;
    Reg_ForEachCell(reg, pCell, j) {
      const Int cxl = pCell->loc().x();
      const Int cxh = cxl + pCell->width();
      const Int cyl = pCell->loc().y();
      const Int cyh = cyl + pCell->height();
      for (Int x = cxl; x < cxh; x += stepX) {
        for (Int y = cyl; y < cyh; y += stepY) {
          sCellLocs.emplace(x, y);
        }
      }
    }

    for (Int x = xl; x < xh; x += stepX) {
      for (Int y = yl; y < yh; y += stepY) {
        const Point<Int> loc(x, y);
        if (sCellLocs.find(loc) == sCellLocs.end()) {
          const Int idx = reg.numDummyCells();
          reg.addDummyCell();
          Cell& c = reg.dummyCell(idx);
          const String& name = "reg" + std::to_string(i) + "_fill_" + std::to_string(idx);
          c.setName(name);
          c.setInstName(name);
          c.setIdx(idx);
          c.setDevMap(pDevMap);
          c.setPrim(dp);
          c.setRegion(pReg);
          const Int gx = (x - xl) / stepX;
          const Int gy = (y - yl) / stepY;
          c.setOrient(gy % 2 ? Orient2dE::fs : Orient2dE::n);
          c.setLoc(x, y, gx, gy);
        }
      }
    }

  }
}

void PlaceMgr::setSymCellOrients() 
{
  // 2-coloring to assign orients for x orients
  Vector<Vector<Int>> vvAdjs(_cir.numCells());
  Vector<Byte> vColors(_cir.numCells(), -1);

  Int i, j;
  const PlaceSymCstr* pSymCstr;
  Cir_ForEachPlaceSymCstr(_cir, pSymCstr, i) {
    const auto& cstr = *pSymCstr;
    for (j = 0; j < cstr.numCells(); ++j) {
      const Cell& c = cstr.cell(j); 
      if (cstr.isPartA(j)) {
        const Cell& symC = cstr.symCell(j);
        vvAdjs[c.idx()].emplace_back(symC.idx());
        vvAdjs[symC.idx()].emplace_back(c.idx());
      }
      //else if (cstr.isSelfSym(j)) {
      //}
    }
  }

  // bfs
  Queue<Int> q;
  for (size_t i = 0; i < vvAdjs.size(); ++i) {
    if (vColors[i] == -1) {
      q.push(i);
      vColors[i] = 0;

      while (!q.empty()) {
        const Int uIdx = q.front();
        q.pop();
        
        for (const Int vIdx : vvAdjs[uIdx]) {
          if (vIdx == uIdx) {
            spdlog::error("[PlaceMgr] Symmetry constraints invalid: self-loop on cell {}", _cir.cell(uIdx).name());
            exit(0);
          }
          if (vColors[vIdx] == -1) {
            vColors[vIdx] = vColors[uIdx] ^ (0x1);
            q.push(vIdx);
          }
          else if (vColors[vIdx] == vColors[uIdx]) {
            spdlog::error("[PlaceMgr] Symmetry constraints conflicts: {} {} orientation infeasible", _cir.cell(uIdx).name(), _cir.cell(vIdx).name());
            exit(0);
          }
        }
      }
    }
  }
  // set orients by colors
  PrePlaceCstr* pPrePlaceCstr;
  Cir_ForEachPrePlaceCstr(_cir, pPrePlaceCstr, i) {
    auto& cstr = *pPrePlaceCstr;
    for (j = 0; j < cstr.numCells(); ++j) {
      Cell& c = cstr.cell(j);
      if (cstr.hasCellOrient(j)) {
        c.setOrient(cstr.cellOrient(j));
      }
    }
  }
  for (size_t i = 0; i < vColors.size(); ++i) {
    Cell& c = _cir.cell(i);
    if (vColors[i] == 1) {
      c.flipX();
    }
  }

}

void PlaceMgr::setRowCellOrients()
{
  Int i, j;
  Region* pReg;
  Cell* pCell;

  // power row 
  Cir_ForEachRegion(_cir, pReg, i) {
    Reg_ForEachCell((*pReg), pCell, j) {
      Cell& c = *pCell;
      if (c.gridLoc().y() % 2) {
        c.flipY();
      }
    }
  }

}


PROJECT_NAMESPACE_END
