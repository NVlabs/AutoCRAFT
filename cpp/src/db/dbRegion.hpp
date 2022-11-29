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

#include "geo/box.hpp"
#include "dbGrid.hpp"
#include "dbCell.hpp"
#include "dbNet.hpp"
#include "cstr/dbPrePlaceCstr.hpp"
#include "cstr/dbPlaceSymCstr.hpp"
#include "cstr/dbPlaceArrayCstr.hpp"
#include "cstr/dbPlaceExtCstr.hpp"
#include "cstr/dbPlaceEdgeDistCstr.hpp"
#include "cstr/dbRouteSymCstr.hpp"
#include "cstr/dbNetPrioCstr.hpp"

PROJECT_NAMESPACE_START
class CirDB;

class Region {
  friend class Parser;

public:
  Region() :
    //_pCir(nullptr),
    _name(""), _idx(-1) {}
  ~Region() {}

  //CirDB&                                    cir()                                             { return *_pCir; }
  //const CirDB&                              cir()                                       const { return *_pCir; }
  //CirDB*                                    pCir()                                            { return _pCir; }
  //const CirDB*                              pCir()                                      const { return _pCir; }
  const String&                             name()                                      const { return _name; }
  Int                                       idx()                                       const { return _idx; }
  
  const PlaceGrid&                          placeGrid()                                 const { return *_pPlaceGrid; }
  const PlaceGrid*                          pPlaceGrid()                                const { return _pPlaceGrid; }

  const RouteGrid&                          routeGrid(const Int i)                      const { return *_vpRouteGrids.at(i); }
  const RouteGrid*                          pRouteGrid(const Int i)                     const { return _vpRouteGrids.at(i); }
  const Vector<const RouteGrid*>&           vpRouteGrids()                              const { return _vpRouteGrids; }
  Int                                       numRouteGrids()                             const { return _vpRouteGrids.size(); }
  
  const RouteGrid&                          powerRouteGrid(const Int i)                 const { return *_vpPowerRouteGrids.at(i); }
  const RouteGrid*                          pPowerRouteGrid(const Int i)                const { return _vpPowerRouteGrids.at(i); }
  const Vector<const RouteGrid*>&           vpPowerRouteGrids()                         const { return _vpPowerRouteGrids; }
  Int                                       numPowerRouteGrids()                        const { return _vpPowerRouteGrids.size(); }

  Cell&                                     cell(const Int i)                                 { return *_vpCells.at(i); }
  const Cell&                               cell(const Int i)                           const { return *_vpCells.at(i); }
  Cell*                                     pCell(const Int i)                                { return _vpCells.at(i); }
  const Cell*                               pCell(const Int i)                          const { return _vpCells.at(i); }
  const Vector<Cell*>&                      vpCells()                                   const { return _vpCells; }
  Int                                       numCells()                                  const { return _vpCells.size(); }

  const Primitive&                          edgePrim(const Int i)                       const { return *_vpEdgePrims.at(i); }
  const Primitive&                          edgePrimEast()                              const { return *_vpEdgePrims.at(0); }
  const Primitive&                          edgePrimWest()                              const { return *_vpEdgePrims.at(1); }
  const Primitive&                          edgePrimSouth()                             const { return *_vpEdgePrims.at(2); }
  const Primitive&                          edgePrimNorth()                             const { return *_vpEdgePrims.at(3); }
  const Primitive&                          edgePrimNorthEast()                         const { return *_vpEdgePrims.at(4); }
  const Primitive&                          edgePrimNorthWest()                         const { return *_vpEdgePrims.at(5); }
  const Primitive&                          edgePrimSouthEast()                         const { return *_vpEdgePrims.at(6); }
  const Primitive&                          edgePrimSouthWest()                         const { return *_vpEdgePrims.at(7); }
  const Primitive*                          pEdgePrim(const Int i)                      const { return _vpEdgePrims.at(i); }
  const Primitive*                          pEdgePrimEast()                             const { return _vpEdgePrims.at(0); }
  const Primitive*                          pEdgePrimWest()                             const { return _vpEdgePrims.at(1); }
  const Primitive*                          pEdgePrimSouth()                            const { return _vpEdgePrims.at(2); }
  const Primitive*                          pEdgePrimNorth()                            const { return _vpEdgePrims.at(3); }
  const Primitive*                          pEdgePrimNorthEast()                        const { return _vpEdgePrims.at(4); }
  const Primitive*                          pEdgePrimNorthWest()                        const { return _vpEdgePrims.at(5); }
  const Primitive*                          pEdgePrimSouthEast()                        const { return _vpEdgePrims.at(6); }
  const Primitive*                          pEdgePrimSouthWest()                        const { return _vpEdgePrims.at(7); }
  Orient2dE                                 edgePrimOrient(const Int i)                 const { return _vEdgePrimOrients.at(i); }
  Orient2dE                                 edgePrimEastOrient()                        const { return _vEdgePrimOrients.at(0); }
  Orient2dE                                 edgePrimWestOrient()                        const { return _vEdgePrimOrients.at(1); }
  Orient2dE                                 edgePrimSouthOrient()                       const { return _vEdgePrimOrients.at(2); }
  Orient2dE                                 edgePrimNorthOrient()                       const { return _vEdgePrimOrients.at(3); }
  Orient2dE                                 edgePrimNorthEastOrient()                   const { return _vEdgePrimOrients.at(4); }
  Orient2dE                                 edgePrimNorthWestOrient()                   const { return _vEdgePrimOrients.at(5); }
  Orient2dE                                 edgePrimSouthEastOrient()                   const { return _vEdgePrimOrients.at(6); }
  Orient2dE                                 edgePrimSouthWestOrient()                   const { return _vEdgePrimOrients.at(7); }

  const Primitive&                          dummyPrim(const Int i)                      const { return *_vpDummyPrims.at(i); }
  const Primitive*                          pDummyPrim(const Int i)                     const { return _vpDummyPrims.at(i); }
  const Vector<const Primitive*>&           vpDummyPrims()                              const { return _vpDummyPrims; }
  Int                                       numDummyPrims()                             const { return _vpDummyPrims.size(); }

  Net&                                      net(const Int i)                                  { return *_vpNets.at(i); }
  const Net&                                net(const Int i)                            const { return *_vpNets.at(i); }
  Net*                                      pNet(const Int i)                                 { return _vpNets.at(i); }
  const Net*                                pNet(const Int i)                           const { return _vpNets.at(i); }
  const Vector<Net*>&                       vpNets()                                    const { return _vpNets; }
  Int                                       numNets()                                   const { return _vpNets.size(); }
  Int                                       numSignalNets()                             const { return _vpNets.size() - _vpPowerNets.size(); }

  Net&                                      powerNet(const Int i)                             { return *_vpPowerNets.at(i); }
  const Net&                                powerNet(const Int i)                       const { return *_vpPowerNets.at(i); }
  Net*                                      pPowerNet(const Int i)                            { return _vpPowerNets.at(i); }
  const Net*                                pPowerNet(const Int i)                      const { return _vpPowerNets.at(i); }
  const Vector<Net*>&                       vpPowerNets()                               const { return _vpPowerNets; }
  Int                                       numPowerNets()                              const { return _vpPowerNets.size(); }
  
  //const PrePlaceCstr&                       prePlaceCstr(const Int i)                   const { return *_vpPrePlaceCstrs.at(i); }
  //const PrePlaceCstr*                       pPrePlaceCstr(const Int i)                  const { return _vpPrePlaceCstrs.at(i); }
  //Int                                       numPrePlaceCstrs()                          const { return _vpPrePlaceCstrs.size(); }

  const PlaceSymCstr&                       placeSymCstr(const Int i)                   const { return *_vpPlaceSymCstrs.at(i); }
  const PlaceSymCstr*                       pPlaceSymCstr(const Int i)                  const { return _vpPlaceSymCstrs.at(i); }
  Int                                       numPlaceSymCstrs()                          const { return _vpPlaceSymCstrs.size(); }

  const PlaceArrayCstr&                     placeArrayCstr(const Int i)                 const { return *_vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr*                     pPlaceArrayCstr(const Int i)                const { return _vpPlaceArrayCstrs.at(i); }
  Int                                       numPlaceArrayCstrs()                        const { return _vpPlaceArrayCstrs.size(); }
  
  const PlaceExtCstr&                       placeExtCstr(const Int i)                   const { return *_vpPlaceExtCstrs.at(i); }
  const PlaceExtCstr*                       pPlaceExtCstr(const Int i)                  const { return _vpPlaceExtCstrs.at(i); }
  Int                                       numPlaceExtCstrs()                          const { return _vpPlaceExtCstrs.size(); }
  
  const PlaceEdgeDistCstr&                  placeEdgeDistCstr(const Int i)              const { return *_vpPlaceEdgeDistCstrs.at(i); }
  const PlaceEdgeDistCstr*                  pPlaceEdgeDistCstr(const Int i)             const { return _vpPlaceEdgeDistCstrs.at(i); }
  Int                                       numPlaceEdgeDistCstrs()                     const { return _vpPlaceEdgeDistCstrs.size(); }
  
  const RouteSymCstr&                       routeSymCstr(const Int i)                   const { return *_vpRouteSymCstrs.at(i); }
  const RouteSymCstr*                       pRouteSymCstr(const Int i)                  const { return _vpRouteSymCstrs.at(i); }
  Int                                       numRouteSymCstrs()                          const { return _vpRouteSymCstrs.size(); }
  
  const NetPrioCstr&                        netPrioCstr(const Int i)                    const { return *_vpNetPrioCstrs.at(i); }
  const NetPrioCstr*                        pNetPrioCstr(const Int i)                   const { return _vpNetPrioCstrs.at(i); }
  Int                                       numNetPrioCstrs()                           const { return _vpNetPrioCstrs.size(); }

  Box<Int>&                                 bboxInner()                                       { return _bboxInner; }
  const Box<Int>&                           bboxInner()                                 const { return _bboxInner; }
  Box<Int>&                                 bboxOuter()                                       { return _bboxOuter; }
  const Box<Int>&                           bboxOuter()                                 const { return _bboxOuter; }
 
  Vector<Cell*>&                            vpCellGroup(const Int i)                          { return _vvpCellGroups.at(i); }
  const Vector<Cell*>&                      vpCellGroup(const Int i)                    const { return _vvpCellGroups.at(i); }
  Vector<Vector<Cell*>>&                    vvpCellGroups()                                   { return _vvpCellGroups; }
  const Vector<Vector<Cell*>>&              vvpCellGroups()                             const { return _vvpCellGroups; }
  FlatHashMap<Pair<Net*, Net*>, Int>&       mPowerNets2CellGroupIdx()                         { return _mPowerNets2CellGroupIdx; }
  const FlatHashMap<Pair<Net*, Net*>, Int>& mPowerNets2CellGroupIdx()                   const { return _mPowerNets2CellGroupIdx; }
  Int                                       numCellGroups()                             const { return _vvpCellGroups.size(); }
  void                                      vCellGroupIds(const Int i, Vector<Int>& v)  const { v.clear(); for (const Cell* p : _vvpCellGroups.at(i)) { v.emplace_back(p->idx()); } }
  void                                      vvCellGroupIds(Vector<Vector<Int>>& v)      const { v.clear(); v.resize(_vvpCellGroups.size()); for (size_t i = 0; i < v.size(); ++i) { for (const Cell* p: _vvpCellGroups.at(i)) v[i].emplace_back(p->idx()); } }
 
  Cell&                                     edgeCell(const Int i)                             { return _vEdgeCells.at(i); }
  const Cell&                               edgeCell(const Int i)                       const { return _vEdgeCells.at(i); }
  Cell*                                     pEdgeCell(const Int i)                            { return &_vEdgeCells.at(i); }
  const Cell*                               pEdgeCell(const Int i)                      const { return &_vEdgeCells.at(i); }
  Vector<Cell>&                             vEdgeCells()                                      { return _vEdgeCells; }
  const Vector<Cell>&                       vEdgeCells()                                const { return _vEdgeCells; }
  Int                                       numEdgeCells()                              const { return _vEdgeCells.size(); }

  Cell&                                     dummyCell(const Int i)                            { return _vDummyCells.at(i); } 
  const Cell&                               dummyCell(const Int i)                      const { return _vDummyCells.at(i); } 
  Cell*                                     pDummyCell(const Int i)                           { return &_vDummyCells.at(i); } 
  const Cell*                               pDummyCell(const Int i)                     const { return &_vDummyCells.at(i); } 
  Vector<Cell>&                             vDummyCells()                                     { return _vDummyCells; }
  const Vector<Cell>&                       vDummyCells()                               const { return _vDummyCells; }
  Int                                       numDummyCells()                             const { return _vDummyCells.size(); }
 
  // funcs 
  void setBboxInner(const Int xl, const Int yl, const Int xh, const Int yh) { _bboxInner.set(xl, yl, xh, yh); }
  void setBboxOuter(const Int xl, const Int yl, const Int xh, const Int yh) { _bboxOuter.set(xl, yl, xh, yh); }
  void addEdgeCell() { _vEdgeCells.emplace_back(); }
  void addDummyCell() { _vDummyCells.emplace_back(); }

private:
  //CirDB*                                    _pCir;
  String                                    _name;
  Int                                       _idx;
  const PlaceGrid*                          _pPlaceGrid;
  Vector<const RouteGrid*>                  _vpRouteGrids;
  Vector<const RouteGrid*>                  _vpPowerRouteGrids;
  Vector<Cell*>                             _vpCells; // all cells in this region
  Vector<Net*>                              _vpNets; // all nets in this region
  Vector<Net*>                              _vpPowerNets;
  
  Vector<Vector<Cell*>>                     _vvpCellGroups; // organized by power nets
  FlatHashMap<Pair<Net*, Net*>, Int>        _mPowerNets2CellGroupIdx;

  Vector<const Primitive*>                  _vpEdgePrims; // east, west, south, north, northEast, northWest, southEast, southWest
  Vector<Orient2dE>                         _vEdgePrimOrients;
 
  Vector<const Primitive*>                  _vpDummyPrims;

  //Vector<const PrePlaceCstr*>               _vpPrePlaceCstrs;
  Vector<const PlaceSymCstr*>               _vpPlaceSymCstrs;
  Vector<const PlaceArrayCstr*>             _vpPlaceArrayCstrs;
  Vector<const PlaceExtCstr*>               _vpPlaceExtCstrs;
  Vector<const PlaceEdgeDistCstr*>          _vpPlaceEdgeDistCstrs;
  Vector<const RouteSymCstr*>               _vpRouteSymCstrs;
  Vector<const NetPrioCstr*>                _vpNetPrioCstrs;

  Vector<Cell>                              _vEdgeCells; // edge cell instances
  Vector<Cell>                              _vDummyCells; // dummy cell instances

  Box<Int>                                  _bboxInner; // w/o edge cells
  Box<Int>                                  _bboxOuter; // w/ edge cells
  
};

#define Reg_ForEachRouteGrid(reg, pRouteGrid_, i) \
  for (i = 0; i < reg.numRouteGrids() and (pRouteGrid_ = reg.pRouteGrid(i)); ++i)

#define Reg_ForEachEdgePrim(reg, pEdgePrim_, i) \
  for (i = 0; i < reg.numEdgePrims() and (pEdgePrim_ = reg.pEdgePrim(i)); ++i)

#define Reg_ForEachDummyPrim(reg, pDummyPrim_, i) \
  for (i = 0; i < reg.numDummyPrims() and (pDummyPrim_ = reg.pDummyPrim(i)); ++i)

#define Reg_ForEachCell(reg, pCell_, i) \
  for (i = 0; i < reg.numCells() and (pCell_ = reg.pCell(i)); ++i)

#define Reg_ForEachNet(reg, pNet_, i) \
  for (i = 0; i < reg.numNets() and (pNet_ = reg.pNet(i)); ++i)

#define Reg_ForEachPowerNet(reg, pPowerNet_, i) \
  for (i = 0; i < reg.numPowerNets() and (pPowerNet_ = reg.pPowerNet(i)); ++i)

#define Reg_ForEachPlaceSymCstr(reg, pPlaceSymCstr_, i) \
  for (i = 0; i < reg.numPlaceSymCstrs() and (pPlaceSymCstr_ = reg.pPlaceSymCstr(i)); ++i)

#define Reg_ForEachPlaceArrayCstr(reg, pPlaceArrayCstr_, i) \
  for (i = 0; i < reg.numPlaceArrayCstrs() and (pPlaceArrayCstr_ = reg.pPlaceArrayCstr(i)); ++i)

#define Reg_ForEachPlaceExtCstr(reg, pPlaceExtCstr_, i) \
  for (i = 0; i < reg.numPlaceExtCstrs() and (pPlaceExtCstr_ = reg.pPlaceExtCstr(i)); ++i)

#define Reg_ForEachPlaceEdgeDistCstr(reg, pPlaceEdgeDistCstr_, i) \
  for (i = 0; i < reg.numPlaceEdgeDistCstrs() and (pPlaceEdgeDistCstr_ = reg.pPlaceEdgeDistCstr(i)); ++i)

#define Reg_ForEachRouteSymCstr(reg, pRouteSymCstr_, i) \
  for (i = 0; i < reg.numRouteSymCstrs() and (pRouteSymCstr_ = reg.pRouteSymCstr(i)); ++i)

#define Reg_ForEachEdgeCell(reg, pCell_, i) \
  for (i = 0; i < reg.numEdgeCells() and (pCell_ = reg.pEdgeCell(i)); ++i)

#define Reg_ForEachDummyCell(reg, pCell_, i) \
  for (i = 0; i < reg.numDummyCells() and (pCell_ = reg.pDummyCell(i)); ++i)

PROJECT_NAMESPACE_END
