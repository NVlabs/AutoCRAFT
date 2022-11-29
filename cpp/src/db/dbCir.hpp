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

#include "dbGrid.hpp"
#include "dbLayer.hpp"
#include "dbCell.hpp"
#include "dbNet.hpp"
#include "dbVia.hpp"
#include "dbRegion.hpp"
#include "geo/spatial.hpp"
#include "cstr/dbPlaceClusterCstr.hpp"
#include "cstr/dbPlaceOrderCstr.hpp"
#include "cstr/dbPlaceAlignCstr.hpp"
#include "cstr/dbPlaceDisjointCstr.hpp"
#include "cstr/dbPlaceRowCstr.hpp"
#include "cstr/dbRoutePathMatchCstr.hpp"

PROJECT_NAMESPACE_START

class CirDB {
  friend class Parser;
  friend class LefReader;

public:
  CirDB() : _physRes(0) {}
  ~CirDB();

  ////////////////////////////////
  //          Basics            //
  ////////////////////////////////
  const String&                       name()                                        const { return _name; }
  const Real                          physRes()                                     const { return _physRes; }
  
  Box<Int>&                           bbox()                                              { return _bbox; }
  const Box<Int>&                     bbox()                                        const { return _bbox; }
  Box<Int>&                           routingBbox()                                       { return _routingBbox; }
  const Box<Int>&                     routingBbox()                                 const { return _routingBbox; }
  
  const PlaceGrid&                    defaultPlaceGrid()                            const { return *_vpPlaceGrids.at(_defaultPlaceGridIdx); }
  const PlaceGrid&                    placeGrid(const Int i)                        const { return *_vpPlaceGrids.at(i); }
  const PlaceGrid&                    placeGrid(const String& n)                    const { return *_vpPlaceGrids.at(name2PlaceGridIdx(n)); }
  const PlaceGrid*                    pDefaGultPlaceGrid()                          const { return _vpPlaceGrids.at(_defaultPlaceGridIdx); }
  const PlaceGrid*                    pPlaceGrid(const Int i)                       const { return _vpPlaceGrids.at(i); }
  const PlaceGrid*                    pPlaceGrid(const String& n)                   const { return _vpPlaceGrids.at(name2PlaceGridIdx(n)); }
  Int                                 name2PlaceGridIdx(const String& n)            const { return _mPlaceGridName2Idx.at(n); }
  Int                                 numPlaceGrids()                               const { return _vpPlaceGrids.size(); }
  bool                                hasPlaceGrid(const String& n)                 const { return _mPlaceGridName2Idx.find(n) != _mPlaceGridName2Idx.end(); }
  const Vector<PlaceGrid*>&           vpPlaceGrids()                                const { return _vpPlaceGrids; }

  // signal route grids
  const RouteGrid&                    defaultRouteGrid(const Int i)                 const { return *_vpRouteGrids.at(_vDefaultRouteGridIds.at(i)); }
  const RouteGrid&                    routeGrid(const Int i)                        const { return *_vpRouteGrids.at(i); }
  const RouteGrid&                    routeGrid(const String& n)                    const { return *_vpRouteGrids.at(name2RouteGridIdx(n)); }
  const RouteGrid*                    pDefaultRouteGrid(const Int i)                const { return _vpRouteGrids.at(_vDefaultRouteGridIds.at(i)); }
  const RouteGrid*                    pRouteGrid(const Int i)                       const { return _vpRouteGrids.at(i); }
  const RouteGrid*                    pRouteGrid(const String& n)                   const { return _vpRouteGrids.at(name2RouteGridIdx(n)); }
  Int                                 name2RouteGridIdx(const String& n)            const { return _mRouteGridName2Idx.at(n); }
  Int                                 numDefaultRouteGrids()                        const { return _vDefaultRouteGridIds.size(); }
  Int                                 numRouteGrids()                               const { return _vpRouteGrids.size(); }
  bool                                hasRouteGrid(const String& n)                 const { return _mRouteGridName2Idx.find(n) != _mRouteGridName2Idx.end(); }
  const Vector<RouteGrid*>&           vpRouteGrids()                                const { return _vpRouteGrids; }
  
  // power route grids
  const RouteGrid&                    defaultPowerRouteGrid(const Int i)            const { return *_vpPowerRouteGrids.at(_vDefaultPowerRouteGridIds.at(i)); }
  const RouteGrid&                    powerRouteGrid(const Int i)                   const { return *_vpPowerRouteGrids.at(i); }
  const RouteGrid&                    powerRouteGrid(const String& n)               const { return *_vpPowerRouteGrids.at(name2PowerRouteGridIdx(n)); }
  const RouteGrid*                    pDefaultPowerRouteGrid(const Int i)           const { return _vpPowerRouteGrids.at(_vDefaultPowerRouteGridIds.at(i)); }
  const RouteGrid*                    pPowerRouteGrid(const Int i)                  const { return _vpPowerRouteGrids.at(i); }
  const RouteGrid*                    pPowerRouteGrid(const String& n)              const { return _vpPowerRouteGrids.at(name2PowerRouteGridIdx(n)); }
  Int                                 name2PowerRouteGridIdx(const String& n)       const { return _mPowerRouteGridName2Idx.at(n); }
  Int                                 numDefaultPowerRouteGrids()                   const { return _vDefaultPowerRouteGridIds.size(); }
  Int                                 numPowerRouteGrids()                          const { return _vpPowerRouteGrids.size(); }
  bool                                hasPowerRouteGrid(const String& n)            const { return _mPowerRouteGridName2Idx.find(n) != _mPowerRouteGridName2Idx.end(); }
  const Vector<RouteGrid*>&           vpPowerRouteGrids()                           const { return _vpPowerRouteGrids; }
  


  ////////////////////////////////    
  //          Layers            //    
  ////////////////////////////////
  const Layer&                        layer(const Int i)                            const { return *_vpLayers.at(i); }
  const Layer&                        layer(const String& n)                        const { return *_vpLayers.at(name2LayerIdx(n)); }
  const Layer*                        pLayer(const Int i)                           const { return _vpLayers.at(i); }
  const Layer*                        pLayer(const String& n)                       const { return _vpLayers.at(name2LayerIdx(n)); }
  Int                                 name2LayerIdx(const String& n)                const { return _mLayerName2Idx.at(n); }
  Int                                 numLayers()                                   const { return _vpLayers.size(); }
  bool                                hasLayer(const String& n)                     const { return _mLayerName2Idx.find(n) != _mLayerName2Idx.end(); }
  const Vector<Layer*>&               vpLayers()                                    const { return _vpLayers;  }

  const MetalLayer&                   metalLayer(const Int i)                       const { return *_vpMetalLayers.at(i); }
  const MetalLayer&                   metalLayer(const String& n)                   const { return *_vpMetalLayers.at(name2MetalLayerIdx(n)); }
  const MetalLayer*                   pMetalLayer(const Int i)                      const { return _vpMetalLayers.at(i); }
  const MetalLayer*                   pMetalLayer(const String& n)                  const { return _vpMetalLayers.at(name2MetalLayerIdx(n)); }
  Int                                 name2MetalLayerIdx(const String& n)           const { return _mMetalLayerName2Idx.at(n); }
  Int                                 numMetalLayers()                              const { return _vpMetalLayers.size(); }
  bool                                hasMetalLayer(const String& n)                const { return _mMetalLayerName2Idx.find(n) != _mMetalLayerName2Idx.end(); }
  const Vector<MetalLayer*>&          vpMetalLayers()                               const { return _vpMetalLayers; }

  const CutLayer&                     cutLayer(const Int i)                         const { return *_vpCutLayers.at(i); }
  const CutLayer&                     cutLayer(const String& n)                     const { return *_vpCutLayers.at(name2CutLayerIdx(n)); }
  const CutLayer*                     pCutLayer(const Int i)                        const { return _vpCutLayers.at(i); }
  const CutLayer*                     pCutLayer(const String& n)                    const { return _vpCutLayers.at(name2CutLayerIdx(n)); }
  Int                                 name2CutLayerIdx(const String& n)             const { return _mCutLayerName2Idx.at(n); }
  Int                                 numCutLayers()                                const { return _vpCutLayers.size(); }
  bool                                hasCutLayer(const String& n)                  const { return _mCutLayerName2Idx.find(n) != _mCutLayerName2Idx.end(); }
  const Vector<CutLayer*>&            vpCutLayers()                                 const { return _vpCutLayers; }

  ////////////////////////////////    
  //        Primitives          //    
  ////////////////////////////////    
  const Primitive&                    prim(const Int i)                             const { return *_vpPrims.at(i); }
  const Primitive&                    prim(const String& n)                         const { return *_vpPrims.at(name2PrimIdx(n)); }
  const Primitive*                    pPrim(const Int i)                            const { return _vpPrims.at(i); }
  const Primitive*                    pPrim(const String& n)                        const { return _vpPrims.at(name2PrimIdx(n)); }
  Int                                 name2PrimIdx(const String& n)                 const { return _mPrimName2Idx.at(n); }
  Int                                 numPrims()                                    const { return _vpPrims.size(); }
  bool                                hasPrim(const String& n)                      const { return _mPrimName2Idx.find(n) != _mPrimName2Idx.end(); }
  const Vector<Primitive*>&           vpPrims()                                     const { return _vpPrims; }
  

  ////////////////////////////////    
  //         Dev Maps           //    
  ////////////////////////////////    
  const DevMap&                       devMap(const Int i)                           const { return *_vpDevMaps.at(i); }
  const DevMap&                       devMap(const String& n)                       const { return *_vpDevMaps.at(name2DevMapIdx(n)); }
  const DevMap*                       pDevMap(const Int i)                          const { return _vpDevMaps.at(i); }
  const DevMap*                       pDevMap(const String& n)                      const { return _vpDevMaps.at(name2DevMapIdx(n)); }
  Int                                 name2DevMapIdx(const String& n)               const { return _mDevName2Idx.at(n); }
  Int                                 numDevMaps()                                  const { return _vpDevMaps.size(); }
  bool                                hasDevMap(const String& n)                    const { return _mDevName2Idx.find(n) != _mDevName2Idx.end(); }
  const Vector<DevMap*>&              vpDevMaps()                                   const { return _vpDevMaps; }
  
  ////////////////////////////////    
  //         Cells              //    
  ////////////////////////////////    
  Cell&                               cell(const Int i)                                   { return *_vpCells.at(i); }
  Cell&                               cell(const String& n)                               { return *_vpCells.at(name2CellIdx(n)); }
  const Cell&                         cell(const Int i)                             const { return *_vpCells.at(i); }
  const Cell&                         cell(const String& n)                         const { return *_vpCells.at(name2CellIdx(n)); }
  Cell*                               pCell(const Int i)                                  { return _vpCells.at(i); }
  Cell*                               pCell(const String& n)                              { return _vpCells.at(name2CellIdx(n)); }
  const Cell*                         pCell(const Int i)                            const { return _vpCells.at(i); }
  const Cell*                         pCell(const String& n)                        const { return _vpCells.at(name2CellIdx(n)); }
  
  Int                                 name2CellIdx(const String& n)                 const { return _mCellName2Idx.at(n); }
  Vector<Cell*>&                      vpCells()                                           { return _vpCells; }    
  const Vector<Cell*>&                vpCells()                                     const { return _vpCells; }    
  bool                                hasCell(const String& n)                      const { return _mCellName2Idx.find(n) != _mCellName2Idx.end(); }
  Int                                 numCells()                                    const { return _vpCells.size(); }
  
  const Vector<Int>&                  vSubmoduleCellIds(const String& n)            const { return _mSubmoduleName2Ids.at(n); }
  bool                                hasSubmodule(const String& n)                 const { return _mSubmoduleName2Ids.find(n) != _mSubmoduleName2Ids.end(); }
  Int                                 numSubmodules()                               const { return _mSubmoduleName2Ids.size(); }

  const Vector<Int>&                  vBaseCellIds(const String& n)                 const { return _mBaseCellName2Ids.at(n); }
  bool                                hasBaseCell(const String& n)                  const { return _mBaseCellName2Ids.find(n) != _mBaseCellName2Ids.end(); }
  Int                                 numBaseCells()                                const { return _mBaseCellName2Ids.size(); }

  ////////////////////////////////    
  //           Pins             //    
  ////////////////////////////////    
  Pin&                                pin(const Int i)                                    { return *_vpPins.at(i); }
  Pin&                                pin(const String& n)                                { return *_vpPins.at(name2PinIdx(n)); }
  const Pin&                          pin(const Int i)                              const { return *_vpPins.at(i); }
  const Pin&                          pin(const String& n)                          const { return *_vpPins.at(name2PinIdx(n)); }
  Pin*                                pPin(const Int i)                                   { return _vpPins.at(i); }
  Pin*                                pPin(const String& n)                               { return _vpPins.at(name2PinIdx(n)); }
  const Pin*                          pPin(const Int i)                             const { return _vpPins.at(i); }
  const Pin*                          pPin(const String& n)                         const { return _vpPins.at(name2PinIdx(n)); }
  Int                                 name2PinIdx(const String& n)                  const { return .at(n); }
  Int                                 numPins()                                     const { return _vpPins.size(); }
  Vector<Pin*>&                       vpPins()                                            { return _vpPins; }
  const Vector<Pin*>&                 vpPins()                                      const { return _vpPins; }
  bool                                hasPin(const String& n)                       const { return .find(n) != .end(); }

  ////////////////////////////////    
  //         Edge+Dummy         //    
  ////////////////////////////////    
  const Primitive&                    edgePrim(const Int i)                         const { return *_vpPrims.at(_vEdgePrimIds.at(i)); }
  const Primitive*                    pEdgePrim(const Int i)                        const { return _vpPrims.at(_vEdgePrimIds.at(i)); }
  Int                                 numEdgePrims()                                const { return _vEdgePrimIds.size(); }

  const Primitive&                    dummyPrim(const Int i)                        const { return *_vpPrims.at(_vDummyPrimIds.at(i)); }
  const Primitive*                    pDummyPrim(const Int i)                       const { return _vpPrims.at(_vDummyPrimIds.at(i)); }
  Int                                 numDummyPrims()                               const { return _vDummyPrimIds.size(); }
  
  ////////////////////////////////    
  //           Vias             //    
  ////////////////////////////////
  Via&                                via(const Int i)                                    { return *_vpVias.at(i); }
  Via&                                via(const String& n)                                { return *_vpVias.at(name2ViaIdx(n)); }
  const Via&                          via(const Int i)                              const { return *_vpVias.at(i); }
  const Via&                          via(const String& n)                          const { return *_vpVias.at(name2ViaIdx(n)); }
  Via*                                pVia(const Int i)                                   { return _vpVias.at(i); }
  Via*                                pVia(const String& n)                               { return _vpVias.at(name2ViaIdx(n)); }
  const Via*                          pVia(const Int i)                             const { return _vpVias.at(i); }
  const Via*                          pVia(const String& n)                         const { return _vpVias.at(name2ViaIdx(n)); }
  Int                                 name2ViaIdx(const String& n)                  const { return _mViaName2Idx.at(n); }
  Int                                 numVias()                                     const { return _vpVias.size(); }
  const Vector<Via*>&                 vpVias()                                      const { return _vpVias; }
  bool                                hasVia(const String& n)                       const { return _mViaName2Idx.find(n) != _mViaName2Idx.end(); }

  ////////////////////////////////    
  //           Nets             //    
  ////////////////////////////////    
  Net&                                net(const Int i)                                    { return *_vpNets.at(i); }
  Net&                                net(const String& n)                                { return *_vpNets.at(name2NetIdx(n)); }
  const Net&                          net(const Int i)                              const { return *_vpNets.at(i); }
  const Net&                          net(const String& n)                          const { return *_vpNets.at(name2NetIdx(n)); }
  Net*                                pNet(const Int i)                                   { return _vpNets.at(i); }
  Net*                                pNet(const String& n)                               { return _vpNets.at(name2NetIdx(n)); }
  const Net*                          pNet(const Int i)                             const { return _vpNets.at(i); }
  const Net*                          pNet(const String& n)                         const { return _vpNets.at(name2NetIdx(n)); }
  Int                                 name2NetIdx(const String& n)                  const { return _mNetName2Idx.at(n); }
  Int                                 numNets()                                     const { return _vpNets.size(); }
  const Vector<Net*>&                 vpNets()                                      const { return _vpNets; } 
  bool                                hasNet(const String& n)                       const { return _mNetName2Idx.find(n) != _mNetName2Idx.end(); }
  
  
  ////////////////////////////////    
  //         IO Pins            //    
  ////////////////////////////////    
  Pin&                                ioPin(const Int i)                                  { return *_vpIOPins.at(i); }
  Pin&                                ioPin(const String& n)                              { return *_vpIOPins.at(name2IOPinIdx(n)); }
  const Pin&                          ioPin(const Int i)                            const { return *_vpIOPins.at(i); }
  const Pin&                          ioPin(const String& n)                        const { return *_vpIOPins.at(name2IOPinIdx(n)); }
  Pin*                                pIOPin(const Int i)                                 { return _vpIOPins.at(i); }
  Pin*                                pIOPin(const String& n)                             { return _vpIOPins.at(name2IOPinIdx(n)); }
  const Pin*                          pIOPin(const Int i)                           const { return _vpIOPins.at(i); }
  const Pin*                          pIOPin(const String& n)                       const { return _vpIOPins.at(name2IOPinIdx(n)); }
  Int                                 name2IOPinIdx(const String& n)                const { return _mIOPinName2Idx.at(n); }
  Int                                 numIOPins()                                   const { return _vpIOPins.size(); }
  const Vector<Pin*>&                 vpIOPins()                                    const { return _vpIOPins; }
  bool                                hasIOPin(const String& n)                     const { return _mIOPinName2Idx.find(n) != _mIOPinName2Idx.end(); }
  IOPinLocE                           ioPinLoc(const Int i)                         const { return _vIOPinLocs.at(i); }
  Box<Int>&                           ioPinBbox(const Int i)                              { return _vIOPinBboxes.at(i); }
  const Box<Int>&                     ioPinBbox(const Int i)                        const { return _vIOPinBboxes.at(i); }
  
  Pin&                                ioPinL(const Int i)                                 { return *_vpIOPins.at(_vLeftIOPinIds.at(i)); }               
  const Pin&                          ioPinL(const Int i)                           const { return *_vpIOPins.at(_vLeftIOPinIds.at(i)); }               
  Pin&                                ioPinR(const Int i)                                 { return *_vpIOPins.at(_vRightIOPinIds.at(i)); }               
  const Pin&                          ioPinR(const Int i)                           const { return *_vpIOPins.at(_vRightIOPinIds.at(i)); }               
  Pin&                                ioPinB(const Int i)                                 { return *_vpIOPins.at(_vBottomIOPinIds.at(i)); }               
  const Pin&                          ioPinB(const Int i)                           const { return *_vpIOPins.at(_vBottomIOPinIds.at(i)); }               
  Pin&                                ioPinT(const Int i)                                 { return *_vpIOPins.at(_vTopIOPinIds.at(i)); }               
  const Pin&                          ioPinT(const Int i)                           const { return *_vpIOPins.at(_vTopIOPinIds.at(i)); }               
  Pin*                                pIOPinL(const Int i)                                { return _vpIOPins.at(_vLeftIOPinIds.at(i)); }               
  const Pin*                          pIOPinL(const Int i)                          const { return _vpIOPins.at(_vLeftIOPinIds.at(i)); }               
  Pin*                                pIOPinR(const Int i)                                { return _vpIOPins.at(_vRightIOPinIds.at(i)); }               
  const Pin*                          pIOPinR(const Int i)                          const { return _vpIOPins.at(_vRightIOPinIds.at(i)); }               
  Pin*                                pIOPinB(const Int i)                                { return _vpIOPins.at(_vBottomIOPinIds.at(i)); }               
  const Pin*                          pIOPinB(const Int i)                          const { return _vpIOPins.at(_vBottomIOPinIds.at(i)); }               
  Pin*                                pIOPinT(const Int i)                                { return _vpIOPins.at(_vTopIOPinIds.at(i)); }               
  const Pin*                          pIOPinT(const Int i)                          const { return _vpIOPins.at(_vTopIOPinIds.at(i)); }               
  Int                                 numLeftIOPins()                               const { return _vLeftIOPinIds.size(); }
  Int                                 numRightIOPins()                              const { return _vRightIOPinIds.size(); }
  Int                                 numBottomIOPins()                             const { return _vBottomIOPinIds.size(); }
  Int                                 numTopIOPins()                                const { return _vTopIOPinIds.size(); }

  ////////////////////////////////    
  //         Constraints        //    
  ////////////////////////////////    
  Region&                             region(const Int i)                                 { return *_vpRegions.at(i); }
  Region&                             region(const String& n)                             { return *_vpRegions.at(name2RegionIdx(n)); }
  const Region&                       region(const Int i)                           const { return *_vpRegions.at(i); }
  const Region&                       region(const String& n)                       const { return *_vpRegions.at(name2RegionIdx(n)); }
  Region*                             pRegion(const Int i)                                { return _vpRegions.at(i); }
  Region*                             pRegion(const String& n)                            { return _vpRegions.at(name2RegionIdx(n)); }
  const Region*                       pRegion(const Int i)                          const { return _vpRegions.at(i); }
  const Region*                       pRegion(const String& n)                      const { return _vpRegions.at(name2RegionIdx(n)); }
  Int                                 name2RegionIdx(const String& n)               const { return _mRegionName2Idx.at(n); }
  Int                                 numRegions()                                  const { return _vpRegions.size(); }
  const Vector<Region*>&              vpRegions()                                   const { return _vpRegions; }
  bool                                hasRegion(const String& n)                    const { return _mRegionName2Idx.find(n) != _mRegionName2Idx.end(); }

  PrePlaceCstr&                       prePlaceCstr(const Int i)                           { return *_vpPrePlaceCstrs.at(i); }
  PrePlaceCstr&                       prePlaceCstr(const String& n)                       { return *_vpPrePlaceCstrs.at(name2PrePlaceCstrIdx(n)); }
  const PrePlaceCstr&                 prePlaceCstr(const Int i)                     const { return *_vpPrePlaceCstrs.at(i); }
  const PrePlaceCstr&                 prePlaceCstr(const String& n)                 const { return *_vpPrePlaceCstrs.at(name2PrePlaceCstrIdx(n)); }
  PrePlaceCstr*                       pPrePlaceCstr(const Int i)                          { return _vpPrePlaceCstrs.at(i); }
  PrePlaceCstr*                       pPrePlaceCstr(const String& n)                      { return _vpPrePlaceCstrs.at(name2PrePlaceCstrIdx(n)); }
  const PrePlaceCstr*                 pPrePlaceCstr(const Int i)                    const { return _vpPrePlaceCstrs.at(i); }
  const PrePlaceCstr*                 pPrePlaceCstr(const String& n)                const { return _vpPrePlaceCstrs.at(name2PrePlaceCstrIdx(n)); }
  Int                                 name2PrePlaceCstrIdx(const String& n)         const { return _mPrePlaceCstrName2Idx.at(n); }
  Int                                 numPrePlaceCstrs()                            const { return _vpPrePlaceCstrs.size(); }
  bool                                hasPrePlaceCstr(const String& n)              const { return _mPrePlaceCstrName2Idx.find(n) != _mPrePlaceCstrName2Idx.end(); }
  Vector<PrePlaceCstr*>&              vpPrePlaceCstrs()                                   { return _vpPrePlaceCstrs; }
  const Vector<PrePlaceCstr*>&        vpPrePlaceCstrs()                             const { return _vpPrePlaceCstrs; }

  PlaceSymCstr&                       placeSymCstr(const Int i)                           { return *_vpPlaceSymCstrs.at(i); }
  PlaceSymCstr&                       placeSymCstr(const String& n)                       { return *_vpPlaceSymCstrs.at(name2PlaceSymCstrIdx(n)); }
  const PlaceSymCstr&                 placeSymCstr(const Int i)                     const { return *_vpPlaceSymCstrs.at(i); }
  const PlaceSymCstr&                 placeSymCstr(const String& n)                 const { return *_vpPlaceSymCstrs.at(name2PlaceSymCstrIdx(n)); }
  PlaceSymCstr*                       pPlaceSymCstr(const Int i)                          { return _vpPlaceSymCstrs.at(i); }
  PlaceSymCstr*                       pPlaceSymCstr(const String& n)                      { return _vpPlaceSymCstrs.at(name2PlaceSymCstrIdx(n)); }
  const PlaceSymCstr*                 pPlaceSymCstr(const Int i)                    const { return _vpPlaceSymCstrs.at(i); }
  const PlaceSymCstr*                 pPlaceSymCstr(const String& n)                const { return _vpPlaceSymCstrs.at(name2PlaceSymCstrIdx(n)); }
  Int                                 name2PlaceSymCstrIdx(const String& n)         const { return _mPlaceSymCstrName2Idx.at(n); }
  Int                                 numPlaceSymCstrs()                            const { return _vpPlaceSymCstrs.size(); }
  bool                                hasPlaceSymCstr(const String& n)              const { return _mPlaceSymCstrName2Idx.find(n) != _mPlaceSymCstrName2Idx.end(); }
  Vector<PlaceSymCstr*>&              vpPlaceSymCstrs()                                   { return _vpPlaceSymCstrs; }
  const Vector<PlaceSymCstr*>&        vpPlaceSymCstrs()                             const { return _vpPlaceSymCstrs; }

  PlaceArrayCstr&                     placeArrayCstr(const Int i)                         { return *_vpPlaceArrayCstrs.at(i); }
  PlaceArrayCstr&                     placeArrayCstr(const String& n)                     { return *_vpPlaceArrayCstrs.at(name2PlaceArrayCstrIdx(n)); }
  const PlaceArrayCstr&               placeArrayCstr(const Int i)                   const { return *_vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr&               placeArrayCstr(const String& n)               const { return *_vpPlaceArrayCstrs.at(name2PlaceArrayCstrIdx(n)); }
  PlaceArrayCstr*                     pPlaceArrayCstr(const Int i)                        { return _vpPlaceArrayCstrs.at(i); }
  PlaceArrayCstr*                     pPlaceArrayCstr(const String& n)                    { return _vpPlaceArrayCstrs.at(name2PlaceArrayCstrIdx(n)); }
  const PlaceArrayCstr*               pPlaceArrayCstr(const Int i)                  const { return _vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr*               pPlaceArrayCstr(const String& n)              const { return _vpPlaceArrayCstrs.at(name2PlaceArrayCstrIdx(n)); }
  Int                                 name2PlaceArrayCstrIdx(const String& n)       const { return _mPlaceArrayCstrName2Idx.at(n); }
  Int                                 numPlaceArrayCstrs()                          const { return _vpPlaceArrayCstrs.size(); }
  bool                                hasPlaceArrayCstr(const String& n)            const { return _mPlaceArrayCstrName2Idx.find(n) != _mPlaceArrayCstrName2Idx.end(); }
  Vector<PlaceArrayCstr*>&            vpPlaceArrayCstrs()                                 { return _vpPlaceArrayCstrs; }
  const Vector<PlaceArrayCstr*>&      vpPlaceArrayCstrs()                           const { return _vpPlaceArrayCstrs; }
  
  PlaceClusterCstr&                   placeClusterCstr(const Int i)                       { return *_vpPlaceClusterCstrs.at(i); }
  PlaceClusterCstr&                   placeClusterCstr(const String& n)                   { return *_vpPlaceClusterCstrs.at(name2PlaceClusterCstrIdx(n)); }
  const PlaceClusterCstr&             placeClusterCstr(const Int i)                 const { return *_vpPlaceClusterCstrs.at(i); }
  const PlaceClusterCstr&             placeClusterCstr(const String& n)             const { return *_vpPlaceClusterCstrs.at(name2PlaceClusterCstrIdx(n)); }
  PlaceClusterCstr*                   pPlaceClusterCstr(const Int i)                      { return _vpPlaceClusterCstrs.at(i); }
  PlaceClusterCstr*                   pPlaceClusterCstr(const String& n)                  { return _vpPlaceClusterCstrs.at(name2PlaceClusterCstrIdx(n)); }
  const PlaceClusterCstr*             pPlaceClusterCstr(const Int i)                const { return _vpPlaceClusterCstrs.at(i); }
  const PlaceClusterCstr*             pPlaceClusterCstr(const String& n)            const { return _vpPlaceClusterCstrs.at(name2PlaceClusterCstrIdx(n)); }
  Int                                 name2PlaceClusterCstrIdx(const String& n)     const { return _mPlaceClusterCstrName2Idx.at(n); }
  Int                                 numPlaceClusterCstrs()                        const { return _vpPlaceClusterCstrs.size(); }
  bool                                hasPlaceClusterCstr(const String& n)          const { return _mPlaceClusterCstrName2Idx.find(n) != _mPlaceClusterCstrName2Idx.end(); }
  Vector<PlaceClusterCstr*>&          vpPlaceClusterCstrs()                               { return _vpPlaceClusterCstrs; }
  const Vector<PlaceClusterCstr*>&    vpPlaceClusterCstrs()                         const { return _vpPlaceClusterCstrs; }
  
  PlaceExtCstr&                       placeExtCstr(const Int i)                           { return *_vpPlaceExtCstrs.at(i); }
  PlaceExtCstr&                       placeExtCstr(const String& n)                       { return *_vpPlaceExtCstrs.at(name2PlaceExtCstrIdx(n)); }
  const PlaceExtCstr&                 placeExtCstr(const Int i)                     const { return *_vpPlaceExtCstrs.at(i); }
  const PlaceExtCstr&                 placeExtCstr(const String& n)                 const { return *_vpPlaceExtCstrs.at(name2PlaceExtCstrIdx(n)); }
  PlaceExtCstr*                       pPlaceExtCstr(const Int i)                          { return _vpPlaceExtCstrs.at(i); }
  PlaceExtCstr*                       pPlaceExtCstr(const String& n)                      { return _vpPlaceExtCstrs.at(name2PlaceExtCstrIdx(n)); }
  const PlaceExtCstr*                 pPlaceExtCstr(const Int i)                    const { return _vpPlaceExtCstrs.at(i); }
  const PlaceExtCstr*                 pPlaceExtCstr(const String& n)                const { return _vpPlaceExtCstrs.at(name2PlaceExtCstrIdx(n)); }
  Int                                 name2PlaceExtCstrIdx(const String& n)         const { return _mPlaceExtCstrName2Idx.at(n); }
  Int                                 numPlaceExtCstrs()                            const { return _vpPlaceExtCstrs.size(); }
  bool                                hasPlaceExtCstr(const String& n)              const { return _mPlaceExtCstrName2Idx.find(n) != _mPlaceExtCstrName2Idx.end(); }
  Vector<PlaceExtCstr*>&              vpPlaceExtCstrs()                                   { return _vpPlaceExtCstrs; }
  const Vector<PlaceExtCstr*>&        vpPlaceExtCstrs()                             const { return _vpPlaceExtCstrs; }
  
  PlaceEdgeDistCstr&                  placeEdgeDistCstr(const Int i)                      { return *_vpPlaceEdgeDistCstrs.at(i); }
  PlaceEdgeDistCstr&                  placeEdgeDistCstr(const String& n)                  { return *_vpPlaceEdgeDistCstrs.at(name2PlaceEdgeDistCstrIdx(n)); }
  const PlaceEdgeDistCstr&            placeEdgeDistCstr(const Int i)                const { return *_vpPlaceEdgeDistCstrs.at(i); }
  const PlaceEdgeDistCstr&            placeEdgeDistCstr(const String& n)            const { return *_vpPlaceEdgeDistCstrs.at(name2PlaceEdgeDistCstrIdx(n)); }
  PlaceEdgeDistCstr*                  pPlaceEdgeDistCstr(const Int i)                     { return _vpPlaceEdgeDistCstrs.at(i); }
  PlaceEdgeDistCstr*                  pPlaceEdgeDistCstr(const String& n)                 { return _vpPlaceEdgeDistCstrs.at(name2PlaceEdgeDistCstrIdx(n)); }
  const PlaceEdgeDistCstr*            pPlaceEdgeDistCstr(const Int i)               const { return _vpPlaceEdgeDistCstrs.at(i); }
  const PlaceEdgeDistCstr*            pPlaceEdgeDistCstr(const String& n)           const { return _vpPlaceEdgeDistCstrs.at(name2PlaceEdgeDistCstrIdx(n)); }
  Int                                 name2PlaceEdgeDistCstrIdx(const String& n)    const { return _mPlaceEdgeDistCstrName2Idx.at(n); }
  Int                                 numPlaceEdgeDistCstrs()                       const { return _vpPlaceEdgeDistCstrs.size(); }
  bool                                hasPlaceEdgeDistCstr(const String& n)         const { return _mPlaceEdgeDistCstrName2Idx.find(n) != _mPlaceEdgeDistCstrName2Idx.end(); }
  Vector<PlaceEdgeDistCstr*>&         vpPlaceEdgeDistCstrs()                              { return _vpPlaceEdgeDistCstrs; }
  const Vector<PlaceEdgeDistCstr*>&   vpPlaceEdgeDistCstrs()                        const { return _vpPlaceEdgeDistCstrs; }
  
  PlaceOrderCstr&                     placeOrderCstr(const Int i)                         { return *_vpPlaceOrderCstrs.at(i); }
  PlaceOrderCstr&                     placeOrderCstr(const String& n)                     { return *_vpPlaceOrderCstrs.at(name2PlaceOrderCstrIdx(n)); }
  const PlaceOrderCstr&               placeOrderCstr(const Int i)                   const { return *_vpPlaceOrderCstrs.at(i); }
  const PlaceOrderCstr&               placeOrderCstr(const String& n)               const { return *_vpPlaceOrderCstrs.at(name2PlaceOrderCstrIdx(n)); }
  PlaceOrderCstr*                     pPlaceOrderCstr(const Int i)                        { return _vpPlaceOrderCstrs.at(i); }
  PlaceOrderCstr*                     pPlaceOrderCstr(const String& n)                    { return _vpPlaceOrderCstrs.at(name2PlaceOrderCstrIdx(n)); }
  const PlaceOrderCstr*               pPlaceOrderCstr(const Int i)                  const { return _vpPlaceOrderCstrs.at(i); }
  const PlaceOrderCstr*               pPlaceOrderCstr(const String& n)              const { return _vpPlaceOrderCstrs.at(name2PlaceOrderCstrIdx(n)); }
  Int                                 name2PlaceOrderCstrIdx(const String& n)       const { return _mPlaceOrderCstrName2Idx.at(n); }
  Int                                 numPlaceOrderCstrs()                          const { return _vpPlaceOrderCstrs.size(); }
  bool                                hasPlaceOrderCstr(const String& n)            const { return _mPlaceOrderCstrName2Idx.find(n) != _mPlaceOrderCstrName2Idx.end(); }
  Vector<PlaceOrderCstr*>&            vpPlaceOrderCstrs()                                 { return _vpPlaceOrderCstrs; }
  const Vector<PlaceOrderCstr*>&      vpPlaceOrderCstrs()                           const { return _vpPlaceOrderCstrs; }
  
  PlaceAlignCstr&                     placeAlignCstr(const Int i)                         { return *_vpPlaceAlignCstrs.at(i); }
  PlaceAlignCstr&                     placeAlignCstr(const String& n)                     { return *_vpPlaceAlignCstrs.at(name2PlaceAlignCstrIdx(n)); }
  const PlaceAlignCstr&               placeAlignCstr(const Int i)                   const { return *_vpPlaceAlignCstrs.at(i); }
  const PlaceAlignCstr&               placeAlignCstr(const String& n)               const { return *_vpPlaceAlignCstrs.at(name2PlaceAlignCstrIdx(n)); }
  PlaceAlignCstr*                     pPlaceAlignCstr(const Int i)                        { return _vpPlaceAlignCstrs.at(i); }
  PlaceAlignCstr*                     pPlaceAlignCstr(const String& n)                    { return _vpPlaceAlignCstrs.at(name2PlaceAlignCstrIdx(n)); }
  const PlaceAlignCstr*               pPlaceAlignCstr(const Int i)                  const { return _vpPlaceAlignCstrs.at(i); }
  const PlaceAlignCstr*               pPlaceAlignCstr(const String& n)              const { return _vpPlaceAlignCstrs.at(name2PlaceAlignCstrIdx(n)); }
  Int                                 name2PlaceAlignCstrIdx(const String& n)       const { return _mPlaceAlignCstrName2Idx.at(n); }
  Int                                 numPlaceAlignCstrs()                          const { return _vpPlaceAlignCstrs.size(); }
  bool                                hasPlaceAlignCstr(const String& n)            const { return _mPlaceAlignCstrName2Idx.find(n) != _mPlaceAlignCstrName2Idx.end(); }
  Vector<PlaceAlignCstr*>&            vpPlaceAlignCstrs()                                 { return _vpPlaceAlignCstrs; }
  const Vector<PlaceAlignCstr*>&      vpPlaceAlignCstrs()                           const { return _vpPlaceAlignCstrs; }
  
  PlaceDisjointCstr&                  placeDisjointCstr(const Int i)                      { return *_vpPlaceDisjointCstrs.at(i); }
  PlaceDisjointCstr&                  placeDisjointCstr(const String& n)                  { return *_vpPlaceDisjointCstrs.at(name2PlaceDisjointCstrIdx(n)); }
  const PlaceDisjointCstr&            placeDisjointCstr(const Int i)                const { return *_vpPlaceDisjointCstrs.at(i); }
  const PlaceDisjointCstr&            placeDisjointCstr(const String& n)            const { return *_vpPlaceDisjointCstrs.at(name2PlaceDisjointCstrIdx(n)); }
  PlaceDisjointCstr*                  pPlaceDisjointCstr(const Int i)                     { return _vpPlaceDisjointCstrs.at(i); }
  PlaceDisjointCstr*                  pPlaceDisjointCstr(const String& n)                 { return _vpPlaceDisjointCstrs.at(name2PlaceDisjointCstrIdx(n)); }
  const PlaceDisjointCstr*            pPlaceDisjointCstr(const Int i)               const { return _vpPlaceDisjointCstrs.at(i); }
  const PlaceDisjointCstr*            pPlaceDisjointCstr(const String& n)           const { return _vpPlaceDisjointCstrs.at(name2PlaceDisjointCstrIdx(n)); }
  Int                                 name2PlaceDisjointCstrIdx(const String& n)    const { return _mPlaceDisjointCstrName2Idx.at(n); }
  Int                                 numPlaceDisjointCstrs()                       const { return _vpPlaceDisjointCstrs.size(); }
  bool                                hasPlaceDisjointCstr(const String& n)         const { return _mPlaceDisjointCstrName2Idx.find(n) != _mPlaceDisjointCstrName2Idx.end(); }
  Vector<PlaceDisjointCstr*>&         vpPlaceDisjointCstrs()                              { return _vpPlaceDisjointCstrs; }
  const Vector<PlaceDisjointCstr*>&   vpPlaceDisjointCstrs()                        const { return _vpPlaceDisjointCstrs; }
  
  PlaceRowCstr&                       placeRowCstr(const Int i)                           { return *_vpPlaceRowCstrs.at(i); }
  PlaceRowCstr&                       placeRowCstr(const String& n)                       { return *_vpPlaceRowCstrs.at(name2PlaceRowCstrIdx(n)); }
  const PlaceRowCstr&                 placeRowCstr(const Int i)                     const { return *_vpPlaceRowCstrs.at(i); }
  const PlaceRowCstr&                 placeRowCstr(const String& n)                 const { return *_vpPlaceRowCstrs.at(name2PlaceRowCstrIdx(n)); }
  PlaceRowCstr*                       pPlaceRowCstr(const Int i)                          { return _vpPlaceRowCstrs.at(i); }
  PlaceRowCstr*                       pPlaceRowCstr(const String& n)                      { return _vpPlaceRowCstrs.at(name2PlaceRowCstrIdx(n)); }
  const PlaceRowCstr*                 pPlaceRowCstr(const Int i)                    const { return _vpPlaceRowCstrs.at(i); }
  const PlaceRowCstr*                 pPlaceRowCstr(const String& n)                const { return _vpPlaceRowCstrs.at(name2PlaceRowCstrIdx(n)); }
  Int                                 name2PlaceRowCstrIdx(const String& n)         const { return _mPlaceRowCstrName2Idx.at(n); }
  Int                                 numPlaceRowCstrs()                            const { return _vpPlaceRowCstrs.size(); }
  bool                                hasPlaceRowCstr(const String& n)              const { return _mPlaceRowCstrName2Idx.find(n) != _mPlaceRowCstrName2Idx.end(); }
  Vector<PlaceRowCstr*>&              vpPlaceRowCstrs()                                   { return _vpPlaceRowCstrs; }
  const Vector<PlaceRowCstr*>&        vpPlaceRowCstrs()                             const { return _vpPlaceRowCstrs; }
  
  RouteSymCstr&                       routeSymCstr(const Int i)                           { return *_vpRouteSymCstrs.at(i); }
  RouteSymCstr&                       routeSymCstr(const String& n)                       { return *_vpRouteSymCstrs.at(name2RouteSymCstrIdx(n)); }
  const RouteSymCstr&                 routeSymCstr(const Int i)                     const { return *_vpRouteSymCstrs.at(i); }
  const RouteSymCstr&                 routeSymCstr(const String& n)                 const { return *_vpRouteSymCstrs.at(name2RouteSymCstrIdx(n)); }
  RouteSymCstr*                       pRouteSymCstr(const Int i)                          { return _vpRouteSymCstrs.at(i); }
  RouteSymCstr*                       pRouteSymCstr(const String& n)                      { return _vpRouteSymCstrs.at(name2RouteSymCstrIdx(n)); }
  const RouteSymCstr*                 pRouteSymCstr(const Int i)                    const { return _vpRouteSymCstrs.at(i); }
  const RouteSymCstr*                 pRouteSymCstr(const String& n)                const { return _vpRouteSymCstrs.at(name2RouteSymCstrIdx(n)); }
  Int                                 name2RouteSymCstrIdx(const String& n)         const { return _mRouteSymCstrName2Idx.at(n); }
  Int                                 numRouteSymCstrs()                            const { return _vpRouteSymCstrs.size(); }
  bool                                hasRouteSymCstr(const String& n)              const { return _mRouteSymCstrName2Idx.find(n) != _mRouteSymCstrName2Idx.end(); }
  Vector<RouteSymCstr*>&              vpRouteSymCstrs()                                   { return _vpRouteSymCstrs; }
  const Vector<RouteSymCstr*>&        vpRouteSymCstrs()                             const { return _vpRouteSymCstrs; }
 
  RoutePathMatchCstr&                 routePathMatchCstr(const Int i)                     { return *_vpRoutePathMatchCstrs.at(i); }
  RoutePathMatchCstr&                 routePathMatchCstr(const String& n)                 { return *_vpRoutePathMatchCstrs.at(name2RoutePathMatchCstrIdx(n)); }
  const RoutePathMatchCstr&           routePathMatchCstr(const Int i)               const { return *_vpRoutePathMatchCstrs.at(i); }
  const RoutePathMatchCstr&           routePathMatchCstr(const String& n)           const { return *_vpRoutePathMatchCstrs.at(name2RoutePathMatchCstrIdx(n)); }
  RoutePathMatchCstr*                 pRoutePathMatchCstr(const Int i)                    { return _vpRoutePathMatchCstrs.at(i); }
  RoutePathMatchCstr*                 pRoutePathMatchCstr(const String& n)                { return _vpRoutePathMatchCstrs.at(name2RoutePathMatchCstrIdx(n)); }
  const RoutePathMatchCstr*           pRoutePathMatchCstr(const Int i)              const { return _vpRoutePathMatchCstrs.at(i); }
  const RoutePathMatchCstr*           pRoutePathMatchCstr(const String& n)          const { return _vpRoutePathMatchCstrs.at(name2RoutePathMatchCstrIdx(n)); }
  Int                                 name2RoutePathMatchCstrIdx(const String& n)   const { return _mRoutePathMatchCstrName2Idx.at(n); }
  Int                                 numRoutePathMatchCstrs()                      const { return _vpRoutePathMatchCstrs.size(); }
  bool                                hasRoutePathMatchCstr(const String& n)        const { return _mRoutePathMatchCstrName2Idx.find(n) != _mRoutePathMatchCstrName2Idx.end(); }
  Vector<RoutePathMatchCstr*>&        vpRoutePathMatchCstrs()                             { return _vpRoutePathMatchCstrs; }
  const Vector<RoutePathMatchCstr*>&  vpRoutePathMatchCstrs()                       const { return _vpRoutePathMatchCstrs; }
  
  NetPrioCstr&                        netPrioCstr(const Int i)                            { return *_vpNetPrioCstrs.at(i); }
  NetPrioCstr&                        netPrioCstr(const String& n)                        { return *_vpNetPrioCstrs.at(name2NetPrioCstrIdx(n)); }
  const NetPrioCstr&                  netPrioCstr(const Int i)                      const { return *_vpNetPrioCstrs.at(i); }
  const NetPrioCstr&                  netPrioCstr(const String& n)                  const { return *_vpNetPrioCstrs.at(name2NetPrioCstrIdx(n)); }
  NetPrioCstr*                        pNetPrioCstr(const Int i)                           { return _vpNetPrioCstrs.at(i); }
  NetPrioCstr*                        pNetPrioCstr(const String& n)                       { return _vpNetPrioCstrs.at(name2NetPrioCstrIdx(n)); }
  const NetPrioCstr*                  pNetPrioCstr(const Int i)                     const { return _vpNetPrioCstrs.at(i); }
  const NetPrioCstr*                  pNetPrioCstr(const String& n)                 const { return _vpNetPrioCstrs.at(name2NetPrioCstrIdx(n)); }
  Int                                 name2NetPrioCstrIdx(const String& n)          const { return _mNetPrioCstrName2Idx.at(n); }
  Int                                 numNetPrioCstrs()                             const { return _vpNetPrioCstrs.size(); }
  bool                                hasNetPrioCstr(const String& n)               const { return _mNetPrioCstrName2Idx.find(n) != _mNetPrioCstrName2Idx.end(); }
  Vector<NetPrioCstr*>&               vpNetPrioCstrs()                                    { return _vpNetPrioCstrs; }
  const Vector<NetPrioCstr*>&         vpNetPrioCstrs()                              const { return _vpNetPrioCstrs; }

  ////////////////////////////////    
  //      Spatial Structures    //    
  ////////////////////////////////
  Pin&                                pin(const Int l, const Int i)                       { return *_vvpPins.at(l).at(i); }
  const Pin&                          pin(const Int l, const Int i)                 const { return *_vvpPins.at(l).at(i); }
  Pin*                                pPin(const Int l, const Int i)                      { return _vvpPins.at(l).at(i); }
  const Pin*                          pPin(const Int l, const Int i)                const { return _vvpPins.at(l).at(i); }
  Vector<Pin*>&                       vpPins(const Int i)                                 { return _vvpPins.at(i); }
  const Vector<Pin*>&                 vpPins(const Int i)                           const { return _vvpPins.at(i); }
  Int                                 numPins(const Int i)                          const { return _vvpPins.at(i).size(); }
  SpatialMap<Int, Pin*>&              spatialPins(const Int i)                            { return _vSpatialPins.at(i); }
  const SpatialMap<Int, Pin*>&        spatialPins(const Int i)                      const { return _vSpatialPins.at(i); }

  Obs&                                obs(const Int l, const Int i)                       { return *_vvpObs.at(l).at(i); }
  const Obs&                          obs(const Int l, const Int i)                 const { return *_vvpObs.at(l).at(i); }
  Obs*                                pObs(const Int l, const Int i)                      { return _vvpObs.at(l).at(i); }
  const Obs*                          pObs(const Int l, const Int i)                const { return _vvpObs.at(l).at(i); }
  Vector<Obs*>&                       vpObs(const Int i)                                  { return _vvpObs.at(i); }
  const Vector<Obs*>&                 vpObs(const Int i)                            const { return _vvpObs.at(i); }
  Int                                 numObs(const Int i)                           const { return _vvpObs.at(i).size(); }
  SpatialMap<Int, Obs*>&              spatialObs(const Int i)                             { return _vSpatialObs.at(i); }
  const SpatialMap<Int, Obs*>&        spatialObs(const Int i)                       const { return _vSpatialObs.at(i); }

  SpatialMap<Int, Net*>&              spatialNets(const Int i)                            { return _vSpatialNets.at(i); }
  const SpatialMap<Int, Net*>&        spatialNets(const Int i)                      const { return _vSpatialNets.at(i); }
  
  ////////////////////////////////    
  //            WSP             //    
  ////////////////////////////////
  Pair<Box<Int>, Int>&                wspBbox(const Int i)                                { return _vWspBboxes.at(i); }
  const Pair<Box<Int>, Int>&          wspBbox(const Int i)                          const { return _vWspBboxes.at(i); }
  Vector<Pair<Box<Int>, Int>>&        vWspBboxes()                                        { return _vWspBboxes; }  
  const Vector<Pair<Box<Int>, Int>>&  vWspBboxes()                                  const { return _vWspBboxes; }  

  Pair<Box<Int>, Int>&                wspFillDrw(const Int i)                             { return _vWspFillDrws.at(i); }
  const Pair<Box<Int>, Int>&          wspFillDrw(const Int i)                       const { return _vWspFillDrws.at(i); }
  Vector<Pair<Box<Int>, Int>>&        vWspFillDrws()                                      { return _vWspFillDrws; }  
  const Vector<Pair<Box<Int>, Int>>&  vWspFillDrws()                                const { return _vWspFillDrws; }  

  // funcs
  void                                updateBbox();
  void                                groupCells();
  void                                initSpatials();
  void                                buildSpatialPinsNObs();
  void                                buildSpatialNets();
  void                                buildSpatialNet(const Net& net);
  void                                setIOPinBbox(const Int i, const Box<Int>& b);
  void                                genRoutingBbox();
  void                                genWsp();

  Real                                segRes(const Segment3d<Int>& seg, const TopoTree::SegData& sd) const;

  Int                                 pathLength(const Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  Int                                 pathVias(const Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  Pair<Int, Int>                      pathLengthVias(const Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  //Real                                pathRes(const Vector<Vector<Segment3d<Int>>>& vvSegs, const Vector<Vector<TopoTree::SegData>>& vvSegData) const;
  //Tuple<Int, Int, Real>               pathStats(const Vector<Vector<Segment3d<Int>>>& vvSegs, const Vector<Vector<TopoTree::SegData>>& vvSegData) const;
  Real                                pathRes(const Vector<Segment3d<Int>>& vSegs) const;
  Real                                pathRes(const Vector<Vector<Segment3d<Int>>>& vvSegs) const;
  Tuple<Int, Int, Real>               pathStats(const Vector<Segment3d<Int>>& vSegs) const;
  Tuple<Int, Int, Real>               pathStats(const Vector<Vector<Segment3d<Int>>>& vvSegs) const;

  Real                                netRes(const Net& net) const;
  Real                                pathRes(const Pin& sPin, const Pin& tPin) const;

  // debug
  void                                showInfo() const;
  void                                showPlaceHpwl(const bool useIO = true) const;
  void                                showRouteWL() const;
  void                                showNetSegs() const;
  void                                debug();

private:
  String                                _name;
  Real                                  _physRes; // physical resolution;

  Box<Int>                              _bbox;
  Box<Int>                              _routingBbox; // routing boundaries

  // Tech
  Vector<PlaceGrid*>                    _vpPlaceGrids; // all used place grids in this design (default + region)
  Vector<RouteGrid*>                    _vpRouteGrids; // all used route grids in this design (default + region)
  Vector<RouteGrid*>                    _vpPowerRouteGrids; // all used power route grids in this design (default + region)
  FlatHashMap<String, Int>              _mPlaceGridName2Idx;
  FlatHashMap<String, Int>              _mRouteGridName2Idx;
  FlatHashMap<String, Int>              _mPowerRouteGridName2Idx;
  
  Vector<Primitive*>                    _vpPrims; // all used primitive templates in this design (except edgecells and dummycells)
  Vector<DevMap*>                       _vpDevMaps; // device -> primitives mapping
  Vector<Cell*>                         _vpCells; // all cells for placement
  Vector<Pin*>                          _vpPins;
  FlatHashMap<String, Int>              _mPrimName2Idx;
  FlatHashMap<String, Int>              _mDevName2Idx;
  FlatHashMap<String, Int>              _mCellName2Idx;
  FlatHashMap<String, Vector<Int>>      _mSubmoduleName2Ids;
  FlatHashMap<String, Vector<Int>>      _mBaseCellName2Ids; // cell name without multi -> ids
  FlatHashMap<String, Int>              ;

  Vector<Int>                           _vEdgePrimIds;
  Vector<Int>                           _vDummyPrimIds;

  // Placement
  Int                                   _defaultPlaceGridIdx; // default placement grid
  Vector<Pin*>                          _vpIOPins; // placed IO pins
  Vector<IOPinLocE>                     _vIOPinLocs;
  Vector<Box<Int>>                      _vIOPinBboxes; // bound valid to generate io pin shapes
  FlatHashMap<String, Int>              _mIOPinName2Idx;
  Vector<Int>                           _vLeftIOPinIds;
  Vector<Int>                           _vRightIOPinIds;
  Vector<Int>                           _vBottomIOPinIds;
  Vector<Int>                           _vTopIOPinIds;

  // Routing
  Vector<Layer*>                        _vpLayers; // merged of all layer
  Vector<MetalLayer*>                   _vpMetalLayers;
  Vector<CutLayer*>                     _vpCutLayers;
  FlatHashMap<String, Int>              _mLayerName2Idx;
  FlatHashMap<String, Int>              _mMetalLayerName2Idx;
  FlatHashMap<String, Int>              _mCutLayerName2Idx;

  Vector<Int>                           _vDefaultRouteGridIds;
  Vector<Int>                           _vDefaultPowerRouteGridIds;

  Vector<Via*>                          _vpVias; // via templates
  FlatHashMap<String, Int>              _mViaName2Idx;

  Vector<Net*>                          _vpNets;
  FlatHashMap<String, Int>              _mNetName2Idx;

  // Constraints
  Vector<Region*>                       _vpRegions;
  FlatHashMap<String, Int>              _mRegionName2Idx;

  Vector<PrePlaceCstr*>                 _vpPrePlaceCstrs;
  FlatHashMap<String, Int>              _mPrePlaceCstrName2Idx;

  Vector<PlaceSymCstr*>                 _vpPlaceSymCstrs;
  FlatHashMap<String, Int>              _mPlaceSymCstrName2Idx;

	Vector<PlaceArrayCstr*>               _vpPlaceArrayCstrs;
  FlatHashMap<String, Int>              _mPlaceArrayCstrName2Idx;

  Vector<PlaceClusterCstr*>             _vpPlaceClusterCstrs;
  FlatHashMap<String, Int>              _mPlaceClusterCstrName2Idx;
  
  Vector<PlaceExtCstr*>                 _vpPlaceExtCstrs;
  FlatHashMap<String, Int>              _mPlaceExtCstrName2Idx;
  
  Vector<PlaceEdgeDistCstr*>            _vpPlaceEdgeDistCstrs;
  FlatHashMap<String, Int>              _mPlaceEdgeDistCstrName2Idx;
	
  Vector<PlaceOrderCstr*>               _vpPlaceOrderCstrs;
  FlatHashMap<String, Int>              _mPlaceOrderCstrName2Idx;
  
  Vector<PlaceAlignCstr*>               _vpPlaceAlignCstrs;
  FlatHashMap<String, Int>              _mPlaceAlignCstrName2Idx;
  
  Vector<PlaceDisjointCstr*>            _vpPlaceDisjointCstrs;
  FlatHashMap<String, Int>              _mPlaceDisjointCstrName2Idx;
  
  Vector<PlaceRowCstr*>                 _vpPlaceRowCstrs;
  FlatHashMap<String, Int>              _mPlaceRowCstrName2Idx;
 
  Vector<RouteSymCstr*>                 _vpRouteSymCstrs;
  FlatHashMap<String, Int>              _mRouteSymCstrName2Idx;
  
  Vector<RoutePathMatchCstr*>           _vpRoutePathMatchCstrs;
  FlatHashMap<String, Int>              _mRoutePathMatchCstrName2Idx;
 
  Vector<NetPrioCstr*>                  _vpNetPrioCstrs;
  FlatHashMap<String, Int>              _mNetPrioCstrName2Idx;

  // Spatial
  Vector<Vector<Pin*>>                  _vvpPins; // pins in each layer
  Vector<Vector<Obs*>>                  _vvpObs; // obs in each layer
  Vector<SpatialMap<Int, Pin*>>         _vSpatialPins;
  Vector<SpatialMap<Int, Obs*>>         _vSpatialObs;
  Vector<SpatialMap<Int, Net*>>         _vSpatialNets;

  // wsp
  Vector<Pair<Box<Int>, Int>>           _vWspBboxes; // wsp layer boundaries
  Vector<Pair<Box<Int>, Int>>           _vWspFillDrws; // wsp end dummy filldrw tracks
};

#define Cir_ForEachLayerIdx(cir, i) \
  for (i = 0; i < cir.numLayers(); ++i)

#define Cir_ForEachLayer(cir, pLayer_, i) \
  for (i = 0; i < cir.numLayers() and (pLayer_ = cir.pLayer(i)); ++i)

#define Cir_ForEachMetalLayer(cir, pMetalLayer_, i) \
  for (i = 0; i < cir.numMetalLayers() and (pMetalLayer_ = cir.pMetalLayer(i)); ++i)

#define Cir_ForEachCutLayer(cir, pCutLayer_, i) \
  for (i = 0; i < cir.numCutLayers() and (pCutLayer_ = cir.pCutLayer(i)); ++i)

#define Cir_ForEachPlaceGrid(cir, pPlaceGrid_, i) \
  for (i = 0; i < cir.numPlaceGrids() and (pPlaceGrid_ = cir.pPlaceGrid(i)); ++i)

#define Cir_ForEachRouteGrid(cir, pRouteGrid_, i) \
  for (i = 0; i < cir.numRouteGrids() and (pRouteGrid_ = cir.pRouteGrid(i)); ++i)

#define Cir_ForEachPrim(cir, pPrim_, i) \
  for (i = 0; i < cir.numPrims() and (pPrim_ = cir.pPrim(i)); ++i)

#define Cir_ForEachEdgePrim(cir, pEdgePrim_, i) \
  for (i = 0; i < cir.numEdgePrims() and (pEdgePrim_ = cir.pEdgePrim(i)); ++i)

#define Cir_ForEachDummyPrim(cir, pDummyPrim_, i) \
  for (i = 0; i < cir.numDummyPrims() and (pDummyPrim_ = cir.pDummyPrim(i)); ++i)

#define Cir_ForEachVia(cir, pVia_, i) \
  for (i = 0; i < cir.numVias() and (pVia_ = cir.pVia(i)); ++i)

#define Cir_ForEachDevMap(cir, pDevMap_, i) \
  for (i = 0; i < cir.numDevMaps() and (pDevMap_ = cir.pDevMap(i)); ++i)

#define Cir_ForEachCell(cir, pCell_, i) \
  for (i = 0; i < cir.numCells() and (pCell_ = cir.pCell(i)); ++i)

#define Cir_ForEachNet(cir, pNet_, i) \
  for (i = 0; i < cir.numNets() and (pNet_ = cir.pNet(i)); ++i)

#define Cir_ForEachRegion(cir, pRegion_, i) \
  for (i = 0; i < cir.numRegions() and (pRegion_ = cir.pRegion(i)); ++i)

#define Cir_ForEachNetPrioCstr(cir, pNetPrioCstr_, i) \
  for (i = 0; i < cir.numNetPrioCstrs() and (pNetPrioCstr_ = cir.pNetPrioCstr(i)); ++i)

#define Cir_ForEachPlaceSymCstr(cir, pPlaceSymCstr_, i) \
  for (i = 0; i < cir.numPlaceSymCstrs() and (pPlaceSymCstr_ = cir.pPlaceSymCstr(i)); ++i)

#define Cir_ForEachPlaceArrayCstr(cir, pPlaceArrayCstr_, i) \
  for (i = 0; i < cir.numPlaceArrayCstrs() and (pPlaceArrayCstr_ = cir.pPlaceArrayCstr(i)); ++i)

#define Cir_ForEachPlaceClusterCstr(cir, pPlaceClusterCstr_, i) \
  for (i = 0; i < cir.numPlaceClusterCstrs() and (pPlaceClusterCstr_ = cir.pPlaceClusterCstr(i)); ++i)

#define Cir_ForEachPlaceExtCstr(cir, pPlaceExtCstr_, i) \
  for (i = 0; i < cir.numPlaceExtCstrs() and (pPlaceExtCstr_ = cir.pPlaceExtCstr(i)); ++i)

#define Cir_ForEachPlaceEdgeDistCstr(cir, pPlaceEdgeDistCstr_, i) \
  for (i = 0; i < cir.numPlaceEdgeDistCstrs() and (pPlaceEdgeDistCstr_ = cir.pPlaceEdgeDistCstr(i)); ++i)

#define Cir_ForEachPlaceOrderCstr(cir, pPlaceOrderCstr_, i) \
  for (i = 0; i < cir.numPlaceOrderCstrs() and (pPlaceOrderCstr_ = cir.pPlaceOrderCstr(i)); ++i)

#define Cir_ForEachPlaceAlignCstr(cir, pPlaceAlignCstr_, i) \
  for (i = 0; i < cir.numPlaceAlignCstrs() and (pPlaceAlignCstr_ = cir.pPlaceAlignCstr(i)); ++i)

#define Cir_ForEachPlaceDisjointCstr(cir, pPlaceDisjointCstr_, i) \
  for (i = 0; i < cir.numPlaceDisjointCstrs() and (pPlaceDisjointCstr_ = cir.pPlaceDisjointCstr(i)); ++i)

#define Cir_ForEachPlaceRowCstr(cir, pPlaceRowCstr_, i) \
  for (i = 0; i < cir.numPlaceRowCstrs() and (pPlaceRowCstr_ = cir.pPlaceRowCstr(i)); ++i)

#define Cir_ForEachPrePlaceCstr(cir, pPrePlaceCstr_, i) \
  for (i = 0; i < cir.numPrePlaceCstrs() and (pPrePlaceCstr_ = cir.pPrePlaceCstr(i)); ++i)

#define Cir_ForEachRouteSymCstr(cir, pRouteSymCstr_, i) \
  for (i = 0; i < cir.numRouteSymCstrs() and (pRouteSymCstr_ = cir.pRouteSymCstr(i)); ++i)

#define Cir_ForEachRoutePathMatchCstr(cir, pRoutePathMatchCstr_, i) \
  for (i = 0; i < cir.numRoutePathMatchCstrs() and (pRoutePathMatchCstr_ = cir.pRoutePathMatchCstr(i)); ++i)

#define Cir_ForEachLayerPin(cir, layerIdx, pPin_, i) \
  for (i = 0; i < cir.numPins(layerIdx) and (pPin_ = cir.pPin(layerIdx, i)); ++i)

#define Cir_ForEachLayerObs(cir, layerIdx, pObs_, i) \
  for (i = 0; i < cir.numObs(layerIdx) and (pObs_ = cir.pObs(layerIdx, i)); ++i)

#define Cir_ForEachIOPin(cir, pPin_, i) \
  for (i = 0; i < cir.numIOPins() and (pPin_ = cir.pIOPin(i)); ++i) 

#define Cir_ForEachLeftIOPin(cir, pPin_, i) \
  for (i = 0; i < cir.numLeftIOPins() and (pPin_ = cir.pIOPinL(i)); ++i) 

#define Cir_ForEachRightIOPin(cir, pPin_, i) \
  for (i = 0; i < cir.numRightIOPins() and (pPin_ = cir.pIOPinR(i)); ++i) 

#define Cir_ForEachBottomIOPin(cir, pPin_, i) \
  for (i = 0; i < cir.numBottomIOPins() and (pPin_ = cir.pIOPinB(i)); ++i) 

#define Cir_ForEachTopIOPin(cir, pPin_, i) \
  for (i = 0; i < cir.numTopIOPins() and (pPin_ = cir.pIOPinT(i)); ++i) 

PROJECT_NAMESPACE_END

