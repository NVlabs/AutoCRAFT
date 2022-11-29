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

#include "dbPin.hpp"
#include "dbPrim.hpp"
#include "dbDevMap.hpp"


PROJECT_NAMESPACE_START

// forward declaration
class PrePlaceCstr;
class PlaceSymCstr;
class PlaceArrayCstr;
class PlaceClusterCstr;
class PlaceOrderCstr;
class PlaceAlignCstr;
class PlaceDisjointCstr;
class PlaceExtCstr;
class PlaceRowCstr;
class PlaceEdgeDistCstr;
class Region;
//class CirDB;

class Cell {
  friend class Parser;

public:
  Cell()
    :
      //_pCir(nullptr),
      _idx(-1), _vddPinIdx(-1), _vssPinIdx(-1),
      _pDevMap(nullptr), _pPrim(nullptr),
      _width(0), _height(0),
      _orient(Orient2dE::n), _pRegion(nullptr), 
      _pPrePlaceCstr(nullptr), 
      _pPlaceExtCstr(nullptr), _pPlaceEdgeDistCstr(nullptr),
      _powerGroupIdx(-1) {}
  ~Cell() {}

  // get
  //CirDB&                            cir()                                 { return *_pCir; }
  //const CirDB&                      cir()                           const { return *_pCir; }
  //CirDB*                            pCir()                                { return _pCir; }
  //const CirDB*                      pCir()                          const { return _pCir; }
  
  const String&                     name()                          const { return _name; }
  const String&                     instName()                      const { return _instName; }
  Int                               idx()                           const { return _idx; }

  const DevMap&                     devMap()                        const { return *_pDevMap; }
  const DevMap*                     pDevMap()                       const { return _pDevMap; }
  const Primitive&                  prim()                          const { return *_pPrim; }
  const Primitive*                  pPrim()                         const { return _pPrim; }

  Pin&                              pin(const Int i)                      { return _vPins.at(i); }
  const Pin&                        pin(const Int i)                const { return _vPins.at(i); }
  Pin*                              pPin(const Int i)                     { return &_vPins.at(i); }
  const Pin*                        pPin(const Int i)               const { return &_vPins.at(i); }
  Vector<Pin>&                      vPins()                               { return _vPins; }
  const Vector<Pin>&                vPins()                         const { return _vPins; }
  Int                               numPins()                       const { return _vPins.size(); }
  Int                               numSignalPins()                 const { return _vPins.size() - hasVddPin() - hasVssPin(); }

  Pin&                              vddPin()                              { return _vPins.at(_vddPinIdx); }
  const Pin&                        vddPin()                        const { return _vPins.at(_vddPinIdx); }
  Pin*                              pVddPin()                             { return &_vPins.at(_vddPinIdx); }
  const Pin*                        pVddPin()                       const { return &_vPins.at(_vddPinIdx); }
  Pin&                              vssPin()                              { return _vPins.at(_vssPinIdx); }
  const Pin&                        vssPin()                        const { return _vPins.at(_vssPinIdx); }
  Pin*                              pVssPin()                             { return &_vPins.at(_vssPinIdx); }
  const Pin*                        pVssPin()                       const { return &_vPins.at(_vssPinIdx); }
  bool                              hasVddPin()                     const { return _vddPinIdx != -1; }
  bool                              hasVssPin()                     const { return _vssPinIdx != -1; }

  Obs&                              obs(const Int i)                      { return _vObs.at(i); }
  const Obs&                        obs(const Int i)                const { return _vObs.at(i); }
  Obs*                              pObs(const Int i)                     { return &_vObs.at(i); }
  const Obs*                        pObs(const Int i)               const { return &_vObs.at(i); }
  Vector<Obs>&                      vObs()                                { return _vObs; }
  const Vector<Obs>&                vObs()                          const { return _vObs; }
  Int                               numObs()                        const { return _vObs.size(); }

  Point<Int>                        loc()                           const { return _loc; }
  Int                               xl()                            const { return _loc.x(); }
  Int                               yl()                            const { return _loc.y(); }
  Int                               xh()                            const { return _loc.x() + _width; }
  Int                               yh()                            const { return _loc.y() + _height; }
  Point<Int>                        gridLoc()                       const { return _gridLoc; }
  Int                               col()                           const { return _gridLoc.x(); }
  Int                               row()                           const { return _gridLoc.y(); }
  Int                               width()                         const { return _width; }
  Int                               height()                        const { return _height; }
  Int                               area()                          const { return prim().area(); }
  Point<Int>                        center()                        const { return Point<Int>(_loc.x() + width() / 2, _loc.y() + height() / 2); }
  Int                               centerX()                       const { return _loc.x() + _width / 2; }
  Int                               centerY()                       const { return _loc.y() + _height / 2; }

  Region&                           region()                              { return *_pRegion; }
  const Region&                     region()                        const { return *_pRegion; }
  Region*                           pRegion()                             { return _pRegion; }
  const Region*                     pRegion()                       const { return _pRegion; }
  Int                               powerGroupIdx()                 const { return _powerGroupIdx; }

  const PrePlaceCstr&               prePlaceCstr()                  const { return *_pPrePlaceCstr; }
  const PrePlaceCstr*               pPrePlaceCstr()                 const { return _pPrePlaceCstr; }
  bool                              hasPrePlace()                   const { return _pPrePlaceCstr != nullptr; }
  
  const PlaceSymCstr&               placeSymCstr(const Int i)       const { return *_vpPlaceSymCstrs.at(i); }
  const PlaceSymCstr*               pPlaceSymCstr(const Int i)      const { return _vpPlaceSymCstrs.at(i); }
  const Vector<PlaceSymCstr*>&      vpPlaceSymCstrs()               const { return _vpPlaceSymCstrs; }
  Int                               numPlaceSymCstrs()              const { return _vpPlaceSymCstrs.size(); }
  bool                              hasSym()                        const { return !_vpPlaceSymCstrs.empty(); }

  const PlaceArrayCstr&             placeArrayCstr(const Int i)     const { return *_vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr*             pPlaceArrayCstr(const Int i)    const { return _vpPlaceArrayCstrs.at(i); }
  const Vector<PlaceArrayCstr*>&    vpPlaceArrayCstrs()             const { return _vpPlaceArrayCstrs; }
  Int                               numPlaceArrayCstrs()            const { return _vpPlaceArrayCstrs.size(); }
  bool                              hasArray()                      const { return !_vpPlaceArrayCstrs.empty(); }
  
  const PlaceClusterCstr&           placeClusterCstr(const Int i)   const { return *_vpPlaceClusterCstrs.at(i); }
  const PlaceClusterCstr*           pPlaceClusterCstr(const Int i)  const { return _vpPlaceClusterCstrs.at(i); }
  const Vector<PlaceClusterCstr*>&  vpPlaceClusterCstrs()           const { return _vpPlaceClusterCstrs; }
  Int                               numPlaceClusterCstrs()          const { return _vpPlaceClusterCstrs.size(); }
  bool                              hasCluster()                    const { return !_vpPlaceClusterCstrs.empty(); }
  
  const PlaceOrderCstr&             placeOrderCstr(const Int i)     const { return *_vpPlaceOrderCstrs.at(i); }
  const PlaceOrderCstr*             pPlaceOrderCstr(const Int i)    const { return _vpPlaceOrderCstrs.at(i); }
  const Vector<PlaceOrderCstr*>&    vpPlaceOrderCstrs()             const { return _vpPlaceOrderCstrs; }
  Int                               numPlaceOrderCstrs()            const { return _vpPlaceOrderCstrs.size(); }
  bool                              hasOrder()                      const { return !_vpPlaceOrderCstrs.empty(); }
  
  const PlaceAlignCstr&             placeAlignCstr(const Int i)     const { return *_vpPlaceAlignCstrs.at(i); }
  const PlaceAlignCstr*             pPlaceAlignCstr(const Int i)    const { return _vpPlaceAlignCstrs.at(i); }
  const Vector<PlaceAlignCstr*>&    vpPlaceAlignCstrs()             const { return _vpPlaceAlignCstrs; }
  Int                               numPlaceAlignCstrs()            const { return _vpPlaceAlignCstrs.size(); }
  bool                              hasAlign()                      const { return !_vpPlaceAlignCstrs.empty(); }
  
  const PlaceDisjointCstr&          placeDisjointCstr(const Int i)  const { return *_vpPlaceDisjointCstrs.at(i); }
  const PlaceDisjointCstr*          pPlaceDisjointCstr(const Int i) const { return _vpPlaceDisjointCstrs.at(i); }
  const Vector<PlaceDisjointCstr*>& vpPlaceDisjointCstrs()          const { return _vpPlaceDisjointCstrs; }
  Int                               numPlaceDisjointCstrs()         const { return _vpPlaceDisjointCstrs.size(); }
  bool                              hasDisjoint()                   const { return !_vpPlaceDisjointCstrs.empty(); }
  
  const PlaceExtCstr&               placeExtCstr()                  const { return *_pPlaceExtCstr; }
  const PlaceExtCstr*               pPlaceExtCstr()                 const { return _pPlaceExtCstr; }
  bool                              hasExtension()                  const { return _pPlaceExtCstr != nullptr; }
  
  const PlaceRowCstr&               placeRowCstr()                  const { return *_pPlaceRowCstr; }
  const PlaceRowCstr*               pPlaceRowCstr()                 const { return _pPlaceRowCstr; }
  bool                              hasRow()                        const { return _pPlaceRowCstr != nullptr; }
 
  const PlaceEdgeDistCstr&          placeEdgeDistCstr()             const { return *_pPlaceEdgeDistCstr; }
  const PlaceEdgeDistCstr*          pPlaceEdgeDistCstr()            const { return _pPlaceEdgeDistCstr; }
  bool                              hasEdgeDist()                   const { return _pPlaceEdgeDistCstr != nullptr; }

  Orient2dE                         orient()                        const { return _orient; }
  
  // set
  void                              setName(const String& n)      { _name = n; }
  void                              setInstName(const String& n)  { _instName = n; }
  void                              setIdx(const Int i)           { _idx = i; }
  void                              setRegion(Region* p)          { _pRegion = p; }
  void                              setDevMap(const DevMap* p)    { _pDevMap = p; }

  void                              setPrim(const Primitive& prim);
  void                              setLoc(const Int x, const Int y, const Int gx, const Int gy);
  void                              setOrient(const Orient2dE o);
  void                              rotate90();
  void                              rotate180();
  void                              rotate270();
  void                              flipX();
  void                              flipY();

  void                              setPowerGroupIdx(const Int i);

private:
  //CirDB*                      _pCir;
  String                      _name;
  String                      _instName;
  Int                         _idx;
  Vector<Pin>                 _vPins;
  Int                         _vddPinIdx;
  Int                         _vssPinIdx;
  Vector<Obs>                 _vObs;

  const DevMap*               _pDevMap;
  const Primitive*            _pPrim;

  Point<Int>                  _loc; // lower left corner
  Point<Int>                  _gridLoc; // lower left corner (grid based)
  Int                         _width;
  Int                         _height;
  Orient2dE                   _orient;

  Region*                     _pRegion;
  PrePlaceCstr*               _pPrePlaceCstr;
  Vector<PlaceSymCstr*>       _vpPlaceSymCstrs;
  Vector<PlaceArrayCstr*>     _vpPlaceArrayCstrs;
  Vector<PlaceClusterCstr*>   _vpPlaceClusterCstrs;
  Vector<PlaceOrderCstr*>     _vpPlaceOrderCstrs;
  Vector<PlaceAlignCstr*>     _vpPlaceAlignCstrs;
  Vector<PlaceDisjointCstr*>  _vpPlaceDisjointCstrs;
  PlaceExtCstr*               _pPlaceExtCstr;
  PlaceRowCstr*               _pPlaceRowCstr;
  PlaceEdgeDistCstr*          _pPlaceEdgeDistCstr;

  Int                         _powerGroupIdx;
};

#define Cell_ForEachPin(cell, pPin_, i) \
  for (i = 0; i < cell.numPins() and (pPin_ = cell.pPin(i)); ++i)

#define Cell_ForEachObs(cell, pObs_, i) \
  for (i = 0; i < cell.numObs() and (pObs_ = cell.pObs(i)); ++i)

PROJECT_NAMESPACE_END
