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

#include "db/dbCell.hpp"

PROJECT_NAMESPACE_START

class Primitive;
class Cell;
class PlaceArrayCstr;
class Region;

class PlaceExtCstr {
  friend class Parser;

public:
  PlaceExtCstr()
    : _idx(0), _type(ExtTypeE::undef)
  {}
  ~PlaceExtCstr() {}

  const String&                       name()                        const { return _name; }
  Int                                 idx()                         const { return _idx; }
  ExtTypeE                            type()                        const { return _type; }
  bool                                isCell()                      const { return _type == ExtTypeE::cell; }
  bool                                isRegion()                    const { return _type == ExtTypeE::region; }
  bool                                isArray()                     const { return _type == ExtTypeE::array; }

  Cell&                               cell(const Int i)                   { return *_vpCells.at(i); }
  const Cell&                         cell(const Int i)             const { return *_vpCells.at(i); }
  Cell*                               pCell(const Int i)                  { return _vpCells.at(i); }
  const Cell*                         pCell(const Int i)            const { return _vpCells.at(i); }
  Vector<Cell*>&                      vpCells()                           { return _vpCells; }
  const Vector<Cell*>&                vpCells()                     const { return _vpCells; }
  Int                                 numCells()                    const { return _vpCells.size(); }

  Region&                             region(const Int i)                 { return *_vpRegions.at(i); }
  const Region&                       region(const Int i)           const { return *_vpRegions.at(i); }
  Region*                             pRegion(const Int i)                { return _vpRegions.at(i); }
  const Region*                       pRegion(const Int i)          const { return _vpRegions.at(i); }
  Vector<Region*>&                    vpRegions()                         { return _vpRegions; }
  const Vector<Region*>&              vpRegions()                   const { return _vpRegions; }
  Int                                 numRegions()                  const { return _vpRegions.size(); }

  PlaceArrayCstr&                     placeArrayCstr(const Int i)         { return *_vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr&               placeArrayCstr(const Int i)   const { return *_vpPlaceArrayCstrs.at(i); }
  PlaceArrayCstr*                     pPlaceArrayCstr(const Int i)        { return _vpPlaceArrayCstrs.at(i); }
  const PlaceArrayCstr*               pPlaceArrayCstr(const Int i)  const { return _vpPlaceArrayCstrs.at(i); }
  Vector<PlaceArrayCstr*>&            vpPlaceArrayCstrs()                 { return _vpPlaceArrayCstrs; }
  const Vector<PlaceArrayCstr*>&      vpPlaceArrayCstrs()           const { return _vpPlaceArrayCstrs; }
  Int                                 numPlaceArrayCstrs()          const { return _vpPlaceArrayCstrs.size(); }

  const Pair<const Primitive*, Int>&  leftExt()                     const { return _left; }
  const Pair<const Primitive*, Int>&  rightExt()                    const { return _right; }
  const Pair<const Primitive*, Int>&  bottomExt()                   const { return _bottom; }
  const Pair<const Primitive*, Int>&  topExt()                      const { return _top; }
  Pair<Int, Int>                      leftExtIdx()                  const { return std::make_pair(_left.first->idx(), _left.second); }
  Pair<Int, Int>                      rightExtIdx()                 const { return std::make_pair(_right.first->idx(), _right.second); }
  Pair<Int, Int>                      bottomExtIdx()                const { return std::make_pair(_bottom.first->idx(), _bottom.second); }
  Pair<Int, Int>                      topExtIdx()                   const { return std::make_pair(_top.first->idx(), _top.second); }

private:
  String                      _name;
  Int                         _idx;

  ExtTypeE                    _type;

  Vector<Cell*>               _vpCells;
  Vector<Region*>             _vpRegions;
  Vector<PlaceArrayCstr*>     _vpPlaceArrayCstrs;

  Pair<const Primitive*, Int> _left; // (prim, numGrids)
  Pair<const Primitive*, Int> _right;
  Pair<const Primitive*, Int> _bottom;
  Pair<const Primitive*, Int> _top;
};

PROJECT_NAMESPACE_END
