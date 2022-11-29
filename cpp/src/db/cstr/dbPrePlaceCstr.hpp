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

class Cell;
class Region;

class PrePlaceCstr {
  friend class Parser;

public:
  PrePlaceCstr()
    : _idx(-1)
  {}
  ~PrePlaceCstr() {}

  const String&            name()                      const { return _name; }
  Int                      idx()                       const { return _idx; }

  Cell&                    cell(const Int i)                 { return *_vpCells.at(i); }
  const Cell&              cell(const Int i)           const { return *_vpCells.at(i); }
  Cell*                    pCell(const Int i)                { return _vpCells.at(i); }
  const Cell*              pCell(const Int i)          const { return _vpCells.at(i); }
  Vector<Cell*>&           vpCells()                         { return _vpCells; }
  const Vector<Cell*>&     vpCells()                   const { return _vpCells; }
  Int                      numCells()                  const { return _vpCells.size(); }
  
  Point<Int>               cellLoc(const Int i)        const { return _vCellLocs.at(i); }
  bool                     hasCellLoc(const Int i)     const { return _vCellLocs.at(i) != Point<Int>(-1, -1); }

  Orient2dE                cellOrient(const Int i)     const { return _vCellOrients.at(i); }
  bool                     hasCellOrient(const Int i)  const { return _vCellOrients.at(i) != Orient2dE::undef; }
  
  Region&                  region(const Int i)               { return *_vpRegions.at(i); }
  const Region&            region(const Int i)         const { return *_vpRegions.at(i); }
  Region*                  pRegion(const Int i)              { return _vpRegions.at(i); }
  const Region*            pRegion(const Int i)        const { return _vpRegions.at(i); }
  Vector<Region*>&         vpRegions()                       { return _vpRegions; }
  const Vector<Region*>&   vpRegions()                 const { return _vpRegions; }
  Int                      numRegions()                const { return _vpRegions.size(); }

  Point<Int>               regLoc(const Int i)         const { return _vRegLocs.at(i); }
  bool                     hasRegLoc(const Int i)      const { return _vRegLocs.at(i) != Point<Int>(-1, -1); }
 

private:
  String              _name;
  Int                 _idx;
  Vector<Cell*>       _vpCells;
  Vector<Point<Int>>  _vCellLocs;
  Vector<Orient2dE>   _vCellOrients;

  Vector<Region*>     _vpRegions;
  Vector<Point<Int>>  _vRegLocs;

};

PROJECT_NAMESPACE_END
