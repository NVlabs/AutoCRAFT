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

class PlaceSymCstr {
  friend class Parser;

public:
  PlaceSymCstr() : _idx(-1), _axis(SymAxisE::undef), _numPartAs(0), _numSelfSyms(0) {}
  ~PlaceSymCstr() {}

  const String&         name()                        const { return _name; }
  Int                   idx()                         const { return _idx; }
  
  SymAxisE              axis()                        const { return _axis; }
  bool                  isHor()                       const { return _axis == SymAxisE::hor; }
  bool                  isVer()                       const { return _axis == SymAxisE::ver; }

  Cell&                 cell(const Int i)                   { return *_vpCells.at(i); }
  const Cell&           cell(const Int i)             const { return *_vpCells.at(i); }
  Cell*                 pCell(const Int i)                  { return _vpCells.at(i); }
  const Cell*           pCell(const Int i)            const { return _vpCells.at(i); }
  Vector<Cell*>&        vpCells()                           { return _vpCells; }
  const Vector<Cell*>&  vpCells()                     const { return _vpCells; }
  Int                   numCells()                    const { return _vpCells.size(); }

  SymPartE              symPart(const Int i)          const { return _vSymPartEs.at(i); }
  SymPartE              symPart(const Cell& c)        const { return _vSymPartEs.at(cellIdx2Idx(c.idx())); }
  bool                  isPartA(const Int i)          const { return symPart(i) == SymPartE::a; }
  bool                  isPartA(const Cell& c)        const { return symPart(c) == SymPartE::a; }
  bool                  isPartB(const Int i)          const { return symPart(i) == SymPartE::b; }
  bool                  isPartB(const Cell& c)        const { return symPart(c) == SymPartE::b; }
  bool                  isSelfSym(const Int i)        const { return symPart(i) == SymPartE::self; }
  bool                  isSelfSym(const Cell& c)      const { return symPart(c) == SymPartE::self; }
  Int                   numSelfSyms()                 const { return _numSelfSyms; }
  Int                   numPartAs()                   const { return _numPartAs; }
  bool                  hasSelfSym()                  const { return _numSelfSyms > 0; }

  Int                   cellIdx2Idx(const Int idx)    const { return _mCellIdx2Idx.at(idx); }
  Int                   cellIdx2SymIdx(const Int idx) const { return _vSymCellIds.at(cellIdx2Idx(idx)); }
  bool                  hasCell(const Cell& c)        const { return _mCellIdx2Idx.find(c.idx()) != _mCellIdx2Idx.end(); }

  Cell&                 symCell(const Int i)                { return *_vpCells.at(_vSymCellIds.at(i)); }
  Cell&                 symCell(const Cell& c)              { return *_vpCells.at(cellIdx2SymIdx(c.idx())); }
  const Cell&           symCell(const Int i)          const { return *_vpCells.at(_vSymCellIds.at(i)); }
  const Cell&           symCell(const Cell& c)        const { return *_vpCells.at(cellIdx2SymIdx(c.idx())); }
  Cell*                 pSymCell(const Int i)               { return _vpCells.at(_vSymCellIds.at(i));  }
  Cell*                 pSymCell(const Cell& c)             { return _vpCells.at(cellIdx2SymIdx(c.idx()));  }
  const Cell*           pSymCell(const Int i)         const { return _vpCells.at(_vSymCellIds.at(i));  }
  const Cell*           pSymCell(const Cell& c)       const { return _vpCells.at(cellIdx2SymIdx(c.idx()));  }

  const Region&         region()                      const { return _vpCells.at(0)->region(); }
  const Region*         pRegion()                     const { return _vpCells.at(0)->pRegion(); }

  // for debug
  bool check() {
    bool b = true;
    const Region* pReg = _vpCells.at(0)->pRegion();
    for (Int i = 1; i < (Int)_vpCells.size(); ++i) {
      Cell& c = *_vpCells.at(i);
      if (c.pRegion() != pReg) {
        b = false;
      }
    }
    return b;
  }

private:

  String                _name;
  Int                   _idx;
  SymAxisE              _axis;

  Vector<Cell*>         _vpCells;
  Vector<SymPartE>      _vSymPartEs;
  Vector<Int>           _vSymCellIds;
  Int                   _numPartAs;
  Int                   _numSelfSyms;

  FlatHashMap<Int, Int> _mCellIdx2Idx; // cell.idx() -> idx in _vpCells

};

PROJECT_NAMESPACE_END
