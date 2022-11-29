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

class PlaceArrayCstr {
  friend class Parser;

public:
  PlaceArrayCstr() : _idx(-1), _col(0), _row(0), _size(0), _pattern(ArrayPatternE::undef) {}
  ~PlaceArrayCstr() {}

  const String&                   name()                                  const { return _name; }
  Int                             idx()                                   const { return _idx; }

  Int                             col()                                   const { return _col; }
  Int                             row()                                   const { return _row; }
  bool                            hasCol()                                const { return _col > 0; }
  bool                            hasRow()                                const { return _row > 0; }

  Int                             size()                                  const { return _size; }
  bool                            hasSize()                               const { return _size > 0; }
  bool                            useAutoSize()                           const { return _size == 0; }
  
  ArrayPatternE                   pattern()                               const { return _pattern; }
  bool                            isID()                                  const { return _pattern == ArrayPatternE::id; }
  bool                            isCC()                                  const { return _pattern == ArrayPatternE::cc; }
  bool                            isCS()                                  const { return _pattern == ArrayPatternE::cs; }
  bool                            hasPattern()                            const { return _pattern != ArrayPatternE::undef; }

  Cell&                           cell(const Int i)                             { return *_vpCells.at(i); }
  const Cell&                     cell(const Int i)                       const { return *_vpCells.at(i); }
  Cell*                           pCell(const Int i)                            { return _vpCells.at(i); }
  const Cell*                     pCell(const Int i)                      const { return _vpCells.at(i); }
  Vector<Cell*>&                  vpCells()                                     { return _vpCells; }
  const Vector<Cell*>&            vpCells()                               const { return _vpCells; }
  Int                             numCells()                              const { return _vpCells.size(); }

  ArrayPartE                      arrayPart(const Int i)                  const { return _vArrayPartEs.at(i); }
  ArrayPartE                      arrayPart(const Cell& c)                const { return _vArrayPartEs.at(cellIdx2Idx(c.idx())); }
  bool                            isPartA(const Int i)                    const { return arrayPart(i) == ArrayPartE::a; }
  bool                            isPartA(const Cell& c)                  const { return arrayPart(c) == ArrayPartE::a; }
  bool                            isPartB(const Int i)                    const { return arrayPart(i) == ArrayPartE::b; }
  bool                            isPartB(const Cell& c)                  const { return arrayPart(c) == ArrayPartE::b; }

  Int                             cellIdx2Idx(const Int idx)              const { return _mCellIdx2Idx.at(idx); }

  PlaceArrayCstr&                 placeArrayCstr(const Int i)                   { return *_vpArrays.at(i); }
  const PlaceArrayCstr&           placeArrayCstr(const Int i)             const { return *_vpArrays.at(i); }
  PlaceArrayCstr*                 pPlaceArrayCstr(const Int i)                  { return _vpArrays.at(i); }
  const PlaceArrayCstr*           pPlaceArrayCstr(const Int i)            const { return _vpArrays.at(i); }
  Vector<PlaceArrayCstr*>&        vpPlaceArrayCstrs()                           { return _vpArrays; }
  const Vector<PlaceArrayCstr*>&  vpPlaceArrayCstrs()                     const { return _vpArrays; }
  Int                             numPlaceArrayCstrs()                    const { return _vpArrays.size(); }

  Int                             arrayIdx2Idx(const Int idx)             const { return _mArrayIdx2Idx.at(idx); }
  bool                            hasArray(const PlaceArrayCstr& arr)     const { return _mArrayIdx2Idx.find(arr.idx()) != _mArrayIdx2Idx.end(); }

  const Region&                   region()                                const { return _vpCells.size() ? const_cast<const Region&>(_vpCells.at(0)->region()) : _vpArrays.at(0)->region(); }
  const Region*                   pRegion()                               const { return _vpCells.size() ? const_cast<const Region*>(_vpCells.at(0)->pRegion()) : _vpArrays.at(0)->pRegion(); }
  
  bool hasCell(const Cell& c) const {
    std::function<bool(const PlaceArrayCstr& ac, const Cell& c)>
    hasCellImpl = [&] (const PlaceArrayCstr& ac, const Cell& c) -> bool {
      if (!ac._mCellIdx2Idx.empty()) {
        return ac._mCellIdx2Idx.find(c.idx()) != ac._mCellIdx2Idx.end();
      }
      else {
        assert(ac._vpArrays.size() > 0);
        for (const PlaceArrayCstr* pArr : ac._vpArrays) {
          if (hasCellImpl(*pArr, c)) {
            return true;
          }
        }
        return false;
      }
    };
    return hasCellImpl(*this, c);
  }

private:
  String                  _name;
  Int                     _idx;
  Int                     _col;
  Int                     _row;
  Int                     _size;
  ArrayPatternE           _pattern;
  
  Vector<Cell*>           _vpCells;
  Vector<ArrayPartE>      _vArrayPartEs;
  FlatHashMap<Int, Int>   _mCellIdx2Idx;

  Vector<PlaceArrayCstr*> _vpArrays; // for array of array constraints
  FlatHashMap<Int, Int>   _mArrayIdx2Idx;
};


PROJECT_NAMESPACE_END
