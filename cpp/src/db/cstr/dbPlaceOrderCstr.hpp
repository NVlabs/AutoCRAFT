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

class PlaceOrderCstr {
  friend class Parser;

public:
  PlaceOrderCstr() : _idx(-1), _dir(Direction2dE::undef), _isStrict(false) {}
  ~PlaceOrderCstr() {}
  
  const String&                 name()                          const { return _name; }
  Int                           idx()                           const { return _idx; }

  Cell&                         cell(const Int i, const Int j)        { return *_vvpCells.at(i).at(j); }
  const Cell&                   cell(const Int i, const Int j)  const { return *_vvpCells.at(i).at(j); }
  Cell*                         pCell(const Int i, const Int j)       { return _vvpCells.at(i).at(j); }
  const Cell*                   pCell(const Int i, const Int j) const { return _vvpCells.at(i).at(j); }
  Vector<Cell*>&                vpCells(const Int i)                  { return _vvpCells.at(i); }
  const Vector<Cell*>&          vpCells(const Int i)            const { return _vvpCells.at(i); }
  Int                           numCells(const Int i)           const { return _vvpCells.at(i).size(); }
  Vector<Vector<Cell*>>&        vvpCells()                            { return _vvpCells; }
  const Vector<Vector<Cell*>>&  vvpCells()                      const { return _vvpCells; }
  Int                           numCellGroups()                 const { return _vvpCells.size(); }

  Direction2dE                  dir()                           const { return _dir; }
  bool                          isStrict()                      const { return _isStrict; }
  bool                          isWeak()                        const { return !_isStrict; }

private:
  String                _name;
  Int                   _idx;
  Vector<Vector<Cell*>> _vvpCells; // should be in the desired order
  Direction2dE          _dir;
  bool                  _isStrict;
};

PROJECT_NAMESPACE_END
