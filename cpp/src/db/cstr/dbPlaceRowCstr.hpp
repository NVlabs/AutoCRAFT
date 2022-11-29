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

class PlaceRowCstr {
  friend class Parser;

public:
  PlaceRowCstr()
    : _idx(-1), _isOdd(false)
  {}
  ~PlaceRowCstr() {}

  const String&           name()                const { return _name; }
  Int                     idx()                 const { return _idx; }

  Cell&                   cell(const Int i)           { return *_vpCells.at(i); }
  const Cell&             cell(const Int i)     const { return *_vpCells.at(i); }
  Cell*                   pCell(const Int i)          { return _vpCells.at(i); }
  const Cell*             pCell(const Int i)    const { return _vpCells.at(i); }
  Vector<Cell*>&          vpCells()                   { return _vpCells; }
  const Vector<Cell*>&    vpCells()             const { return _vpCells; }
  Int                     numCells()            const { return _vpCells.size(); }

  bool                    isOdd()               const { return _isOdd; }
  bool                    isEven()              const { return !_isOdd; }


private:
  String          _name;
  Int             _idx;
  Vector<Cell*>   _vpCells;
  bool            _isOdd;
};

PROJECT_NAMESPACE_END

