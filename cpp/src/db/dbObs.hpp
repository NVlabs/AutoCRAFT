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

PROJECT_NAMESPACE_START

class Cell; 

class Obs {
  friend class Cell;
  friend class Parser;
  friend class LefReader;

public:
  Obs()
    : _pCell(nullptr), _layerIdx(-1) {}
  Obs(const Box<Int>& b, const Int z)
    : _pCell(nullptr), _box(b), _layerIdx(z) {}
  ~Obs() {}
  Cell&           cell()             { return *_pCell; }
  const Cell&     cell()       const { return *_pCell; }
  Cell*           pCell()            { return _pCell; }
  const Cell*     pCell()      const { return _pCell; }

  Box<Int>&       box()             { return _box; }
  const Box<Int>& box()       const { return _box; }
  Int             layerIdx()  const { return _layerIdx; }

private:
  Cell*     _pCell;
  Box<Int>  _box; 
  Int       _layerIdx;
};

PROJECT_NAMESPACE_END
