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

#include "ds/hash.hpp"
#include "dbPin.hpp"
#include "dbObs.hpp"

PROJECT_NAMESPACE_START


class Primitive {
  friend class Parser;
  friend class LefReader;
    
public:

  Primitive()
    : _idx(-1), _originX(0), _originY(0), _sizeX(0), _sizeY(0), _foreignX(0), _foreignY(0),
      _isXSym(false), _isYSym(false), _is90Sym(false) {}
  ~Primitive() {}

  const String&           name()                        const { return _name; }
  Int                     idx()                         const { return _idx; }
  const String&           classType()                   const { return _classType; }

  Int                     originX()                     const { return _originX; }
  Int                     originY()                     const { return _originY; }
  Int                     sizeX()                       const { return _sizeX; }
  Int                     sizeY()                       const { return _sizeY; }
  Int                     area()                        const { return _sizeX * _sizeY; }

  const String&           foreignCellName()             const { return _foreignCellName; }
  Int                     foreignX()                    const { return _foreignX; }
  Int                     foreignY()                    const { return _foreignY; }

  const Pin&              pin(const Int i)              const { return _vPins.at(i); }
  const Pin&              pin(const String& n)          const { return _vPins.at(pinName2Idx(n)); }
  Int                     pinName2Idx(const String& n)  const { return _mPinName2Idx.at(n); }
  const Vector<Pin>&      vPins()                       const { return _vPins; }
  Int                     numPins()                     const { return _vPins.size(); }

  const Obs&              obs(const Int i)              const { return _vObs.at(i); }
  const Vector<Obs>&      vObs()                        const { return _vObs; }
  Int                     numObs()                      const { return _vObs.size(); }

private:
  String                   _name;
  Int                      _idx;
  String                   _classType;
  Int                      _originX, _originY;
  Int                      _sizeX, _sizeY;
  String                   _foreignCellName;
  Int                      _foreignX, _foreignY;
  bool                     _isXSym;
  bool                     _isYSym;
  bool                     _is90Sym;

  Vector<Pin>              _vPins;
  FlatHashMap<String, Int> _mPinName2Idx;

  Vector<Obs>              _vObs;
};


PROJECT_NAMESPACE_END
