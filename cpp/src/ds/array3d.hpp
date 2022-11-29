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

#include "global/global.hpp"

PROJECT_NAMESPACE_START

template<typename T>
class Array3d {
 public:
  Array3d() : _numX(0), _numY(0), _numZ(0), _numYZ(0) {}
  Array3d(const Int x, const Int y, const Int z) : _numX(x), _numY(y), _numZ(z), _numYZ(y * z), _vec(x * y * z) {}
  Array3d(const Int x, const Int y, const Int z, const T& val) : _numX(x), _numY(y), _numZ(z), _numYZ(y * z), _vec(x * y * z, val) {}
  ~Array3d() {}

  // get
  Int       numX()                                    const { return _numX; }
  Int       numY()                                    const { return _numY; }
  Int       numZ()                                    const { return _numZ; }
  Int       size()                                    const { return _vec.size(); }
  T&        at(const Int i)                                 { return _vec.at(i); }
  const T&  at(const Int i)                           const { return _vec.at(i); }
  T&        at(const Int x, const Int y, const Int z)       { return _vec.at(flatIdx(x, y, z)); }
  const T&  at(const Int x, const Int y, const Int z) const { return _vec.at(flatIdx(x, y, z)); }

  // iterator
  inline typename Vector<T>::iterator       begin()        { return _vec.begin(); }
  inline typename Vector<T>::const_iterator cbegin() const { return _vec.cbegin(); }
  inline typename Vector<T>::iterator       end()          { return _vec.end(); }
  inline typename Vector<T>::const_iterator cend()   const { return _vec.cend(); }

  // set
  void clear() { _numX = _numY = _numZ = _numYZ = 0; _vec.clear(); }
  void resize(const Int x, const Int y, const Int z) { _numX = x; _numY = y; _numZ = z; _numYZ = y * z; _vec.resize(x * y * z); }
  void resize(const Int x, const Int y, const Int z, const T& val) { _numX = x; _numY = y; _numZ = z; _numYZ = y * z; _vec.resize(x * y * z, val); }
  void set(const Int x, const Int y, const Int z, const T& val) { _vec[flatIdx(x, y, z)] = val; }
  
  T& operator [] (const Int i) { return _vec.at(i); }

 private:
  Int       _numX;
  Int       _numY;
  Int       _numZ;
  Int       _numYZ;
  Vector<T> _vec;

  Int flatIdx(const Int x, const Int y, const Int z) const {
    assert(0 <= x and x < _numX);
    assert(0 <= y and y < _numY);
    assert(0 <= z and z < _numZ);
    return x * _numYZ + y * _numZ + z;
  }
};

#define Array3d_ForEachEntryX(a3d, y, z, pEntry_, i) \
  for (i = 0; i < a3d.numX() and (pEntry_ = &a3d.at(i, y, z)); ++i)

#define Array3d_ForEachEntryY(a3d, x, z, pEntry_, i) \
  for (i = 0; i < a3d.numY() and (pEntry_ = &a3d.at(x, i, z)); ++i)

#define Array3d_ForEachEntryZ(a3d, x, y, pEntry_, i) \
  for (i = 0; i < a3d.numZ() and (pEntry_ = &a3d.at(x, y, i)); ++i)

PROJECT_NAMESPACE_END

