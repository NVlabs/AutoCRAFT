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
class Array2d {
 public:
  Array2d() : _numX(0), _numY(0) {}
  Array2d(const Int x, const Int y) : _numX(x), _numY(y), _vec(x * y) {}
  Array2d(const Int x, const Int y, const T& val) : _numX(x), _numY(y), _vec(x * y, val) {}
  ~Array2d() {}

  // get
  Int       numX()                        const  { return _numX; }
  Int       numY()                        const  { return _numY; }
  Int       size()                        const  { return _vec.size(); }
  T&        at(const Int i)                      { return _vec.at(i); }
  const T&  at(const Int i)               const  { return _vec.at(i); }
  T&        at(const Int x, const Int y)         { return _vec.at(flatIdx(x, y)); }
  const T&  at(const Int x, const Int y)  const  { return _vec.at(flatIdx(x, y)); }
  
  // iterator
  inline typename Vector<T>::iterator       begin()        { return _vec.begin(); }
  inline typename Vector<T>::const_iterator cbegin() const { return _vec.cbegin(); }
  inline typename Vector<T>::iterator       end()          { return _vec.end(); }
  inline typename Vector<T>::const_iterator cend()   const { return _vec.cend(); }

  // set
  void clear() { _numX = _numY = 0; _vec.clear(); }
  void resize(const Int x, const Int y) { _numX = x; _numY = y; _vec.resize(x * y); }
  void resize(const Int x, const Int y, const T& val) { _numX = x; _numY = y; _vec.resize(x * y, val); }
  void set(const Int x, const Int y, const T& val) { _vec[flatIdx(x, y)] = val; }   

  T& operator [] (const Int i) { return _vec.at(i); }

 private:
  Int       _numX;
  Int       _numY;
  Vector<T> _vec;

  Int flatIdx(const Int x, const Int y) const {
    assert(0 <= x and x < _numX);
    assert(0 <= y and y < _numY);
    return x * _numY +  y;
  }
  Pair<Int, Int>  nestedIdx(const Int i) const { return std::make_pair(i / _numY, i % _numY); }
};


#define Array2d_ForEachEntryX(a2d, y, pEntry_, i) \
  for (i = 0; i < a2d.numX() and (pEntry_ = &a2d.at(i, y)); ++i)

#define Array2d_ForEachEntryY(a2d, x, pEntry_, i) \
  for (i = 0; i < a2d.numY() and (pEntry_ = &a2d.at(x, i)); ++i)


PROJECT_NAMESPACE_END

