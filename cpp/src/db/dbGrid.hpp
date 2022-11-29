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

#include "ds/array2d.hpp"
#include "ds/hash.hpp"
#include "geo/box.hpp"
#include "geo/point.hpp"
#include "geo/point3d.hpp"
#include "dbVia.hpp"

PROJECT_NAMESPACE_START

class PlaceGrid {
  friend class Parser;

public:
  PlaceGrid() : _name(""), _stepX(0), _stepY(0) {}
  ~PlaceGrid() {}

  const String& name()              const { return _name; }
  Point<Int>    origin()            const { return _origin; }
  Int           originX()           const { return _origin.x(); }
  Int           originY()           const { return _origin.y(); }
  Int           stepX()             const { return _stepX; }
  Int           stepY()             const { return _stepY; }

  Int           floorX(Int x)       const { x -= _origin.x(); return x - modulo(x, _stepX); }
  Int           floorY(Int y)       const { y -= _origin.y(); return y - modulo(y, _stepY); }
  Int           ceilX(Int x)        const { const Int m = modulo(x, _stepX); return m == 0 ? x : x + _stepX - m; }
  Int           ceilY(Int y)        const { const Int m = modulo(y, _stepY); return m == 0 ? y : y + _stepY - m; }
  
  bool isOnGridX(const Int x) const
  {
    return modulo((x - _origin.x()), _stepX) == 0; 
  }
  bool isOnGridY(const Int y) const
  {
    return modulo((y - _origin.y()), _stepY) == 0; 
  }

  bool isOnGrid(const Point<Int>& p) const
  {
    return isOnGridX(p.x()) and isOnGridY(p.y());
  }

  bool isOnGrid(const Box<Int>& b) const
  {
    return isOnGrid(b.min_corner()) and isOnGrid(b.max_corner());
  }

private:
  String     _name;
  Point<Int> _origin;
  Int        _stepX;
  Int        _stepY;
  
  unsigned modulo(const int v, unsigned m) const
  {
    int mod = v % static_cast<int>(m);
    m &= mod >> std::numeric_limits<int>::digits;
    return mod + m;
  }
};

class Via;

class RouteGrid {
  friend class Parser;

public:
  RouteGrid() : _stepX(0), _stepY(0), _xLayerIdx(-1), _yLayerIdx(-1) {}
  ~RouteGrid() {}

  const String&             name()                              const { return _name; }
  Point<Int>                origin()                            const { return _origin; } 
  Int                       originX()                           const { return _origin.x(); }
  Int                       originY()                           const { return _origin.y(); }
  Int                       stepX()                             const { return _stepX; }
  Int                       stepY()                             const { return _stepY; }
  Int                       xLayerIdx()                         const { return _xLayerIdx; }
  Int                       yLayerIdx()                         const { return _yLayerIdx; }
  Int                       lowerLayerIdx()                     const { return std::min(_xLayerIdx, _yLayerIdx); }
  Int                       upperLayerIdx()                     const { return std::max(_xLayerIdx, _yLayerIdx); }
  Int                       cutLayerIdx()                       const { return (_xLayerIdx + _yLayerIdx) / 2; }
  Int                       numXGrids()                         const { return _vXGrids.size(); }
  Int                       numYGrids()                         const { return _vYGrids.size(); }
  Int                       xGrid(const Int i)                  const { return _vXGrids.at(i); }
  Int                       yGrid(const Int i)                  const { return _vYGrids.at(i); }
  Int                       xWidth(const Int i)                 const { return _vXWidths.at(i); }
  Int                       yWidth(const Int i)                 const { return _vYWidths.at(i); }
  TrackUseE                 xUse(const Int i)                   const { return _vXUses.at(i); }               
  TrackUseE                 yUse(const Int i)                   const { return _vYUses.at(i); }               
  bool                      isXPowerTrack(const Int i)          const { return _vXUses.at(i) == TrackUseE::power; }
  bool                      isXSignalTrack(const Int i)         const { return _vXUses.at(i) == TrackUseE::signal; }
  bool                      isXFreeTrack(const Int i)           const { return _vXUses.at(i) == TrackUseE::free; }
  bool                      isYPowerTrack(const Int i)          const { return _vYUses.at(i) == TrackUseE::power; }
  bool                      isYSignalTrack(const Int i)         const { return _vYUses.at(i) == TrackUseE::signal; }
  bool                      isYFreeTrack(const Int i)           const { return _vYUses.at(i) == TrackUseE::free; }

  bool                      isHor(const Int i)                  const { return i == _yLayerIdx; }
  bool                      isVer(const Int i)                  const { return i == _xLayerIdx; }
  bool                      isLowerLayerHor()                   const { return _yLayerIdx == lowerLayerIdx(); }
  bool                      isLowerLayerVer()                   const { return _xLayerIdx == lowerLayerIdx(); }
  bool                      isUpperLayerHor()                   const { return _yLayerIdx == upperLayerIdx(); }
  bool                      isUpperLayerVer()                   const { return _xLayerIdx == upperLayerIdx(); }
  bool                      isOnXTrack(const Int x)             const { return std::binary_search(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)); }
  bool                      isOnYTrack(const Int y)             const { return std::binary_search(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)); }
  bool                      isOnGrid(const Int x, const Int y)  const { return isOnXTrack(x) and isOnYTrack(y); }
  bool                      isOnGrid(const Point<Int>& p)       const { return isOnXTrack(p.x()) and isOnYTrack(p.y()); }

  const Via&                via(const Int i)                    const { return *_vpVias.at(i); }
  const Via*                pVia(const Int i)                   const { return _vpVias.at(i); }
  const Vector<const Via*>& vpVias()                            const { return _vpVias; }

  Int prevX(Int x) const { return floorX(x - 1); }
  Int nextX(Int x) const { return ceilX(x + 1); }
  Int prevY(Int y) const { return floorY(y - 1); }
  Int nextY(Int y) const { return ceilY(y + 1); }

  Int prevX(Int x, const Int n) const { for (Int i = 0; i < n; ++i) x = prevX(x); return x; }
  Int nextX(Int x, const Int n) const { for (Int i = 0; i < n; ++i) x = nextX(x); return x; }
  Int prevY(Int y, const Int n) const { for (Int i = 0; i < n; ++i) y = prevY(y); return y; }
  Int nextY(Int y, const Int n) const { for (Int i = 0; i < n; ++i) y = nextY(y); return y; }

  Int prevSignalX(Int x) const { do { x = prevX(x); } while (isXLocPowerTrack(x)); return x; }
  Int nextSignalX(Int x) const { do { x = nextX(x); } while (isXLocPowerTrack(x)); return x; }
  Int prevSignalY(Int y) const { do { y = prevY(y); } while (isYLocPowerTrack(y)); return y; }
  Int nextSignalY(Int y) const { do { y = nextY(y); } while (isYLocPowerTrack(y)); return y; }
 
  Int prevSignalX(Int x, const Int n) const { for (Int i = 0; i < n; ++i) x = prevSignalX(x); return x; }
  Int nextSignalX(Int x, const Int n) const { for (Int i = 0; i < n; ++i) x = nextSignalX(x); return x; }
  Int prevSignalY(Int y, const Int n) const { for (Int i = 0; i < n; ++i) y = prevSignalY(y); return y; }
  Int nextSignalY(Int y, const Int n) const { for (Int i = 0; i < n; ++i) y = nextSignalY(y); return y; }

  Int floorX(Int x) const 
  {
    x -= _origin.x();
    const Int r = modulo(x, _stepX);
    const Int d = x >= 0 ? x / _stepX : (r == 0 ? x / _stepX : x / _stepX - 1);
    Int idx = std::upper_bound(_vXGrids.begin(), _vXGrids.end(), r) - _vXGrids.begin() - 1;
    if (idx < 0) {
      return (d - 1) * _stepX + _vXGrids.back() + _origin.x();
    }
    return d * _stepX + _vXGrids.at(idx) + _origin.x();
  }

  Int ceilX(Int x) const 
  {
    x -= _origin.x();
    const Int r = modulo(x, _stepX);
    const Int d = x >= 0 ? x / _stepX : (r == 0 ? x / _stepX : x / _stepX - 1);
    Int idx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), r) - _vXGrids.begin();
    if (idx >= static_cast<Int>(_vXGrids.size())) {
      return (d + 1) * _stepX + _vXGrids.front() + _origin.x();
    }
    return d * _stepX + _vXGrids.at(idx) + _origin.x();
  }

  Int floorY(Int y) const 
  {
    y -= _origin.y();
    const Int r = modulo(y, _stepY);
    const Int d = y >= 0 ? y / _stepY : (r == 0 ? y / _stepY : y / _stepY - 1);
    Int idx = std::upper_bound(_vYGrids.begin(), _vYGrids.end(), r) - _vYGrids.begin() - 1;
    if (idx < 0) {
      return (d - 1) * _stepY + _vYGrids.back() + _origin.y();
    }
    //if (!(0 <= idx and idx < _vYGrids.size())) {
      //std::cerr << y << std::endl;
      //std::cerr << d << std::endl;
      //std::cerr << r << std::endl;
      //std::cerr << _stepY << std::endl << std::endl;;

      //for (const Int i : _vYGrids) {
        //std::cerr << i << std::endl;
      //}
    //}
    return d * _stepY + _vYGrids.at(idx) + _origin.y();
  }

  Int ceilY(Int y) const 
  {
    y -= _origin.y();
    const Int r = modulo(y, _stepY);
    const Int d = y >= 0 ? y / _stepY : (r == 0 ? y / _stepY : y / _stepY - 1);
    Int idx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), r) - _vYGrids.begin();
    if (idx >= static_cast<Int>(_vYGrids.size())) {
      return (d + 1) * _stepY + _vYGrids.front() + _origin.y();
    }
    return d * _stepY + _vYGrids.at(idx) + _origin.y();
  }

  Int xLocWidth(const Int x) const
  {
    const Int idx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    return _vXWidths.at(idx);
  }

  Int yLocWidth(const Int y) const
  {
    const Int idx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return _vYWidths.at(idx);
  }

  Int wireWidth(const Point3d<Int>& pt) const
  {
    if (pt.z() == _xLayerIdx) {
      return xLocWidth(pt.x()); 
    }
    else {
      assert(pt.z() == _yLayerIdx);
      return yLocWidth(pt.y());
    }
  }
  
  bool isXLocPowerTrack(const Int x) const
  {
    if (!isOnXTrack(x)) {
      return false;
    }
    const Int idx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    return isXPowerTrack(idx);   
  }

  bool isXLocSignalTrack(const Int x) const
  {
    if (!isOnXTrack(x)) {
      return false;
    }
    const Int idx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    return isXSignalTrack(idx);   
  }

  bool isXLocFreeTrack(const Int x) const
  {
    if (!isOnXTrack(x)) {
      return false;
    }
    const Int idx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    return isXFreeTrack(idx);   
  }

  bool isYLocPowerTrack(const Int y) const
  {
    if (!isOnYTrack(y)) {
      return false;
    }
    const Int idx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return isYPowerTrack(idx);   
  }

  bool isYLocSignalTrack(const Int y) const
  {
    if (!isOnYTrack(y)) {
      return false;
    }
    const Int idx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return isYSignalTrack(idx);   
  }

  bool isYLocFreeTrack(const Int y) const
  {
    if (!isOnYTrack(y)) {
      return false;
    }
    const Int idx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return isYFreeTrack(idx);   
  }


  //bool isValidVia(const String& n, Int x, Int y) const
  //{
    //x %= _stepX;
    //y %= _stepY;
    //auto it = _viaMap.find(n);
    //if (it != _viaMap.end())
      //return it->second.at(x, y);
    //return false;
  //}
  
  const Vector<const Via*>& vpValidVias(const Int x, const Int y) const {
    assert(isOnGrid(x, y));
    const Int xIdx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    const Int yIdx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return _viaMap.at(xIdx, yIdx);
  }

  bool hasValidVias(const Int x, const Int y) const {
    assert(isOnGrid(x, y));
    const Int xIdx = std::lower_bound(_vXGrids.begin(), _vXGrids.end(), modulo((x - _origin.x()), _stepX)) - _vXGrids.begin();
    const Int yIdx = std::lower_bound(_vYGrids.begin(), _vYGrids.end(), modulo((y - _origin.y()), _stepY)) - _vYGrids.begin();
    return _viaMap.at(xIdx, yIdx).size() > 0;
  }


private:
  String                _name;
  Point<Int>            _origin;
  Int                   _stepX; // width of a routing grid
  Int                   _stepY; // height of a routing grid
  Int                   _xLayerIdx, _yLayerIdx;
  Vector<Int>           _vXGrids, _vYGrids; // subgrids inside a routing grid, sorted
  Vector<Int>           _vXWidths, _vYWidths;
  Vector<TrackUseE>     _vXUses, _vYUses;

  // key: via name; val: valid grids
  //FlatHashMap<String, Array2d<Byte>> _viaMap;
  Vector<const Via*>          _vpVias;
  Array2d<Vector<const Via*>> _viaMap;

  unsigned modulo(const int v, unsigned m) const
  {
    int mod = v % static_cast<int>(m);
    m &= mod >> std::numeric_limits<int>::digits;
    return mod + m;
  }
};

PROJECT_NAMESPACE_END
