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

#include "point.hpp"
#include "interval.hpp"

PROJECT_NAMESPACE_START

template<typename T>
class Box {
public:
  typedef Interval<T> interval_type;
  Box(T l = 0, T b = 0, T r = 0, T t = 0)
    : _bl(l, b), _tr(r, t) {
    assert(l <= r && b <= t);
  }
  Box(const Point<T>& p0, const Point<T>& p1) {
    assert(p0.x() <= p1.x() && p0.y() <= p1.y());
    _bl = p0;
    _tr = p1;
  }
  ~Box() {}

  // Basic setting functions
  void  setBL(const Point<T>& p)        { _bl = p; }
  void  setTR(const Point<T>& p)        { _tr = p; }
  void  setXL(T l)                      { _bl.setX(l); }
  void  setXH(T r)                      { _tr.setX(r); }
  void  setYL(T b)                      { _bl.setY(b); }
  void  setYH(T t)                      { _tr.setY(t); }
  void  set(T l, T b, T r, T t)         { setXL(l); setYL(b); setXH(r); setYH(t); }
  void  set(const Box<T>& b)            { _bl = b._bl; _tr = b._tr; }
  // Basic access functions
  T          xl()               const { return _bl.x(); }
  T          yl()               const { return _bl.y(); }
  T          xh()               const { return _tr.x(); }
  T          yh()               const { return _tr.y(); }
  T          width()            const { return xh() - xl(); }
  T          height()           const { return yh() - yl(); }
  T          centerX()          const { return (xl() + xh()) / 2; }
  T          centerY()          const { return (yl() + yh()) / 2; }
  T          hpwl()             const { return width() + height(); }
  T          perimeter()        const { return 2 * hpwl(); }
  T          area()             const { return width() * height(); }
  
  // Points
  Point<T>&         bl()               { return _bl; }
  const Point<T>&   bl()         const { return _bl; }
  Point<T>&         tr()               { return _tr; }
  const Point<T>&   tr()         const { return _tr; }
  Point<T>&         min_corner()       { return _bl; }
  const Point<T>&   min_corner() const { return _bl; }
  Point<T>&         max_corner()       { return _tr; }
  const Point<T>&   max_corner() const { return _tr; }
  Point<T>          center()     const { return Point<T>(centerX(), centerY());  }

  // utils
  void shiftX(const T x);
  void shiftY(const T y);
  void shift(const T x, const T y);
  void rotate90(const T x, const T y, const bool bClockWise); // rotate 90 degree with respect to (x, y)
  void rotate180(const T x, const T y);                       // rotate 180 degree with respect to (x, y)
  void flipX(const T x);                                      // flip by line x = x
  void flipY(const T y);                                      // flip by line y = y
  void expand(const T s);
  void expand(const T s, const int dim);
  void expandX(const T s);
  void expandY(const T s);
  void shrink(const T s);
  void shrinkX(const T s);
  void shrinkY(const T s);
  void difference(const Box& r, Vector<Box>& result);
  void multi_diff(const Vector<Box>& vBox, Vector<Box>& result) const;
  void multi_diff(const std::list<const Box*>& vBox, Vector<Box>& result) const;
  void multi_diff(const std::list<Uint>& ord, const Vector<Box>& vBox, Vector<Box>& result) const;
  void coverPoint(const Point<T> &pt) 
  {
    _bl.setX(std::min(pt.x(), _bl.x()));
    _bl.setY(std::min(pt.y(), _bl.y()));
    _tr.setX(std::max(pt.x(), _tr.x()));
    _tr.setY(std::max(pt.y(), _tr.y()));
  }

  //static functions
  static T     Mdistance(const Box& box1, const Box& box2);
  static T     Mcenterdistance(const Box& box1, const Box& box2);
  static T     Mdistance(const Box& box1, const Point<T>& pt);
  static bool  bOverlap(const Box& box1, const Box& box2);
  static bool  bConnect(const Box& box1, const Box& box2);
  static bool  bCover(const Box& box1, const Box& box2);
  static bool  bContain(const Box& box1, const Box& box2);
  static bool  bInside(const Box& box, const Point<T>& pt);
  static bool  bConnect(const Box& box, const Point<T>& pt);
  static bool  bOnBoundary(const Box& box, const Point<T>& pt);
  static bool  bOnBoundary(const Box& box, const Point<T>& pt, const Int t);
  static bool  bOnBoundaryL(const Box& box, const Point<T>& pt);
  static bool  bOnBoundaryR(const Box& box, const Point<T>& pt);
  static bool  bOnBoundaryB(const Box& box, const Point<T>& pt);
  static bool  bOnBoundaryT(const Box& box, const Point<T>& pt);
  static T     overlapArea(const Box& box1, const Box& box2);
  static void  intersection(const Box& box1, const Box& box2, Vector<Box>& result);
  static void  intersection2(const Box& box1, const Box& box2, Vector<Box>& result);
  static void  difference2(const Box& box1, const Box& box2, Vector<Box>& result);

  //operator
  bool operator < (const Box<T>& box) const {
    //if (_bl.x() != box._bl.x())
      //return _bl.x() < box._bl.x();
    //else if (_bl.y() != box._bl.y())
      //return _bl.y() < box._bl.y();
    //else if (_tr.x() != box._tr.x())
      //return _tr.x() < box._tr.x();
    //else if (_tr.y() != box._tr.y())
      //return _tr.y() < box._tr.y();
    //else
      //return false;
    return std::tie(_bl, _tr) < std::tie(box._bl, box._tr);
  }

  bool operator > (const Box<T>& box) const {
    //if (_bl.x() != box._bl.x())
      //return _bl.x() > box._bl.x();
    //else if (_bl.y() != box._bl.y())
      //return _bl.y() > box._bl.y();
    //else if (_tr.x() != box._tr.x())
      //return _tr.x() > box._tr.x();
    //else if (_tr.y() != box._tr.y())
      //return _tr.y() > box._tr.y();
    //else
      //return false;
    return std::tie(_bl, _tr) > std::tie(box._bl, box._tr);
  }

  bool operator <= (const Box<T>& box) const {
    return !(*this > box);
  }

  bool operator >= (const Box<T>& box) const {
    return !(*this < box);
  }
  
  bool operator == (const Box<T>& box) const {
    //return _bl.x() == box.xl() && _bl.y() == box.yl() && _tr.x() == box.xh() && _tr.y() == box.yh();
    return std::tie(_bl, _tr) == std::tie(box._bl, box._tr);
  }
  
  bool operator != (const Box<T>& box) const {
    return !(*this == box);
  }
  
  friend std::ostream& operator << (std::ostream& os, const Box& r) {
    os << '(' << r._bl.x() << ' ' << r._bl.y() << ' ' << r._tr.x() << ' ' << r._tr.y() << ')';
    return os;
  }

  // hasher
  struct hasher {
    size_t operator() (const Box& b) const {
      size_t seed = 0;
      boost::hash_combine(seed, b._bl.x());
      boost::hash_combine(seed, b._bl.y());
      boost::hash_combine(seed, b._tr.x());
      boost::hash_combine(seed, b._tr.y());
      return seed;
    }
  };
  friend size_t hash_value(const Box& b) {
    return phmap::HashState().combine(0, b._bl.x(), b._bl.y(), b._tr.x(), b._tr.y());
  }

  //Debugging
  void printBoxInfo() const {
    std::cout << "  Bound : " << '(' << _bl.x() << ' ' << _bl.y() << ' ' << _tr.x() << ' ' << _tr.y() << ')';
    std::cout << "  CenterXY : " << ' ' << (_bl.x() + _tr.x()) / 2  << ' ' << (_bl.y() + _tr.y()) / 2 << ')' << std::endl;
  }

private:
  Point<T> _bl; // bottom left
  Point<T> _tr; // top right
};

// member fucntions definition
template<typename T>
void Box<T>::shiftX(const T x) {
  setXL(_bl.x() + x);
  setXH(_tr.x() + x);
}

template<typename T>
void Box<T>::shiftY(const T y) {
  setYL(_bl.y() + y);
  setYH(_tr.y() + y);
}

template<typename T>
void Box<T>::shift(const T x, const T y) {
  shiftX(x);
  shiftY(y);
}

template<typename T>
void Box<T>::rotate90(const T x, const T y, const bool bClockWise) {
  Point<T> p0(_bl);
  Point<T> p1(_tr);
  p0.rotate90(x, y, bClockWise);
  p1.rotate90(x, y, bClockWise);
  setXL(std::min(p0.x(), p1.x()));
  setYL(std::min(p0.y(), p1.y()));
  setXH(std::max(p0.x(), p1.x()));
  setYH(std::max(p0.y(), p1.y()));
}

template<typename T>
void Box<T>::rotate180(const T x, const T y) {
  Point<T> p0(_bl);
  Point<T> p1(_tr);
  p0.rotate180(x, y);
  p1.rotate180(x, y);
  setXL(std::min(p0.x(), p1.x()));
  setXH(std::max(p0.x(), p1.x()));
  setYL(std::min(p0.y(), p1.y()));
  setYH(std::max(p0.y(), p1.y()));
}

template<typename T>
void Box<T>::flipX(const T x) {
  const Int l = _bl.x();
  const Int r = _tr.x();
  setXL(x + (x - r));
  setXH(x + (x - l));
}

template<typename T>
void Box<T>::flipY(const T y) {
  const Int b = _bl.y();
  const Int t = _tr.y();
  setYL(y + (y - t));
  setYH(y + (y - b));
}

template<typename T>
void Box<T>::expand(const T s) {
  setXL(_bl.x() - s);
  setYL(_bl.y() - s);
  setXH(_tr.x() + s);
  setYH(_tr.y() + s);
}

template<typename T>
void Box<T>::expand(const T s, const int dim) {
  if (dim == 0) {
    setXL(_bl.x() - s);
    setXH(_tr.x() + s);
  }
  else {
    assert(dim == 1);
    setYL(_bl.y() - s);
    setYH(_tr.y() + s);
  }
}

template<typename T>
void Box<T>::expandX(const T s) {
  setXL(_bl.x() - s);
  setXH(_tr.x() + s);
}

template<typename T>
void Box<T>::expandY(const T s) {
  setYL(_bl.y() - s);
  setYH(_tr.y() + s);
}

template<typename T>
void Box<T>::shrink(const T s) {
  setXL(_bl.x() + s);
  setYL(_bl.y() + s);
  setXH(_tr.x() - s);
  setYH(_tr.y() - s);
}

template<typename T>
void Box<T>::shrinkX(const T s) {
  setXL(_bl.x() + s);
  setXH(_tr.x() - s);
}

template<typename T>
void Box<T>::shrinkY(const T s) {
  setYL(_bl.y() + s);
  setYH(_tr.y() - s);
}

template<typename T>
void Box<T>::difference(const Box<T>& r, Vector<Box<T> >& result) {
  T a = std::min(_bl.x(), r._bl.x());
  T b = std::max(_bl.x(), r._bl.x());
  T c = std::min(_tr.x(), r._tr.x());
  T d = std::max(_tr.x(), r._tr.x());

  T e = std::min(_bl.y(), r._bl.y());
  T f = std::max(_bl.y(), r._bl.y());
  T g = std::min(_tr.y(), r._tr.y());
  T h = std::max(_tr.y(), r._tr.y());

  if (f - e > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, e, b, f);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
    if (c - b > 0) {
      Box<T> tmp(b, e, c, f);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, e, d, f);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
  }
  if (g - f > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, f, b, g);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, f, d, g);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
  }
  if (h - g > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, g, b, h);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
    if (c - b > 0) {
      Box<T> tmp(b, g, c, h);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, g, d, h);
      if (bOverlap(*this, tmp))
        result.push_back(tmp);
    }
  }
}

template<typename T>
void Box<T>::multi_diff(const Vector<Box<T>>& vBox, Vector<Box<T>>& result) const {
  Vector<T> hor = { _bl.y(), _tr.y() };
  Vector<T> ver = { _bl.x(), _tr.x() };
  for (Uint i = 0; i < vBox.size(); ++i) {
    Box<T>& box= vBox[i];
    if (box.xl() > _bl.x() && box.xl() < _tr.x())
      ver.push_back(box.xl());
    if (box.xh() > _bl.x() && box.xh() < _tr.x())
      ver.push_back(box.xh());
    if (box.yl() > _bl.y() && box.yl() < _tr.y())
      hor.push_back(box.yl());
    if (box.yh() > _bl.y() && box.yh() < _tr.y())
      hor.push_back(box.yh());
  }
  std::sort(hor.begin(), hor.end());
  std::sort(ver.begin(), ver.end());
  hor.resize(std::unique(hor.begin(), hor.end()) - hor.begin());
  ver.resize(std::unique(ver.begin(), ver.end()) - ver.begin());
  for (Uint i = 1; i < ver.size(); ++i)
    for (Uint j = 1; j < hor.size(); ++j)
      result.push_back(Box(ver[i - 1], hor[j - 1], ver[i], hor[j]));
}

template<typename T>
void Box<T>::multi_diff(const std::list<Uint>& ord, const Vector<Box<T>>& vBox, Vector<Box<T>>& result) const {
  Vector<T> hor = { _bl.y(), _tr.y() };
  Vector<T> ver = { _bl.x(), _tr.x() };
  for (Uint i : ord) {
    const Box<T>& box = vBox[i];
    if (box.xl() > _bl.x() && box.xl() < _tr.x())
      ver.push_back(box.xl());
    if (box.xh() > _bl.x() && box.xh() < _tr.x())
      ver.push_back(box.xh());
    if (box.yl() > _bl.y() && box.yl() < _tr.y())
      hor.push_back(box.yl());
    if (box.yh() > _bl.y() && box.yh() < _tr.y())
      hor.push_back(box.yh());
  }
  std::sort(hor.begin(), hor.end());
  std::sort(ver.begin(), ver.end());
  hor.resize(std::unique(hor.begin(), hor.end()) - hor.begin());
  ver.resize(std::unique(ver.begin(), ver.end()) - ver.begin());
  for (Uint i = 1; i < ver.size(); ++i)
    for (Uint j = 1; j < hor.size(); ++j)
      result.push_back(Box(ver[i - 1], hor[j - 1], ver[i], hor[j]));
}

template<typename T>
void Box<T>::multi_diff(const std::list<const Box<T>*>& vBox, Vector<Box<T>>& result) const {
  Vector<T> hor = { _bl.y(), _tr.y() };
  Vector<T> ver = { _bl.x(), _tr.x() };
  for (const Box<T>* box: vBox) {
    if (box->xl() > _bl.x() && box->xl() < _tr.x())
      ver.push_back(box->xl());
    if (box->xh() > _bl.x() && box->xh() < _tr.x())
      ver.push_back(box->xh());
    if (box->yl() > _bl.y() && box->yl() < _tr.y())
      hor.push_back(box->yl());
    if (box->yh() > _bl.y() && box->yh() < _tr.y())
      hor.push_back(box->yh());
  }
  std::sort(hor.begin(), hor.end());
  std::sort(ver.begin(), ver.end());
  hor.resize(std::unique(hor.begin(), hor.end()) - hor.begin());
  ver.resize(std::unique(ver.begin(), ver.end()) - ver.begin());
  for (Uint i = 1; i < ver.size(); ++i)
    for (Uint j = 1; j < hor.size(); ++j)
      result.push_back(Box<T>(ver[i - 1], hor[j - 1], ver[i], hor[j]));
}

// static functions
template<typename T>
T Box<T>::Mdistance(const Box<T>& box1, const Box<T>& box2) {
  T d1 = max({box1._bl.x() - box2._tr.x(), box2._bl.x() - box1._tr.x(), (T)0});
  T d2 = max({box1._bl.y() - box2._tr.y(), box2._bl.y() - box1._tr.y(), (T)0});
  return d1 + d2;
}

template<typename T>
T Box<T>::Mcenterdistance(const Box<T>& box1, const Box<T>& box2) {
  return abs((box1.xl() + box1.xh()) / 2 - (box2.xl() + box2.xh()) / 2)
       + abs((box1.yl() + box1.yh()) / 2 - (box2.yl() + box2.yh()) / 2);
}

template<typename T>
T Box<T>::Mdistance(const Box<T>& box, const Point<T>& pt) {
  T d1 = 0, d2 = 0;
  if (pt.x() < box.xl())
    d1 = box.xl() - pt.x();
  else if (pt.x() > box.xh())
    d1 = pt.x() - box.xh();
  if (pt.y() < box.yl())
    d2 = box.yl() - pt.y();
  else if
    (pt.y() > box.yh()) d2 = pt.y() - box.yh();
  return d1 + d2;
}

template<typename T>
bool Box<T>::bOverlap(const Box<T>& box1, const Box<T>& box2) {
  if (box1._bl.y() >= box2._tr.y()) return false;
  if (box1._tr.y() <= box2._bl.y()) return false;
  if (box1._bl.x() >= box2._tr.x()) return false;
  if (box1._tr.x() <= box2._bl.x()) return false;
  return true;
}

template<typename T>
bool Box<T>::bConnect(const Box<T>& box1, const Box<T>& box2) {
  if (box1._bl.y() > box2._tr.y()) return false;
  if (box1._tr.y() < box2._bl.y()) return false;
  if (box1._bl.x() > box2._tr.x()) return false;
  if (box1._tr.x() < box2._bl.x()) return false;
  return true;
}

template<typename T>
bool Box<T>::bOnBoundary(const Box& box, const Point<T>& pt) {
  if (pt.x() == box.xl()) {
    if (pt.y() < box.yl()) return false;
    if (pt.y() > box.yh()) return false;
  }
  else if (pt.x() == box.xh()) {
    if (pt.y() < box.yl()) return false;
    if (pt.y() > box.yh()) return false;
  }
  else {
    if (pt.y() != box.yl()) return false;
    if (pt.y() != box.yh()) return false;
  }
  return true;
}

template<typename T>
bool Box<T>::bOnBoundary(const Box& box, const Point<T>& pt, const Int t) {
  switch (t) {
    case 0: return Box<T>::bOnBoundaryL(box, pt);
    case 1: return Box<T>::bOnBoundaryB(box, pt);
    case 2: return Box<T>::bOnBoundaryR(box, pt);
    case 3: return Box<T>::bOnBoundaryT(box, pt);
    default: assert(false);
  }
  return false;
}

template<typename T>
bool Box<T>::bOnBoundaryL(const Box& box, const Point<T>& pt){
  if (pt.x() != box.xl()) return false;
  if (pt.y() < box.yl()) return false;
  if (pt.y() > box.yh()) return false;
  return true;
}

template<typename T>
bool Box<T>::bOnBoundaryR(const Box& box, const Point<T>& pt) {
  if (pt.x() != box.xh()) return false;
  if (pt.y() < box.yl()) return false;
  if (pt.y() > box.yh()) return false;
  return true;
}

template<typename T>
bool Box<T>::bOnBoundaryB(const Box& box, const Point<T>& pt) {
  if (pt.y() != box.yl()) return false;
  if (pt.x() < box.xl()) return false;
  if (pt.x() > box.xh()) return false;
  return true;
}

template<typename T>
bool Box<T>::bOnBoundaryT(const Box& box, const Point<T>& pt) {
  if (pt.y() != box.yh()) return false;
  if (pt.x() < box.xl()) return false;
  if (pt.x() > box.xh()) return false;
  return true;
}

template<typename T>
bool Box<T>::bCover(const Box& box1, const Box& box2) {
  return bConnect(box1, box2.min_corner()) &&
         bConnect(box1, box2.max_corner());
}

template<typename T>
bool Box<T>::bContain(const Box& box1, const Box& box2) {
  return bInside(box1, box2.min_corner()) &&
         bInside(box1, box2.max_corner());
}

template<typename T>
bool Box<T>::bInside(const Box<T>& box, const Point<T>& pt) {
  return (pt.x() > box.xl() && pt.x() < box.xh() &&
          pt.y() > box.yl() && pt.y() < box.yh());
}

template<typename T>
bool Box<T>::bConnect(const Box<T>& box, const Point<T>& pt) {
  return (pt.x() >= box.xl() && pt.x() <= box.xh() &&
          pt.y() >= box.yl() && pt.y() <= box.yh());
}

template<typename T>
T Box<T>::overlapArea(const Box<T>& box1, const Box<T>& box2) {
  T overlapH = std::min(box1._tr.x(), box2._tr.x()) - std::max(box1._bl.x(), box2._bl.x());
  T overlapV = std::min(box1._tr.y(), box2._tr.y()) - std::max(box1._bl.y(), box2._bl.y());
  if (overlapH <= 0 || overlapV <= 0)
    return 0;
  return overlapH * overlapV;
}

template<typename T>
void Box<T>::intersection(const Box<T>& box1, const Box<T>& box2, Vector<Box<T> >& result) {
  if (!bOverlap(box1, box2))
    return;
  Box<T> box;
  box.setXL(std::max(box1._bl.x(), box2._bl.x()));
  box.setXH(std::min(box1._tr.x(), box2._tr.x()));
  box.setYL(std::max(box1._bl.y(), box2._bl.y()));
  box.setYH(std::min(box1._tr.y(), box2._tr.y()));
  result.push_back(box);
}

template<typename T>
void Box<T>::intersection2(const Box<T>& box1, const Box<T>& box2, Vector<Box<T> >& result) {
  if (!bConnect(box1, box2))
    return;
  Box<T> box;
  box.setXL(std::max(box1._bl.x(), box2._bl.x()));
  box.setXH(std::min(box1._tr.x(), box2._tr.x()));
  box.setYL(std::max(box1._bl.y(), box2._bl.y()));
  box.setYH(std::min(box1._tr.y(), box2._tr.y()));
  result.push_back(box);
}

template<typename T>
void Box<T>::difference2(const Box<T>& box1, const Box<T>& box2, Vector<Box<T> >& result) {
  // X = intersection, 0-7 = possible difference areas
  // h +-+-+-+
  // . |5|6|7|
  // g +-+-+-+
  // . |3|X|4|
  // f +-+-+-+
  // . |0|1|2|
  // e +-+-+-+
  // . a b c d
  T a = std::min(box1._bl.x(), box2._bl.x());
  T b = std::max(box1._bl.x(), box2._bl.x());
  T c = std::min(box1._tr.x(), box2._tr.x());
  T d = std::max(box1._tr.x(), box2._tr.x());

  T e = std::min(box1._bl.y(), box2._bl.y());
  T f = std::max(box1._bl.y(), box2._bl.y());
  T g = std::min(box1._tr.y(), box2._tr.y());
  T h = std::max(box1._tr.y(), box2._tr.y());

  if (f - e > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, e, b, f);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
    if (c - b > 0) {
      Box<T> tmp(b, e, c, f);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, e, d, f);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
  }
  if (g - f > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, f, b, g);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, f, d, g);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
  }
  if (h - g > 0) {
    if (b - a > 0) {
      Box<T> tmp(a, g, b, h);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
    if (c - b > 0) {
      Box<T> tmp(b, g, c, h);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
    if (d - c > 0) {
      Box<T> tmp(c, g, d, h);
      if (bOverlap(box1, tmp) || bOverlap(box2, tmp))
        result.push_back(tmp);
    }
  }
}

PROJECT_NAMESPACE_END

// boost geometry traits
#include <boost/geometry/geometries/register/box.hpp>

namespace boost { namespace geometry { namespace traits {
  
  template<typename CoordType>
    struct tag<PROJECT_NAMESPACE::Box<CoordType>> {
      typedef box_tag type;
    };

  template<typename CoordType>
    struct point_type<PROJECT_NAMESPACE::Box<CoordType>> {
      typedef PROJECT_NAMESPACE::Point<CoordType> type;
    };
  
  template<typename CoordType>
    struct indexed_access<PROJECT_NAMESPACE::Box<CoordType>, min_corner, 0> {

      static inline CoordType get(const PROJECT_NAMESPACE::Box<CoordType>& b) {
        return b.min_corner().x();
      }
      
      static inline void set(PROJECT_NAMESPACE::Box<CoordType>& b, CoordType const& value) {
        b.setXL(value);
      }
    };
  
  template<typename CoordType>
    struct indexed_access<PROJECT_NAMESPACE::Box<CoordType>, min_corner, 1> {

      static inline CoordType get(const PROJECT_NAMESPACE::Box<CoordType>& b) {
        return b.min_corner().y();
      }
      
      static inline void set(PROJECT_NAMESPACE::Box<CoordType>& b, CoordType const& value) {
        b.setYL(value);
      }
    };
  
  template<typename CoordType>
    struct indexed_access<PROJECT_NAMESPACE::Box<CoordType>, max_corner, 0> {

      static inline CoordType get(const PROJECT_NAMESPACE::Box<CoordType>& b) {
        return b.max_corner().x();
      }
      
      static inline void set(PROJECT_NAMESPACE::Box<CoordType>& b, CoordType const& value) {
        b.setXH(value);
      }
    };
  
  template<typename CoordType>
    struct indexed_access<PROJECT_NAMESPACE::Box<CoordType>, max_corner, 1> {

      static inline CoordType get(const PROJECT_NAMESPACE::Box<CoordType>& b) {
        return b.max_corner().y();
      }
      
      static inline void set(PROJECT_NAMESPACE::Box<CoordType>& b, CoordType const& value) {
        b.setYH(value);
      }
    };
  

} } }

// boost polygon traits
#include <boost/polygon/rectangle_traits.hpp>
#include <boost/polygon/polygon.hpp>
namespace boost { namespace polygon {
  template<typename CoordType>
  struct geometry_concept<PROJECT_NAMESPACE::Box<CoordType>> { typedef rectangle_concept type; };

  template<typename CoordType>
  struct rectangle_traits<PROJECT_NAMESPACE::Box<CoordType>, typename gtl_same_type<typename PROJECT_NAMESPACE::Box<CoordType>::interval_type, typename PROJECT_NAMESPACE::Box<CoordType>::interval_type>::type>
  {
      typedef CoordType coordinate_type;
      typedef typename PROJECT_NAMESPACE::Box<CoordType>::interval_type interval_type;
      static inline interval_type get(const PROJECT_NAMESPACE::Box<CoordType> &rectangle, orientation_2d orient)
      {
          if (orient == HORIZONTAL)
          {
              return boost::polygon::interval_mutable_traits<interval_type>::construct(rectangle.xl(), rectangle.xh());
          }
          else
          {
              return boost::polygon::interval_mutable_traits<interval_type>::construct(rectangle.yl(), rectangle.yh());
          }
      }
  };
  template<typename CoordType>
  struct rectangle_mutable_traits<PROJECT_NAMESPACE::Box<CoordType>>
  {
      template<typename T2>
      static inline void set(PROJECT_NAMESPACE::Box<CoordType> &rectangle, orientation_2d orient, const T2& interval)
      {
          if (orient == HORIZONTAL)
          {
              rectangle.setXL(boost::polygon::interval_traits<T2>::get(interval, LOW));
              rectangle.setXH(boost::polygon::interval_traits<T2>::get(interval, HIGH));
          }
          else
          {
              rectangle.setYL(boost::polygon::interval_traits<T2>::get(interval, LOW));
              rectangle.setYH(boost::polygon::interval_traits<T2>::get(interval, HIGH));
          }
      }
      template<typename T2, typename T3>
      static inline PROJECT_NAMESPACE::Box<CoordType> construct(const T2& interval_horizontal,
                                                                const T3& interval_vertical)
      {
          return PROJECT_NAMESPACE::Box<CoordType>
              (
                   boost::polygon::interval_traits<T2>::get(interval_horizontal, LOW),
                   boost::polygon::interval_traits<T2>::get(interval_vertical, LOW),
                   boost::polygon::interval_traits<T2>::get(interval_horizontal, HIGH),
                   boost::polygon::interval_traits<T2>::get(interval_vertical, HIGH)
              );
      }
  };

}} // boost
