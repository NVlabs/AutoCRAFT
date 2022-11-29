/**
 * @file   segment3d.hpp
 * @brief  Geometric Data Structure: 3D segment type
 * @author Hao Chen
 * @date   09/05/2019
 *
 **/

#pragma once

#include "point3d.hpp"

PROJECT_NAMESPACE_START

template<class T>
class Segment3d {
public:
  Segment3d(T x0 = 0, T y0 = 0, T z0 = 0, T x1 = 0, T y1 = 0, T z1 = 0)
    : _p0(x0, y0, z0), _p1(x1, y1, z1) {}
  Segment3d(const Point3d<T>& p0, const Point3d<T>& p1)
    : _p0(p0), _p1(p1) {}
  ~Segment3d() {}

  // Basic setting functions

  // Basic access functions
  Point3d<T>&       p0()                { return _p0; }
  const Point3d<T>& p0()          const { return _p0; }
  Point3d<T>&       p1()                { return _p1; }
  const Point3d<T>& p1()          const { return _p1; }
  
  T                 xl()          const { return std::min(_p0.x(), _p1.x()); }
  T                 xh()          const { return std::max(_p0.x(), _p1.x()); }
  T                 yl()          const { return std::min(_p0.y(), _p1.y()); }
  T                 yh()          const { return std::max(_p0.y(), _p1.y()); }
  T                 zl()          const { return std::min(_p0.z(), _p1.z()); }
  T                 zh()          const { return std::max(_p0.z(), _p1.z()); }
  T                 centerX()     const { return (xl() + xh()) / 2; }
  T                 centerY()     const { return (yl() + yh()) / 2; }
  T                 centerZ()     const { return (zl() + zh()) / 2; }
  Point3d<T>        center()      const { return Point3d<T>(centerX(), centerY(), centerZ()); }
  Point3d<T>        min_corner()  const { return Point3d<T>(xl(), yl(), zl()); }
  Point3d<T>        max_corner()  const { return Point3d<T>(xh(), yh(), zh()); }
  T                 length()      const { return Point3d<T>::Mdistance(_p0, _p1); }
  bool              bHor()        const { return _p0.x() != _p1.x() and _p0.y() == _p1.y() and _p0.z() == _p1.z(); }
  bool              bVer()        const { return _p0.x() == _p1.x() and _p0.y() != _p1.y() and _p0.z() == _p1.z(); }
  bool              bVia()        const { return _p0.x() == _p1.x() and _p0.y() == _p1.y() and _p0.z() != _p1.z(); }
  bool              b90()         const { return bHor() or bVer() or bVia(); }


  // operators
  friend std::ostream&  operator <<  (std::ostream& os, const Segment3d& s)       { os << s._p0 << s._p1; return os; }
  bool                  operator ==  (const Segment3d& s)                   const { return _p0 == s._p0 && _p1 == s._p1; }
  bool                  operator !=  (const Segment3d& s)                   const { return !(*this == s); }
  void                  operator =   (const Segment3d& s)                         { _p0 = s._p0; _p1 = s._p1; }
  bool                  operator <   (const Segment3d& s)                   const { return std::tie(_p0, _p1) < std::tie(s._p0, s._p1); } 
  bool                  operator >   (const Segment3d& s)                   const { return std::tie(_p0, _p1) > std::tie(s._p0, s._p1); } 
  bool                  operator <=  (const Segment3d& s)                   const { return !(*this > s); }
  bool                  operator >=  (const Segment3d& s)                   const { return !(*this < s); }

  // 90 utils
  static bool bConnect(const Segment3d& s0, const Segment3d& s1);

  // hasher
  struct hasher {
    size_t operator() (const Segment3d& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, s._p0.x());
      boost::hash_combine(seed, s._p0.y());
      boost::hash_combine(seed, s._p0.z());
      boost::hash_combine(seed, s._p1.x());
      boost::hash_combine(seed, s._p1.y());
      boost::hash_combine(seed, s._p1.z());
      return seed;
    }
  };
  friend size_t hash_value(const Segment3d& s) {
    return phmap::HashState().combine(0, s._p0.x(), s._p0.y(), s._p0.z(), s._p1.x(), s._p1.y(), s._p1.z());
  }
private:
  Point3d<T> _p0, _p1;
};

template<typename T>
bool Segment3d<T>::bConnect(const Segment3d<T>& s0, const Segment3d<T>& s1) {
  assert(s0.b90() and s1.b90());
  if (s0.bHor()) {
    if (s1.bHor()) { // s1: hor, s2: hor
      if (s0.p0().z() != s1.p0().z()) return false;
      if (s0.p0().y() != s1.p0().y()) return false;
      if (s0.xl() > s1.xh()) return false;
      if (s0.xh() < s1.xl()) return false;
      return true;
    }
    else if (s1.bVer()) { // s1: hor, s2: ver
      if (s0.p0().z() != s1.p0().z()) return false;
      if (s0.p0().y() > s1.yh()) return false;
      if (s0.p0().y() < s1.yl()) return false;
      if (s0.xl() > s1.p0().x()) return false;
      if (s0.xh() < s1.p0().x()) return false;
      return true;
    }
    else { // s1: hor, s2: via
      if (s0.p0().y() != s1.p0().y()) return false;
      if (s0.p0().z() > s1.zh()) return false;
      if (s0.p0().z() < s1.zl()) return false;
      if (s0.xl() > s1.p0().x()) return false;
      if (s0.xh() < s1.p0().x()) return false;
      return true;
    }
  }
  else if (s0.bVer()) {
    if (s1.bHor()) { // s1: ver, s2: hor
      if (s0.p0().z() != s1.p0().z()) return false;
      if (s0.p0().x() > s1.xh()) return false;
      if (s0.p0().x() < s1.xl()) return false;
      if (s0.yl() > s1.p0().y()) return false;
      if (s0.yh() < s1.p0().y()) return false;
      return true;
    }
    else if (s1.bVer()) { // s1: ver, s2: ver
      if (s0.p0().z() != s1.p0().z()) return false;
      if (s0.p0().x() != s1.p0().x()) return false;
      if (s0.yl() > s1.yh()) return false;
      if (s0.yh() < s1.yl()) return false;
      return true;
    }
    else { // s1: ver, s2: via
      if (s0.p0().x() != s1.p0().x()) return false;
      if (s0.p0().z() > s1.zh()) return false;
      if (s0.p0().z() < s1.zl()) return false;
      if (s0.yl() > s1.p0().y()) return false;
      if (s0.yh() < s1.p0().y()) return false;
      return true;
    }
  }
  else {
    if (s1.bHor()) { // s1: via, s2: hor
      if (s0.p0().y() != s1.p0().y()) return false;
      if (s0.zl() > s1.p0().z()) return false;
      if (s0.zh() < s1.p0().z()) return false;
      if (s0.p0().x() < s1.xl()) return false;
      if (s0.p0().x() > s1.xh()) return false;
      return true;
    }
    else if (s1.bVer()) { // s1: via, s2: ver
      if (s0.p0().x() != s1.p0().x()) return false;
      if (s0.zl() > s1.p0().z()) return false;
      if (s0.zh() < s1.p0().z()) return false;
      if (s0.p0().y() < s1.yl()) return false;
      if (s0.p0().y() > s1.yh()) return false;
      return true;
    }
    else { // s1: via, s2: via
      if (s0.p0().x() != s1.p0().x()) return false;
      if (s0.p0().y() != s1.p0().y()) return false;
      if (s0.zl() > s1.zh()) return false;
      if (s0.zh() < s1.zl()) return false;
      return true;
    }
  }
}


PROJECT_NAMESPACE_END

