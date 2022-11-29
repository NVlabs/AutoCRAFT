/**
 * @file   segment.hpp
 * @brief  Geometric Data Structure: 2D segment type
 * @author Hao Chen
 * @date   09/05/2019
 *
 **/


#pragma once

#include "point.hpp"

PROJECT_NAMESPACE_START

template<typename T>
class Segment {
public:
  Segment(T x0 = 0, T y0 = 0, T x1 = 0, T y1 = 0)
    : _p0(x0, y0), _p1(x1, y1) {}
  Segment(const Point<T>& p0, const Point<T>& p1)
    : _p0(p0), _p1(p1) {}
  ~Segment() {}

  // Basic access functions
  Point<T>&        p0()           { return _p0; }
  const Point<T>&  p0()     const { return _p0; }
  Point<T>&        p1()           { return _p1; }
  const Point<T>&  p1()     const { return _p1; }

  T         length()        const { return Point<T>::Mdistance(_p0, _p1); }
  T         xl()            const { return std::min(_p0.x(), _p1.x()); }
  T         xh()            const { return std::max(_p0.x(), _p1.x()); }
  T         yl()            const { return std::min(_p0.y(), _p1.y()); }
  T         yh()            const { return std::max(_p0.y(), _p1.y()); }
  T         centerX()       const { return (xl() + xh()) / 2; }
  T         centerY()       const { return (yl() + yh()) / 2; }
  Point<T>  center()        const { return Point<T>(centerX(), centerY()); }
  Point<T>  min_corner()    const { return Point<T>(xl(), yl()); }
  Point<T>  max_corner()    const { return Point<T>(xh(), yh()); }
  bool      bHor()          const { return _p0.x() != _p1.x() && _p0.y() == _p1.y(); }
  bool      bVer()          const { return _p0.x() == _p1.x() && _p0.y() != _p1.y(); }
  bool      b90()           const { return bHor() or bVer(); }

  // util functions
  void shiftX(const T x)               { _p0.shiftX(x); _p1.shiftX(x); }
  void shiftY(const T y)               { _p0.shiftY(y); _p1.shiftY(y); }
  void shiftXY(const T x, const T y)   { shiftX(x); shiftY(y); }

  // operators
  friend std::ostream&  operator <<  (std::ostream& os, const Segment& s)         { os << s._p0 << s._p1; return os; }
  bool                  operator ==  (const Segment& s)                    const  { return _p0 == s._p0 && _p1 == s._p1; }
  bool                  operator !=  (const Segment& s)                    const  { return !(*this == s); }
  void                  operator =   (const Segment& s)                           { _p0 = s._p0; _p1 = s._p1; }
  bool                  operator <   (const Segment& s)                    const  { return std::tie(_p0, _p1) < std::tie(s._p0, s._p1); }
  bool                  operator >   (const Segment& s)                    const  { return std::tie(_p0, _p1) > std::tie(s._p0, s._p1); }
  bool                  operator <=  (const Segment& s)                    const  { return !(*this > s); }
  bool                  operator >=  (const Segment& s)                    const  { return !(*this < s); }
  
  // static funcs
  static bool  bConnect(const Segment& s0, const Segment& s1);

  // hasher
  struct hasher {
    size_t operator() (const Segment& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, s._p0.x());
      boost::hash_combine(seed, s._p0.y());
      boost::hash_combine(seed, s._p1.x());
      boost::hash_combine(seed, s._p1.y());
      return seed;
    }
  };
  friend size_t hash_value(const Segment& s) {
    return phmap::HashState().combine(0, s._p0.x(), s._p0.y(), s._p1.x(), s._p1.y());
  }

private:
  Point<T> _p0, _p1;  
};

template<typename T>
bool Segment<T>::bConnect(const Segment& s0, const Segment& s1) {
  Int o1 = Point<T>::orientation(s0.p0(), s0.p1(), s1.p0());
  Int o2 = Point<T>::orientation(s0.p0(), s0.p1(), s1.p1());
  Int o3 = Point<T>::orientation(s1.p0(), s1.p1(), s0.p0());
  Int o4 = Point<T>::orientation(s1.p0(), s1.p1(), s0.p1());

  // general case
  if (o1 != o2 and o3 != o4)
    return true;

  // special cases
  if (o1 == 0 and Point<T>::bOnSegment(s0.p0(), s1.p0(), s0.p1()))
    return true; 
  if (o2 == 0 and Point<T>::bOnSegment(s0.p0(), s1.p1(), s0.p1()))
    return true; 
  if (o3 == 0 and Point<T>::bOnSegment(s1.p0(), s0.p0(), s1.p1()))
    return true; 
  if (o4 == 0 and Point<T>::bOnSegment(s1.p0(), s0.p1(), s1.p1()))
    return true; 

  return false;
}


PROJECT_NAMESPACE_END

