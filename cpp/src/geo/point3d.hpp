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

#include <boost/geometry.hpp>
#include <boost/functional/hash.hpp>
#include <parallel_hashmap/phmap_utils.h>
#include "global/type.hpp"
#include "geo/point.hpp"

PROJECT_NAMESPACE_START

template<typename T>
class Point3d {
 public:
  Point3d(T x = 0, T y = 0, T z = 0) : _d{x, y, z} {}
  Point3d(const Point3d& p) { memcpy(_d, p.d(), 3 * sizeof(T)); }
  ~Point3d() {}

  // Basic setting functions
  void set(const T v, const int i)                 { assert(i >= 0 && i <= 3); _d[i] = v; }
  void setX(const T x)                             { _d[0] = x;                           }
  void setY(const T y)                             { _d[1] = y;                           }
  void setZ(const T z)                             { _d[2] = z;                           }
  void setXYZ(const T x, const T y, const T z)     { _d[0] = x; _d[1] = y; _d[2] = z;     }
  void shift(const T v, const int i)               { _d[i] += v;                          }
  void shiftX(const T x)                           { _d[0] += x;                          }
  void shiftY(const T y)                           { _d[1] += y;                          }
  void shiftZ(const T z)                           { _d[2] += z;                          }
  void shiftXYZ(const T x, const T y, const T z)   { _d[0] += x; _d[1] += y; _d[2] += z;  }

  // Basic access functions
  T         x()               const { return _d[0]; }
  T         y()               const { return _d[1]; }
  T         z()               const { return _d[2]; }
  T*        d()                     { return _d;    }
  const T*  d()               const { return _d;    }
  T         val(const int i)  const { assert(i >= 0 && i < 3); return _d[i]; }
  T&        val(const int i)        { assert(i >= 0 && i < 3); return _d[i]; }
  Point<T>  to2d()            const { return Point<Int>(_d[0], _d[1]); }

  // util functions
  static T Mdistance(const Point3d& p0, const Point3d& p1) {
    return (p0._d[0] > p1._d[0] ? p0._d[0] - p1._d[0] : p1._d[0] - p0._d[0]) + 
      (p0._d[1] > p1._d[1] ? p0._d[1] - p1._d[1] : p1._d[1] - p0._d[1]) +
      (p0._d[2] > p1._d[2] ? p0._d[2] - p1._d[2] : p1._d[2] - p0._d[2]);
  }
  static T dot(const Point3d& p0, const Point3d& p1) {
    return p0.x() * p1.x() + p0.y() * p1.y() + p0.z() * p1.z();
  }
  static T norm(const Point3d& p) {
    return p.x() * p.x() + p.y() * p.y() + p.z() * p.z();
  }
  static Point3d cross(const Point3d& p0, const Point3d& p1) {
    return Point3d(p0.y() * p1.z() - p1.y() * p0.z(), 
                   p0.z() * p1.x() - p1.z() * p0.x(), 
                   p0.x() * p1.y() - p1.x() * p0.y());
  }

  void flipX(const T x) { _d[0] = 2 * x - _d[0]; }
  void flipY(const T y) { _d[1] = 2 * y - _d[1]; }
  void flipZ(const T z) { _d[2] = 2 * z - _d[2]; }

  // operators
  friend std::ostream&  operator <<  (std::ostream& os, const Point3d& p)       { os << '(' << p._d[0] << ' ' << p._d[1] << ' ' << p._d[2] << ')'; return os; }
  bool                  operator ==  (const Point3d& p)                   const { return std::tie(_d[0], _d[1], _d[2]) == std::tie(p._d[0], p._d[1], p._d[2]); }
  bool                  operator !=  (const Point3d& p)                   const { return !(*this == p); }
  bool                  operator <   (const Point3d& p)                   const { return std::tie(_d[2], _d[0], _d[1]) < std::tie(p._d[2], p._d[0], p._d[1]); }
  bool                  operator >   (const Point3d& p)                   const { return std::tie(_d[2], _d[0], _d[1]) > std::tie(p._d[2], p._d[0], p._d[1]); }
  bool                  operator <=  (const Point3d& p)                   const { return !(*this > p); }
  bool                  operator >=  (const Point3d& p)                   const { return !(*this < p); }
  void                  operator =   (const Point3d& p)                         { setXYZ(p._d[0], p._d[1], p._d[2]); }
  Point3d               operator +   (const Point3d& p)                   const { return Point3d(_d[0] + p._d[0], _d[1] + p._d[1], _d[2] + p._d[2]); }
  Point3d               operator -   (const Point3d& p)                   const { return Point3d(_d[0] - p._d[0], _d[1] - p._d[1], _d[2] + p._d[2]); }

  // comparators
  struct Compare {
    bool operator() (const Point3d& p0, const Point3d& p1) const {
      if (p0.z() != p1.z()) return p0.z() < p1.z();
      else if (p0.x() != p1.x()) return p0.x() < p1.x();
      else return p0.y() < p1.y();
    }
  };
  // hasher
  struct hasher {
    std::size_t operator() (const Point3d& p) const {
      return boost::hash_range(p._d, p._d + 3);
    }
  };

  friend size_t hash_value(const Point3d& p) {
    return phmap::HashState().combine(0, p._d[0], p._d[1], p._d[2]);
  }
  // string
  String str() const 
  {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

 private:
  T _d[3];
};

PROJECT_NAMESPACE_END

namespace boost { namespace geometry { namespace traits {
  template<typename CoordType>
  struct tag<PROJECT_NAMESPACE::Point3d<CoordType>> {
    typedef point_tag type;
  };

  template<typename CoordType>
  struct coordinate_type<PROJECT_NAMESPACE::Point3d<CoordType>> {
    typedef CoordType type;
  };

  template<typename CoordType>
  struct coordinate_system<PROJECT_NAMESPACE::Point3d<CoordType>> {
    typedef boost::geometry::cs::cartesian type;
  };
  
  template<typename CoordType>
  struct dimension<PROJECT_NAMESPACE::Point3d<CoordType>>
    : boost::mpl::int_<3> {};

  template<typename CoordType>
  struct access<PROJECT_NAMESPACE::Point3d<CoordType>, 0> {
    static inline CoordType get(const PROJECT_NAMESPACE::Point3d<CoordType>& p) {
      return p.x();
    }
    static inline void set(PROJECT_NAMESPACE::Point3d<CoordType>& p, CoordType const& value) {
      p.setX(value);
    }
  };

  template<typename CoordType>
  struct access<PROJECT_NAMESPACE::Point3d<CoordType>, 1> {
    static inline CoordType get(const PROJECT_NAMESPACE::Point3d<CoordType>& p) {
      return p.y();
    }
    static inline void set(PROJECT_NAMESPACE::Point3d<CoordType> & p, CoordType const& value) {
      p.setY(value);
    }
  };

  template<typename CoordType>
  struct access<PROJECT_NAMESPACE::Point3d<CoordType>, 2> {
    static inline CoordType get(PROJECT_NAMESPACE::Point3d<CoordType>  const& p) {
      return p.z();
    }
    static inline void set(PROJECT_NAMESPACE::Point3d<CoordType> & p, CoordType const& value) {
      p.setZ(value);
    }
  };

}}}


