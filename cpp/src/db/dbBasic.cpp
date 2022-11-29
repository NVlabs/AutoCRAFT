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

#include "dbBasic.hpp"

PROJECT_NAMESPACE_START

namespace orient2d {

Orient2dE rotate90(const Orient2dE o) 
{
  switch (o) {
    case Orient2dE::n:  return Orient2dE::w; 
    case Orient2dE::w:  return Orient2dE::s; 
    case Orient2dE::s:  return Orient2dE::e; 
    case Orient2dE::e:  return Orient2dE::n; 
    case Orient2dE::fn: return Orient2dE::fe;
    case Orient2dE::fw: return Orient2dE::fn;
    case Orient2dE::fs: return Orient2dE::fw;
    case Orient2dE::fe: return Orient2dE::fs;
    default: assert(false);                 
  }
  return Orient2dE::undef;
}

Orient2dE rotate180(const Orient2dE o) 
{
  switch (o) {
    case Orient2dE::n:  return Orient2dE::s;  
    case Orient2dE::w:  return Orient2dE::e;  
    case Orient2dE::s:  return Orient2dE::n;  
    case Orient2dE::e:  return Orient2dE::w;  
    case Orient2dE::fn: return Orient2dE::fs; 
    case Orient2dE::fw: return Orient2dE::fe; 
    case Orient2dE::fs: return Orient2dE::fn; 
    case Orient2dE::fe: return Orient2dE::fw; 
    default: assert(false);                 
  }
  return Orient2dE::undef;
}

Orient2dE rotate270(const Orient2dE o)
{
  switch (o) {
    case Orient2dE::n:  return Orient2dE::e; 
    case Orient2dE::w:  return Orient2dE::n; 
    case Orient2dE::s:  return Orient2dE::w; 
    case Orient2dE::e:  return Orient2dE::s; 
    case Orient2dE::fn: return Orient2dE::fw;
    case Orient2dE::fw: return Orient2dE::fs;
    case Orient2dE::fs: return Orient2dE::fe;
    case Orient2dE::fe: return Orient2dE::fn;
    default: assert(false);                 
  }
  return Orient2dE::undef;
}

Orient2dE flipX(const Orient2dE o)
{
  switch (o) {
    case Orient2dE::n:  return Orient2dE::fn;  
    case Orient2dE::w:  return Orient2dE::fw; 
    case Orient2dE::s:  return Orient2dE::fs; 
    case Orient2dE::e:  return Orient2dE::fe; 
    case Orient2dE::fn: return Orient2dE::n;  
    case Orient2dE::fw: return Orient2dE::w;  
    case Orient2dE::fs: return Orient2dE::s;  
    case Orient2dE::fe: return Orient2dE::e;  
    default: assert(false);                 
  }
  return Orient2dE::undef;
}

Orient2dE flipY(const Orient2dE o)
{
  switch (o) {
    case Orient2dE::n:  return Orient2dE::fs;  
    case Orient2dE::w:  return Orient2dE::fe; 
    case Orient2dE::s:  return Orient2dE::fn; 
    case Orient2dE::e:  return Orient2dE::fw; 
    case Orient2dE::fn: return Orient2dE::s;  
    case Orient2dE::fw: return Orient2dE::e;  
    case Orient2dE::fs: return Orient2dE::n;  
    case Orient2dE::fe: return Orient2dE::w;  
    default: assert(false);                 
  }
  return Orient2dE::undef;
}

}

namespace direction3d {

Direction3dE findDir(const Point3d<Int>& u, const Point3d<Int>& v)
{
  if (u.z() == v.z()) {
    if (u.x() == v.x()) {
      assert(u.y() != v.y());
      return u.y() < v.y() ? Direction3dE::up : Direction3dE::down;
    }
    else {
      assert(u.x() != v.x());
      return u.x() < v.x() ? Direction3dE::right : Direction3dE::left;
    }
  }
  else {
    assert(u.x() == v.x() and u.y() == v.y());
    return u.z() < v.z() ? Direction3dE::via_up : Direction3dE::via_down;
  }
}

}

PROJECT_NAMESPACE_END
